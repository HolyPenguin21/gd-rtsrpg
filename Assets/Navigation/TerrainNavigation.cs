using System;
using System.Collections.Generic;
using UnityEngine;

namespace GameDev.RtsRpg
{
    /// <summary>One runtime navigation snapshot for one builder; publishes invalidation, never repaths.</summary>
    [DisallowMultipleComponent]
    [RequireComponent(typeof(TerrainBuilder))]
    public sealed class TerrainNavigation : MonoBehaviour
    {
        public NavigationGrid Grid { get; private set; }
        public bool HasCurrentTransform => builder != null && builder.transform.localToWorldMatrix == matrix;
        public string Error { get; private set; }
        public event Action Changed;
        private TerrainBuilder builder;
        private Matrix4x4 matrix;

        // Live occupancy only. The static grid never receives unit blockers or revision churn.
        internal int TrafficRevision { get; private set; }
        internal void NotifyTrafficChanged() => TrafficRevision++;
        private static readonly Unity.Profiling.ProfilerMarker TickMarker = new("Navigation.MotionTick");
        private static readonly Unity.Profiling.ProfilerMarker ContactMarker = new("Navigation.FindContact");
        private const float BucketSize = 4f;
        private readonly Dictionary<Vector2Int, List<UnitMovement>> traffic = new();
        private readonly Stack<List<UnitMovement>> spareBuckets = new();
        private readonly List<UnitMovement> units = new();
        private readonly Dictionary<(RectInt Area, bool AlongX), PassageState> lanes = new();
        private readonly Stack<PassageState> freePassages = new();
        private readonly List<(RectInt Area, bool AlongX)> emptyPassages = new();
        private int tick, firstUnit, laneRevision;

        private sealed class PassageState
        {
            internal readonly List<UnitMovement> Waiting = new();
            internal readonly List<(UnitMovement Unit, int Direction, int Width)> Inside = new();
        }

        internal void Track(UnitMovement unit, Vector2 centre)
        {
            units.Add(unit);
            Register(unit, centre);
        }

        internal void Untrack(UnitMovement unit)
        {
            units.Remove(unit);
            Unregister(unit);
        }

        private void FixedUpdate()
        {
            if (Grid == null || !Grid.IsAlive || !HasCurrentTransform || !Application.isFocused || units.Count == 0) return;
            using var marker = TickMarker.Auto();
            tick++;
            if (laneRevision != Grid.Revision)
            {
                ResetPassages();
                laneRevision = Grid.Revision;
            }
            foreach (var pair in lanes)
            {
                var state = pair.Value;
                state.Waiting.Clear();
                var area = ToRect(pair.Key.Area);
                for (int i = state.Inside.Count - 1; i >= 0; i--)
                {
                    var unit = state.Inside[i].Unit;
                    if (unit == null || !unit.isActiveAndEnabled || !UnitMovement.Overlaps(unit.Occupancy, area))
                        state.Inside.RemoveAt(i);
                }
            }
            foreach (var unit in units) unit.CaptureTick();
            foreach (var unit in units)
            {
                unit.PrepareTick(tick);
                if (unit.PassageDirection == 0) continue;
                var key = PassageKey(unit);
                if (!lanes.TryGetValue(key, out var state))
                {
                    state = freePassages.Count == 0 ? new PassageState() : freePassages.Pop();
                    lanes.Add(key, state);
                }
                if (!UnitMovement.Overlaps(unit.Occupancy, ToRect(unit.Passage))) state.Waiting.Add(unit);
                else EnterPassage(unit);
            }

            // Everyone predicts against the same snapshot. Rotate the order
            // of live collision commits so the first spawned unit cannot dominate.
            firstUnit = (firstUnit + 1) % units.Count;
            for (int i = 0; i < units.Count; i++) units[(firstUnit + i) % units.Count].MoveTick(Time.fixedDeltaTime);
            emptyPassages.Clear();
            foreach (var pair in lanes)
                if (pair.Value.Waiting.Count == 0 && pair.Value.Inside.Count == 0) emptyPassages.Add(pair.Key);
            foreach (var key in emptyPassages)
            {
                freePassages.Push(lanes[key]);
                lanes.Remove(key);
            }
        }

        private void ResetPassages()
        {
            foreach (var state in lanes.Values)
            {
                state.Inside.Clear();
                state.Waiting.Clear();
                freePassages.Push(state);
            }
            lanes.Clear();
        }

        private static (RectInt Area, bool AlongX) PassageKey(UnitMovement unit) =>
            (unit.Passage, Math.Abs(unit.PassageDirection) == 1);

        private static Rect ToRect(RectInt area) => new(area.x, area.y, area.width, area.height);

        internal bool CanEnterPassage(UnitMovement unit)
        {
            if (UnitMovement.Overlaps(unit.Occupancy, ToRect(unit.Passage))) return true;
            var key = PassageKey(unit);
            if (!lanes.TryGetValue(key, out var state)) return false;
            int width = key.AlongX ? key.Area.height : key.Area.width;
            int direction = unit.PassageDirection;
            foreach (var inside in state.Inside)
                if (inside.Direction != direction && inside.Width + unit.PassageWidth > width) return false;

            foreach (var other in state.Waiting)
            {
                if (other == unit || other.PassageDirection == direction || other.PassageWidth + unit.PassageWidth <= width) continue;
                // Once the opposite side has waited longer, let the current
                // wave drain. A stable tie-break makes the first grant unique.
                if (other.PassageSince < unit.PassageSince || (other.PassageSince == unit.PassageSince
                    && other.GetInstanceID() < unit.GetInstanceID())) return false;
            }
            return true;
        }

        internal void EnterPassage(UnitMovement unit)
        {
            if (!lanes.TryGetValue(PassageKey(unit), out var state)) return;
            foreach (var inside in state.Inside) if (inside.Unit == unit) return;
            state.Inside.Add((unit, unit.PassageDirection, unit.PassageWidth));
        }

        private static Vector2Int BucketAt(float x, float z) => new(
            Mathf.FloorToInt(x / BucketSize), Mathf.FloorToInt(z / BucketSize));

        internal void Register(UnitMovement unit, Vector2 centre)
        {
            var key = BucketAt(centre.x, centre.y);
            if (key == unit.Bucket && traffic.TryGetValue(key, out var existing) && existing.Contains(unit)) return;
            Unregister(unit);
            NotifyTrafficChanged();
            if (!traffic.TryGetValue(key, out var bucket))
                traffic.Add(key, bucket = spareBuckets.Count > 0 ? spareBuckets.Pop() : new List<UnitMovement>());
            bucket.Add(unit);
            unit.Bucket = key;
        }

        internal void Unregister(UnitMovement unit)
        {
            if (!traffic.TryGetValue(unit.Bucket, out var bucket) || !bucket.Remove(unit)) return;
            NotifyTrafficChanged();
            if (bucket.Count != 0) return;
            traffic.Remove(unit.Bucket);
            spareBuckets.Push(bucket);
        }

        internal UnitMovement FindContact(UnitMovement requester, Rect area,
            bool search = false, bool avoidAllies = false, float horizon = 0f)
        {
            using var marker = ContactMarker.Auto();
            // A diagonal translating horse can retain its old anchor up to a
            // whole cell away, plus its rotated body's half extent (~1.061).
            float margin = horizon > 0f ? 3.45f : 2.2f;
            var min = BucketAt(area.xMin - margin, area.yMin - margin);
            var max = BucketAt(area.xMax + margin, area.yMax + margin);
            for (int z = min.y; z <= max.y; z++)
            for (int x = min.x; x <= max.x; x++)
                if (traffic.TryGetValue(new Vector2Int(x, z), out var bucket))
                    foreach (var unit in bucket)
                        if (unit != requester && requester.SharesGrid(unit)
                            && (UnitMovement.Overlaps(area, PredictedArea(unit, horizon))
                                || (horizon > 0f && UnitMovement.Overlaps(area, PredictedArea(unit, horizon * 0.5f))))
                            && (!search || requester.Avoids(unit, avoidAllies))) return unit;
            return null;
        }

        private static Rect PredictedArea(UnitMovement unit, float horizon)
        {
            var area = unit.Occupancy;
            area.position += Vector2.ClampMagnitude(unit.PredictionVelocity * horizon, 1.25f);
            return area;
        }

        private void OnEnable()
        {
            builder = GetComponent<TerrainBuilder>();
            builder.SurfaceChanged += Rebuild;
            Rebuild();
        }
        private void OnDisable()
        {
            if (builder != null) builder.SurfaceChanged -= Rebuild;
            Clear();
        }
        private void LateUpdate()
        {
            // Only compare 16 floats. Build once on an actual transform change, never search here.
            if (builder != null && builder.transform.localToWorldMatrix != matrix) Rebuild();
        }
        private void Clear()
        {
            ResetPassages();
            if (Grid != null) { Grid.Changed -= Notify; Grid.Retire(); Grid = null; }
            Changed?.Invoke();
        }
        private void Notify() => Changed?.Invoke();
        [ContextMenu("Rebuild Navigation (does not replan)")]
        public void Rebuild()
        {
            var previous = Grid;
            Clear(); Error = null;
            if (builder == null) builder = GetComponent<TerrainBuilder>();
            matrix = builder.transform.localToWorldMatrix;
            if (builder.Surface == null) { Error = "Terrain is unavailable; rebuild Terrain Builder first."; return; }
            try
            {
                Grid = new NavigationGrid(builder.Surface, matrix, previous);
                Grid.Changed += Notify;
            }
            catch (ArgumentException e) { Error = e.Message; Debug.LogError(Error, this); }
            Changed?.Invoke();
        }
    }
}
