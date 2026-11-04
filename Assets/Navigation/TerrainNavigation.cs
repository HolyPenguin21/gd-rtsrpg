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
        private readonly Dictionary<NavigationGrid.Passage, PassageState> lanes = new();
        private readonly Stack<PassageState> freePassages = new();
        private readonly List<NavigationGrid.Passage> emptyPassages = new();
        private int tick, firstUnit, laneRevision;

        private sealed class PassageState
        {
            internal readonly List<UnitMovement> Waiting = new();
            internal readonly List<(UnitMovement Unit, Vector2Int Mouth, int Direction, int Width)> Inside = new();
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
            Grid.AdvancePassages(192);
            foreach (var pair in lanes)
            {
                var state = pair.Value;
                state.Waiting.Clear();
                for (int i = state.Inside.Count - 1; i >= 0; i--)
                {
                    var unit = state.Inside[i].Unit;
                    if (unit == null || !unit.isActiveAndEnabled || !unit.ReservesPassage(pair.Key))
                        state.Inside.RemoveAt(i);
                }
            }
            foreach (var unit in units) unit.CaptureTick();
            foreach (var unit in units)
            {
                unit.PrepareTick(tick);
                var key = unit.Passage;
                if (key == null || !key.Ready) continue;
                if (!lanes.TryGetValue(key, out var state))
                {
                    state = freePassages.Count == 0 ? new PassageState() : freePassages.Pop();
                    lanes.Add(key, state);
                }
                if (!key.Overlaps(unit.Occupancy)) state.Waiting.Add(unit);
                else EnterPassage(unit);
            }

            // Everyone predicts against the same snapshot. Rotate the order
            // of live collision commits so the first spawned unit cannot dominate.
            foreach (var unit in units) unit.PrepareSteering(Time.fixedDeltaTime);
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

        internal bool CanEnterPassage(UnitMovement unit)
        {
            var passage = unit.Passage;
            if (passage == null || !passage.Ready || !lanes.TryGetValue(passage, out var state)) return false;
            if (passage.Overlaps(unit.Occupancy)) return true;

            foreach (var inside in state.Inside)
                if (Conflicts(inside.Mouth, inside.Direction, inside.Width)) return false;
            foreach (var other in state.Waiting)
            {
                if (other == unit || !Conflicts(other.PassageMouth, other.PassageDirection, other.PassageWidth)) continue;
                if (other.PassageSince < unit.PassageSince || (other.PassageSince == unit.PassageSince
                    && other.GetInstanceID() < unit.GetInstanceID())) return false;
            }
            return true;

            bool Conflicts(Vector2Int mouth, int direction, int width)
            {
                if (!passage.Complex)
                    return direction != unit.PassageDirection && width + unit.PassageWidth > passage.Width;
                if (direction != unit.PassageDirection) return true;
                var delta = mouth - unit.PassageMouth;
                return Math.Abs(direction) == 1 ? delta.x != 0 || Math.Abs(delta.y) >= passage.Width
                    : delta.y != 0 || Math.Abs(delta.x) >= passage.Width;
            }
        }

        internal void EnterPassage(UnitMovement unit)
        {
            if (unit.Passage == null || !lanes.TryGetValue(unit.Passage, out var state)) return;
            foreach (var inside in state.Inside) if (inside.Unit == unit) return;
            state.Inside.Add((unit, unit.PassageMouth, unit.PassageDirection, unit.PassageWidth));
        }

        private readonly List<UnitMovement> steeringNeighbors = new();

        internal void CollectSteeringNeighbors(UnitMovement requester, Vector2 centre, float range)
        {
            steeringNeighbors.Clear();
            var min = BucketAt(centre.x - range, centre.y - range);
            var max = BucketAt(centre.x + range, centre.y + range);
            for (int z = min.y; z <= max.y; z++)
            for (int x = min.x; x <= max.x; x++)
                if (traffic.TryGetValue(new Vector2Int(x, z), out var bucket))
                    foreach (var other in bucket)
                        if (other != requester && requester.SharesGrid(other)) steeringNeighbors.Add(other);
        }

        internal float TimeToContact(Rect body, Vector2 velocity, float horizon)
        {
            float earliest = horizon;
            foreach (var other in steeringNeighbors)
            {
                var obstacle = other.Occupancy;
                var relative = velocity - other.PredictionVelocity;
                float enter = 0f, leave = earliest;
                if (Axis(body.xMin, body.xMax, obstacle.xMin, obstacle.xMax, relative.x, ref enter, ref leave)
                    && Axis(body.yMin, body.yMax, obstacle.yMin, obstacle.yMax, relative.y, ref enter, ref leave))
                    earliest = enter;
                if (earliest <= 0f) break;
            }
            return earliest;

            static bool Axis(float min, float max, float low, float high, float speed, ref float enter, ref float leave)
            {
                const float epsilon = 0.0001f;
                low += epsilon;
                high -= epsilon;
                if (Math.Abs(speed) < epsilon) return min < high && max > low;
                float a = (low - max) / speed, b = (high - min) / speed;
                enter = Math.Max(enter, Math.Min(a, b));
                leave = Math.Min(leave, Math.Max(a, b));
                return enter < leave;
            }
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
