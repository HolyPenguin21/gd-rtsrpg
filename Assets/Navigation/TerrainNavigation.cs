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
        private static readonly Unity.Profiling.ProfilerMarker ContactMarker = new("Navigation.FindContact");
        private const float BucketSize = 4f;
        private readonly Dictionary<Vector2Int, List<UnitMovement>> traffic = new();
        private readonly Stack<List<UnitMovement>> spareBuckets = new();

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
            bool search = false, bool avoidAllies = false)
        {
            using var marker = ContactMarker.Auto();
            // A diagonal translating horse can retain its old anchor up to a
            // whole cell away, plus its rotated body's half extent (~1.061).
            const float margin = 2.2f;
            var min = BucketAt(area.xMin - margin, area.yMin - margin);
            var max = BucketAt(area.xMax + margin, area.yMax + margin);
            for (int z = min.y; z <= max.y; z++)
            for (int x = min.x; x <= max.x; x++)
                if (traffic.TryGetValue(new Vector2Int(x, z), out var bucket))
                    foreach (var unit in bucket)
                        if (unit != requester && requester.SharesGrid(unit)
                            && UnitMovement.Overlaps(area, unit.Occupancy)
                            && (!search || requester.Avoids(unit, avoidAllies))) return unit;
            return null;
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
