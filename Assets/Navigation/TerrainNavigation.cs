using System;
using UnityEngine;

namespace GameDev.RtsRpg
{
    /// <summary>One runtime navigation snapshot for one builder; publishes invalidation, never repaths.</summary>
    [DisallowMultipleComponent]
    [RequireComponent(typeof(TerrainBuilder))]
    public sealed class TerrainNavigation : MonoBehaviour
    {
        public NavigationGrid Grid { get; private set; }
        public string Error { get; private set; }
        public event Action Changed;
        private TerrainBuilder builder;
        private Matrix4x4 matrix;

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
            Clear(); Error = null;
            if (builder == null) builder = GetComponent<TerrainBuilder>();
            matrix = builder.transform.localToWorldMatrix;
            if (builder.Surface == null) { Error = "Terrain is unavailable; rebuild Terrain Builder first."; return; }
            try
            {
                Grid = new NavigationGrid(builder.Surface, matrix);
                Grid.Changed += Notify;
            }
            catch (ArgumentException e) { Error = e.Message; Debug.LogError(Error, this); }
            Changed?.Invoke();
        }
    }
}
