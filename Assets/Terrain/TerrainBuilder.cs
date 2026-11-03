using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

namespace GameDev.RtsRpg
{
    /// <summary>One static mesh from one validated surface. Explicit rebuild, never per frame.</summary>
    [DisallowMultipleComponent]
    [RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
    public sealed class TerrainBuilder : MonoBehaviour
    {
        [SerializeField] private TerrainMapData map;
        public TerrainSurface Surface { get; private set; }
        private Mesh ownedMesh;

        private void OnEnable() => Rebuild();
        private void OnDisable() => Release();
        private void OnDestroy() => Release();

        [ContextMenu("Rebuild Terrain")]
        public void Rebuild()
        {
            // Clear obsolete output too when the next input is invalid.
            Release();
            try
            {
                if (map == null) throw new ArgumentException("Terrain Builder needs a TerrainMapData asset.");
                TerrainSurface surface = map.CreateSurface();
                Mesh mesh = BuildMesh(surface);
                ownedMesh = mesh;
                GetComponent<MeshFilter>().sharedMesh = mesh;
                Surface = surface;
            }
            catch (ArgumentException error)
            {
                Debug.LogError(error.Message, this);
            }
        }

        private void Release()
        {
            Surface = null;
            if (ownedMesh == null) return;
            var filter = GetComponent<MeshFilter>();
            if (filter.sharedMesh == ownedMesh) filter.sharedMesh = null;
            if (Application.isPlaying) Destroy(ownedMesh);
            else DestroyImmediate(ownedMesh);
            ownedMesh = null;
        }

        /// <summary>Caller owns the returned mesh and must destroy it.</summary>
        public static Mesh BuildMesh(TerrainSurface surface)
        {
            if (surface == null) throw new ArgumentNullException(nameof(surface));
            var vertices = new List<Vector3>();
            var triangles = new List<int>();
            for (int z = 0; z < surface.CellsZ; z++)
            for (int x = 0; x < surface.CellsX; x++)
            {
                var cell = surface.GetCell(x, z);
                Vector3 sw = surface.Position(x, z, cell.Corner(0, 0));
                Vector3 nw = surface.Position(x, z + 1, cell.Corner(0, 1));
                Vector3 ne = surface.Position(x + 1, z + 1, cell.Corner(1, 1));
                Vector3 se = surface.Position(x + 1, z, cell.Corner(1, 0));
                Quad(sw, nw, ne, se, Vector3.up, vertices, triangles);

                // Each shared edge exactly once; west/south are only the outside perimeter.
                Wall(surface, x, z, TerrainDirection.North, vertices, triangles);
                Wall(surface, x, z, TerrainDirection.East, vertices, triangles);
                if (z == 0) Wall(surface, x, z, TerrainDirection.South, vertices, triangles);
                if (x == 0) Wall(surface, x, z, TerrainDirection.West, vertices, triangles);
            }
            var mesh = new Mesh { name = "Terrain (generated)", hideFlags = HideFlags.DontSave,
                indexFormat = vertices.Count > 65535 ? IndexFormat.UInt32 : IndexFormat.UInt16 };
            try
            {
                mesh.SetVertices(vertices);
                mesh.SetTriangles(triangles, 0);
                mesh.RecalculateNormals();
                mesh.RecalculateBounds();
                return mesh;
            }
            catch
            {
                if (Application.isPlaying) Destroy(mesh); else DestroyImmediate(mesh);
                throw;
            }
        }

        private static void Wall(TerrainSurface surface, int x, int z, TerrainDirection side,
            List<Vector3> vertices, List<int> triangles)
        {
            surface.GetEdge(x, z, side, out int a, out int b);
            Vector2Int step = TerrainSurface.Step(side);
            int c = surface.BaseQuarterLevel, d = c;
            if (surface.Contains(x + step.x, z + step.y))
                surface.GetEdge(x + step.x, z + step.y, TerrainSurface.Opposite(side), out c, out d);
            if (a == c && b == d) return;

            int ax = x, az = z, bx = x, bz = z;
            switch (side)
            {
                case TerrainDirection.North: az++; bx++; bz++; break;
                case TerrainDirection.East: ax++; bx++; bz++; break;
                case TerrainDirection.South: bx++; break;
                case TerrainDirection.West: bz++; break;
            }
            Vector3 outward = new Vector3(step.x, 0, step.y);
            if (a < c || b < d) outward = -outward;
            // Exact matching endpoints collapse to a triangle at a ramp tip. Never emit
            // overlapping double-sided walls or interpolate a cliff into a walkable top.
            Quad(surface.Position(ax, az, a), surface.Position(bx, bz, b),
                surface.Position(bx, bz, d), surface.Position(ax, az, c), outward, vertices, triangles);
        }

        private static void Quad(Vector3 a, Vector3 b, Vector3 c, Vector3 d, Vector3 outward,
            List<Vector3> vertices, List<int> triangles)
        {
            Triangle(a, b, c, outward, vertices, triangles);
            Triangle(a, c, d, outward, vertices, triangles);
        }

        private static void Triangle(Vector3 a, Vector3 b, Vector3 c, Vector3 outward,
            List<Vector3> vertices, List<int> triangles)
        {
            Vector3 cross = Vector3.Cross(b - a, c - a);
            if (cross.x == 0f && cross.y == 0f && cross.z == 0f) return;
            int first = vertices.Count;
            vertices.Add(a); vertices.Add(b); vertices.Add(c);
            triangles.Add(first);
            bool reverse = Vector3.Dot(cross, outward) < 0;
            triangles.Add(first + (reverse ? 2 : 1));
            triangles.Add(first + (reverse ? 1 : 2));
        }
    }
}
