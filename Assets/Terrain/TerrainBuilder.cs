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
        public enum Source { Explicit, DirectionReview, Procedural }
        [SerializeField] private Source source;
        [SerializeField] private TerrainMapData directionReviewMap;
        [SerializeField] private ProceduralMapSettings procedural = new ProceduralMapSettings();
        [SerializeField] private TerrainMapData map;
        [Tooltip("Slots: grass, cliff, ramp. Shared assets are never modified at runtime.")]
        [SerializeField] private Material grassMaterial;
        [SerializeField] private Material cliffMaterial;
        [SerializeField] private Material rampMaterial;
        public TerrainSurface Surface { get; private set; }
        public event Action SurfaceChanged;
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
                TerrainSurface surface;
                if (source == Source.Procedural) surface = ProceduralTerrainGenerator.Generate(procedural);
                else
                {
                    var input = source == Source.DirectionReview ? directionReviewMap : map;
                    if (input == null) throw new ArgumentException("Terrain Builder needs a TerrainMapData asset.");
                    surface = input.CreateSurface();
                }
                if (grassMaterial == null || cliffMaterial == null || rampMaterial == null)
                    throw new ArgumentException("Terrain Builder needs grass, cliff and ramp materials.");
                Mesh mesh = BuildMesh(surface);
                GetComponent<MeshRenderer>().sharedMaterials = new[] { grassMaterial, cliffMaterial, rampMaterial };
                ownedMesh = mesh;
                GetComponent<MeshFilter>().sharedMesh = mesh;
                Surface = surface;
                SurfaceChanged?.Invoke();
            }
            catch (ArgumentException error)
            {
                Debug.LogError(error.Message, this);
            }
        }

        [ContextMenu("Show Explicit 6x5")]
        public void ShowExplicit() { source = Source.Explicit; Rebuild(); }

        [ContextMenu("Show Four Ramp Directions")]
        public void ShowDirections() { source = Source.DirectionReview; Rebuild(); }

        [ContextMenu("Show Procedural Seed 104729")]
        private void ShowFirstSeed() { ShowProceduralSeed(104729); }

        [ContextMenu("Show Procedural Seed 130363")]
        private void ShowSecondSeed() { ShowProceduralSeed(130363); }

        public void ShowProceduralSeed(int seed)
        {
            source = Source.Procedural;
            procedural = new ProceduralMapSettings { seed = seed };
            Rebuild();
        }

        private void Release()
        {
            bool hadSurface = Surface != null;
            Surface = null;
            if (hadSurface) SurfaceChanged?.Invoke();
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
            var triangles = new[] { new List<int>(), new List<int>(), new List<int>() };
            for (int z = 0; z < surface.CellsZ; z++)
            for (int x = 0; x < surface.CellsX; x++)
            {
                var cell = surface.GetCell(x, z);
                Vector3 sw = surface.Position(x, z, cell.Corner(0, 0));
                Vector3 nw = surface.Position(x, z + 1, cell.Corner(0, 1));
                Vector3 ne = surface.Position(x + 1, z + 1, cell.Corner(1, 1));
                Vector3 se = surface.Position(x + 1, z, cell.Corner(1, 0));
                Quad(sw, nw, ne, se, Vector3.up, vertices, triangles[cell.IsRamp ? 2 : 0]);

                // Each shared edge exactly once; west/south are only the outside perimeter.
                Wall(surface, x, z, TerrainDirection.North, vertices, triangles[1]);
                Wall(surface, x, z, TerrainDirection.East, vertices, triangles[1]);
                if (z == 0) Wall(surface, x, z, TerrainDirection.South, vertices, triangles[1]);
                if (x == 0) Wall(surface, x, z, TerrainDirection.West, vertices, triangles[1]);
            }
            var mesh = new Mesh { name = "Terrain (generated)", hideFlags = HideFlags.DontSave,
                indexFormat = vertices.Count > 65535 ? IndexFormat.UInt32 : IndexFormat.UInt16 };
            try
            {
                mesh.SetVertices(vertices);
                mesh.subMeshCount = 3;
                for (int i = 0; i < 3; i++) mesh.SetTriangles(triangles[i], i);
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
            Vector3 p = surface.Position(ax, az, a), q = surface.Position(bx, bz, b);
            Vector3 r = surface.Position(bx, bz, d), t = surface.Position(ax, az, c);
            // All four boundary edges remain EXACT. An irregular inset centre makes real
            // stone facets inside the solid rock, never outside into a ramp or a lower top.
            // Adjacent segments share their untouched perimeter, including ramp tips.
            uint hash = DetailHash(ax, az, bx, bz, surface.DetailSeed);
            float along = 0.35f + (hash & 255) / 255f * 0.3f;
            float vertical = 0.28f + ((hash >> 8) & 255) / 255f * 0.4f;
            Vector3 upper = Vector3.Lerp(p, q, along), lower = Vector3.Lerp(t, r, along);
            float depth = Mathf.Min(surface.BlockWorldSize / 4f * 0.12f,
                Mathf.Abs(upper.y - lower.y) * 0.12f);
            Vector3 centre = Vector3.Lerp(upper, lower, vertical) - outward * depth;
            Triangle(p, q, centre, outward, vertices, triangles);
            Triangle(q, r, centre, outward, vertices, triangles);
            Triangle(r, t, centre, outward, vertices, triangles);
            Triangle(t, p, centre, outward, vertices, triangles);
        }

        private static uint DetailHash(int ax, int az, int bx, int bz, int seed)
        {
            unchecked
            {
                uint h = (uint)seed ^ (uint)ax * 73856093u ^ (uint)az * 19349663u
                    ^ (uint)bx * 83492791u ^ (uint)bz * 2654435761u;
                h ^= h >> 16; h *= 2246822519u; h ^= h >> 13;
                return h;
            }
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
