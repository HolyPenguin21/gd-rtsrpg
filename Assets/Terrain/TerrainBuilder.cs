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
        public enum Source { Explicit, DirectionReview, Procedural, Landforms }
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
        public string BuildReport { get; private set; }
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
                var clock = System.Diagnostics.Stopwatch.StartNew();
                long memoryStart = GC.GetAllocatedBytesForCurrentThread();
                if (source == Source.Procedural || source == Source.Landforms)
                {
                    procedural.naturalLandforms = source == Source.Landforms;
                    surface = ProceduralTerrainGenerator.Generate(procedural);
                }
                else
                {
                    var input = source == Source.DirectionReview ? directionReviewMap : map;
                    if (input == null) throw new ArgumentException("Terrain Builder needs a TerrainMapData asset.");
                    surface = input.CreateSurface();
                }
                if (grassMaterial == null || cliffMaterial == null || rampMaterial == null)
                    throw new ArgumentException("Terrain Builder needs grass, cliff and ramp materials.");
                double generationMs = clock.Elapsed.TotalMilliseconds;
                long generationBytes = GC.GetAllocatedBytesForCurrentThread() - memoryStart;
                clock.Restart();
                memoryStart = GC.GetAllocatedBytesForCurrentThread();
                Mesh mesh = BuildMesh(surface);
                double meshMs = clock.Elapsed.TotalMilliseconds;
                long meshBytes = GC.GetAllocatedBytesForCurrentThread() - memoryStart;
                BuildReport = $"{surface.CellsX}×{surface.CellsZ} cells. Generation {generationMs:F1} ms / {generationBytes:N0} B; "
                    + $"mesh {meshMs:F1} ms / {meshBytes:N0} B managed; {mesh.vertexCount:N0} vertices / "
                    + $"{(mesh.GetIndexCount(0) + mesh.GetIndexCount(1) + mesh.GetIndexCount(2)) / 3:N0} triangles.";
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

        public void ShowLandformSeed(int seed)
        {
            source = Source.Landforms;
            procedural.seed = seed;
            procedural.width = Mathf.Max(64, procedural.width);
            procedural.length = Mathf.Max(64, procedural.length);
            Rebuild();
        }

        private void Release()
        {
            bool hadSurface = Surface != null;
            Surface = null;
            BuildReport = null;
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
            var edgeMask = new List<Vector2>();
            var rampJoinVertices = new List<int>();
            byte[] upperEdges = CollectUpperEdges(surface);
            var nearby = new List<Vector4>(12);
            var landings = new List<Vector4>(4);
            var triangles = new[] { new List<int>(), new List<int>(), new List<int>() };
            for (int z = 0; z < surface.CellsZ; z++)
            for (int x = 0; x < surface.CellsX; x++)
            {
                var cell = surface.GetCell(x, z);
                Vector3 sw = surface.Position(x, z, cell.Corner(0, 0));
                Vector3 nw = surface.Position(x, z + 1, cell.Corner(0, 1));
                Vector3 ne = surface.Position(x + 1, z + 1, cell.Corner(1, 1));
                Vector3 se = surface.Position(x + 1, z, cell.Corner(1, 0));
                nearby.Clear();
                landings.Clear();
                if (!cell.IsRamp)
                {
                    CollectNearbyEdges(surface, upperEdges, x, z, cell.Level, nearby);
                    CollectRampLandings(surface, x, z, landings);
                }

                int firstTopVertex = vertices.Count;
                BuildTop(sw, nw, ne, se, nearby, landings, vertices, edgeMask, triangles[cell.IsRamp ? 2 : 0]);
                if (cell.IsRamp)
                    MarkRampJoins(surface, cell, x, z, firstTopVertex, rampJoinVertices);

                // Each shared edge exactly once; west/south are only the outside perimeter.
                Wall(surface, x, z, TerrainDirection.North, vertices, triangles, edgeMask);
                Wall(surface, x, z, TerrainDirection.East, vertices, triangles, edgeMask);
                if (z == 0) Wall(surface, x, z, TerrainDirection.South, vertices, triangles, edgeMask);
                if (x == 0) Wall(surface, x, z, TerrainDirection.West, vertices, triangles, edgeMask);
            }
            var mesh = new Mesh { name = "Terrain (generated)", hideFlags = HideFlags.DontSave,
                indexFormat = vertices.Count > 65535 ? IndexFormat.UInt32 : IndexFormat.UInt16 };
            try
            {
                mesh.SetVertices(vertices);
                mesh.SetUVs(0, edgeMask);
                mesh.subMeshCount = 3;
                for (int i = 0; i < 3; i++) mesh.SetTriangles(triangles[i], i);
                mesh.RecalculateNormals();
                if (rampJoinVertices.Count != 0)
                {
                    Vector3[] normals = mesh.normals;
                    foreach (int index in rampJoinVertices) normals[index] = Vector3.up;
                    mesh.normals = normals;
                }
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
            List<Vector3> vertices, List<int>[] materials, List<Vector2> edgeMask)
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
            // Carve a recessed rock band below the lip. Keep the complete outer
            // perimeter fixed so the collision top, adjacent faces and ramp tips coincide.
            bool firstHigh = a > c || b > d;
            int hx = firstHigh ? x : x + step.x, hz = firstHigh ? z : z + step.y;
            int lx = firstHigh ? x + step.x : x, lz = firstHigh ? z + step.y : z;
            List<int> triangles = materials[1];
            Vector3 upperA = firstHigh ? p : t, upperB = firstHigh ? q : r;
            Vector3 lowerA = firstHigh ? t : p, lowerB = firstHigh ? r : q;
            var tangent = new Vector2Int(bx - ax, bz - az);
            var normal = firstHigh ? step : -step;
            var high = new Vector2Int(hx, hz);
            var lipA = RimAt(surface, high, new Vector2Int(ax, az), firstHigh ? a : c, normal, -tangent);
            var lipB = RimAt(surface, high, new Vector2Int(bx, bz), firstHigh ? b : d, normal, tangent);
            BuildRim(surface, upperA, upperB, lowerA, lowerB, lipA, lipB, outward, tangent, hx, hz,
                DetailHash(ax, az, bx, bz, surface.DetailSeed), vertices, materials);

            var before = step.x == 0 ? TerrainDirection.West : TerrainDirection.South;
            var after = TerrainSurface.Opposite(before);
            bool cornerA = IsCorner(before), cornerB = IsCorner(after);
            bool IsCorner(TerrainDirection direction) =>
                (surface.Contains(hx, hz) && surface.IsBoundaryBlocked(hx, hz, direction))
                || (surface.Contains(lx, lz) && surface.IsBoundaryBlocked(lx, lz, direction));
            uint hash = DetailHash(ax, az, bx, bz, surface.DetailSeed);
            float u = (cornerA ? 0.32f : 0.23f) + (hash & 255) / 255f * 0.09f;
            float v = (cornerB ? 0.59f : 0.68f) + ((hash >> 8) & 255) / 255f * 0.09f;
            Vector3 Profile(float along, float down, float inset)
            {
                Vector3 first = Vector3.Lerp(p, q, along), second = Vector3.Lerp(t, r, along);
                Vector3 upper = first.y > second.y ? first : second;
                Vector3 lower = first.y > second.y ? second : first;
                float depth = Mathf.Min(surface.BlockWorldSize * 0.065f, (upper.y - lower.y) * 0.22f);
                // Keep a face inside its corner wedge so recessed perpendicular
                // faces cannot intersect at an outside turn.
                depth = Mathf.Min(depth, surface.BlockWorldSize * 0.125f * Mathf.Min(along, 1f - along));
                return Vector3.Lerp(upper, lower, down) - outward * (depth * inset);
            }

            // The rock meets the underside of the turf on the same exact
            // edge at both ends. Both wall bands taper to the ramp tips.
            upperA -= Vector3.up * lipA.Depth;
            upperB -= Vector3.up * lipB.Depth;
            // At shoulders the lower band narrows continuously into the ramp tip.
            while (edgeMask.Count < vertices.Count) edgeMask.Add(Vector2.right);
            int firstRockVertex = vertices.Count;
            Vector3 leftLip = Profile(u, 0.20f, 0.72f), rightLip = Profile(v, 0.25f, 0.83f);
            Vector3 leftBase = Profile(u, 0.72f, 1f), rightBase = Profile(v, 0.79f, 0.76f);
            Quad(upperA, upperB, rightLip, leftLip, outward, vertices, triangles);
            Quad(leftLip, rightLip, rightBase, leftBase, outward, vertices, triangles);
            Quad(leftBase, rightBase, lowerB, lowerA, outward, vertices, triangles);
            Quad(upperA, leftLip, leftBase, lowerA, outward, vertices, triangles);
            Quad(upperB, rightLip, rightBase, lowerB, outward, vertices, triangles);
            for (int i = firstRockVertex; i < vertices.Count; i++)
            {
                Vector3 vertex = vertices[i];
                float along = tangent.x != 0
                    ? Mathf.InverseLerp(upperA.x, upperB.x, vertex.x)
                    : Mathf.InverseLerp(upperA.z, upperB.z, vertex.z);
                float underLip = Mathf.Lerp(upperA.y, upperB.y, along) - vertex.y;
                edgeMask.Add(new Vector2(0f, 1f - Mathf.Clamp01(underLip / (surface.BlockWorldSize * 0.095f))));
            }
        }

        private readonly struct RimCorner
        {
            internal readonly Vector3 Offset;
            internal readonly float Depth, SkirtDepth;
            internal readonly bool Convex, Concave;
            internal RimCorner(Vector3 offset, float depth, float skirtDepth, bool convex, bool concave)
            { Offset = offset; Depth = depth; SkirtDepth = skirtDepth; Convex = convex; Concave = concave; }
        }

        private static Vector3 Horizontal(Vector2Int direction) => new(direction.x, 0f, direction.y);

        private static RimCorner RimAt(TerrainSurface surface, Vector2Int upper, Vector2Int vertex,
            int height, Vector2Int outward, Vector2Int beyond)
        {
            int Corner(Vector2Int cell) => surface.Contains(cell.x, cell.y)
                ? surface.GetCell(cell.x, cell.y).Corner(vertex.x - cell.x, vertex.y - cell.y)
                : surface.BaseQuarterLevel;

            int across = Corner(upper + outward);
            int beside = Corner(upper + beyond);
            int diagonal = Corner(upper + outward + beyond);
            if (across >= height || beside > height || diagonal > height
                || (beside < height && diagonal == height)) return default;

            // Both incident edges see the same four corner heights and the
            // same hash. Shared endpoints do not depend on edge traversal.
            int nextLower = Math.Max(across, Math.Max(beside < height ? beside : across,
                diagonal < height ? diagonal : across));
            float drop = (height - nextLower) * surface.HeightStepWorld * 0.25f;
            float cellSize = surface.BlockWorldSize * 0.25f;
            uint hash = DetailHash(vertex.x, vertex.y, vertex.x, vertex.y, surface.DetailSeed);
            float width = Math.Min(cellSize * (0.075f + (hash & 255) * (0.065f / 255f)), RimReach(surface, drop));
            float depth = Math.Min(cellSize * 0.065f, drop * 0.15f);
            bool convex = beside < height, concave = diagonal == height;
            // Adjacent concave strips share the diagonal end cut in the low
            // quadrant. The nearby strip station rounds off the inside bend.
            Vector3 offset = concave ? Horizontal(outward - beyond) * (width * 1.25f)
                : Horizontal(outward) * width;
            return new RimCorner(offset, depth, depth, convex, concave);
        }

        // Keep the skirt above the lowest clearance even when the lower
        // surface is a ramp. An inside corner can extend on both axes.
        private static float RimReach(TerrainSurface surface, float gap)
            => gap / (4f * (1f + surface.HeightStepWorld / surface.BlockWorldSize));

        private static void BuildRim(TerrainSurface surface, Vector3 a, Vector3 b, Vector3 lowA, Vector3 lowB,
            RimCorner first, RimCorner last, Vector3 outward, Vector2Int tangent, int hx, int hz, uint hash,
            List<Vector3> vertices, List<int>[] materials)
        {
            float cell = surface.BlockWorldSize * 0.25f;
            List<int> cover = materials[surface.GetCell(hx, hz).IsRamp ? 2 : 0];
            Vector3 previous = a;
            var previousSection = first;
            for (int i = 1; i <= 4; i++)
            {
                float along = i == 1 ? 0.2f : i == 2 ? 0.46f + (hash & 63) * (0.08f / 63f) : i == 3 ? 0.8f : 1f;
                Vector3 top = Vector3.Lerp(a, b, along);
                float gap = top.y - Mathf.Lerp(lowA.y, lowB.y, along);
                var section = last;
                if (i < 4)
                {
                    float detail = ((hash >> (i * 7)) & 127) / 127f;
                    float width = cell * (0.045f + detail * detail * 0.19f);
                    width = Math.Min(width, RimReach(surface, gap));
                    // Keep the first/last stations next to an inside corner
                    // far enough back that the strips meet only at their end cut.
                    if ((i == 1 && first.Concave) || (i == 3 && last.Concave))
                        width = Math.Min(width, cell * 0.085f);
                    float depth = Mathf.Lerp(first.Depth, last.Depth, along);
                    float skirt = Math.Max(depth, Math.Min(cell * 0.055f, gap * 0.15f));
                    section = new RimCorner(outward * width, depth, skirt, false, false);
                }
                Strip(previous, top, previousSection, section, outward);
                previous = top;
                previousSection = section;
            }

            // A convex quadrant is emitted by its X-running edge only.
            // The opposite end is exactly the endpoint of the perpendicular strip.
            if (tangent.x != 0)
            {
                if (first.Convex) Fan(a, first, Horizontal(-tangent));
                if (last.Convex) Fan(b, last, Horizontal(tangent));
            }

            void Fan(Vector3 corner, RimCorner section, Vector3 direction)
            {
                float width = section.Offset.magnitude;
                var from = section;
                for (int i = 1; i <= 3; i++)
                {
                    Vector3 normal = (outward * (3 - i) + direction * i).normalized;
                    var to = new RimCorner(normal * width, section.Depth, section.SkirtDepth, false, false);
                    Strip(corner, corner, from, to, (from.Offset + to.Offset).normalized);
                    from = to;
                }
            }

            void Strip(Vector3 start, Vector3 end, RimCorner left, RimCorner right, Vector3 normal)
            {
                if (left.Offset == Vector3.zero && right.Offset == Vector3.zero) return;
                Vector3 outerA = start + left.Offset - Vector3.up * (left.SkirtDepth * 0.35f);
                Vector3 outerB = end + right.Offset - Vector3.up * (right.SkirtDepth * 0.35f);
                Vector3 bottomA = start - Vector3.up * left.Depth;
                Vector3 bottomB = end - Vector3.up * right.Depth;
                Vector3 skirtA = start + left.Offset * 0.82f - Vector3.up * left.SkirtDepth;
                Vector3 skirtB = end + right.Offset * 0.82f - Vector3.up * right.SkirtDepth;
                Quad(start, end, outerB, outerA, Vector3.up, vertices, cover);
                Vector3 bevelA = Vector3.Lerp(outerA, skirtA, 0.42f) + left.Offset * 0.04f;
                Vector3 bevelB = Vector3.Lerp(outerB, skirtB, 0.42f) + right.Offset * 0.04f;
                Quad(outerA, outerB, bevelB, bevelA, normal + Vector3.up * 0.5f, vertices, cover);
                Quad(bevelA, bevelB, skirtB, skirtA, normal, vertices, cover);
                Quad(skirtA, skirtB, bottomB, bottomA, Vector3.down, vertices, cover);
            }
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

        private static byte[] CollectUpperEdges(TerrainSurface surface)
        {
            var edges = new byte[surface.CellsX * surface.CellsZ];
            for (int z = 0; z < surface.CellsZ; z++)
            for (int x = 0; x < surface.CellsX; x++)
            {
                var cell = surface.GetCell(x, z);
                if (cell.IsRamp) continue;
                int height = cell.Level * 4;
                for (int i = 0; i < 4; i++)
                {
                    var side = (TerrainDirection)i;
                    var step = TerrainSurface.Step(side);
                    int a = surface.BaseQuarterLevel, b = a;
                    if (surface.Contains(x + step.x, z + step.y))
                        surface.GetEdge(x + step.x, z + step.y, TerrainSurface.Opposite(side), out a, out b);
                    if (a < height || b < height)
                        edges[x + z * surface.CellsX] |= (byte)(1 << i);
                }
            }
            return edges;
        }

        private static void CollectNearbyEdges(TerrainSurface surface, byte[] edges, int x, int z,
            int level, List<Vector4> nearby)
        {
            // The transition reaches zero before one cell. Only same-height upper edges
            // can contribute, including edges ending at a diagonal corner of this cell.
            for (int dz = -1; dz <= 1; dz++)
            for (int dx = -1; dx <= 1; dx++)
            {
                int nx = x + dx, nz = z + dz;
                if (!surface.Contains(nx, nz)) continue;
                int sides = edges[nx + nz * surface.CellsX];
                if (sides == 0 || surface.GetCell(nx, nz).Level != level) continue;
                if ((sides & (1 << (int)TerrainDirection.North)) != 0)
                    nearby.Add(new Vector4(dx, dz + 1, dx + 1, dz + 1));
                if ((sides & (1 << (int)TerrainDirection.East)) != 0)
                    nearby.Add(new Vector4(dx + 1, dz, dx + 1, dz + 1));
                if ((sides & (1 << (int)TerrainDirection.South)) != 0)
                    nearby.Add(new Vector4(dx, dz, dx + 1, dz));
                if ((sides & (1 << (int)TerrainDirection.West)) != 0)
                    nearby.Add(new Vector4(dx, dz, dx, dz + 1));
            }
        }

        private static void CollectRampLandings(TerrainSurface surface, int x, int z, List<Vector4> landings)
        {
            // Include diagonal neighbors so the dirt fades around both ends of a landing.
            for (int dz = -1; dz <= 1; dz++)
            for (int dx = -1; dx <= 1; dx++)
            {
                int nx = x + dx, nz = z + dz;
                if (!surface.Contains(nx, nz) || surface.GetCell(nx, nz).IsRamp
                    || surface.GetCell(nx, nz).Level != surface.GetCell(x, z).Level) continue;
                for (int i = 0; i < 4; i++)
                {
                    var side = (TerrainDirection)i;
                    var step = TerrainSurface.Step(side);
                    int rx = nx + step.x, rz = nz + step.y;
                    if (!surface.Contains(rx, rz)) continue;
                    var ramp = surface.GetCell(rx, rz);
                    if (!ramp.IsRamp || (ramp.Uphill != side && ramp.Uphill != TerrainSurface.Opposite(side))
                        || surface.IsBoundaryBlocked(nx, nz, side)) continue;
                    switch (side)
                    {
                        case TerrainDirection.North: landings.Add(new Vector4(dx, dz + 1, dx + 1, dz + 1)); break;
                        case TerrainDirection.East: landings.Add(new Vector4(dx + 1, dz, dx + 1, dz + 1)); break;
                        case TerrainDirection.South: landings.Add(new Vector4(dx, dz, dx + 1, dz)); break;
                        case TerrainDirection.West: landings.Add(new Vector4(dx, dz, dx, dz + 1)); break;
                    }
                }
            }
        }

        private static float EdgeInfluence(float u, float v, List<Vector4> edges)
        {
            float distanceSquared = 1f;
            foreach (Vector4 edge in edges)
            {
                float dx = u - Mathf.Clamp(u, edge.x, edge.z);
                float dz = v - Mathf.Clamp(v, edge.y, edge.w);
                distanceSquared = Mathf.Min(distanceSquared, dx * dx + dz * dz);
            }
            return 1f - Mathf.Sqrt(distanceSquared);
        }

        private static void BuildTop(Vector3 sw, Vector3 nw, Vector3 ne, Vector3 se, List<Vector4> nearby, List<Vector4> landings,
            List<Vector3> vertices, List<Vector2> edgeMask, List<int> triangles)
        {
            int divisions = nearby.Count == 0 && landings.Count == 0 ? 1 : 4;
            int first = vertices.Count;
            int stride = divisions + 1;
            for (int z = 0; z <= divisions; z++)
            for (int x = 0; x <= divisions; x++)
            {
                float u = (float)x / divisions, v = (float)z / divisions;
                vertices.Add(Vector3.Lerp(Vector3.Lerp(sw, se, u), Vector3.Lerp(nw, ne, u), v));
                edgeMask.Add(new Vector2(EdgeInfluence(u, v, nearby), EdgeInfluence(u, v, landings)));
                if (x == divisions || z == divisions) continue;
                int index = first + x + z * stride;
                triangles.Add(index); triangles.Add(index + stride); triangles.Add(index + stride + 1);
                triangles.Add(index); triangles.Add(index + stride + 1); triangles.Add(index + 1);
            }
        }

        private static void MarkRampJoins(TerrainSurface surface, TerrainSurface.Cell cell, int x, int z,
            int first, List<int> indices)
        {
            Mark(cell.Uphill);
            Mark(TerrainSurface.Opposite(cell.Uphill));

            void Mark(TerrainDirection side)
            {
                Vector2Int step = TerrainSurface.Step(side);
                int nx = x + step.x, nz = z + step.y;
                if (!surface.Contains(nx, nz) || surface.GetCell(nx, nz).IsRamp
                    || surface.IsBoundaryBlocked(x, z, side)) return;

                // Blend the lighting into the landing while keeping exact shared heights for picking.
                switch (side)
                {
                    case TerrainDirection.North: indices.Add(first + 2); indices.Add(first + 3); break;
                    case TerrainDirection.East: indices.Add(first + 1); indices.Add(first + 3); break;
                    case TerrainDirection.South: indices.Add(first); indices.Add(first + 1); break;
                    case TerrainDirection.West: indices.Add(first); indices.Add(first + 2); break;
                }
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
