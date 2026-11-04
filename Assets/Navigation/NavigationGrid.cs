using System;
using System.Collections.Generic;
using Unity.Profiling;
using UnityEngine;

namespace GameDev.RtsRpg
{
    /// <summary>Owns static occupancy only. Geometry always comes from the immutable TerrainSurface.</summary>
    public sealed class NavigationGrid
    {
        private static readonly ProfilerMarker PickMarker = new("Navigation.TryPickSurface");
        private static readonly ProfilerMarker StepMarker = new("Navigation.CanStep");
        private static readonly ProfilerMarker TurnMarker = new("Navigation.CanTurn");
        public const int MaxCells = 262144;
        public TerrainSurface Surface { get; }
        public Matrix4x4 LocalToWorld { get; }
        private readonly Matrix4x4 worldToLocal;
        private readonly int[] blocked;
        private readonly byte[] cliffs;
        private readonly ushort[][] geometryEdges;
        private readonly int maxQuarterHeight;
        private int CellCount => blocked.Length;
        internal int StateId(NavigationPose pose) =>
            pose.Cell.x + pose.Cell.y * Surface.CellsX + (int)pose.Facing * CellCount;
        internal NavigationPose Pose(int state)
        {
            int cell = state % CellCount;
            return new NavigationPose(new Vector2Int(cell % Surface.CellsX, cell / Surface.CellsX),
                (NavigationFacing)(state / CellCount));
        }
        private readonly Dictionary<int, RectInt> blockers = new Dictionary<int, RectInt>();
        private int nextBlocker = 1;
        public int Revision { get; private set; } = 1;
        public bool IsAlive { get; private set; } = true;
        public event Action Changed;

        public NavigationGrid(TerrainSurface surface, Matrix4x4 localToWorld) : this(surface, localToWorld, null) { }

        internal NavigationGrid(TerrainSurface surface, Matrix4x4 localToWorld, NavigationGrid previous)
        {
            Surface = surface ?? throw new ArgumentNullException(nameof(surface));
            if ((long)surface.CellsX * surface.CellsZ > MaxCells)
                throw new ArgumentException($"Navigation supports at most {MaxCells} cells; split or reduce the input map explicitly.");
            // XZ must stay a horizontal square lattice. Y scale can differ. No silent geometry correction.
            Vector3 x = localToWorld.MultiplyVector(Vector3.right);
            Vector3 y = localToWorld.MultiplyVector(Vector3.up);
            Vector3 z = localToWorld.MultiplyVector(Vector3.forward);
            for (int i = 0; i < 16; i++)
                if (!Finite(localToWorld[i])) throw new ArgumentException("Navigation transform must be finite.");
            if (!Finite(x.magnitude) || !Finite(z.magnitude)
                || x.magnitude < 0.0001f || y.y < 0.0001f || z.magnitude < 0.0001f
                || Mathf.Abs(x.y) > 0.00001f || Mathf.Abs(z.y) > 0.00001f
                || Mathf.Abs(y.x) > 0.00001f || Mathf.Abs(y.z) > 0.00001f
                || Mathf.Abs(Vector3.Dot(x.normalized, z.normalized)) > 0.00001f
                || Mathf.Abs(x.magnitude - z.magnitude) > x.magnitude * 0.00001f
                || Vector3.Dot(Vector3.Cross(z, x), y) <= 0
                || localToWorld.m30 != 0 || localToWorld.m31 != 0 || localToWorld.m32 != 0 || localToWorld.m33 != 1)
                throw new ArgumentException("Navigation requires translation, yaw, positive equal XZ scales, positive Y scale; no tilt, reflection or shear.");
            LocalToWorld = localToWorld; worldToLocal = localToWorld.inverse;
            for (int i = 0; i < 16; i++)
                if (!Finite(worldToLocal[i])) throw new ArgumentException("Navigation transform inverse must be finite.");
            blocked = new int[surface.CellsX * surface.CellsZ];
            if (previous != null && ReferenceEquals(surface, previous.Surface))
            {
                cliffs = previous.cliffs;
                geometryEdges = previous.geometryEdges;
                maxQuarterHeight = previous.maxQuarterHeight;
            }
            else
            {
                maxQuarterHeight = surface.BaseQuarterLevel;
                cliffs = new byte[CellCount];
                geometryEdges = new[] { new ushort[CellCount], new ushort[CellCount], new ushort[CellCount * 8] };
                for (int zc = 0; zc < surface.CellsZ; zc++)
                for (int xc = 0; xc < surface.CellsX; xc++)
                {
                    var c = surface.GetCell(xc, zc);
                    cliffs[xc + zc * surface.CellsX] = (byte)(
                        (surface.IsBoundaryBlocked(xc, zc, TerrainDirection.East) ? 1 : 0)
                        | (surface.IsBoundaryBlocked(xc, zc, TerrainDirection.North) ? 2 : 0));
                    for (int dz = 0; dz <= 1; dz++)
                    for (int dx = 0; dx <= 1; dx++) maxQuarterHeight = Math.Max(maxQuarterHeight, c.Corner(dx, dz));
                }
            }
            // An affine transform reaches its extrema at bounding box corners.
            for (int i = 0; i < 8; i++)
            {
                Vector3 p = localToWorld.MultiplyPoint3x4(surface.Position((i & 1) == 0 ? 0 : surface.CellsX,
                    (i & 2) == 0 ? 0 : surface.CellsZ, (i & 4) == 0 ? surface.BaseQuarterLevel : maxQuarterHeight));
                if (!Finite(p.x) || !Finite(p.y) || !Finite(p.z))
                    throw new ArgumentException("Navigation transformed surface exceeds finite world coordinates.");
            }
        }

        public bool TryWorldToCell(Vector3 world, out Vector2Int cell)
        {
            cell = default;
            if (!IsAlive || !Finite(world.x) || !Finite(world.y) || !Finite(world.z)) return false;
            Vector3 p = worldToLocal.MultiplyPoint3x4(world);
            double x = (double)p.x * 4 / Surface.BlockWorldSize, z = (double)p.z * 4 / Surface.BlockWorldSize;
            if (double.IsNaN(x) || double.IsNaN(z) || x < 0 || z < 0 || x >= Surface.CellsX || z >= Surface.CellsZ) return false;
            cell = new Vector2Int((int)x, (int)z); return true;
        }

        // Traverse only cells crossed by the ray, in front-to-back order. The same
        // planar corner heights define both the mesh top and WorldPoint.
        internal bool TryPickSurface(Ray ray, out Vector2Int cell)
        {
            using var marker = PickMarker.Auto();
            cell = default;
            if (!IsAlive) return false;
            float scale = 4f / Surface.BlockWorldSize;
            Vector3 origin = worldToLocal.MultiplyPoint3x4(ray.origin);
            Vector3 direction = worldToLocal.MultiplyVector(ray.direction);
            float ox = origin.x * scale, oz = origin.z * scale;
            float dx = direction.x * scale, dz = direction.z * scale;
            float enter = 0f, exit = float.PositiveInfinity;
            if (!Clip(ox, dx, Surface.CellsX, ref enter, ref exit)
                || !Clip(oz, dz, Surface.CellsZ, ref enter, ref exit)) return false;

            int x = Mathf.Clamp(Mathf.FloorToInt(ox + enter * dx), 0, Surface.CellsX - 1);
            int z = Mathf.Clamp(Mathf.FloorToInt(oz + enter * dz), 0, Surface.CellsZ - 1);
            int stepX = Math.Sign(dx), stepZ = Math.Sign(dz);
            float heightScale = Surface.HeightStepWorld * 0.25f;
            while (Surface.Contains(x, z))
            {
                float tx = stepX == 0 ? float.PositiveInfinity : (x + (stepX > 0 ? 1 : 0) - ox) / dx;
                float tz = stepZ == 0 ? float.PositiveInfinity : (z + (stepZ > 0 ? 1 : 0) - oz) / dz;
                float leave = Mathf.Min(exit, Mathf.Min(tx, tz));
                var top = Surface.GetCell(x, z);
                float h = top.Corner(0, 0) * heightScale;
                float sx = (top.Corner(1, 0) - top.Corner(0, 0)) * heightScale;
                float sz = (top.Corner(0, 1) - top.Corner(0, 0)) * heightScale;
                float slope = direction.y - sx * dx - sz * dz;
                if (slope < -0.000001f)
                {
                    float hit = (h + sx * (ox - x) + sz * (oz - z) - origin.y) / slope;
                    if (hit >= enter - 0.0001f && hit <= leave + 0.0001f)
                    {
                        cell = new Vector2Int(x, z);
                        return true;
                    }
                    // A near-side cliff hides the lower surface behind it. Reject the
                    // cliff face instead of picking unrelated ground through the rock.
                    if (hit < enter - 0.0001f) return false;
                }
                if (leave >= exit) break;
                if (tx <= tz) x += stepX;
                if (tz <= tx) z += stepZ;
                enter = leave;
            }
            return false;
        }

        private static bool Clip(float origin, float direction, int bound, ref float enter, ref float exit)
        {
            if (direction == 0f) return origin >= 0f && origin <= bound;
            float a = -origin / direction, b = (bound - origin) / direction;
            enter = Mathf.Max(enter, Mathf.Min(a, b));
            exit = Mathf.Min(exit, Mathf.Max(a, b));
            return enter <= exit;
        }

        public Vector3 WorldPoint(float cellX, float cellZ)
        {
            if (!IsAlive || !Finite(cellX) || !Finite(cellZ) || cellX < 0 || cellZ < 0
                || cellX > Surface.CellsX || cellZ > Surface.CellsZ) throw new ArgumentOutOfRangeException(nameof(cellX));
            int x = Math.Min((int)cellX, Surface.CellsX - 1), z = Math.Min((int)cellZ, Surface.CellsZ - 1);
            float u = cellX - x, v = cellZ - z;
            var c = Surface.GetCell(x, z);
            double q = (1 - u) * (1 - v) * (double)c.Corner(0, 0) + u * (1 - v) * (double)c.Corner(1, 0)
                + (1 - u) * v * (double)c.Corner(0, 1) + u * v * (double)c.Corner(1, 1);
            return LocalToWorld.MultiplyPoint3x4(new Vector3((float)((double)cellX * Surface.BlockWorldSize / 4),
                (float)(q * Surface.HeightStepWorld / 4), (float)((double)cellZ * Surface.BlockWorldSize / 4)));
        }

        public NavigationPoint Point(NavigationPose pose, NavigationFootprint footprint)
        {
            Vector2 centre = pose.Centre(footprint);
            return new NavigationPoint(pose, WorldPoint(centre.x, centre.y));
        }

        public bool IsBlocked(int x, int z) => !IsAlive || !Surface.Contains(x, z) || blocked[x + z * Surface.CellsX] > 0;

        public bool CanOccupy(NavigationPose pose, NavigationFootprint footprint)
        {
            if (!IsAlive || !footprint.IsValid || (uint)pose.Facing > 7) return false;
            Vector2Int size = footprint.Size(pose.Facing);
            return ClearRectangle(pose.Cell.x, pose.Cell.y, size.x, size.y, blockers.Count != 0)
                && (!footprint.NeedsTurns || RampDirectionFits(pose.Centre(footprint), NavigationPose.Steps[(int)pose.Facing]));
        }

        // Internal edges matter too: a footprint cannot straddle a cliff even at rest.
        private bool ClearRectangle(int x, int z, int width, int length, bool checkBlockers)
        {
            int stride = Surface.CellsX;
            if (x < 0 || z < 0 || x > stride - width || z > Surface.CellsZ - length) return false;
            for (int dz = 0; dz < length; dz++)
            {
                int index = x + (z + dz) * stride;
                int north = dz + 1 < length ? 2 : 0;
                for (int dx = 0; dx < width; dx++, index++)
                {
                    int edges = north | (dx + 1 < width ? 1 : 0);
                    if ((cliffs[index] & edges) != 0 || (checkBlockers && blocked[index] != 0)) return false;
                }
            }
            return true;
        }

        // Use the same blocker counts and blocked cell edges for continuous
        // segments as for graph edges. Short pieces keep the swept rectangle tight.
        internal bool CanSweep(Vector2 from, Vector2 to, Vector2 halfExtent)
        {
            if (!IsAlive) return false;
            int pieces = Mathf.Max(1, Mathf.CeilToInt(Mathf.Max(Mathf.Abs(to.x - from.x), Mathf.Abs(to.y - from.y)) * 4f));
            Vector2 previous = from;
            bool checkBlockers = blockers.Count != 0;
            const float epsilon = 0.00001f;
            for (int i = 1; i <= pieces; i++)
            {
                Vector2 current = Vector2.Lerp(from, to, (float)i / pieces);
                int x = Mathf.FloorToInt(Mathf.Min(previous.x, current.x) - halfExtent.x + epsilon);
                int z = Mathf.FloorToInt(Mathf.Min(previous.y, current.y) - halfExtent.y + epsilon);
                int endX = Mathf.CeilToInt(Mathf.Max(previous.x, current.x) + halfExtent.x - epsilon);
                int endZ = Mathf.CeilToInt(Mathf.Max(previous.y, current.y) + halfExtent.y - epsilon);
                if (!ClearRectangle(x, z, endX - x, endZ - z, checkBlockers)) return false;
                previous = current;
            }
            return true;
        }

        internal bool NearBoundary(NavigationPose pose, NavigationFootprint footprint)
        {
            var size = footprint.Size(pose.Facing);
            return !ClearRectangle(pose.Cell.x - 1, pose.Cell.y - 1, size.x + 2, size.y + 2, blockers.Count != 0);
        }

        internal int EdgeCost(NavigationPose from, NavigationPose to, NavigationFootprint footprint, int edge)
        {
            int cost = NavigationPathfinder.EdgeCost(edge);
            if (edge >= 8)
            {
                var at = from.Centre(footprint);
                return cost + (Surface.GetCell((int)at.x, (int)at.y).IsRamp ? 15 : 2);
            }
            // A soft terrain-only cost. Narrow routes remain reachable.
            int x = to.Cell.x, z = to.Cell.y;
            int index = x + z * Surface.CellsX;
            return cost + ((cliffs[index] != 0 || x == 0 || z == 0
                || (cliffs[index - 1] & 1) != 0 || (cliffs[index - Surface.CellsX] & 2) != 0) ? 2 : 0);
        }

        // Shared topology for straight corridors, bends and junctions. One
        // connected component is built at a time so competing entrances
        // cannot acquire different reservations while discovery is suspended.
        internal sealed class Passage
        {
            internal readonly HashSet<Vector2Int> Cells = new();
            internal bool Ready, Complex;
            internal int Width = 4;
            internal int Axes;

            internal bool Overlaps(Rect area)
            {
                const float inset = 0.0001f;
                int maxX = Mathf.CeilToInt(area.xMax - inset);
                int maxZ = Mathf.CeilToInt(area.yMax - inset);
                for (int z = Mathf.FloorToInt(area.yMin + inset); z < maxZ; z++)
                for (int x = Mathf.FloorToInt(area.xMin + inset); x < maxX; x++)
                    if (Cells.Contains(new Vector2Int(x, z))) return true;
                return false;
            }
        }

        private int passageRevision;
        private byte[] narrowCells, corePassageCells;
        private readonly Dictionary<Vector2Int, Passage> passages = new();
        private readonly Queue<Vector2Int> passageFrontier = new();
        private Passage buildingPassage;

        private void RefreshPassages()
        {
            if (passageRevision == Revision) return;
            passageRevision = Revision;
            passages.Clear();
            passageFrontier.Clear();
            buildingPassage = null;
            if (narrowCells == null)
            {
                narrowCells = new byte[CellCount];
                corePassageCells = new byte[CellCount];
            }
            else
            {
                Array.Clear(narrowCells, 0, narrowCells.Length);
                Array.Clear(corePassageCells, 0, corePassageCells.Length);
            }
        }

        internal void AdvancePassages(int budget)
        {
            RefreshPassages();
            while (buildingPassage != null && budget-- > 0)
            {
                var cell = passageFrontier.Dequeue();
                int shape = PassageShape(cell);
                buildingPassage.Axes |= shape;
                buildingPassage.Complex |= buildingPassage.Axes == 3;
                if ((shape & 1) != 0 && CrossSection(cell.x, cell.y, true, out _, out int widthX))
                    buildingPassage.Width = Math.Min(buildingPassage.Width, widthX);
                if ((shape & 2) != 0 && CrossSection(cell.y, cell.x, false, out _, out int widthZ))
                    buildingPassage.Width = Math.Min(buildingPassage.Width, widthZ);
                for (int i = 0; i < 8; i += 2)
                {
                    var neighbor = cell + NavigationPose.Steps[i];
                    if (passages.ContainsKey(neighbor) || PassageShape(neighbor) == 0 || !OpenCardinal(cell, neighbor)) continue;
                    AddPassageCell(neighbor);
                }
                if (passageFrontier.Count != 0) continue;
                buildingPassage.Ready = true;
                buildingPassage = null;
            }
        }

        private void AddPassageCell(Vector2Int cell)
        {
            passages.Add(cell, buildingPassage);
            buildingPassage.Cells.Add(cell);
            passageFrontier.Enqueue(cell);
        }

        private int CorePassageAxes(Vector2Int cell)
        {
            if (IsBlocked(cell.x, cell.y)) return 0;
            int id = cell.x + cell.y * Surface.CellsX;
            if (corePassageCells[id] != 0) return corePassageCells[id] - 1;
            bool east = OpenCardinal(cell, cell + Vector2Int.right);
            bool west = OpenCardinal(cell, cell - Vector2Int.right);
            bool north = OpenCardinal(cell, cell + Vector2Int.up);
            bool south = OpenCardinal(cell, cell - Vector2Int.up);
            int shape = ((east || west) && CrossSection(cell.x, cell.y, true, out _, out _) ? 1 : 0)
                | ((north || south) && CrossSection(cell.y, cell.x, false, out _, out _) ? 2 : 0);
            corePassageCells[id] = (byte)(shape + 1);
            return shape;
        }

        private int PassageShape(Vector2Int cell)
        {
            if (IsBlocked(cell.x, cell.y)) return 0;
            int id = cell.x + cell.y * Surface.CellsX;
            if (narrowCells[id] != 0) return narrowCells[id] - 1;
            int shape = CorePassageAxes(cell);
            if (shape != 3)
            {
                int connected = 0;
                for (int i = 0; i < 8; i += 2)
                {
                    var neighbor = cell + NavigationPose.Steps[i];
                    int axis = i == 0 || i == 4 ? 2 : 1;
                    if ((connected & axis) == 0 && OpenCardinal(cell, neighbor)
                        && (CorePassageAxes(neighbor) & axis) != 0) connected |= axis;
                }
                // A wider central tile can be the crossing point of two
                // narrow branches. A single opening into a plaza is not one.
                if (connected == 3) shape = 3;
            }
            narrowCells[id] = (byte)(shape + 1);
            return shape;
        }

        private bool OpenCardinal(Vector2Int a, Vector2Int b) => ClearRectangle(
            Math.Min(a.x, b.x), Math.Min(a.y, b.y), 1 + Math.Abs(a.x - b.x), 1 + Math.Abs(a.y - b.y),
            blockers.Count != 0);

        internal bool PassageAt(NavigationPose from, NavigationPose to, NavigationFootprint footprint,
            out Vector2Int cell, out int direction, out int bodyWidth)
        {
            RefreshPassages();
            var delta = to.Cell - from.Cell;
            if (delta == Vector2Int.zero) delta = NavigationPose.Steps[(int)to.Facing];
            bool alongX = Math.Abs(delta.x) > Math.Abs(delta.y);
            direction = alongX ? Math.Sign(delta.x) : Math.Sign(delta.y) * 2;
            var size = footprint.Size(to.Facing);
            bodyWidth = alongX ? size.y : size.x;
            // Query only the next footprint. Longer shortcuts are separately
            // validated to stay off bottlenecks.
            for (int z = 0; z < size.y; z++)
            for (int x = 0; x < size.x; x++)
            {
                cell = to.Cell + new Vector2Int(x, z);
                if (PassageShape(cell) != 0) return true;
            }
            var fromSize = footprint.Size(from.Facing);
            int minX = Math.Min(from.Cell.x, to.Cell.x), minZ = Math.Min(from.Cell.y, to.Cell.y);
            int maxX = Math.Max(from.Cell.x + fromSize.x, to.Cell.x + size.x);
            int maxZ = Math.Max(from.Cell.y + fromSize.y, to.Cell.y + size.y);
            for (int z = minZ; z < maxZ; z++)
            for (int x = minX; x < maxX; x++)
            {
                if (x >= from.Cell.x && x < from.Cell.x + fromSize.x && z >= from.Cell.y && z < from.Cell.y + fromSize.y) continue;
                cell = new Vector2Int(x, z);
                if (PassageShape(cell) != 0) return true;
            }
            cell = default;
            return false;
        }

        internal bool CanSteerAcrossPassage(Rect sweep, Rect current, Passage allowed)
        {
            RefreshPassages();
            const float inset = 0.0001f;
            int maxX = Mathf.CeilToInt(sweep.xMax - inset), maxZ = Mathf.CeilToInt(sweep.yMax - inset);
            for (int z = Mathf.FloorToInt(sweep.yMin + inset); z < maxZ; z++)
            for (int x = Mathf.FloorToInt(sweep.xMin + inset); x < maxX; x++)
            {
                var cell = new Vector2Int(x, z);
                if (allowed != null && allowed.Cells.Contains(cell)) continue;
                if (x < current.xMax - inset && x + 1 > current.xMin + inset
                    && z < current.yMax - inset && z + 1 > current.yMin + inset) continue;
                if (PassageShape(cell) != 0) return false;
            }
            return true;
        }

        internal Passage RequestPassage(Vector2Int cell)
        {
            if (passages.TryGetValue(cell, out var existing)) return existing;
            if (buildingPassage != null) return null;
            buildingPassage = new Passage();
            AddPassageCell(cell);
            return buildingPassage;
        }

        internal bool IsRampPose(NavigationPose pose, NavigationFootprint footprint)
        {
            var at = pose.Centre(footprint);
            return Surface.Contains((int)at.x, (int)at.y) && Surface.GetCell((int)at.x, (int)at.y).IsRamp;
        }

        internal float FormationClearance(NavigationPose pose, NavigationFootprint footprint, Vector2 direction)
        {
            var axis = Math.Abs(direction.x) > Math.Abs(direction.y) ? Vector2Int.up : Vector2Int.right;
            float width = 0f;
            for (int side = -1; side <= 1; side += 2)
            for (int distance = 1; distance <= 2; distance++)
            {
                var from = new NavigationPose(pose.Cell + axis * (side * (distance - 1)), pose.Facing);
                if (!StepClear(from, axis * side, footprint, blockers.Count != 0)) break;
                width += 1f;
            }
            return width * 0.25f;
        }

        private bool CrossSection(int position, int cross, bool alongX, out int first, out int width)
        {
            first = cross;
            width = 1;
            if (!Clear(cross, 1)) return false;
            while (width < 4 && Clear(first - 1, width + 1)) { first--; width++; }
            while (width < 4 && Clear(first, width + 1)) width++;
            return width < 4;

            bool Clear(int start, int span) => alongX
                ? ClearRectangle(position, start, 1, span, blockers.Count != 0)
                : ClearRectangle(start, position, span, 1, blockers.Count != 0);
        }

        public bool CanStep(NavigationPose from, NavigationPose to, NavigationFootprint footprint)
        {
            using var marker = StepMarker.Auto();
            if (!IsAlive || !footprint.IsValid || (uint)from.Facing > 7 || from.Facing != to.Facing
                || !Surface.Contains(from.Cell.x, from.Cell.y) || !Surface.Contains(to.Cell.x, to.Cell.y)) return false;
            Vector2Int delta = to.Cell - from.Cell;
            if (footprint.NeedsTurns && (delta != NavigationPose.Steps[(int)from.Facing]
                || !RampAligned(from, delta, footprint))) return false;
            if (Math.Abs(delta.x) > 1 || Math.Abs(delta.y) > 1 || delta == Vector2Int.zero) return false;
            return StepClear(from, delta, footprint, blockers.Count != 0);
        }

        private bool RampAligned(NavigationPose from, Vector2Int delta, NavigationFootprint footprint)
        {
            var centre = from.Centre(footprint);
            return RampDirectionFits(centre, delta) && RampDirectionFits(centre + (Vector2)delta, delta);
        }

        private bool RampDirectionFits(Vector2 point, Vector2Int direction)
        {
            int x = Mathf.FloorToInt(point.x), z = Mathf.FloorToInt(point.y);
            if (!Surface.Contains(x, z)) return false;
            var cell = Surface.GetCell(x, z);
            if (!cell.IsRamp) return true;
            var uphill = TerrainSurface.Step(cell.Uphill);
            return direction == uphill || direction == -uphill;
        }

        // Yield may reverse along a ramp but still needs its full swept footprint.
        internal bool CanYieldStep(NavigationPose from, NavigationPose to, NavigationFootprint footprint)
        {
            Vector2Int delta = to.Cell - from.Cell;
            return IsAlive && StepClear(from, delta, footprint, blockers.Count != 0)
                && (!footprint.NeedsTurns || RampAligned(from, delta, footprint));
        }

        private bool StepClear(NavigationPose from, Vector2Int delta, NavigationFootprint footprint, bool checkBlockers)
        {
            Vector2Int size = footprint.Size(from.Facing);
            // Includes both orthogonal alternatives of a diagonal step.
            return ClearRectangle(from.Cell.x + Math.Min(0, delta.x), from.Cell.y + Math.Min(0, delta.y),
                size.x + Math.Abs(delta.x), size.y + Math.Abs(delta.y), checkBlockers);
        }

        public bool CanTurn(NavigationPose from, NavigationPose to, NavigationFootprint footprint)
        {
            using var marker = TurnMarker.Auto();
            if (!IsAlive || !footprint.NeedsTurns || from.Cell != to.Cell
                || (uint)from.Facing > 7 || (uint)to.Facing > 7) return false;
            int delta = ((int)to.Facing - (int)from.Facing + 8) & 7;
            return (delta == 1 || delta == 7) && TurnClear(from, to, footprint, blockers.Count != 0);
        }

        private bool TurnClear(NavigationPose from, NavigationPose to, NavigationFootprint footprint, bool checkBlockers)
        {
            // Every intermediate body orientation fits in a radius of sqrt(5)/2.
            // Both centres use half-cell increments, so that radius always reaches
            // at least one cell past the floor/ceil of either centre.
            if (!Surface.Contains(from.Cell.x, from.Cell.y) || IsRampPose(from, footprint) || IsRampPose(to, footprint)) return false;
            Vector2Int a = footprint.Size(from.Facing), b = footprint.Size(to.Facing);
            int minX = ((Math.Min(a.x, b.x) + 1) >> 1) - 2;
            int minZ = ((Math.Min(a.y, b.y) + 1) >> 1) - 2;
            int maxX = (Math.Max(a.x, b.x) >> 1) + 2;
            int maxZ = (Math.Max(a.y, b.y) >> 1) + 2;
            return ClearRectangle(from.Cell.x + minX, from.Cell.y + minZ,
                maxX - minX, maxZ - minZ, checkBlockers);
        }

        // Terrain-only transitions survive blocker edits. Blockers are overlaid
        // at query time and never invalidate the immutable geometry cache.
        internal int Edges(NavigationPose pose, NavigationFootprint footprint)
        {
            var cache = geometryEdges[footprint.Graph];
            int index = StateId(pose);
            int mask = cache[index];
            if (mask == 0)
            {
                mask = 1 << 10;
                int first = footprint.NeedsTurns ? (int)pose.Facing : 0;
                int last = footprint.NeedsTurns ? first + 1 : 8;
                for (int i = first; i < last; i++)
                    if (StepClear(pose, NavigationPose.Steps[i], footprint, false)
                        && (!footprint.NeedsTurns || RampAligned(pose, NavigationPose.Steps[i], footprint))) mask |= 1 << i;
                if (footprint.NeedsTurns)
                {
                    if (TurnClear(pose, pose.Turn(-1), footprint, false)) mask |= 1 << 8;
                    if (TurnClear(pose, pose.Turn(1), footprint, false)) mask |= 1 << 9;
                }
                cache[index] = (ushort)mask;
            }
            if (blockers.Count == 0) return mask & 1023;
            for (int i = 0; i < 8; i++)
                if ((mask & (1 << i)) != 0 && !StepClear(pose, NavigationPose.Steps[i], footprint, true)) mask &= ~(1 << i);
            if ((mask & 256) != 0 && !TurnClear(pose, pose.Turn(-1), footprint, true)) mask &= ~256;
            if ((mask & 512) != 0 && !TurnClear(pose, pose.Turn(1), footprint, true)) mask &= ~512;
            return mask & 1023;
        }

        public bool TryAddBlocker(RectInt rectangle, out int handle, out string error)
        {
            handle = 0; error = null;
            if (!IsAlive || rectangle.width < 1 || rectangle.height < 1 || rectangle.x < 0 || rectangle.y < 0
                || (long)rectangle.x + rectangle.width > Surface.CellsX || (long)rectangle.y + rectangle.height > Surface.CellsZ
                || nextBlocker == int.MaxValue)
            { error = "Blocker needs a positive in-map rectangle and a live grid with available handles."; return false; }
            handle = nextBlocker++; blockers.Add(handle, rectangle); Apply(rectangle, 1); Invalidate(); return true;
        }

        public bool RemoveBlocker(int handle)
        {
            if (!IsAlive || !blockers.TryGetValue(handle, out RectInt rectangle)) return false;
            blockers.Remove(handle); Apply(rectangle, -1); Invalidate(); return true;
        }

        public IReadOnlyCollection<RectInt> Blockers => blockers.Values;
        private void Apply(RectInt r, int delta)
        {
            for (int z = r.y; z < r.yMax; z++)
            for (int x = r.x; x < r.xMax; x++) blocked[x + z * Surface.CellsX] += delta;
        }
        private void Invalidate() { Revision++; Changed?.Invoke(); }
        public void Retire() { if (!IsAlive) return; IsAlive = false; Invalidate(); }
        private static bool Finite(float f) => !float.IsNaN(f) && !float.IsInfinity(f);
    }
}
