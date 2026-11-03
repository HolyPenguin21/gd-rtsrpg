using System;
using System.Collections.Generic;
using UnityEngine;

namespace GameDev.RtsRpg
{
    /// <summary>Owns static occupancy only. Geometry always comes from the immutable TerrainSurface.</summary>
    public sealed class NavigationGrid
    {
        public const int MaxCells = 262144;
        public TerrainSurface Surface { get; }
        public Matrix4x4 LocalToWorld { get; }
        private readonly Matrix4x4 worldToLocal;
        private readonly int[] blocked;
        private readonly Dictionary<int, RectInt> blockers = new Dictionary<int, RectInt>();
        private int nextBlocker = 1;
        public int Revision { get; private set; } = 1;
        public bool IsAlive { get; private set; } = true;
        public event Action Changed;

        public NavigationGrid(TerrainSurface surface, Matrix4x4 localToWorld)
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
            // Reject finite inputs whose transformed endpoints overflow float coordinates.
            for (int zc = 0; zc < surface.CellsZ; zc++)
            for (int xc = 0; xc < surface.CellsX; xc++)
            {
                var c = surface.GetCell(xc, zc);
                for (int dz = 0; dz <= 1; dz++)
                for (int dx = 0; dx <= 1; dx++)
                {
                    Vector3 p = localToWorld.MultiplyPoint3x4(surface.Position(xc + dx, zc + dz, c.Corner(dx, dz)));
                    if (!Finite(p.x) || !Finite(p.y) || !Finite(p.z))
                        throw new ArgumentException("Navigation transformed surface exceeds finite world coordinates.");
                }
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
            Vector2Int size = footprint.Size(pose.Facing);
            return new NavigationPoint(pose, WorldPoint(pose.Cell.x + size.x * 0.5f, pose.Cell.y + size.y * 0.5f));
        }

        public bool IsBlocked(int x, int z) => !IsAlive || !Surface.Contains(x, z) || blocked[x + z * Surface.CellsX] > 0;

        public bool CanOccupy(NavigationPose pose, NavigationFootprint footprint)
        {
            if (!IsAlive || !footprint.IsValid || (uint)pose.Facing > 3) return false;
            Vector2Int size = footprint.Size(pose.Facing);
            return ClearRectangle(pose.Cell.x, pose.Cell.y, size.x, size.y);
        }

        // Internal edges matter too: a large footprint cannot straddle a cliff even at rest.
        private bool ClearRectangle(int x, int z, int width, int length)
        {
            if (x < 0 || z < 0 || width < 1 || length < 1 || (long)x + width > Surface.CellsX
                || (long)z + length > Surface.CellsZ) return false;
            for (int dz = 0; dz < length; dz++)
            for (int dx = 0; dx < width; dx++)
            {
                int cx = x + dx, cz = z + dz;
                if (IsBlocked(cx, cz)) return false;
                if (dx + 1 < width && Surface.IsBoundaryBlocked(cx, cz, TerrainDirection.East)) return false;
                if (dz + 1 < length && Surface.IsBoundaryBlocked(cx, cz, TerrainDirection.North)) return false;
            }
            return true;
        }

        public bool CanStep(NavigationPose from, NavigationPose to, NavigationFootprint footprint)
        {
            if (!CanOccupy(from, footprint) || !CanOccupy(to, footprint) || from.Facing != to.Facing) return false;
            int dx = to.Cell.x - from.Cell.x, dz = to.Cell.y - from.Cell.y;
            if (Math.Abs(dx) > 1 || Math.Abs(dz) > 1 || (dx == 0 && dz == 0)) return false;
            Vector2Int size = footprint.Size(from.Facing);
            // Entire swept rectangle, including both orthogonal alternatives of a diagonal.
            return ClearRectangle(Math.Min(from.Cell.x, to.Cell.x), Math.Min(from.Cell.y, to.Cell.y),
                size.x + Math.Abs(dx), size.y + Math.Abs(dz));
        }

        public bool CanTurn(NavigationPose from, NavigationPose to, NavigationFootprint footprint)
        {
            if (!footprint.NeedsTurns || from.Cell != to.Cell || !CanOccupy(from, footprint) || !CanOccupy(to, footprint)) return false;
            int delta = ((int)to.Facing - (int)from.Facing + 4) % 4;
            if (delta != 1 && delta != 3) return false;
            // Lower-left anchor is stable; rectangle centre shifts by half a cell while turning.
            // Bound the rotation AND that centre shift with the union of circumscribed discs.
            // This deliberately conservative envelope needs a clear 4x4 turning pad for 1x2.
            Vector2Int a = footprint.Size(from.Facing), b = footprint.Size(to.Facing);
            float radius = Mathf.Sqrt(footprint.Width * footprint.Width + footprint.Length * footprint.Length) * 0.5f;
            int minX = Mathf.FloorToInt(from.Cell.x + Math.Min(a.x, b.x) * 0.5f - radius);
            int minZ = Mathf.FloorToInt(from.Cell.y + Math.Min(a.y, b.y) * 0.5f - radius);
            int maxX = Mathf.CeilToInt(from.Cell.x + Math.Max(a.x, b.x) * 0.5f + radius);
            int maxZ = Mathf.CeilToInt(from.Cell.y + Math.Max(a.y, b.y) * 0.5f + radius);
            return ClearRectangle(minX, minZ, maxX - minX, maxZ - minZ);
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
