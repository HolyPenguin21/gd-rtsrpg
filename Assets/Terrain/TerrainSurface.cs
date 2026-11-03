using System;
using UnityEngine;

namespace GameDev.RtsRpg
{
    /// <summary>Immutable, validated static geometry; no graph, occupancy or path search.</summary>
    public sealed class TerrainSurface
    {
        public readonly struct Cell
        {
            public int Level { get; }
            public int RampIndex { get; }
            public TerrainDirection Uphill { get; }
            public bool IsRamp => RampIndex >= 0;
            private readonly int localX, localZ;

            internal Cell(int level, int rampIndex, TerrainDirection uphill, int x, int z)
            {
                Level = level;
                RampIndex = rampIndex;
                Uphill = uphill;
                localX = x;
                localZ = z;
            }

            /// <summary>Exact height in quarter-levels at a corner (0 or 1 on each cell axis).</summary>
            public int Corner(int x, int z)
            {
                if ((uint)x > 1 || (uint)z > 1) throw new ArgumentOutOfRangeException("Corner must be 0 or 1.");
                int rise = 0;
                if (IsRamp)
                {
                    switch (Uphill)
                    {
                        case TerrainDirection.North: rise = localZ + z; break;
                        case TerrainDirection.East: rise = localX + x; break;
                        case TerrainDirection.South: rise = 4 - localZ - z; break;
                        case TerrainDirection.West: rise = 4 - localX - x; break;
                    }
                }
                return Level * 4 + rise;
            }
        }

        private readonly Cell[] cells;
        public int CellsX { get; }
        public int CellsZ { get; }
        public float BlockWorldSize { get; }
        public float HeightStepWorld { get; }
        public int BaseQuarterLevel { get; }

        internal TerrainSurface(int x, int z, float blockSize, float heightStep, int baseHeight, Cell[] data)
        {
            CellsX = x; CellsZ = z; BlockWorldSize = blockSize; HeightStepWorld = heightStep;
            BaseQuarterLevel = baseHeight; cells = data;
        }

        public bool Contains(int x, int z) => x >= 0 && z >= 0 && x < CellsX && z < CellsZ;

        public Cell GetCell(int x, int z)
        {
            if (!Contains(x, z)) throw new ArgumentOutOfRangeException($"Subcell ({x},{z}) is outside the terrain.");
            return cells[x + z * CellsX];
        }

        // Edges always run along +X (N/S) or +Z (E/W), including the opposite cell.
        public void GetEdge(int x, int z, TerrainDirection side, out int a, out int b)
        {
            Cell cell = GetCell(x, z);
            switch (side)
            {
                case TerrainDirection.North: a = cell.Corner(0, 1); b = cell.Corner(1, 1); break;
                case TerrainDirection.East: a = cell.Corner(1, 0); b = cell.Corner(1, 1); break;
                case TerrainDirection.South: a = cell.Corner(0, 0); b = cell.Corner(1, 0); break;
                case TerrainDirection.West: a = cell.Corner(0, 0); b = cell.Corner(0, 1); break;
                default: throw new ArgumentOutOfRangeException(nameof(side));
            }
        }

        /// <summary>True for a map edge or a height discontinuity along this entire shared edge.
        /// False only describes geometry, not unit passability or diagonal corner-cutting.</summary>
        public bool IsBoundaryBlocked(int x, int z, TerrainDirection side)
        {
            GetEdge(x, z, side, out int a, out int b);
            Vector2Int step = Step(side);
            if (!Contains(x + step.x, z + step.y)) return true;
            GetEdge(x + step.x, z + step.y, Opposite(side), out int c, out int d);
            return a != c || b != d;
        }

        public Vector3 Position(int latticeX, int latticeZ, int quarterLevel) => new Vector3(
            (float)((double)latticeX * BlockWorldSize / 4),
            (float)((double)quarterLevel * HeightStepWorld / 4),
            (float)((double)latticeZ * BlockWorldSize / 4));

        public static TerrainDirection Opposite(TerrainDirection direction) => (TerrainDirection)(((int)direction + 2) % 4);

        public static Vector2Int Step(TerrainDirection direction)
        {
            switch (direction)
            {
                case TerrainDirection.North: return new Vector2Int(0, 1);
                case TerrainDirection.East: return new Vector2Int(1, 0);
                case TerrainDirection.South: return new Vector2Int(0, -1);
                case TerrainDirection.West: return new Vector2Int(-1, 0);
                default: throw new ArgumentOutOfRangeException(nameof(direction));
            }
        }
    }
}
