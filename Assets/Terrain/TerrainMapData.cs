using System;
using UnityEngine;

namespace GameDev.RtsRpg
{
    public enum TerrainDirection { North, East, South, West }

    [Serializable]
    public struct TerrainRamp
    {
        public Vector2Int block;
        public TerrainDirection uphill;
        [Tooltip("Across X for North/South; across Z for East/West. Never mirrored.")]
        public int startSubcell;
        public int widthSubcells;
    }

    /// <summary>Authored input, independent of meshes. CreateSurface validates and copies it.</summary>
    [CreateAssetMenu(menuName = "GameDev/Terrain Map")]
    public sealed class TerrainMapData : ScriptableObject
    {
        [SerializeField] private int width = 1;
        [SerializeField] private int length = 1;
        [SerializeField] private float blockWorldSize = 4f;
        [SerializeField] private float heightStepWorld = 2f;
        [Tooltip("Row-major: x + z * width; negative levels are allowed.")]
        [SerializeField] private int[] levels = { 0 };
        [SerializeField] private TerrainRamp[] ramps = Array.Empty<TerrainRamp>();

        public TerrainSurface CreateSurface()
        {
            if (width <= 0 || length <= 0 || (long)width * length > int.MaxValue / 16)
                throw new ArgumentException("Terrain: dimensions must be positive and fit the 4x4 surface array.");
            if (!FinitePositive(blockWorldSize) || !FinitePositive(heightStepWorld))
                throw new ArgumentException("Terrain: blockWorldSize and heightStepWorld must be finite and positive.");
            if (levels == null || levels.LongLength != (long)width * length)
                throw new ArgumentException($"Terrain: expected { (long)width * length } levels (x + z * width).");
            // Integer quarter-levels keep all shared endpoints exact, including negative elevations.
            int minimum = int.MaxValue;
            foreach (int level in levels)
            {
                if (level < int.MinValue / 4 + 1 || level > int.MaxValue / 4 - 1)
                    throw new ArgumentException("Terrain: elevation exceeds the integer quarter-level range.");
                minimum = Math.Min(minimum, level);
                if (!FinitePositive(Math.Abs((double)level * heightStepWorld) + heightStepWorld))
                    throw new ArgumentException("Terrain: elevation times heightStepWorld exceeds finite mesh coordinates.");
            }
            if (!FinitePositive((double)width * blockWorldSize)
                || !FinitePositive((double)length * blockWorldSize))
                throw new ArgumentException("Terrain: map extent exceeds finite mesh coordinates.");

            int cellsX = checked(width * 4), cellsZ = checked(length * 4);
            var cells = new TerrainSurface.Cell[cellsX * cellsZ];
            for (int z = 0; z < cellsZ; z++)
            for (int x = 0; x < cellsX; x++)
                cells[x + z * cellsX] = new TerrainSurface.Cell(levels[x / 4 + z / 4 * width], -1,
                    TerrainDirection.North, 0, 0);

            TerrainRamp[] definitions = ramps ?? Array.Empty<TerrainRamp>();
            for (int i = 0; i < definitions.Length; i++)
            {
                TerrainRamp ramp = definitions[i];
                string prefix = $"Terrain ramp {i} at {ramp.block}: ";
                if (!Enum.IsDefined(typeof(TerrainDirection), ramp.uphill))
                    throw new ArgumentException(prefix + "invalid uphill direction.");
                if (!Inside(ramp.block.x, ramp.block.y))
                    throw new ArgumentException(prefix + "block is outside the map.");
                if (ramp.startSubcell < 0 || ramp.widthSubcells < 1 || ramp.widthSubcells > 4
                    || ramp.startSubcell > 4 - ramp.widthSubcells)
                    throw new ArgumentException(prefix + "width must be 1..4 and start + width <= 4.");
                Vector2Int step = TerrainSurface.Step(ramp.uphill);
                Vector2Int upper = ramp.block + step, lower = ramp.block - step;
                if (!Inside(upper.x, upper.y) || !Inside(lower.x, lower.y))
                    throw new ArgumentException(prefix + "both the upper exit and lower entry need an in-map block.");
                int level = levels[ramp.block.x + ramp.block.y * width];
                if (levels[upper.x + upper.y * width] != level + 1
                    || levels[lower.x + lower.y * width] != level)
                    throw new ArgumentException(prefix + "needs a flat lower entry at L and upper exit at L+1.");

                for (int z = 0; z < 4; z++)
                for (int x = 0; x < 4; x++)
                {
                    bool northSouth = ramp.uphill == TerrainDirection.North || ramp.uphill == TerrainDirection.South;
                    int across = northSouth ? x : z;
                    if (across < ramp.startSubcell || across >= ramp.startSubcell + ramp.widthSubcells) continue;
                    int index = ramp.block.x * 4 + x + (ramp.block.y * 4 + z) * cellsX;
                    if (cells[index].IsRamp)
                        throw new ArgumentException(prefix + "overlaps another ramp.");
                    cells[index] = new TerrainSurface.Cell(level, i, ramp.uphill, x, z);
                }
            }

            var surface = new TerrainSurface(cellsX, cellsZ, blockWorldSize, heightStepWorld,
                (minimum - 1) * 4, cells);
            // Validate the actual entry/exit profiles after ALL ramps have been applied.
            for (int z = 0; z < cellsZ; z++)
            for (int x = 0; x < cellsX; x++)
            {
                var cell = surface.GetCell(x, z);
                for (int d = 0; d < 4; d++)
                {
                    var direction = (TerrainDirection)d;
                    if (cell.IsRamp && (direction == cell.Uphill || direction == TerrainSurface.Opposite(cell.Uphill)))
                    {
                        if (surface.IsBoundaryBlocked(x, z, direction))
                            throw new ArgumentException($"Terrain ramp {cell.RampIndex}: discontinuous entry/exit at subcell ({x},{z}).");
                    }
                    surface.GetEdge(x, z, direction, out int a, out int b);
                    Vector2Int next = new Vector2Int(x, z) + TerrainSurface.Step(direction);
                    if (!surface.Contains(next.x, next.y)) continue;
                    surface.GetEdge(next.x, next.y, TerrainSurface.Opposite(direction), out int c, out int e);
                    if ((a < c && b > e) || (a > c && b < e))
                        throw new ArgumentException($"Terrain: conflicting crossing edge profiles at subcell ({x},{z}), {direction}.");
                }
            }
            return surface;
        }

        private bool Inside(int x, int z) => x >= 0 && z >= 0 && x < width && z < length;
        private static bool FinitePositive(double value) => value > 0 && value <= float.MaxValue
            && !double.IsNaN(value) && !double.IsInfinity(value);
    }
}
