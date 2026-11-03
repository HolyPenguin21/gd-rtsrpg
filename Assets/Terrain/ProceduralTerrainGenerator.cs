using System;
using System.Collections.Generic;
using UnityEngine;

namespace GameDev.RtsRpg
{
    [Serializable]
    public sealed class ProceduralMapSettings
    {
        public int seed = 104729;
        [Range(8, 48)] public int width = 10;
        [Range(8, 48)] public int length = 10;
        public float blockWorldSize = 4f;
        public float heightStepWorld = 2f;
        public TerrainDirection uphill = TerrainDirection.North;
        [Tooltip("Number of elevation transitions (cliff density). Requires room for flat entries between cuts.")]
        [Range(1, 4)] public int terraceCount = 2;
        [Range(1, 8)] public int rampsPerTerrace = 1;
        [Range(3, 4)] public int rampWidth = 4;
        [Tooltip("Probability of a one-block irregularity in an unreserved cliff edge.")]
        [Range(0f, 1f)] public float edgeVariation = 0.65f;
        public Vector2Int startZone = new Vector2Int(1, 1);
        public Vector2Int goalZone = new Vector2Int(8, 8);
        [Tooltip("Additional required flat, connected blocks. Invalid layouts fail explicitly.")]
        public Vector2Int[] keyZones = Array.Empty<Vector2Int>();
    }

    /// <summary>Only generates INPUT. One bounded layout pass, shared terrain validation,
    /// then a four-edge connectivity check. No navigation routes or second mesh builder.</summary>
    public static class ProceduralTerrainGenerator
    {
        public static TerrainSurface Generate(ProceduralMapSettings settings)
        {
            if (settings == null) throw new ArgumentException("Procedural terrain needs settings.");
            var s = settings;
            if (s.width < 8 || s.width > 48 || s.length < 8 || s.length > 48
                || s.terraceCount < 1 || s.terraceCount > 4 || s.rampWidth < 3 || s.rampWidth > 4
                || !Enum.IsDefined(typeof(TerrainDirection), s.uphill)
                || float.IsNaN(s.edgeVariation) || s.edgeVariation < 0 || s.edgeVariation > 1)
                throw new ArgumentException("Procedural terrain: dimensions 8..48, terraces 1..4, width 3..4 and variation 0..1 required.");
            bool alongZ = s.uphill == TerrainDirection.North || s.uphill == TerrainDirection.South;
            int acrossCount = alongZ ? s.width : s.length;
            int depthCount = alongZ ? s.length : s.width;
            bool reversed = s.uphill == TerrainDirection.South || s.uphill == TerrainDirection.West;
            int Depth(Vector2Int b) => reversed ? depthCount - 1 - (alongZ ? b.y : b.x) : (alongZ ? b.y : b.x);
            Vector2Int Block(int depth, int across)
            {
                if (reversed) depth = depthCount - 1 - depth;
                return alongZ ? new Vector2Int(across, depth) : new Vector2Int(depth, across);
            }
            void CheckZone(Vector2Int b)
            {
                if (b.x < 0 || b.y < 0 || b.x >= s.width || b.y >= s.length)
                    throw new ArgumentException($"Procedural terrain: required zone {b} is outside the map.");
            }
            // 1. Reserve exact key blocks BEFORE assigning plateaus or passes.
            var zones = new List<Vector2Int> { s.startZone, s.goalZone };
            if (s.keyZones != null) zones.AddRange(s.keyZones);
            foreach (var b in zones) CheckZone(b);
            int firstDepth = Depth(s.startZone), lastDepth = Depth(s.goalZone);
            int spare = lastDepth - firstDepth - (s.terraceCount * 3 + 1);
            var candidates = new List<int>();
            for (int cross = 1; cross < acrossCount - 1; cross += 3) candidates.Add(cross);
            if (spare < 0 || s.rampsPerTerrace < 1 || s.rampsPerTerrace > candidates.Count)
                throw new ArgumentException("Procedural terrain: zones need 3 * terraceCount + 1 blocks of uphill separation; too many ramps need a wider map.");
            var random = new SeededRandom(s.seed);
            var borders = new int[s.terraceCount, acrossCount];
            var lanes = new int[s.terraceCount, s.rampsPerTerrace];
            // 2. Flat terraces, spaced to leave lower entries between successive passes.
            int offset = 0;
            for (int terrace = 0; terrace < s.terraceCount; terrace++)
            {
                int extra = random.Next(spare + 1);
                offset += extra; spare -= extra;
                int border = firstDepth + 3 + terrace * 3 + offset;
                for (int cross = 0; cross < acrossCount; cross++) borders[terrace, cross] = border;
                // 3. Required passes; candidates are separated by two rock shoulder blocks.
                for (int i = candidates.Count - 1; i > 0; i--)
                {
                    int j = random.Next(i + 1);
                    int temp = candidates[i]; candidates[i] = candidates[j]; candidates[j] = temp;
                }
                for (int i = 0; i < s.rampsPerTerrace; i++) lanes[terrace, i] = candidates[i];
            }
            var levels = new int[s.width * s.length];
            var ramps = new List<TerrainRamp>();
            for (int terrace = 0; terrace < s.terraceCount; terrace++)
            for (int i = 0; i < s.rampsPerTerrace; i++)
            {
                int cross = lanes[terrace, i];
                var rampBlock = Block(borders[terrace, cross] - 1, cross);
                foreach (var zone in zones)
                    if (zone == rampBlock)
                        throw new ArgumentException($"Procedural terrain: zone {zone} conflicts with a required ramp; change seed or zone.");
                ramps.Add(new TerrainRamp { block = rampBlock, uphill = s.uphill,
                    startSubcell = s.rampWidth == 4 ? 0 : random.Next(2),
                    widthSubcells = s.rampWidth, cutIntoCliff = true });
            }
            void FillLevels()
            {
                for (int cross = 0; cross < acrossCount; cross++)
                for (int depth = 0; depth < depthCount; depth++)
                {
                    int level = 0;
                    for (int t = 0; t < s.terraceCount; t++) if (depth >= borders[t, cross]) level++;
                    Vector2Int b = Block(depth, cross);
                    levels[b.x + b.y * s.width] = level;
                }
                // Rock shoulders span one block on each side of the whole pass, also for width 4.
                for (int t = 0; t < s.terraceCount; t++)
                for (int i = 0; i < s.rampsPerTerrace; i++)
                for (int side = -1; side <= 1; side += 2)
                {
                    int cross = lanes[t, i];
                    Vector2Int b = Block(borders[t, cross] - 1, cross + side);
                    levels[b.x + b.y * s.width] = t + 1;
                }
            }
            TerrainSurface Snapshot() => TerrainMapData.CreateSurface(s.width, s.length, s.blockWorldSize,
                s.heightStepWorld, levels, ramps.ToArray(), s.seed);
            FillLevels();
            // 4. Shared validation and required-zone connectivity BEFORE natural variation.
            CheckConnectivity(Snapshot(), zones);
            // 5. Change only unreserved boundary columns, never the ramp, its shoulders or keys.
            for (int t = 0; t < s.terraceCount; t++)
            for (int cross = 0; cross < acrossCount; cross++)
            {
                bool reserved = false;
                for (int i = 0; i < s.rampsPerTerrace; i++)
                    reserved |= Math.Abs(cross - lanes[t, i]) <= 1;
                foreach (var zone in zones) reserved |= cross == (alongZ ? zone.x : zone.y);
                if (!reserved && random.Fraction() < s.edgeVariation) borders[t, cross]++;
            }
            FillLevels();
            TerrainSurface surface = Snapshot();
            CheckConnectivity(surface, zones);
            return surface;
        }

        private static void CheckConnectivity(TerrainSurface surface, List<Vector2Int> zones)
        {
            var visited = new bool[surface.CellsX * surface.CellsZ];
            var queue = new Queue<Vector2Int>();
            Vector2Int first = zones[0] * 4 + new Vector2Int(2, 2);
            queue.Enqueue(first); visited[first.x + first.y * surface.CellsX] = true;
            while (queue.Count > 0)
            {
                Vector2Int cell = queue.Dequeue();
                for (int d = 0; d < 4; d++)
                {
                    var side = (TerrainDirection)d;
                    if (surface.IsBoundaryBlocked(cell.x, cell.y, side)) continue;
                    Vector2Int next = cell + TerrainSurface.Step(side);
                    int index = next.x + next.y * surface.CellsX;
                    if (visited[index]) continue;
                    visited[index] = true; queue.Enqueue(next);
                }
            }
            foreach (var zone in zones)
            {
                int x = zone.x * 4, z = zone.y * 4;
                int level = surface.GetCell(x, z).Level;
                for (int dz = 0; dz < 4; dz++)
                for (int dx = 0; dx < 4; dx++)
                {
                    var cell = surface.GetCell(x + dx, z + dz);
                    if (cell.IsRamp || cell.Level != level || !visited[x + dx + (z + dz) * surface.CellsX])
                        throw new ArgumentException($"Procedural terrain: required zone {zone} must be flat and connected.");
                }
            }
        }

        // Specified xorshift32, independent of UnityEngine.Random and framework Random versions.
        private struct SeededRandom
        {
            private uint state;
            public SeededRandom(int seed) { state = unchecked((uint)seed) ^ 0x9e3779b9u; if (state == 0) state = 1; }
            private uint NextUInt() { state ^= state << 13; state ^= state >> 17; state ^= state << 5; return state; }
            public int Next(int exclusiveMax) => (int)(NextUInt() % (uint)exclusiveMax);
            public float Fraction() => (NextUInt() >> 8) / 16777216f;
        }
    }
}
