using System;
using System.Collections.Generic;
using UnityEngine;

namespace GameDev.RtsRpg
{
    [Serializable]
    public sealed class ProceduralMapSettings
    {
        public int seed = 104729;
        public bool naturalLandforms;
        [Range(8, 128)] public int width = 10;
        [Range(8, 128)] public int length = 10;
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
            if (s.naturalLandforms) return GenerateLandforms(s);
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

        private static TerrainSurface GenerateLandforms(ProceduralMapSettings s)
        {
            if (s.width < 64 || s.width > 128 || s.length < 64 || s.length > 128)
                throw new ArgumentException("Natural landforms require 64..128 blocks on each axis.");

            var random = new SeededRandom(s.seed);
            var levels = new int[s.width * s.length];
            var ramps = new List<TerrainRamp>();
            var zones = new List<Vector2Int> { new Vector2Int(1, 1) };
            var isolated = new List<Vector2Int>();
            if (s.keyZones != null) zones.AddRange(s.keyZones);
            var slots = new[] { new Vector2Int(1, 1), new Vector2Int(3, 1),
                new Vector2Int(5, 1), new Vector2Int(1, 3), new Vector2Int(4, 3) };
            // The mountain reserves the larger northern slot. Mirror the layout by seed.
            int mirror = random.Next(2);
            for (int i = 3; i > 0; i--)
            {
                int j = random.Next(i + 1);
                (slots[i], slots[j]) = (slots[j], slots[i]);
            }

            Vector2Int[] landmarks = new Vector2Int[5];
            for (int i = 0; i < landmarks.Length; i++)
            {
                int x = slots[i].x * s.width / 6;
                var centre = new Vector2Int((mirror == 0 ? x : s.width - 1 - x) + random.Next(3) - 1,
                    slots[i].y * s.length / 5 + random.Next(3) - 1);
                landmarks[i] = centre;
                if (i == 4)
                {
                    for (int tier = 0; tier < 3; tier++)
                    {
                        int radius = 11 - tier * 4;
                        Shape(centre, radius, tier + 1);
                        Cut(centre, radius, tier, (TerrainDirection)((mirror + tier) % 4), tier == 1 ? 1 : 2);
                    }
                    zones.Add(centre);
                    continue;
                }
                int level = i < 2 ? -1 : 1;
                Shape(centre, 6, level);
                if (i % 2 == 0)
                {
                    Cut(centre, 6, Math.Min(0, level), (TerrainDirection)random.Next(4), i == 0 ? 1 : 2);
                    zones.Add(centre);
                }
                else isolated.Add(centre);
            }

            var surface = TerrainMapData.CreateSurface(s.width, s.length, s.blockWorldSize,
                s.heightStepWorld, levels, ramps.ToArray(), s.seed, landmarks);
            CheckConnectivity(surface, zones, isolated);
            return surface;

            void Set(Vector2Int block, int level) => levels[block.x + block.y * s.width] = level;

            void Shape(Vector2Int centre, int radius, int level)
            {
                float phase = random.Fraction() * 2f * Mathf.PI;
                for (int z = -radius - 1; z <= radius + 1; z++)
                for (int x = -radius - 1; x <= radius + 1; x++)
                {
                    float angle = Mathf.Atan2(z, x);
                    float edge = radius + 0.8f * Mathf.Sin(3 * angle + phase)
                        + 0.45f * Mathf.Cos(5 * angle - phase);
                    if (x * x + z * z <= edge * edge)
                        Set(centre + new Vector2Int(x, z), level);
                }
            }

            void Cut(Vector2Int centre, int radius, int low, TerrainDirection side, int width)
            {
                Vector2Int outward = TerrainSurface.Step(side);
                Vector2Int across = new Vector2Int(outward.y, -outward.x);
                bool basin = low < 0;
                var block = centre + outward * (basin ? radius - 1 : radius);
                var uphill = basin ? side : TerrainSurface.Opposite(side);
                Vector2Int step = TerrainSurface.Step(uphill);
                // Reserve the entire apron through the irregular rim. The lower apron
                // ends on the surrounding floor and the upper apron on its terrace.
                for (int depth = -2; depth <= 2; depth++)
                for (int lane = -1; lane <= 1; lane++)
                    Set(block + depth * step + lane * across, low + (depth > 0 || depth == 0 && lane != 0 ? 1 : 0));
                ramps.Add(new TerrainRamp { block = block, uphill = uphill, startSubcell = 1,
                    widthSubcells = width, cutIntoCliff = true, flareExit = true });
            }
        }

        private static void CheckConnectivity(TerrainSurface surface, List<Vector2Int> zones, List<Vector2Int> isolated = null)
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
                if (!surface.Contains(x, z))
                    throw new ArgumentException($"Procedural terrain: required zone {zone} is outside the map.");
                int level = surface.GetCell(x, z).Level;
                for (int dz = 0; dz < 4; dz++)
                for (int dx = 0; dx < 4; dx++)
                {
                    var cell = surface.GetCell(x + dx, z + dz);
                    if (cell.IsRamp || cell.Level != level || !visited[x + dx + (z + dz) * surface.CellsX])
                        throw new ArgumentException($"Procedural terrain: required zone {zone} must be flat and connected.");
                }
            }
            if (isolated != null)
                foreach (var zone in isolated)
                    if (visited[zone.x * 4 + 2 + (zone.y * 4 + 2) * surface.CellsX])
                        throw new ArgumentException($"Procedural terrain: isolated landmark {zone} has a route from the plain.");
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
