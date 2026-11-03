using System;
using System.Collections.Generic;
using UnityEngine;

namespace GameDev.RtsRpg
{
    public readonly struct NavigationMember
    {
        public NavigationPose Start { get; }
        public NavigationFootprint Footprint { get; }
        public NavigationMember(NavigationPose start, NavigationFootprint footprint) { Start = start; Footprint = footprint; }
    }

    public sealed class NavigationGroupResult
    {
        private readonly NavigationGrid grid;
        public int Revision { get; }
        public bool IsCurrent => grid.IsAlive && Revision == grid.Revision;
        public IReadOnlyList<NavigationPath> Paths { get; }
        public IReadOnlyList<Vector2Int> Corridor { get; }
        public int Accepted { get; }
        public int Expanded { get; }
        public NavigationStatus Status { get; }
        internal NavigationGroupResult(NavigationGrid grid, List<NavigationPath> paths,
            List<Vector2Int> corridor, int accepted, int expanded)
        {
            this.grid = grid; Revision = grid.Revision; Paths = paths.AsReadOnly(); Corridor = corridor.AsReadOnly();
            Accepted = accepted; Expanded = expanded;
            Status = accepted == paths.Count && accepted > 0 ? NavigationStatus.Success
                : accepted > 0 ? NavigationStatus.Partial : paths.Exists(p => p.Status == NavigationStatus.BudgetExceeded)
                    ? NavigationStatus.BudgetExceeded : NavigationStatus.NoPath;
        }
    }

    /// <summary>Shared guide corridor + bounded local A*, temporary destination reservations only.</summary>
    public sealed class NavigationGroupPlanner
    {
        private readonly NavigationGrid grid;
        private readonly NavigationPathfinder finder;
        public NavigationGroupPlanner(NavigationGrid grid)
        { this.grid = grid ?? throw new ArgumentNullException(nameof(grid)); finder = new NavigationPathfinder(grid); }

        public NavigationGroupResult Plan(IReadOnlyList<NavigationMember> members, Vector2Int target,
            int radius = 12, int totalExpanded = 80000, int localExpanded = 8000, int candidatesPerMember = 32)
            => Plan(members, target, radius, totalExpanded, localExpanded, candidatesPerMember, null, null, null, null);

        internal NavigationGroupResult Plan(IReadOnlyList<NavigationMember> members, Vector2Int target,
            int radius, int totalExpanded, int localExpanded, int candidatesPerMember,
            HashSet<Vector2Int> reservations, IReadOnlyList<Vector2Int> targetArea, HashSet<Vector2Int> sharedCorridor,
            Func<NavigationPose, NavigationPose, bool> allowsEdge)
        {
            if (members == null || members.Count < 1 || members.Count > 100 || radius < 0 || radius > 32
                || totalExpanded < 1 || totalExpanded > 1000000 || localExpanded < 1 || localExpanded > 1000000
                || candidatesPerMember < 1 || candidatesPerMember > 128)
                throw new ArgumentException("Group: 1..100 members, radius 0..32, budgets 1..1000000, candidates 1..128 required.");
            var budget = new NavigationBudget(totalExpanded);
            var results = new List<NavigationPath>();
            var guide = new List<Vector2Int>();
            var corridor = sharedCorridor ?? new HashSet<Vector2Int>();
            var reserved = reservations ?? new HashSet<Vector2Int>();
            var candidates = targetArea ?? BuildTargetArea(target, radius);
            // Reject unbounded target arithmetic; outside clicks are reported for each member.
            bool targetInside = grid.Surface.Contains(target.x, target.y);
            // One shared route for a 1x1 guide. Only actual success seeds a corridor.
            if (targetInside && sharedCorridor == null)
            {
                var small = new NavigationFootprint(1, 1);
                var common = finder.Find(new NavigationRequest(new NavigationPose(members[0].Start.Cell),
                    new NavigationPose(target), small, Math.Min(localExpanded, totalExpanded)), budget);
                if (common.Status == NavigationStatus.Success)
                foreach (var p in common.Points)
                {
                    guide.Add(p.Pose.Cell);
                    for (int z = -2; z <= 2; z++)
                    for (int x = -2; x <= 2; x++) corridor.Add(p.Pose.Cell + new Vector2Int(x, z));
                }
            }
            // Stable larger-first allocation reduces starvation; results retain input indices.
            var order = new List<int>();
            for (int i = 0; i < members.Count; i++) { order.Add(i); results.Add(null); }
            order.Sort((a, b) =>
            {
                long aa = (long)members[a].Footprint.Width * members[a].Footprint.Length;
                long bb = (long)members[b].Footprint.Width * members[b].Footprint.Length;
                int size = bb.CompareTo(aa); return size != 0 ? size : a.CompareTo(b);
            });
            int accepted = 0;
            foreach (int index in order)
            {
                var member = members[index];
                NavigationPath failure = null;
                if (!grid.IsAlive) failure = new NavigationPath(grid, NavigationStatus.Stale, "Grid retired.");
                else if (!member.Footprint.IsValid || (uint)member.Start.Facing > 3)
                    failure = new NavigationPath(grid, NavigationStatus.InvalidRequest, "Unsupported footprint or facing.");
                else if (!grid.CanOccupy(member.Start, member.Footprint))
                    failure = new NavigationPath(grid, NavigationStatus.InvalidStart, "Invalid group member start.");
                else if (!targetInside) failure = new NavigationPath(grid, NavigationStatus.InvalidGoal, "Group target is outside the map.");
                if (failure != null) { results[index] = failure; continue; }
                int tried = 0, memberRemaining = localExpanded;
                foreach (var goal in DestinationCandidates(member.Footprint, candidates, reserved))
                {
                    if (tried++ >= candidatesPerMember || budget.Remaining == 0 || memberRemaining == 0) break;
                    NavigationPath path = null;
                    int before = budget.Remaining;
                    if (corridor.Contains(member.Start.Cell) && corridor.Contains(goal.Cell))
                    {
                        int corridorBudget = Math.Min(memberRemaining, budget.Remaining) / 3;
                        if (corridorBudget > 0)
                            path = finder.Find(new NavigationRequest(member.Start, goal, member.Footprint, corridorBudget),
                                budget, corridor, true, allowsEdge);
                        memberRemaining -= before - budget.Remaining;
                    }
                    if ((path == null || path.Status != NavigationStatus.Success) && memberRemaining > 0 && budget.Remaining > 0)
                    {
                        before = budget.Remaining;
                        path = finder.Find(new NavigationRequest(member.Start, goal, member.Footprint, memberRemaining), budget, corridor, allowsEdge: allowsEdge);
                        memberRemaining -= before - budget.Remaining;
                    }
                    if (path == null) break;
                    failure = path;
                    if (path.Status != NavigationStatus.Success) continue;
                    results[index] = path; accepted++; Reserve(goal, member.Footprint, reserved); break;
                }
                if (results[index] == null)
                    results[index] = new NavigationPath(grid,
                        budget.Remaining == 0 || tried >= candidatesPerMember || memberRemaining == 0 ? NavigationStatus.BudgetExceeded
                            : failure?.Status ?? NavigationStatus.NoPath,
                        budget.Remaining == 0 || tried >= candidatesPerMember || memberRemaining == 0
                            ? "Group expansion/candidate cap reached; member not assigned. Explicit retry is required."
                            : failure?.Reason ?? "No reachable, non-overlapping destination within the search radius.");
            }
            return new NavigationGroupResult(grid, results, guide, accepted, totalExpanded - budget.Remaining);
        }

        internal List<Vector2Int> BuildTargetArea(Vector2Int target, int radius)
        {
            var cells = new List<Vector2Int>();
            var single = new NavigationFootprint(1, 1);
            if (!grid.CanOccupy(new NavigationPose(target), single)) return cells;
            var visited = new HashSet<Vector2Int> { target };
            cells.Add(target);
            // Flood only from the pointed surface, inside the local search window.
            // A nearby anchor across a cliff is eligible only via a real local passage.
            for (int i = 0; i < cells.Count; i++)
            {
                var from = new NavigationPose(cells[i]);
                for (int d = 0; d < 4; d++)
                {
                    var to = new NavigationPose(from.Cell + TerrainSurface.Step((TerrainDirection)d));
                    if (Math.Max(Math.Abs(to.Cell.x - target.x), Math.Abs(to.Cell.y - target.y)) <= radius
                        && !visited.Contains(to.Cell) && grid.CanStep(from, to, single))
                    {
                        visited.Add(to.Cell);
                        cells.Add(to.Cell);
                    }
                }
            }
            return cells;
        }

        internal IEnumerable<NavigationPose> DestinationCandidates(NavigationFootprint footprint,
            IReadOnlyList<Vector2Int> area, HashSet<Vector2Int> reserved, NavigationPose? ownSlot = null)
        {
            RectInt ownArea = ownSlot.HasValue
                ? new RectInt(ownSlot.Value.Cell, footprint.Size(ownSlot.Value.Facing)) : default;
            int orientations = footprint.NeedsTurns ? 4 : 1;
            foreach (var cell in area)
            for (int facing = 0; facing < orientations; facing++)
            {
                var pose = new NavigationPose(cell, (TerrainDirection)facing);
                if (grid.CanOccupy(pose, footprint) && !Overlaps(pose, footprint, reserved, ownArea))
                    yield return pose;
            }
        }

        private static bool Overlaps(NavigationPose p, NavigationFootprint f, HashSet<Vector2Int> reserved,
            RectInt ownArea)
        {
            Vector2Int s = f.Size(p.Facing);
            for (int z = 0; z < s.y; z++)
            for (int x = 0; x < s.x; x++)
            {
                var cell = p.Cell + new Vector2Int(x, z);
                if (reserved.Contains(cell) && !ownArea.Contains(cell)) return true;
            }
            return false;
        }
        internal static void Reserve(NavigationPose p, NavigationFootprint f, HashSet<Vector2Int> reserved, bool add = true)
        {
            Vector2Int s = f.Size(p.Facing);
            for (int z = 0; z < s.y; z++)
            for (int x = 0; x < s.x; x++)
            {
                var cell = p.Cell + new Vector2Int(x, z);
                if (add) reserved.Add(cell); else reserved.Remove(cell);
            }
        }
    }
}
