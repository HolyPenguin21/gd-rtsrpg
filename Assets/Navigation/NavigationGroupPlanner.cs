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
                else if (!member.Footprint.IsValid || (uint)member.Start.Facing > 7)
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
            var flood = new TargetFlood(grid, target, radius);
            flood.Advance(new NavigationBudget(int.MaxValue));
            return flood.Cells;
        }

        internal sealed class TargetFlood
        {
            private static readonly NavigationFootprint Single = new(1, 1);
            private static readonly Unity.Profiling.ProfilerMarker Marker = new("Navigation.TargetArea");
            private readonly NavigationGrid grid;
            private readonly Vector2Int target;
            private readonly int radius, revision;
            private readonly bool[] visited;
            private int cursor, boundaryCursor;
            internal List<Vector2Int> Cells { get; } = new();
            internal List<Vector2Int> EntryCells { get; } = new();

            internal TargetFlood(NavigationGrid grid, Vector2Int target, int radius)
            {
                this.grid = grid;
                this.target = target;
                this.radius = radius;
                revision = grid.Revision;
                int width = radius * 2 + 1;
                visited = new bool[width * width];
                if (!grid.CanOccupy(new NavigationPose(target), Single)) return;
                visited[Index(target)] = true;
                Cells.Add(target);
            }

            private int Index(Vector2Int cell) =>
                cell.x - target.x + radius + (cell.y - target.y + radius) * (radius * 2 + 1);

            internal bool Contains(Vector2Int cell) => Math.Abs(cell.x - target.x) <= radius
                && Math.Abs(cell.y - target.y) <= radius && visited[Index(cell)];

            internal bool Advance(NavigationBudget budget)
            {
                using var marker = Marker.Auto();
                if (!grid.IsAlive || revision != grid.Revision) { Cells.Clear(); EntryCells.Clear(); return true; }
                while (cursor < Cells.Count)
                {
                    if (!budget.Take()) return false;
                    var pose = new NavigationPose(Cells[cursor++]);
                    int mask = grid.Edges(pose, Single);
                    for (int i = 0; i < 8; i += 2)
                    {
                        if ((mask & (1 << i)) == 0) continue;
                        var cell = pose.Cell + NavigationPose.Steps[i];
                        if (Math.Abs(cell.x - target.x) > radius || Math.Abs(cell.y - target.y) > radius) continue;
                        int index = Index(cell);
                        if (visited[index]) continue;
                        visited[index] = true;
                        Cells.Add(cell);
                    }
                }
                while (boundaryCursor < Cells.Count)
                {
                    if (!budget.Take()) return false;
                    Vector2Int cell = Cells[boundaryCursor++];
                    foreach (var step in NavigationPose.Steps)
                    {
                        Vector2Int neighbor = cell + step;
                        if (!grid.Surface.Contains(neighbor.x, neighbor.y) || Contains(neighbor)) continue;
                        EntryCells.Add(cell);
                        break;
                    }
                }
                return true;
            }
        }

        internal IEnumerable<NavigationPose> DestinationCandidates(NavigationFootprint footprint,
            IReadOnlyList<Vector2Int> area, HashSet<Vector2Int> reserved, NavigationPose? ownSlot = null)
        {
            int cursor = 0;
            while (cursor < area.Count * footprint.Orientations)
                if (TryCandidate(footprint, area, reserved, ownSlot, ref cursor, out var pose)) yield return pose;
        }

        // Exactly one candidate per call, including rejected slots. The demo
        // can suspend even when almost all positions are already reserved.
        internal bool TryCandidate(NavigationFootprint footprint, IReadOnlyList<Vector2Int> area,
            HashSet<Vector2Int> reserved, NavigationPose? ownSlot, ref int cursor, out NavigationPose pose,
            NavigationFacing preferred = NavigationFacing.North)
        {
            int orientations = footprint.Orientations;
            int rank = cursor % orientations;
            int facing = footprint.NeedsTurns ? ((int)preferred + rank * 2 + rank / 4) & 7 : 0;
            pose = new NavigationPose(area[cursor / orientations], (NavigationFacing)facing);
            cursor++;
            if (!grid.CanOccupy(pose, footprint)) return false;
            return CanReserve(pose, footprint, reserved, ownSlot);
        }

        internal static bool CanReserve(NavigationPose p, NavigationFootprint f, HashSet<Vector2Int> reserved,
            NavigationPose? ownSlot)
        {
            RectInt ownArea = ownSlot.HasValue ? new RectInt(ownSlot.Value.Cell, f.Size(ownSlot.Value.Facing)) : default;
            Vector2Int s = f.Size(p.Facing);
            for (int z = 0; z < s.y; z++)
            for (int x = 0; x < s.x; x++)
            {
                var cell = p.Cell + new Vector2Int(x, z);
                if (reserved.Contains(cell) && !ownArea.Contains(cell)) return false;
            }
            return true;
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
