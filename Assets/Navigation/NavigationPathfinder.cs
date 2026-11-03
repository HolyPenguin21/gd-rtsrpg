using System;
using System.Collections.Generic;
using UnityEngine;

namespace GameDev.RtsRpg
{
    /// <summary>Explicit bounded A*. No Update, physics, smoothing, movement or persistent search cache.</summary>
    public sealed class NavigationPathfinder
    {
        private readonly NavigationGrid grid;
        public NavigationPathfinder(NavigationGrid grid) { this.grid = grid ?? throw new ArgumentNullException(nameof(grid)); }
        private static readonly Vector2Int[] Steps = {
            new Vector2Int(0, 1), new Vector2Int(1, 0), new Vector2Int(0, -1), new Vector2Int(-1, 0),
            new Vector2Int(1, 1), new Vector2Int(1, -1), new Vector2Int(-1, -1), new Vector2Int(-1, 1) };

        public NavigationPath Find(NavigationRequest request, NavigationBudget budget = null) => Find(request, budget, null);

        // Group local pass can restrict the corridor; its fallback uses the same solver unrestricted.
        internal NavigationPath Find(NavigationRequest request, NavigationBudget budget, HashSet<Vector2Int> corridor, bool restrictToCorridor = false)
        {
            NavigationPath Fail(NavigationStatus s, string reason) => new NavigationPath(grid, s, reason);
            if (!grid.IsAlive) return Fail(NavigationStatus.Stale, "Terrain snapshot was retired; rebuild navigation and replan explicitly.");
            if (!request.Footprint.IsValid || (uint)request.Start.Facing > 3 || (uint)request.Goal.Facing > 3
                || request.MaxExpanded < 1 || request.MaxExpanded > 1000000)
                return Fail(NavigationStatus.InvalidRequest, "Supported footprints: 1x1, 1x2, 2x2; facing N/E/S/W; budget 1..1000000.");
            var footprint = request.Footprint;
            // A square has no geometric orientation state; all its output poses use canonical North.
            NavigationPose Canonical(NavigationPose p) => footprint.NeedsTurns ? p : new NavigationPose(p.Cell);
            NavigationPose start = Canonical(request.Start), goal = Canonical(request.Goal);
            if (!grid.CanOccupy(start, footprint)) return Fail(NavigationStatus.InvalidStart, "Start footprint is outside, blocked or straddles a cliff.");
            if (!grid.CanOccupy(goal, footprint)) return Fail(NavigationStatus.InvalidGoal, "Goal footprint is outside, blocked or straddles a cliff.");
            budget = budget ?? new NavigationBudget(request.MaxExpanded);
            var nodes = new Dictionary<NavigationPose, Node>();
            var open = new Heap();
            int Heuristic(NavigationPose p)
            {
                int dx = Math.Abs(p.Cell.x - goal.Cell.x), dz = Math.Abs(p.Cell.y - goal.Cell.y);
                return 14 * Math.Min(dx, dz) + 10 * Math.Abs(dx - dz);
            }
            int Preference(NavigationPose p) => corridor == null || corridor.Contains(p.Cell) ? 0 : 1;
            var initial = new Node { Cost = 0 };
            nodes.Add(start, initial);
            open.Push(new Entry(start, 0, Heuristic(start), Preference(start)));
            NavigationPose best = start;
            int bestH = Heuristic(start), expanded = 0;
            bool exhausted = false, found = false;
            while (open.Count > 0)
            {
                Entry entry = open.Pop();
                Node current = nodes[entry.Pose];
                if (current.Closed || current.Cost != entry.Cost) continue;
                // Reaching the goal needs no expansion, including start == goal at zero shared budget.
                if (entry.Pose.Equals(goal)) { best = goal; found = true; break; }
                if (expanded >= request.MaxExpanded || !budget.Take()) { exhausted = true; break; }
                current.Closed = true; expanded++;
                int h = Heuristic(entry.Pose);
                if (h < bestH) { bestH = h; best = entry.Pose; }
                void Relax(NavigationPose next, int cost)
                {
                    int candidate = current.Cost + cost;
                    if (nodes.TryGetValue(next, out Node existing) && (existing.Closed || existing.Cost <= candidate)) return;
                    if (existing == null) { existing = new Node(); nodes.Add(next, existing); }
                    existing.Cost = candidate; existing.Parent = entry.Pose; existing.HasParent = true;
                    open.Push(new Entry(next, candidate, Heuristic(next), Preference(next)));
                }
                foreach (var step in Steps)
                {
                    var next = new NavigationPose(entry.Pose.Cell + step, entry.Pose.Facing);
                    if ((!restrictToCorridor || corridor.Contains(next.Cell)) && grid.CanStep(entry.Pose, next, footprint)) Relax(next, step.x != 0 && step.y != 0 ? 14 : 10);
                }
                if (footprint.NeedsTurns)
                for (int turn = -1; turn <= 1; turn += 2)
                {
                    var next = new NavigationPose(entry.Pose.Cell, (TerrainDirection)(((int)entry.Pose.Facing + turn + 4) % 4));
                    if (grid.CanTurn(entry.Pose, next, footprint)) Relax(next, 10);
                }
            }
            NavigationStatus status = found ? NavigationStatus.Success : exhausted ? NavigationStatus.BudgetExceeded : NavigationStatus.NoPath;
            string reason = found ? "Reached exact goal footprint and orientation." : exhausted
                ? "Expansion cap reached; retry explicitly with a larger budget." : "No route for this footprint and orientation.";
            if (!found && (!request.AllowPartial || best.Equals(start)))
                return new NavigationPath(grid, status, reason, expanded: expanded);
            if (!found) { status = NavigationStatus.Partial; reason += " Partial path ends at the closest expanded state, not at the goal."; }
            var poses = new List<NavigationPose>();
            NavigationPose cursor = best;
            while (true)
            {
                poses.Add(cursor);
                Node n = nodes[cursor];
                if (!n.HasParent) break;
                cursor = n.Parent;
            }
            poses.Reverse();
            var points = new List<NavigationPoint>(poses.Count);
            foreach (var pose in poses) points.Add(grid.Point(pose, footprint));
            return new NavigationPath(grid, status, reason, points, expanded, nodes[best].Cost);
        }

        private sealed class Node
        {
            public int Cost;
            public bool Closed, HasParent;
            public NavigationPose Parent;
        }
        private readonly struct Entry
        {
            public readonly NavigationPose Pose;
            public readonly int Cost, H, Preference;
            public Entry(NavigationPose pose, int cost, int h, int preference)
            { Pose = pose; Cost = cost; H = h; Preference = preference; }
        }
        private sealed class Heap
        {
            private readonly List<Entry> values = new List<Entry>();
            public int Count => values.Count;
            private static bool Before(Entry a, Entry b)
            {
                int f = (a.Cost + a.H).CompareTo(b.Cost + b.H);
                if (f != 0) return f < 0;
                if (a.Preference != b.Preference) return a.Preference < b.Preference;
                if (a.H != b.H) return a.H < b.H;
                if (a.Pose.Cell.y != b.Pose.Cell.y) return a.Pose.Cell.y < b.Pose.Cell.y;
                if (a.Pose.Cell.x != b.Pose.Cell.x) return a.Pose.Cell.x < b.Pose.Cell.x;
                return (int)a.Pose.Facing < (int)b.Pose.Facing;
            }
            public void Push(Entry value)
            {
                int i = values.Count; values.Add(value);
                while (i > 0)
                {
                    int parent = (i - 1) / 2;
                    if (!Before(value, values[parent])) break;
                    values[i] = values[parent]; i = parent;
                }
                values[i] = value;
            }
            public Entry Pop()
            {
                Entry first = values[0], last = values[values.Count - 1];
                values.RemoveAt(values.Count - 1);
                if (values.Count == 0) return first;
                int i = 0;
                while (i * 2 + 1 < values.Count)
                {
                    int child = i * 2 + 1;
                    if (child + 1 < values.Count && Before(values[child + 1], values[child])) child++;
                    if (!Before(values[child], last)) break;
                    values[i] = values[child]; i = child;
                }
                values[i] = last; return first;
            }
        }
    }
}
