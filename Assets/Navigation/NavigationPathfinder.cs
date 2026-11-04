using System;
using System.Collections.Generic;
using Unity.Profiling;
using UnityEngine;

namespace GameDev.RtsRpg
{
    /// <summary>Indexed A* with pooled workspaces and resumable route reconstruction.</summary>
    public sealed class NavigationPathfinder
    {
        private static readonly ProfilerMarker SearchMarker = new("Navigation.AStar");
        private static readonly ProfilerMarker FieldMarker = new("Navigation.GroupField");
        private readonly NavigationGrid grid;
        private readonly Stack<Frontier> free = new();
        public NavigationPathfinder(NavigationGrid grid) { this.grid = grid ?? throw new ArgumentNullException(nameof(grid)); }

        public NavigationPath Find(NavigationRequest request, NavigationBudget budget = null) => Find(request, budget, null);

        internal NavigationPath Find(NavigationRequest request, NavigationBudget budget, HashSet<Vector2Int> corridor,
            bool restrictToCorridor = false, Func<NavigationPose, NavigationPose, bool> allowsEdge = null)
        {
            using var search = new Search(this, request, corridor, restrictToCorridor, allowsEdge, false);
            return search.Advance(budget ?? new NavigationBudget(request.MaxExpanded));
        }

        internal Search BeginSearch(NavigationRequest request, HashSet<Vector2Int> corridor = null,
            Func<NavigationPose, NavigationPose, bool> allowsEdge = null, bool restrictToCorridor = false)
            => new(this, request, corridor, restrictToCorridor, allowsEdge, true);

        internal Search BeginGuide(Vector2Int target, HashSet<int> anchors, RectInt bounds)
            => new(this, new NavigationRequest(new NavigationPose(target), default, new NavigationFootprint(1, 1),
                grid.Surface.CellsX * grid.Surface.CellsZ), null, false, null, true, anchors, bounds);

        private Frontier Rent() => free.Count == 0 ? new Frontier() : free.Pop();
        private void Return(Frontier workspace)
        {
            workspace.Clear();
            free.Push(workspace);
        }

        internal sealed class Search : IDisposable
        {
            private readonly NavigationGrid grid;
            private readonly int revision;
            private IEnumerator<NavigationPath> work;
            private NavigationPath result;
            internal NavigationBudget Budget { get; private set; }

            internal Search(NavigationPathfinder finder, NavigationRequest request, HashSet<Vector2Int> corridor,
                bool restrictToCorridor, Func<NavigationPose, NavigationPose, bool> allowsEdge, bool sliced,
                HashSet<int> anchors = null, RectInt anchorBounds = default)
            {
                grid = finder.grid;
                revision = grid.Revision;
                work = finder.Solve(this, request, corridor, restrictToCorridor, allowsEdge, sliced, anchors, anchorBounds);
            }

            // Null means suspended, never unreachable.
            internal NavigationPath Advance(NavigationBudget budget)
            {
                using var marker = SearchMarker.Auto();
                if (work == null) return result;
                if (!grid.IsAlive || revision != grid.Revision)
                    result = new NavigationPath(grid, NavigationStatus.Stale, "Navigation changed during search.");
                else
                {
                    Budget = budget;
                    work.MoveNext();
                    result = work.Current;
                }
                if (result != null) Dispose();
                return result;
            }

            public void Dispose()
            {
                work?.Dispose();
                work = null;
                Budget = null;
            }
        }

        private static int Distance(Vector2Int a, Vector2Int b)
        {
            int dx = Math.Abs(a.x - b.x), dz = Math.Abs(a.y - b.y);
            return 14 * Math.Min(dx, dz) + 10 * Math.Abs(dx - dz);
        }

        internal static int EdgeCost(int edge) => edge >= 8 ? 5 : (edge & 1) == 0 ? 10 : 14;
        private static NavigationPose FollowEdge(NavigationPose pose, int edge) => edge < 8
            ? new NavigationPose(pose.Cell + NavigationPose.Steps[edge], pose.Facing)
            : pose.Turn(edge == 8 ? -1 : 1);

        private IEnumerator<NavigationPath> Solve(Search search, NavigationRequest request,
            HashSet<Vector2Int> corridor, bool restrictToCorridor,
            Func<NavigationPose, NavigationPose, bool> allowsEdge, bool sliced,
            HashSet<int> anchors, RectInt anchorBounds)
        {
            var footprint = request.Footprint;
            if (!footprint.IsValid || (uint)request.Start.Facing > 7 || (uint)request.Goal.Facing > 7
                || request.MaxExpanded < 1 || request.MaxExpanded > 4000000)
            {
                yield return new NavigationPath(grid, NavigationStatus.InvalidRequest, "Invalid footprint, facing or expansion cap.");
                yield break;
            }

            NavigationPose start = footprint.NeedsTurns ? request.Start : new NavigationPose(request.Start.Cell);
            NavigationPose goal = footprint.NeedsTurns ? request.Goal : new NavigationPose(request.Goal.Cell);
            if (!grid.CanOccupy(start, footprint))
            {
                yield return new NavigationPath(grid, NavigationStatus.InvalidStart, "Start footprint is outside, blocked or straddles a cliff.");
                yield break;
            }
            if (anchors == null && !grid.CanOccupy(goal, footprint))
            {
                yield return new NavigationPath(grid, NavigationStatus.InvalidGoal, "Goal footprint is outside, blocked or straddles a cliff.");
                yield break;
            }

            int Estimate(Vector2Int cell)
            {
                if (anchors == null) return Distance(cell, goal.Cell);
                var closest = new Vector2Int(Mathf.Clamp(cell.x, anchorBounds.xMin, anchorBounds.xMax - 1),
                    Mathf.Clamp(cell.y, anchorBounds.yMin, anchorBounds.yMax - 1));
                return Distance(cell, closest);
            }

            var frontier = Rent();
            try
            {
                int bestH = Estimate(start.Cell);
                int best = frontier.Relax(grid.StateId(start), -1, 0, bestH, 0);
                int expanded = 0, goalId = grid.StateId(goal);
                bool exhausted = false, found = false;
                while (frontier.Count != 0)
                {
                    if (anchors == null ? frontier.Peek.State == goalId : anchors.Contains(frontier.Peek.State))
                    {
                        best = frontier.PeekIndex;
                        found = true;
                        break;
                    }
                    if (expanded == request.MaxExpanded) { exhausted = true; break; }
                    while (sliced && !search.Budget.CanContinue) yield return null;
                    if (!search.Budget.Take()) { exhausted = true; break; }

                    int parent = frontier.Pop();
                    var current = frontier[parent];
                    NavigationPose from = grid.Pose(current.State);
                    expanded++;
                    if (current.H < bestH) { bestH = current.H; best = parent; }

                    int edges = grid.Edges(from, footprint);
                    for (int i = 0; edges != 0; i++, edges >>= 1)
                    {
                        if ((edges & 1) == 0) continue;
                        var to = FollowEdge(from, i);
                        bool onCorridor = corridor == null || corridor.Contains(to.Cell);
                        if (restrictToCorridor && !onCorridor) continue;
                        int id = grid.StateId(to), cost = current.Cost + grid.EdgeCost(from, to, footprint, i);
                        // Skip settled/uncompetitive edges before touching live traffic.
                        if (!frontier.CanImprove(id, cost) || (allowsEdge != null && !allowsEdge(from, to))) continue;
                        frontier.Relax(id, parent, cost, Estimate(to.Cell), onCorridor ? 0 : 1);
                    }
                }

                var status = found ? NavigationStatus.Success : exhausted ? NavigationStatus.BudgetExceeded : NavigationStatus.NoPath;
                string reason = found ? "Reached exact goal footprint and orientation." : exhausted
                    ? "Expansion cap reached." : "No route for this footprint and orientation.";
                if (anchors != null || (!found && (!request.AllowPartial || frontier[best].Parent < 0)))
                {
                    yield return new NavigationPath(grid, status, reason, expanded: expanded);
                    yield break;
                }
                if (!found) { status = NavigationStatus.Partial; reason += " Ends at the closest expanded state."; }

                var points = new List<NavigationPoint>();
                for (int index = best; index >= 0; index = frontier[index].Parent)
                {
                    while (sliced && !search.Budget.HasTime) yield return null;
                    points.Add(grid.Point(grid.Pose(frontier[index].State), footprint));
                }
                for (int i = 0, j = points.Count - 1; i < j; i++, j--)
                {
                    while (sliced && !search.Budget.HasTime) yield return null;
                    var point = points[i];
                    points[i] = points[j];
                    points[j] = point;
                }
                yield return new NavigationPath(grid, status, reason, points, expanded, frontier[best].Cost, footprint);
            }
            finally { Return(frontier); }
        }

        // One lazy reverse Dijkstra per footprint and command. Every settled pose
        // has a forward-valid successor toward the connected target area.
        internal sealed class ApproachField : IDisposable
        {
            private readonly NavigationPathfinder owner;
            private readonly NavigationFootprint footprint;
            private readonly IReadOnlyList<Vector2Int> seeds;
            private readonly int revision;
            private Frontier frontier;
            private int seedIndex;

            internal ApproachField(NavigationPathfinder owner, NavigationFootprint footprint, IReadOnlyList<Vector2Int> seeds)
            {
                this.owner = owner;
                this.footprint = footprint;
                this.seeds = seeds;
                revision = owner.grid.Revision;
                frontier = owner.Rent();
            }

            internal bool AdvanceTo(NavigationPose start, NavigationBudget budget, ref int limit, out bool reached)
            {
                using var marker = FieldMarker.Auto();
                var grid = owner.grid;
                reached = false;
                if (frontier == null || !grid.IsAlive || revision != grid.Revision || !grid.CanOccupy(start, footprint)) return true;
                int target = grid.StateId(start);
                while (seedIndex < seeds.Count)
                {
                    if (!budget.HasTime) return false;
                    var cell = seeds[seedIndex++];
                    for (int i = 0; i < footprint.Orientations; i++)
                    {
                        var pose = new NavigationPose(cell, (NavigationFacing)i);
                        if (grid.CanOccupy(pose, footprint)) frontier.Relax(grid.StateId(pose), -1, 0, 0, 0);
                    }
                }
                while (true)
                {
                    if (frontier.TryGet(target, out var node) && (node.HeapIndex < 0 || node.Parent < 0)) { reached = true; return true; }
                    if (frontier.Count == 0 || limit == 0) return true;
                    if (!budget.Take()) return false;
                    limit--;
                    int parent = frontier.Pop();
                    var current = frontier[parent];
                    var to = grid.Pose(current.State);
                    if (footprint.NeedsTurns)
                    {
                        int facing = (int)to.Facing;
                        Visit(new NavigationPose(to.Cell - NavigationPose.Steps[facing], to.Facing), facing);
                        Visit(to.Turn(1), 8);
                        Visit(to.Turn(-1), 9);
                    }
                    else
                    {
                        int reverseEdges = grid.Edges(to, footprint);
                        for (int i = 0; reverseEdges != 0; i++, reverseEdges >>= 1)
                            if ((reverseEdges & 1) != 0)
                            {
                                var from = FollowEdge(to, i);
                                frontier.Relax(grid.StateId(from), parent,
                                    current.Cost + grid.EdgeCost(from, to, footprint, (i + 4) & 7), 0, 0);
                            }
                    }

                    void Visit(NavigationPose from, int edge)
                    {
                        if (!grid.Surface.Contains(from.Cell.x, from.Cell.y)) return;
                        int id = grid.StateId(from);
                        int cost = current.Cost + grid.EdgeCost(from, to, footprint, edge);
                        if (frontier.CanImprove(id, cost) && (grid.Edges(from, footprint) & (1 << edge)) != 0)
                            frontier.Relax(id, parent, cost, 0, 0);
                    }
                }
            }

            internal bool TryNext(NavigationPose from, Vector2 preferred, out NavigationPose next)
            {
                next = default;
                var grid = owner.grid;
                if (!frontier.TryGet(grid.StateId(from), out var node) || node.Parent < 0) return false;
                next = grid.Pose(frontier[node.Parent].State);
                float best = float.PositiveInfinity;
                float preferredDistance = (from.Centre(footprint) - preferred).magnitude;
                int mask = grid.Edges(from, footprint);
                for (int i = 0; mask != 0; i++, mask >>= 1)
                {
                    if ((mask & 1) == 0) continue;
                    var option = FollowEdge(from, i);
                    if (!frontier.TryGet(grid.StateId(option), out var neighbor)
                        || (neighbor.HeapIndex >= 0 && neighbor.Parent >= 0) || neighbor.Cost >= node.Cost) continue;
                    float score = neighbor.Cost + grid.EdgeCost(from, option, footprint, i)
                        + ((option.Centre(footprint) - preferred).magnitude - preferredDistance) * 2f;
                    if (score >= best) continue;
                    best = score;
                    next = option;
                }
                return true;
            }

            public void Dispose()
            {
                if (frontier == null) return;
                owner.Return(frontier);
                frontier = null;
            }
        }

        private struct Node
        {
            internal int State, Parent, Cost, H, Preference, HeapIndex;
        }

        // Paged state IDs index packed value-type nodes. The heap has one slot
        // per open state, with decrease-key rather than duplicate stale entries.
        private sealed class Frontier
        {
            private const int PageBits = 10;
            private const int PageSize = 1 << PageBits;
            private readonly int[][] lookup = new int[(NavigationGrid.MaxCells * 8) / PageSize][];
            private readonly Pages<Node> nodes = new();
            private readonly Pages<int> heap = new();
            internal Node this[int index] => nodes[index];
            internal int Count => heap.Count;
            internal int PeekIndex => heap[0];
            internal Node Peek => nodes[PeekIndex];
            internal bool TryGet(int state, out Node node)
            {
                int index = IndexOf(state);
                if (index >= 0) { node = nodes[index]; return true; }
                node = default;
                return false;
            }

            private int IndexOf(int state)
            {
                int[] page = lookup[state >> PageBits];
                if (page == null) return -1;
                int index = page[state & (PageSize - 1)] - 1;
                // Returning the workspace clears the live node count. Old page
                // entries are harmless unless they point back to this exact state.
                return (uint)index < (uint)nodes.Count && nodes[index].State == state ? index : -1;
            }

            internal bool CanImprove(int state, int cost) =>
                !TryGet(state, out var node) || (node.HeapIndex >= 0 && cost < node.Cost);

            internal int Relax(int state, int parent, int cost, int h, int preference)
            {
                int index = IndexOf(state);
                if (index >= 0)
                {
                    var old = nodes[index];
                    if (old.HeapIndex < 0 || cost >= old.Cost) return index;
                    old.Parent = parent;
                    old.Cost = cost;
                    nodes[index] = old;
                    BubbleUp(index, old.HeapIndex);
                    return index;
                }

                index = nodes.Count;
                int[] page = lookup[state >> PageBits] ??= new int[PageSize];
                page[state & (PageSize - 1)] = index + 1;
                nodes.Add(new Node { State = state, Parent = parent, Cost = cost, H = h, Preference = preference, HeapIndex = heap.Count });
                heap.Add(index);
                BubbleUp(index, heap.Count - 1);
                return index;
            }

            private bool Before(int a, int b)
            {
                var x = nodes[a];
                var y = nodes[b];
                int cmp = (x.Cost + x.H).CompareTo(y.Cost + y.H);
                if (cmp == 0) cmp = x.Preference.CompareTo(y.Preference);
                if (cmp == 0) cmp = x.H.CompareTo(y.H);
                return cmp == 0 ? x.State < y.State : cmp < 0;
            }

            private void Place(int index, int slot)
            {
                heap[slot] = index;
                var node = nodes[index];
                node.HeapIndex = slot;
                nodes[index] = node;
            }

            private void BubbleUp(int index, int slot)
            {
                while (slot > 0)
                {
                    int parent = (slot - 1) / 2;
                    if (!Before(index, heap[parent])) break;
                    Place(heap[parent], slot);
                    slot = parent;
                }
                Place(index, slot);
            }

            internal int Pop()
            {
                int first = heap[0], last = heap[heap.Count - 1];
                heap.RemoveLast();
                var node = nodes[first];
                node.HeapIndex = -1;
                nodes[first] = node;
                if (heap.Count == 0) return first;
                int slot = 0;
                while (slot * 2 + 1 < heap.Count)
                {
                    int child = slot * 2 + 1;
                    if (child + 1 < heap.Count && Before(heap[child + 1], heap[child])) child++;
                    if (!Before(heap[child], last)) break;
                    Place(heap[child], slot);
                    slot = child;
                }
                Place(last, slot);
                return first;
            }

            internal void Clear() { nodes.Clear(); heap.Clear(); }
        }
        private sealed class Pages<T> where T : struct
        {
            private const int Bits = 10;
            private const int Size = 1 << Bits;
            private readonly List<T[]> pages = new();
            internal int Count { get; private set; }
            internal T this[int index]
            {
                get => pages[index >> Bits][index & (Size - 1)];
                set => pages[index >> Bits][index & (Size - 1)] = value;
            }

            internal void Add(T item)
            {
                if (Count >> Bits == pages.Count) pages.Add(new T[Size]);
                this[Count++] = item;
            }

            internal void RemoveLast() => Count--;
            internal void Clear() => Count = 0;
        }
    }
}
