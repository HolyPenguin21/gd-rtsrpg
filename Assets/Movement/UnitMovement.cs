using System.Collections.Generic;
using Unity.Profiling;
using UnityEngine;

namespace GameDev.RtsRpg
{
    /// <summary>Follows verified grid routes with continuous motion and short, checked detours.</summary>
    public sealed class UnitMovement : MonoBehaviour
    {
        private static readonly ProfilerMarker Marker = new("Navigation.UnitMovement");
        private const int LookAhead = 5;
        public enum Motion { Moving, Yielding, Waiting, RepathPending, Blocked, Arrived, Invalidated }
        internal enum Hold { None, Planning, Geometry, Traffic, Passage }

        private TerrainNavigation navigation;
        private NavigationGrid grid;
        private NavigationPath path, pending;
        private NavigationPose restingPose, edgeEnd, pendingJoin;
        private NavigationPose? yieldTarget, mountedStep;
        private Vector2 centre, previousCentre, edgeStart, velocity;
        private float facing, previousFacing, edgeYaw, edgeAngle, progress, speed, wait, yieldUntil, nextDetour;
        private float cellSize, height, exitSpeed;
        private Quaternion terrainYaw;
        private int next, edgeIndex, pendingIndex, revision, detourSide;
        private bool activeEdge, retreating, provisional, pendingProvisional, localStep, finishingMountedStep;
        private RectInt passage, previousPassage;
        private int passageDirection, previousPassageDirection, passageWidth, passageSince;
        internal Vector2Int Bucket { get; set; }
        public Motion State { get; private set; }
        public NavigationStatus Status { get; private set; }
        internal Hold HoldReason { get; private set; }
        internal NavigationFootprint Footprint { get; private set; }
        internal NavigationPose Pose => restingPose;
        internal NavigationPose PlanningPose => provisional && path != null && path.IsCurrent ? path.Points[path.Points.Count - 1].Pose
            : activeEdge && progress > 0f && !retreating ? edgeEnd : restingPose;
        internal bool AtAnchor => progress == 0f && !yieldTarget.HasValue && !retreating;
        internal bool HasRoute => (path != null && path.IsCurrent) || (pending != null && pending.IsCurrent);
        internal bool CanLaunch => AtAnchor && !HasRoute && Time.time >= yieldUntil;
        internal Rect Occupancy { get; private set; }
        internal Vector2 PredictionVelocity { get; private set; }
        internal bool Planning { get; set; }
        internal int Team { get; private set; }
        internal RectInt Passage => passage;
        internal int PassageDirection => passageDirection;
        internal int PassageWidth => passageWidth;
        internal int PassageSince => passageSince;
        internal int MotionTicks { get; private set; }
        internal int TurnCount { get; private set; }
        internal int StopCount { get; private set; }

        internal bool Reserves(Rect area) => Overlaps(Occupancy, area)
            || (activeEdge && progress > 0f && Overlaps(EdgeSweep(edgeStart, Centre(edgeEnd)), area));

        public bool Begin(TerrainNavigation owner, NavigationPath route, NavigationFootprint size, int faction = 0)
        {
            if (navigation != null || owner == null || owner.Grid == null || route == null
                || !route.IsCurrent || !route.BelongsTo(owner.Grid, size) || route.Points.Count == 0
                || !owner.Grid.CanOccupy(route.Points[0].Pose, size)) return false;
            navigation = owner;
            grid = owner.Grid;
            Footprint = size;
            Team = faction;
            cellSize = grid.LocalToWorld.MultiplyVector(Vector3.right).magnitude * grid.Surface.BlockWorldSize / 4;
            height = cellSize * 0.75f;
            terrainYaw = Quaternion.LookRotation(grid.LocalToWorld.MultiplyVector(Vector3.forward));
            restingPose = route.Points[0].Pose;
            previousCentre = centre = Centre(restingPose);
            previousFacing = facing = restingPose.Yaw;
            revision = grid.Revision;
            Occupancy = BoundsAt(centre, facing);
            if (Blocker(Occupancy) != null || !Assign(route)) { navigation = null; return false; }
            if (route.Points.Count == 1 && route.Status == NavigationStatus.Success)
            {
                path = null;
                State = Motion.Arrived;
            }
            navigation.Track(this, centre);
            transform.localScale = new Vector3(size.Width * cellSize, height, size.Length * cellSize);
            Render(centre, facing);
            return true;
        }

        internal bool Assign(NavigationPath route, bool preview = false)
        {
            if (!route.IsCurrent || !route.BelongsTo(grid, Footprint) || route.Points.Count == 0) return false;
            var join = activeEdge && progress > 0f ? (retreating ? restingPose : edgeEnd) : restingPose;
            if (TryJoin(join, route, out int index))
            {
                if (AtAnchor && !activeEdge) Install(route, index, preview);
                else if (AtAnchor && !yieldTarget.HasValue)
                {
                    activeEdge = false;
                    Install(route, index, preview);
                }
                else
                {
                    pending = route;
                    pendingJoin = join;
                    pendingIndex = index;
                    pendingProvisional = preview;
                }
                return true;
            }
            // The short launch may still be running while its continuation is planned.
            if (provisional && path != null && !retreating && path.Points[path.Points.Count - 1].Pose.Equals(route.Points[0].Pose))
            {
                pending = route;
                pendingJoin = route.Points[0].Pose;
                pendingIndex = 1;
                pendingProvisional = preview;
                return true;
            }
            return false;
        }

        private bool TryJoin(NavigationPose from, NavigationPath route, out int index)
        {
            int end = Mathf.Min(route.Points.Count, LookAhead + 1);
            for (int i = 0; i < end; i++)
                if (from.Equals(route.Points[i].Pose)) { index = i + 1; return true; }
            for (int i = end - 1; i >= 0; i--)
                if (CanShortcut(from, route.Points[i].Pose, 2f)) { index = i; return true; }
            index = 0;
            return false;
        }

        private void Install(NavigationPath route, int index, bool preview)
        {
            revision = grid.Revision;
            path = route;
            pending = null;
            mountedStep = null;
            next = index;
            wait = 0f;
            provisional = preview;
            Status = route.Status;
            State = Motion.Moving;
        }

        internal void CancelPending() => pending = null;

        internal void ReportFailure(NavigationStatus status)
        {
            if (HasRoute) return;
            provisional = false;
            Status = status;
            State = Motion.Blocked;
        }

        private void OnDisable()
        {
            if (navigation != null) navigation.Untrack(this);
            navigation = null;
            grid = null;
            path = pending = null;
            yieldTarget = mountedStep = null;
            activeEdge = retreating = Planning = provisional = false;
            progress = speed = wait = yieldUntil = 0f;
            velocity = PredictionVelocity = Vector2.zero;
            passageDirection = previousPassageDirection = 0;
            State = Motion.Invalidated;
        }

        internal void CaptureTick()
        {
            previousCentre = centre;
            previousFacing = facing;
            PredictionVelocity = velocity;
            previousPassage = passage;
            previousPassageDirection = passageDirection;
            passageDirection = 0;
        }

        internal void PrepareTick(int tick)
        {
            if (!CurrentGrid) return;
            if (grid.Revision != revision)
            {
                pending = null;
                if (activeEdge && progress > 0f && !EdgeStillClear())
                {
                    Status = NavigationStatus.Stale;
                    State = Motion.Invalidated;
                    return;
                }
                revision = grid.Revision;
                if (!activeEdge || progress == 0f) StopAtAnchor();
            }
            if (!activeEdge && pending != null && pending.IsCurrent)
            {
                if (pendingJoin.Equals(restingPose)) Install(pending, pendingIndex, pendingProvisional);
                else if (path == null || next >= path.Points.Count)
                {
                    if (TryJoin(restingPose, pending, out int index)) Install(pending, index, pendingProvisional);
                    else pending = null;
                }
            }
            if (!activeEdge && yieldTarget.HasValue)
                SetEdge(yieldTarget.Value, next, true);
            if (!activeEdge && mountedStep.HasValue)
            {
                var step = mountedStep.Value;
                mountedStep = null;
                if (grid.CanStep(restingPose, step, Footprint))
                {
                    SetEdge(step, next, true);
                    finishingMountedStep = true;
                }
                else StopAtAnchor();
            }
            if (!activeEdge && path != null && next < path.Points.Count)
                SelectEdge();
            else if (activeEdge && progress == 0f && HoldReason == Hold.Traffic && path != null && Time.fixedTime >= nextDetour)
            {
                nextDetour = Time.fixedTime + 0.10f;
                TryDetour(edgeEnd);
            }
            if (!activeEdge || retreating || !grid.TryPassage(restingPose, edgeEnd, Footprint,
                out passage, out passageDirection, out passageWidth))
            {
                passageDirection = 0;
                return;
            }
            if (previousPassageDirection != passageDirection || !previousPassage.Equals(passage)) passageSince = tick;
        }

        internal void MoveTick(float dt)
        {
            using var marker = Marker.Auto();
            velocity = Vector2.zero;
            if (!CurrentGrid || revision != grid.Revision)
            {
                speed = 0f;
                HoldReason = Hold.Geometry;
                State = Motion.Invalidated;
                return;
            }
            if (!activeEdge)
            {
                speed = 0f;
                if (Planning || provisional) HoldReason = Hold.Planning;
                else if (!Recovering) HoldReason = Hold.None;
                if (path != null)
                {
                    if (provisional) State = Motion.RepathPending;
                    else State = path.Status == NavigationStatus.Partial ? Motion.Blocked : Motion.Arrived;
                    path = null;
                }
                return;
            }

            Vector2 end = Centre(edgeEnd);
            float distance = (end - edgeStart).magnitude;
            float length = Mathf.Max(distance, Mathf.Abs(edgeAngle) / 55f);
            length = Mathf.Max(length, 0.01f);
            float remaining = length * (retreating ? progress : 1f - progress);
            float maxSpeed = 4f / cellSize;
            float acceleration = 12f / cellSize;
            float endSpeed = retreating ? 0f : exitSpeed;
            float targetSpeed = Mathf.Min(maxSpeed, Mathf.Sqrt(endSpeed * endSpeed + 2f * acceleration * remaining));
            speed = Mathf.MoveTowards(speed, targetSpeed, acceleration * dt);
            float increment = Mathf.Min(0.25f, Mathf.Max(speed * dt, Mathf.Min(remaining, 0.0001f)));
            float proposedProgress = Mathf.MoveTowards(progress, retreating ? 0f : 1f, increment / length);
            Vector2 proposed = Vector2.Lerp(edgeStart, end, proposedProgress);
            float proposedYaw = edgeYaw + edgeAngle * proposedProgress;
            Rect sweep = edgeAngle == 0f ? Union(BoundsAt(centre, facing), BoundsAt(proposed, proposedYaw)) : EdgeSweep(edgeStart, end);
            if (!retreating && passageDirection != 0 && !navigation.CanEnterPassage(this))
            {
                Wait(Hold.Passage, dt);
                return;
            }
            var obstruction = Blocker(sweep);
            if (obstruction != null)
            {
                if (obstruction.Team == Team && wait >= 0.15f && !retreating)
                    obstruction.TryYield(end - centre, this);
                Wait(Hold.Traffic, dt);
                if (wait >= 0.85f && !retreating)
                {
                    if (progress == 0f) StopAtAnchor();
                    else retreating = true;
                }
                else if (retreating && wait >= 1.7f && Blocker(EdgeSweep(centre, end)) == null)
                    retreating = false;
                return;
            }

            if (passageDirection != 0 && !retreating) navigation.EnterPassage(this);
            if (progress == 0f && edgeAngle != 0f) TurnCount++;
            velocity = (proposed - centre) / dt;
            if (velocity.sqrMagnitude > 0.000001f) MotionTicks++;
            centre = proposed;
            facing = proposedYaw;
            progress = proposedProgress;
            wait = 0f;
            HoldReason = Hold.None;
            State = retreating ? Motion.RepathPending : localStep ? Motion.Yielding : Motion.Moving;
            Rect occupancy = edgeAngle != 0f && progress < 1f && !(retreating && progress <= 0f)
                ? TurnSweep(edgeStart, end) : BoundsAt(centre, facing);
            if (Occupancy != occupancy) navigation.NotifyTrafficChanged();
            Occupancy = occupancy;
            navigation.Register(this, centre);
            if (retreating && progress <= 0f) StopAtAnchor();
            else if (progress >= 1f) FinishEdge();
        }

        private void Wait(Hold reason, float dt)
        {
            if (HoldReason == Hold.None && MotionTicks > 0) StopCount++;
            speed = 0f;
            wait += dt;
            HoldReason = reason;
            State = Motion.Waiting;
        }

        private void SelectEdge()
        {
            var destination = path.Points[next].Pose;
            int index = next;
            if (destination.Facing != restingPose.Facing)
            {
                // Join successive 45-degree graph turns into one continuous pivot
                // when the whole swing fits, including the change of centre.
                float firstTurn = Mathf.DeltaAngle(restingPose.Yaw, destination.Yaw);
                for (int i = next + 1; i < path.Points.Count && i <= next + 2; i++)
                {
                    var candidate = path.Points[i].Pose;
                    float delta = Mathf.DeltaAngle(restingPose.Yaw, candidate.Yaw);
                    if (candidate.Cell != restingPose.Cell || delta * firstTurn <= 0f
                        || Mathf.Abs(delta) <= Mathf.Abs(Mathf.DeltaAngle(restingPose.Yaw, destination.Yaw))
                        || !grid.CanSweep(centre, Centre(candidate), TurnHalfExtent)) break;
                    destination = candidate;
                    index = i;
                }
            }
            else if (!grid.NearBoundary(restingPose, Footprint))
            {
                for (int i = next + 1; i < path.Points.Count && i <= next + LookAhead; i++)
                {
                    var candidate = path.Points[i].Pose;
                    if (grid.NearBoundary(candidate, Footprint) || !CanShortcut(restingPose, candidate, 3f)) break;
                    destination = candidate;
                    index = i;
                }
            }

            bool turning = destination.Facing != restingPose.Facing;
            Rect immediate = turning ? TurnSweep(centre, Centre(destination))
                : TranslationSweep(centre, Vector2.MoveTowards(centre, Centre(destination), 1f), facing);
            bool obstructed = navigation.FindContact(this, immediate, horizon: 0.24f) != null;
            if (obstructed && TryDetour(destination)) return;
            if (!turning && !grid.CanSweep(centre, Centre(destination), HalfExtent(facing)))
            {
                HoldReason = Hold.Geometry;
                StopAtAnchor();
                return;
            }
            SetEdge(destination, index, false);
        }

        private bool TryDetour(NavigationPose destination)
        {
            if (yieldTarget.HasValue || path == null) return false;
            // Leave actual single lanes to passage arbitration. A detour is
            // useful only if there is a verified way back to the route ahead.
            if (grid.TryPassage(restingPose, destination, Footprint, out var lane, out int axis, out int bodyWidth)
                && (Mathf.Abs(axis) == 1 ? lane.height : lane.width) < bodyWidth * 2) return false;
            if (Footprint.NeedsTurns) return TryMountedDetour();
            var desired = Centre(destination) - centre;
            if (desired.sqrMagnitude < 0.01f) return false;
            float best = float.NegativeInfinity;
            NavigationPose chosen = default;
            int rejoin = next;
            int side = 0;
            int mask = grid.Edges(restingPose, Footprint);
            for (int i = 0; i < 8; i++)
            {
                if ((mask & (1 << i)) == 0) continue;
                var delta = NavigationPose.Steps[i];
                var option = new NavigationPose(restingPose.Cell + delta, restingPose.Facing);
                if (option.Equals(destination) || Vector2.Dot(desired, delta) < 0f
                    || grid.TryPassage(restingPose, option, Footprint, out _, out _, out _)
                    || navigation.FindContact(this, TranslationSweep(centre, Centre(option), facing), horizon: 0.24f) != null) continue;
                for (int j = Mathf.Min(path.Points.Count - 1, next + LookAhead); j >= next; j--)
                {
                    var join = path.Points[j].Pose;
                    if (!option.Equals(join) && !CanShortcut(option, join, 4f)) continue;
                    if (!option.Equals(join) && Blocker(BoundsAt(Centre(join), join.Yaw)) != null) continue;
                    int optionSide = desired.x * delta.y - desired.y * delta.x >= 0f ? 1 : -1;
                    float score = j + Vector2.Dot(desired.normalized, ((Vector2)delta).normalized)
                        + (optionSide == detourSide ? 1.5f : 0f) - (grid.NearBoundary(option, Footprint) ? 1f : 0f);
                    if (score > best)
                    {
                        best = score;
                        chosen = option;
                        rejoin = option.Equals(join) ? j + 1 : j;
                        side = optionSide;
                    }
                    break;
                }
            }
            if (side == 0) return false;
            detourSide = side;
            SetEdge(chosen, rejoin, true);
            return true;
        }

        private bool TryMountedDetour()
        {
            if (mountedStep.HasValue || finishingMountedStep || grid.NearBoundary(restingPose, Footprint)
                || Time.fixedTime < yieldUntil) return false;
            int first = detourSide == 0 ? 1 : detourSide;
            for (int i = 0; i < 2; i++)
            {
                int side = i == 0 ? first : -first;
                var turn = restingPose.Turn(side);
                if (!grid.CanTurn(restingPose, turn, Footprint)) continue;
                var step = new NavigationPose(turn.Cell + NavigationPose.Steps[(int)turn.Facing], turn.Facing);
                if (!grid.CanStep(turn, step, Footprint) || grid.NearBoundary(step, Footprint)
                    || navigation.FindContact(this, TurnSweep(centre, Centre(turn)), horizon: 0.24f) != null
                    || navigation.FindContact(this, TranslationSweep(Centre(turn), Centre(step), turn.Yaw), horizon: 0.24f) != null)
                    continue;
                detourSide = side;
                mountedStep = step;
                SetEdge(turn, next, true);
                return true;
            }
            return false;
        }

        private void SetEdge(NavigationPose to, int index, bool local)
        {
            edgeEnd = to;
            edgeStart = centre;
            edgeYaw = facing;
            edgeAngle = Mathf.DeltaAngle(facing, to.Yaw);
            edgeIndex = index;
            localStep = local;
            finishingMountedStep = false;
            exitSpeed = 0f;
            if (!local && path != null && index + 1 < path.Points.Count && edgeAngle == 0f)
            {
                var following = path.Points[index + 1].Pose;
                if (following.Facing == to.Facing)
                {
                    var direction = (Centre(to) - centre).normalized;
                    var nextDirection = (Centre(following) - Centre(to)).normalized;
                    exitSpeed = (4f / cellSize) * Mathf.Clamp01(Vector2.Dot(direction, nextDirection));
                }
            }
            activeEdge = true;
        }

        private void FinishEdge()
        {
            restingPose = edgeEnd;
            activeEdge = false;
            progress = 0f;
            if (finishingMountedStep)
            {
                StopAtAnchor();
                yieldUntil = Time.time + 0.18f;
                return;
            }
            if (yieldTarget.HasValue)
            {
                StopAtAnchor();
                yieldUntil = Time.time + 0.45f;
                return;
            }
            next = localStep ? edgeIndex : edgeIndex + 1;
            if (!localStep) detourSide = 0;
            if (path != null && !path.IsCurrent) StopAtAnchor();
        }

        private void StopAtAnchor()
        {
            activeEdge = retreating = finishingMountedStep = false;
            progress = speed = wait = 0f;
            path = null;
            yieldTarget = mountedStep = null;
            State = Motion.RepathPending;
        }

        internal bool TryLaunch(Vector2 target)
        {
            if (!CanLaunch) return false;
            var from = restingPose;
            var points = new List<NavigationPoint>(6) { grid.Point(from, Footprint) };
            float distance = (Centre(from) - target).sqrMagnitude;
            bool advanced = false;
            for (int step = 0; step < 5; step++)
            {
                var desired = target - Centre(from);
                if (desired.sqrMagnitude < 1f) break;
                int mask = grid.Edges(from, Footprint);
                float angle = Mathf.Atan2(desired.x, desired.y) * Mathf.Rad2Deg;
                float best = float.PositiveInfinity;
                NavigationPose chosen = default;
                for (int i = 0; mask != 0; i++, mask >>= 1)
                {
                    if ((mask & 1) == 0) continue;
                    bool turn = i >= 8;
                    var option = turn ? from.Turn(i == 8 ? -1 : 1)
                        : new NavigationPose(from.Cell + NavigationPose.Steps[i], from.Facing);
                    bool visited = false;
                    for (int j = 0; j < points.Count; j++) visited |= points[j].Pose.Equals(option);
                    if (visited) continue;
                    if (!turn && grid.TryPassage(from, option, Footprint, out _, out _, out _)) continue;
                    if (!turn && Vector2.Dot(desired, Centre(option) - Centre(from)) < 0f) continue;
                    if (turn && Mathf.Abs(Mathf.DeltaAngle(option.Yaw, angle)) >= Mathf.Abs(Mathf.DeltaAngle(from.Yaw, angle))) continue;
                    Rect sweep = turn ? TurnSweep(Centre(from), Centre(option)) : TranslationSweep(Centre(from), Centre(option), from.Yaw);
                    float score = (Centre(option) - target).magnitude + (turn ? 0.3f : 0f)
                        + (grid.NearBoundary(option, Footprint) ? 0.6f : 0f)
                        + (navigation.FindContact(this, sweep, horizon: 0.24f) == null ? 0f : 3f);
                    if (score >= best) continue;
                    best = score;
                    chosen = option;
                }
                if (float.IsPositiveInfinity(best)) break;
                advanced |= chosen.Cell != from.Cell;
                from = chosen;
                points.Add(grid.Point(from, Footprint));
            }
            return advanced && (Centre(from) - target).sqrMagnitude < distance
                && Assign(new NavigationPath(grid, NavigationStatus.Partial, "Safe initial segment.", points, footprint: Footprint), true);
        }

        internal bool CanPlan => !retreating && Time.time >= yieldUntil;
        internal bool Recovering => State == Motion.RepathPending || State == Motion.Blocked || State == Motion.Waiting;

        private void TryYield(Vector2 approach, UnitMovement requester)
        {
            if (!AtAnchor || grid.Revision != revision || Time.time < yieldUntil || (HasRoute && !Recovering)) return;
            Vector2Int forward = Mathf.Abs(approach.x) >= Mathf.Abs(approach.y)
                ? Vector2Int.right * (approach.x >= 0 ? 1 : -1)
                : Vector2Int.up * (approach.y >= 0 ? 1 : -1);
            var side = new Vector2Int(-forward.y, forward.x);
            NavigationPose? queued = null;
            for (int i = 0; i < 3; i++)
            {
                var delta = i == 0 ? side : i == 1 ? -side : forward;
                var target = new NavigationPose(restingPose.Cell + delta, restingPose.Facing);
                if (!grid.CanYieldStep(restingPose, target, Footprint)
                    || Vector2.Dot(Centre(target) - requester.centre, (Vector2)delta) <= 0f) continue;
                var obstruction = Blocker(TranslationSweep(centre, Centre(target), facing));
                if (obstruction == null)
                {
                    activeEdge = false;
                    mountedStep = null;
                    yieldTarget = target;
                    wait = 0f;
                    return;
                }
                if (obstruction != requester && obstruction.Team == Team && obstruction.AtAnchor
                    && Vector2.Dot(obstruction.centre - centre, (Vector2)delta) > 0f)
                    queued ??= target;
            }
            if (queued.HasValue) { activeEdge = false; mountedStep = null; yieldTarget = queued; wait = 0f; }
        }

        internal bool AllowsSearchEdge(NavigationPose from, NavigationPose to, bool avoidAllies)
        {
            Rect sweep = from.Facing != to.Facing ? TurnSweep(Centre(from), Centre(to))
                : TranslationSweep(Centre(from), Centre(to), from.Yaw);
            return navigation.FindContact(this, sweep, search: true, avoidAllies: avoidAllies) == null;
        }

        internal bool SharesGrid(UnitMovement other) => grid == other.grid;
        internal bool Avoids(UnitMovement other, bool avoidAllies) =>
            other.Team != Team || (avoidAllies && (other.Recovering || !other.HasRoute));
        private bool CurrentGrid => navigation != null && grid == navigation.Grid && grid.IsAlive;
        private Vector2 Centre(NavigationPose pose) => pose.Centre(Footprint);
        private Vector2 TurnHalfExtent => Vector2.one * 1.118034f;
        private bool EdgeStillClear() => grid.CanSweep(edgeStart, Centre(edgeEnd), edgeAngle == 0f ? HalfExtent(edgeYaw) : TurnHalfExtent);
        private UnitMovement Blocker(Rect sweep) => navigation.FindContact(this, sweep);

        private bool CanShortcut(NavigationPose from, NavigationPose to, float maxDistance)
        {
            if (from.Facing != to.Facing) return false;
            var delta = to.Cell - from.Cell;
            if (delta == Vector2Int.zero || ((Vector2)delta).sqrMagnitude > maxDistance * maxDistance) return false;
            if (Footprint.NeedsTurns)
            {
                var step = NavigationPose.Steps[(int)from.Facing];
                if (step.x * delta.y != step.y * delta.x || step.x * delta.x + step.y * delta.y <= 0) return false;
            }
            var a = Centre(from);
            var b = Centre(to);
            if (delta.sqrMagnitude > 2)
            {
                int steps = Mathf.CeilToInt(((Vector2)delta).magnitude * 2f);
                for (int i = 0; i <= steps; i++)
                {
                    Vector2 point = Vector2.Lerp(a, b, (float)i / steps);
                    var anchor = point - (Vector2)Footprint.Size(from.Facing) * 0.5f;
                    var pose = new NavigationPose(new Vector2Int(Mathf.RoundToInt(anchor.x), Mathf.RoundToInt(anchor.y)), from.Facing);
                    if (grid.NearBoundary(pose, Footprint)) return false;
                }
            }
            return grid.CanSweep(a, b, HalfExtent(from.Yaw));
        }

        internal static bool Overlaps(Rect a, Rect b)
        {
            const float epsilon = 0.0001f;
            return a.xMin < b.xMax - epsilon && a.xMax > b.xMin + epsilon
                && a.yMin < b.yMax - epsilon && a.yMax > b.yMin + epsilon;
        }

        private static Rect Union(Rect a, Rect b) => Rect.MinMaxRect(Mathf.Min(a.xMin, b.xMin), Mathf.Min(a.yMin, b.yMin),
            Mathf.Max(a.xMax, b.xMax), Mathf.Max(a.yMax, b.yMax));

        private Vector2 HalfExtent(float yaw)
        {
            if (!Footprint.NeedsTurns) return new Vector2(Footprint.Width, Footprint.Length) * 0.5f;
            float radians = yaw * Mathf.Deg2Rad;
            float sin = Mathf.Abs(Mathf.Sin(radians)), cos = Mathf.Abs(Mathf.Cos(radians));
            return new Vector2(cos + 2f * sin, sin + 2f * cos) * 0.5f;
        }

        private Rect BoundsAt(Vector2 at, float yaw)
        {
            var half = HalfExtent(yaw);
            return new Rect(at - half, half * 2f);
        }

        private Rect TurnSweep(Vector2 from, Vector2 to)
        {
            var half = TurnHalfExtent;
            return Union(new Rect(from - half, half * 2f), new Rect(to - half, half * 2f));
        }

        private Rect TranslationSweep(Vector2 from, Vector2 to, float yaw) => Union(BoundsAt(from, yaw), BoundsAt(to, yaw));
        private Rect EdgeSweep(Vector2 from, Vector2 to) => edgeAngle == 0f ? TranslationSweep(from, to, edgeYaw) : TurnSweep(from, to);

        private void LateUpdate()
        {
            if (!CurrentGrid) return;
            float t = Mathf.Clamp01((Time.time - Time.fixedTime) / Time.fixedDeltaTime);
            Render(Vector2.Lerp(previousCentre, centre, t), Mathf.LerpAngle(previousFacing, facing, t));
        }

        private void Render(Vector2 at, float yaw)
        {
            Vector3 surface = grid.WorldPoint(at.x, at.y);
            transform.SetPositionAndRotation(surface + Vector3.up * height * 0.5f,
                terrainYaw * Quaternion.Euler(0f, yaw, 0f));
        }
    }
}
