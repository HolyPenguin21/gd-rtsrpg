using System;
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
        private bool activeEdge, retreating, provisional, pendingProvisional, localStep, finishingMountedStep, endProvisionalAfterEdge;
        private bool revisionHold, revisionRecovery;
        private NavigationGrid.Passage passage, previousPassage;
        private Vector2Int passageMouth;
        private Vector2 steeringPoint;
        private readonly List<Vector2> edgeTrail = new(32);
        private float travelHeading, steeringHeading, formationLane, formationWidth = 1f;
        private bool hasTravelHeading, steeringReady, passagePending;
        private Hold steeringHold;
        private NavigationPose failedCorner, failedDirection;
        private bool leaveCornerAfterEdge;
        private int cornerRevision, cornerFailures;
        private static readonly float[] SteeringAngles = { 0f, 18f, -18f, 36f, -36f, 62f, -62f };
        private static readonly float[] SteeringSpeeds = { 1f, 0.65f, 0.32f };
        private int passageDirection, previousPassageDirection, passageWidth, passageSince;
        internal Vector2Int Bucket { get; set; }
        public Motion State { get; private set; }
        public NavigationStatus Status { get; private set; }
        internal Hold HoldReason { get; private set; }
        internal NavigationFootprint Footprint { get; private set; }
        internal NavigationPose Pose => restingPose;
        internal NavigationPose PlanningPose => provisional && path != null && path.IsCurrent ? path.Points[path.Points.Count - 1].Pose
            : activeEdge && progress > 0f && !retreating ? edgeEnd : restingPose;
        internal bool IsRevisionHeld => revisionHold;
        internal bool HasPlanningAnchor => CurrentGrid && revision == grid.Revision && !revisionHold;
        internal bool IsRecoveringRevision => CurrentGrid && (revisionHold || revisionRecovery || revision != grid.Revision);
        internal bool AtAnchor => !revisionHold && progress == 0f && !yieldTarget.HasValue && !retreating;
        internal bool HasRoute => (path != null && path.IsCurrent) || (pending != null && pending.IsCurrent);
        internal bool RequiresCornerRecovery => cornerFailures > 0 && cornerRevision == grid.Revision
            && (restingPose.Cell - failedCorner.Cell).sqrMagnitude <= 4;
        internal bool CanLaunch => CanPlan && AtAnchor && !HasRoute && Time.time >= yieldUntil;
        internal Rect Occupancy { get; private set; }
        internal Vector2 PredictionVelocity { get; private set; }
        internal bool Planning { get; set; }
        internal int Team { get; private set; }
        internal NavigationGrid.Passage Passage => passage;
        internal Vector2Int PassageMouth => passageMouth;
        internal int PassageDirection => passageDirection;
        internal int PassageWidth => passageWidth;
        internal int PassageSince => passageSince;
        internal int MotionTicks { get; private set; }
        internal int TurnCount { get; private set; }
        internal int StopCount { get; private set; }

        internal bool ReservesPassage(NavigationGrid.Passage zone) => !revisionHold && (zone.Overlaps(Occupancy)
            || (activeEdge && progress > 0f && zone.Overlaps(BoundsAt(edgeStart, edgeYaw))));

        internal bool Reserves(Rect area) => Overlaps(Occupancy, area)
            || (revisionRecovery && edgeAngle != 0f && Overlaps(TurnSlice(progress, retreating ? 0f : 1f), area))
            || (activeEdge && progress > 0f && (!revisionRecovery || edgeAngle == 0f) && (Overlaps(EdgeSweep(edgeStart, Centre(edgeEnd)), area)
                || Overlaps(EdgeSweep(centre, Centre(edgeEnd)), area)));

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
            if (!CurrentGrid || revisionHold || revision != grid.Revision
                || !route.IsCurrent || !route.BelongsTo(grid, Footprint) || route.Points.Count == 0) return false;
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
            endProvisionalAfterEdge = false;
            Status = route.Status;
            State = Motion.Moving;
        }

        internal bool GetReusableRoute(NavigationPose from, out NavigationPath route, out int index)
        {
            route = path;
            index = 0;
            if (route == null || !route.IsCurrent) return false;
            for (int i = Mathf.Max(0, next - 1); i < route.Points.Count && i <= next + LookAhead; i++)
            {
                if (!route.Points[i].Pose.Equals(from)) continue;
                index = i;
                return true;
            }
            return false;
        }

        internal void EndProvisionalRoute()
        {
            if (!provisional) return;
            if (activeEdge && progress > 0f) endProvisionalAfterEdge = true;
            else StopAtAnchor();
        }

        internal void CancelPending() => pending = null;

        internal void ReportFailure(NavigationStatus status)
        {
            if (HasRoute || revisionHold || revisionRecovery) return;
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
            activeEdge = retreating = Planning = provisional = revisionHold = revisionRecovery = false;
            progress = speed = wait = yieldUntil = 0f;
            velocity = PredictionVelocity = Vector2.zero;
            passageDirection = previousPassageDirection = 0;
            passage = previousPassage = null;
            hasTravelHeading = false;
            formationWidth = 1f;
            cornerFailures = 0;
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
            passage = null;
            passagePending = false;
        }

        internal void ConfigureFormation(float lane) => formationLane = Mathf.Clamp(lane, -1f, 1f);

        internal void PrepareTick(int tick)
        {
            if (!CurrentGrid) return;
            if (grid.Revision != revision) RecoverRevision();
            if (revisionHold) return;
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
            if (!activeEdge || retreating || !grid.PassageAt(restingPose, edgeEnd, Footprint,
                out passageMouth, out passageDirection, out passageWidth)) return;
            passage = grid.RequestPassage(passageMouth);
            passagePending = passage == null || !passage.Ready;
            if (previousPassageDirection != passageDirection || !ReferenceEquals(previousPassage, passage)) passageSince = tick;
        }

        private void RecoverRevision()
        {
            // Consume the revision even if geometry leaves neither endpoint
            // reachable. A later edit can retry from the same physical pose.
            revision = grid.Revision;
            path = pending = null;
            yieldTarget = mountedStep = null;
            provisional = false;
            speed = wait = 0f;
            hasTravelHeading = false;
            Status = NavigationStatus.Stale;
            if (progress <= 0f && grid.CanOccupy(restingPose, Footprint))
            {
                StopAtAnchor();
                return;
            }

            bool canReturn = progress > 0f && grid.CanOccupy(restingPose, Footprint)
                && (edgeAngle == 0f ? CanReturnAlongTrail() : CanSweepTurn(progress, 0f));
            bool canFinish = progress > 0f && grid.CanOccupy(edgeEnd, Footprint)
                && (edgeAngle == 0f ? grid.CanSweep(centre, Centre(edgeEnd), HalfExtent(facing)) : CanSweepTurn(progress, 1f));
            activeEdge = revisionRecovery = canReturn || canFinish;
            revisionHold = !revisionRecovery;
            retreating = canReturn && (retreating || !canFinish);
            if (retreating && edgeAngle == 0f && edgeTrail.Count > 1
                && (centre - edgeTrail[edgeTrail.Count - 1]).sqrMagnitude < 0.00000001f)
                edgeTrail.RemoveAt(edgeTrail.Count - 1);
            State = revisionHold ? Motion.Invalidated : Motion.RepathPending;
            HoldReason = revisionHold ? Hold.Geometry : Hold.None;
            UpdateOccupancy(BoundsAt(centre, facing));
        }

        private bool CanReturnAlongTrail()
        {
            var current = centre;
            var half = HalfExtent(facing);
            for (int i = edgeTrail.Count - 1; i >= 0; i--)
            {
                var previous = edgeTrail[i];
                if (!grid.CanSweep(current, previous, half)) return false;
                current = previous;
            }
            return edgeTrail.Count != 0;
        }

        private bool CanSweepTurn(float from, float to)
        {
            int pieces = Mathf.Max(1, Mathf.CeilToInt(Mathf.Abs(edgeAngle * (to - from)) / 5f));
            float previous = from;
            for (int i = 1; i <= pieces; i++)
            {
                float nextFraction = Mathf.Lerp(from, to, (float)i / pieces);
                var a = Vector2.Lerp(edgeStart, Centre(edgeEnd), previous);
                var b = Vector2.Lerp(edgeStart, Centre(edgeEnd), nextFraction);
                if (!grid.CanSweep(a, b, TurnSliceExtent(previous, nextFraction))) return false;
                previous = nextFraction;
            }
            return true;
        }

        private Vector2 TurnSliceExtent(float from, float to)
        {
            float first = edgeYaw + edgeAngle * from, last = edgeYaw + edgeAngle * to;
            var a = HalfExtent(first);
            var b = HalfExtent(last);
            float middle = (first + last) * 0.5f, span = Mathf.Abs(last - first) * 0.5f;
            // Extent maxima for a rotating 1x2 footprint. Endpoints alone
            // miss the widest part of an arc that straddles one of these angles.
            return new Vector2(ContainsPeak(63.43495f) ? TurnHalfExtent.x : Mathf.Max(a.x, b.x),
                ContainsPeak(26.56505f) ? TurnHalfExtent.y : Mathf.Max(a.y, b.y));

            bool ContainsPeak(float peak) =>
                Mathf.Abs(Mathf.DeltaAngle(middle, peak)) <= span || Mathf.Abs(Mathf.DeltaAngle(middle, -peak)) <= span
                || Mathf.Abs(Mathf.DeltaAngle(middle, 180f + peak)) <= span
                || Mathf.Abs(Mathf.DeltaAngle(middle, 180f - peak)) <= span;
        }

        private Rect TurnSlice(float from, float to)
        {
            var half = TurnSliceExtent(from, to);
            var a = Vector2.Lerp(edgeStart, Centre(edgeEnd), from);
            var b = Vector2.Lerp(edgeStart, Centre(edgeEnd), to);
            return Union(new Rect(a - half, half * 2f), new Rect(b - half, half * 2f));
        }

        internal void PrepareSteering(float dt)
        {
            steeringReady = false;
            steeringHold = Hold.None;
            if (!activeEdge || edgeAngle != 0f || !CurrentGrid || revision != grid.Revision || passagePending) return;
            Vector2 end = retreating && edgeTrail.Count != 0 ? edgeTrail[edgeTrail.Count - 1] : Centre(edgeEnd);
            Vector2 toEnd = end - centre;
            float remaining = toEnd.magnitude;
            if (remaining <= 0.0001f)
            {
                steeringPoint = end;
                steeringHeading = travelHeading;
                steeringReady = true;
                return;
            }

            Vector2 direct = toEnd / remaining;
            Vector2 half = HalfExtent(facing);
            float maxSpeed = 4f / cellSize;
            float acceleration = 12f / cellSize;
            float finalSpeed = retreating ? 0f : exitSpeed;
            float limit = Mathf.Min(maxSpeed, Mathf.Sqrt(finalSpeed * finalSpeed + 2f * acceleration * remaining));
            float targetSpeed = Mathf.Min(0.25f / dt, Mathf.MoveTowards(speed, limit, acceleration * dt));
            const float horizon = 0.28f;
            navigation.CollectSteeringNeighbors(this, centre, 4f + Mathf.Min(4f, maxSpeed * horizon));

            bool steerLaterally = !Footprint.NeedsTurns && passage == null && !localStep && !retreating && !revisionRecovery
                && !grid.PassageAt(restingPose, restingPose, Footprint, out _, out _, out _)
                && !grid.IsRampPose(restingPose, Footprint) && !grid.IsRampPose(edgeEnd, Footprint);
            Vector2 desired = direct;
            if (steerLaterally && formationLane != 0f)
            {
                float clear = Mathf.Min(grid.FormationClearance(restingPose, Footprint, direct),
                    grid.FormationClearance(edgeEnd, Footprint, direct));
                if (path != null && edgeIndex + 1 < path.Points.Count)
                    clear = Mathf.Min(clear, grid.FormationClearance(path.Points[edgeIndex + 1].Pose, Footprint, direct));
                formationWidth = Mathf.MoveTowards(formationWidth, clear, dt * 2f);
                var across = new Vector2(-direct.y, direct.x);
                Vector2 onEdge = Vector2.Lerp(edgeStart, end, progress);
                float lateralError = formationLane * formationWidth * 0.55f - Vector2.Dot(centre - onEdge, across);
                desired = (direct + across * (lateralError * Mathf.Clamp01((remaining - 0.45f) / 1.2f))).normalized;
            }
            else formationWidth = Mathf.MoveTowards(formationWidth, 0f, dt * 4f);

            float desiredAngle = Heading(desired);
            float initialHeading = hasTravelHeading && !Footprint.NeedsTurns && !revisionRecovery ? travelHeading : Heading(direct);
            float best = float.NegativeInfinity;
            bool geometricallyClear = false;
            steeringHeading = Mathf.MoveTowardsAngle(initialHeading, desiredAngle, 420f * dt);
            Rect body = BoundsAt(centre, facing);
            // Close to the checked anchor, take the exact remaining step so
            // handoff, occupancy and path completion all share the same pose.
            if (remaining <= targetSpeed * dt && grid.CanSweep(centre, end, half)
                && (!hasTravelHeading || Footprint.NeedsTurns || Mathf.Abs(Mathf.DeltaAngle(initialHeading, Heading(direct))) <= 420f * dt))
            {
                Vector2 snapVelocity = toEnd / dt;
                if (navigation.TimeToContact(body, snapVelocity, dt) >= dt
                    && (retreating || revisionRecovery || grid.CanSteerAcrossPassage(TranslationSweep(centre, end, facing), body, passage)))
                {
                    steeringPoint = end;
                    steeringHeading = Heading(direct);
                    steeringReady = true;
                    return;
                }
            }

            int angles = steerLaterally ? SteeringAngles.Length : 1;
            for (int i = 0; i < angles; i++)
            {
                float angle = Footprint.NeedsTurns ? Heading(direct)
                    : Mathf.MoveTowardsAngle(initialHeading, desiredAngle + SteeringAngles[i], 420f * dt);
                float radians = angle * Mathf.Deg2Rad;
                Vector2 direction = new Vector2(Mathf.Sin(radians), Mathf.Cos(radians));
                float forward = Vector2.Dot(direct, direction);
                if (forward < 0.05f) continue;
                for (int j = 0; j < SteeringSpeeds.Length; j++)
                {
                    float candidateSpeed = Mathf.Min(0.25f / dt,
                        Mathf.MoveTowards(speed, limit * SteeringSpeeds[j], acceleration * dt));
                    float amount = Mathf.Min(remaining, candidateSpeed * dt);
                    if (amount <= 0.000001f) continue;
                    Vector2 point = centre + direction * amount;
                    if (!grid.CanSweep(centre, point, half) || (!retreating && !revisionRecovery
                        && !grid.CanSteerAcrossPassage(TranslationSweep(centre, point, facing), body, passage))) continue;
                    geometricallyClear = true;
                    float safe = navigation.TimeToContact(body, (point - centre) / dt, horizon);
                    if (safe <= dt) continue;
                    // Reward progress, clearance and the previously selected
                    // side. A slower candidate can beat a fast collision course.
                    float side = direct.x * direction.y - direct.y * direction.x;
                    float score = forward * SteeringSpeeds[j] + safe / horizon * 1.5f
                        + Vector2.Dot(desired, direction) * 0.22f
                        + (detourSide != 0 && side * detourSide > 0.05f ? 0.12f : 0f);
                    if (score <= best) continue;
                    best = score;
                    steeringPoint = point;
                    steeringHeading = angle;
                    steeringReady = true;
                    if (i == 0 && j == 0 && safe >= horizon) return;
                }
            }
            if (!steeringReady)
            {
                bool aligning = !Footprint.NeedsTurns && Mathf.Abs(Mathf.DeltaAngle(initialHeading, Heading(direct))) > 420f * dt;
                steeringHold = geometricallyClear ? Hold.Traffic : aligning ? Hold.None : Hold.Geometry;
            }
        }

        private static float Heading(Vector2 direction) => Mathf.Atan2(direction.x, direction.y) * Mathf.Rad2Deg;

        internal void MoveTick(float dt)
        {
            using var marker = Marker.Auto();
            velocity = Vector2.zero;
            if (!CurrentGrid || revision != grid.Revision || revisionHold)
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
            if (!retreating && (passagePending || (passage != null && !navigation.CanEnterPassage(this))))
            {
                Wait(Hold.Passage, dt);
                return;
            }

            Vector2 proposed;
            float proposedProgress, proposedYaw;
            if (edgeAngle == 0f)
            {
                travelHeading = steeringHeading;
                hasTravelHeading = true;
                if (!steeringReady)
                {
                    if (steeringHold == Hold.None) { speed = 0f; return; }
                    Wait(steeringHold, dt);
                    if (steeringHold == Hold.Traffic && wait >= 0.15f && !retreating)
                        Blocker(TranslationSweep(centre, Vector2.MoveTowards(centre, end, 0.35f), facing))
                            ?.YieldForFriendly(end - centre, this);
                    if (!revisionRecovery && wait >= 0.85f && !retreating) RecoverBlockedEdge();
                    else if (!revisionRecovery && retreating && wait >= 1.7f && grid.CanSweep(centre, end, HalfExtent(facing))
                        && Blocker(TranslationSweep(centre, Vector2.MoveTowards(centre, end, 0.35f), facing)) == null)
                        retreating = false;
                    return;
                }
                proposed = steeringPoint;
                proposedYaw = facing;
                bool reachedBreadcrumb = retreating && edgeTrail.Count != 0
                    && (proposed - edgeTrail[edgeTrail.Count - 1]).sqrMagnitude < 0.00000001f;
                if (reachedBreadcrumb)
                    proposedProgress = edgeTrail.Count == 1 ? 0f : Mathf.Max(0.0001f, progress);
                else if (!retreating && (proposed - end).sqrMagnitude < 0.00000001f)
                    proposedProgress = 1f;
                else
                {
                    var edge = end - edgeStart;
                    proposedProgress = Mathf.Clamp(Vector2.Dot(proposed - edgeStart, edge) / edge.sqrMagnitude, 0.0001f, 0.9999f);
                }
            }
            else
            {
                float length = Mathf.Max((end - edgeStart).magnitude, Mathf.Abs(edgeAngle) / 55f);
                float remaining = length * (retreating ? progress : 1f - progress);
                float acceleration = 12f / cellSize;
                float targetSpeed = Mathf.Min(4f / cellSize, Mathf.Sqrt(2f * acceleration * remaining));
                speed = Mathf.MoveTowards(speed, targetSpeed, acceleration * dt);
                float increment = Mathf.Min(0.25f, Mathf.Max(speed * dt, Mathf.Min(remaining, 0.0001f)));
                proposedProgress = Mathf.MoveTowards(progress, retreating ? 0f : 1f, increment / length);
                proposed = Vector2.Lerp(edgeStart, end, proposedProgress);
                proposedYaw = edgeYaw + edgeAngle * proposedProgress;
            }

            Rect sweep = edgeAngle == 0f ? TranslationSweep(centre, proposed, facing)
                : revisionRecovery ? TurnSlice(progress, proposedProgress) : EdgeSweep(edgeStart, end);
            var obstruction = Blocker(sweep);
            if (obstruction != null)
            {
                if (obstruction.Team == Team && wait >= 0.15f && !retreating)
                    obstruction.YieldForFriendly(end - centre, this);
                Wait(Hold.Traffic, dt);
                if (!revisionRecovery && wait >= 0.85f && !retreating)
                {
                    RecoverBlockedEdge();
                }
                else if (!revisionRecovery && retreating && wait >= 1.7f && Blocker(EdgeSweep(centre, end)) == null)
                    retreating = false;
                return;
            }

            if (passage != null && !retreating) navigation.EnterPassage(this);
            if (progress == 0f && edgeAngle != 0f) TurnCount++;
            velocity = (proposed - centre) / dt;
            if (edgeAngle == 0f) speed = velocity.magnitude;
            if (velocity.sqrMagnitude > 0.000001f) MotionTicks++;
            if (edgeAngle == 0f)
            {
                if (retreating)
                {
                    if (edgeTrail.Count > 1 && (proposed - edgeTrail[edgeTrail.Count - 1]).sqrMagnitude < 0.00000001f)
                        edgeTrail.RemoveAt(edgeTrail.Count - 1);
                }
                else if (edgeTrail.Count != 0)
                {
                    var last = edgeTrail[edgeTrail.Count - 1];
                    if (!grid.CanSweep(last, proposed, HalfExtent(facing))) edgeTrail.Add(centre);
                    else if ((proposed - last).sqrMagnitude >= 0.04f) edgeTrail.Add(proposed);
                }
            }
            centre = proposed;
            facing = proposedYaw;
            progress = proposedProgress;
            wait = 0f;
            if (edgeAngle == 0f)
            {
                var direction = (end - edgeStart).normalized;
                float side = direction.x * velocity.y - direction.y * velocity.x;
                if (Mathf.Abs(side) > 0.15f) detourSide = Math.Sign(side);
            }
            HoldReason = Hold.None;
            State = retreating ? Motion.RepathPending : localStep ? Motion.Yielding : Motion.Moving;
            Rect occupancy = !revisionRecovery && edgeAngle != 0f && progress < 1f && !(retreating && progress <= 0f)
                ? TurnSweep(edgeStart, end) : BoundsAt(centre, facing);
            UpdateOccupancy(occupancy);
            navigation.Register(this, centre);
            if (retreating && progress <= 0f) StopAtAnchor();
            else if (progress >= 1f) FinishEdge();
        }

        private void UpdateOccupancy(Rect occupancy)
        {
            if (Occupancy != occupancy) navigation.NotifyTrafficChanged();
            Occupancy = occupancy;
        }

        private void RecoverBlockedEdge()
        {
            if (HoldReason == Hold.Geometry) RecordCornerFailure(edgeEnd);
            if (progress == 0f)
            {
                if (HoldReason == Hold.Geometry && TryDetour(edgeEnd, true)) return;
                StopAtAnchor();
            }
            else
            {
                retreating = true;
                if (edgeAngle == 0f && edgeTrail.Count > 1 && (centre - edgeTrail[edgeTrail.Count - 1]).sqrMagnitude < 0.00000001f)
                    edgeTrail.RemoveAt(edgeTrail.Count - 1);
            }
        }

        private void RecordCornerFailure(NavigationPose destination)
        {
            if (cornerRevision == grid.Revision && failedCorner.Equals(restingPose) && failedDirection.Equals(destination))
                cornerFailures++;
            else
            {
                cornerRevision = grid.Revision;
                failedCorner = restingPose;
                failedDirection = destination;
                cornerFailures = 1;
            }
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
            int staging = RampStagingIndex();
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
                for (int i = next + 1; i < path.Points.Count && i <= next + LookAhead && i <= staging; i++)
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
                RecordCornerFailure(destination);
                if (!TryDetour(destination, true)) StopAtAnchor();
                return;
            }
            SetEdge(destination, index, false);
        }

        private int RampStagingIndex()
        {
            if (grid.IsRampPose(restingPose, Footprint)) return next;
            int end = Mathf.Min(path.Points.Count - 1, next + LookAhead);
            for (int i = next; i <= end; i++)
            {
                var point = path.Points[i].Pose;
                if (!grid.IsRampPose(point, Footprint)) continue;
                // Keep the last flat aligned pose as a checked checkpoint.
                // The ramp edge can then carry speed through the entry.
                return Mathf.Max(next, i - 1);
            }
            return end;
        }

        private bool TryDetour(NavigationPose destination, bool cornerRecovery = false)
        {
            if (yieldTarget.HasValue || path == null) return false;
            // Leave actual single lanes to passage arbitration. A detour is
            // useful only if there is a verified way back to the route ahead.
            if (!cornerRecovery && passage != null && (passage.Complex || passage.Width < passageWidth * 2)) return false;
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
                if (option.Equals(destination) || (!cornerRecovery && Vector2.Dot(desired, delta) < 0f)
                    || (!cornerRecovery && grid.PassageAt(restingPose, option, Footprint, out _, out _, out _))
                    || navigation.FindContact(this, TranslationSweep(centre, Centre(option), facing), horizon: 0.24f) != null) continue;
                for (int j = Mathf.Min(path.Points.Count - 1, next + LookAhead); j >= next; j--)
                {
                    var join = path.Points[j].Pose;
                    if (!option.Equals(join) && !CanShortcut(option, join, 4f)) continue;
                    if (!option.Equals(join) && Blocker(BoundsAt(Centre(join), join.Yaw)) != null) continue;
                    int optionSide = desired.x * delta.y - desired.y * delta.x >= 0f ? 1 : -1;
                    float score = j + Vector2.Dot(desired.normalized, ((Vector2)delta).normalized)
                        + (optionSide == detourSide ? 1.5f : 0f) - (grid.NearBoundary(option, Footprint) ? 1f : 0f);
                    if (cornerRecovery && !grid.NearBoundary(option, Footprint)) score += 2f;
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
            if (side == 0) return cornerRecovery && TryLeaveCorner(desired);
            detourSide = side;
            SetEdge(chosen, rejoin, true);
            return true;
        }

        private bool TryLeaveCorner(Vector2 desired)
        {
            if (Footprint.NeedsTurns) return TryMountedDetour();
            float best = float.NegativeInfinity;
            NavigationPose chosen = default;
            int edges = grid.Edges(restingPose, Footprint);
            for (int i = 0; i < 8; i++)
            {
                if ((edges & (1 << i)) == 0) continue;
                var step = NavigationPose.Steps[i];
                var candidate = new NavigationPose(restingPose.Cell + step, restingPose.Facing);
                if (candidate.Equals(failedDirection) || grid.PassageAt(restingPose, candidate, Footprint, out _, out _, out _)
                    || Blocker(TranslationSweep(centre, Centre(candidate), facing)) != null) continue;
                bool inside = !grid.NearBoundary(candidate, Footprint);
                float forward = Vector2.Dot(desired.normalized, ((Vector2)step).normalized);
                // On the edge, follow the boundary. A step away from the
                // goal is only useful when it takes the body back inland.
                if (!inside && (forward < -0.05f || cornerFailures > 1)) continue;
                float score = (inside ? 3f : 0f) + forward;
                if (score <= best) continue;
                chosen = candidate;
                best = score;
            }
            if (float.IsNegativeInfinity(best)) return false;
            SetEdge(chosen, next, true);
            leaveCornerAfterEdge = true;
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
            edgeTrail.Clear();
            edgeTrail.Add(centre);
            edgeYaw = facing;
            edgeAngle = Mathf.DeltaAngle(facing, to.Yaw);
            edgeIndex = index;
            localStep = local;
            finishingMountedStep = leaveCornerAfterEdge = false;
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
            if (revisionRecovery) { StopAtAnchor(); return; }
            if (finishingMountedStep || leaveCornerAfterEdge)
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
            if (endProvisionalAfterEdge) StopAtAnchor();
            if (!localStep) detourSide = 0;
            if (path != null && !path.IsCurrent) StopAtAnchor();
        }

        private void StopAtAnchor()
        {
            activeEdge = retreating = finishingMountedStep = endProvisionalAfterEdge = leaveCornerAfterEdge = false;
            revisionHold = revisionRecovery = false;
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
                    if (!turn && grid.PassageAt(from, option, Footprint, out _, out _, out _)) continue;
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

        internal bool CanPlan => HasPlanningAnchor && !retreating && Time.time >= yieldUntil;
        internal bool Recovering => State == Motion.RepathPending || State == Motion.Blocked || State == Motion.Waiting;

        private void YieldForFriendly(Vector2 approach, UnitMovement requester)
        {
            if (requester.Team != Team || !AtAnchor || grid.Revision != revision || Time.time < yieldUntil || (HasRoute && !Recovering)) return;
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
            if (cornerFailures >= 2 && cornerRevision == grid.Revision && from.Equals(failedCorner)
                && to.Equals(failedDirection)) return false;
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
        private UnitMovement Blocker(Rect sweep) => navigation.FindContact(this, sweep);

        private bool CanShortcut(NavigationPose from, NavigationPose to, float maxDistance)
        {
            if (from.Facing != to.Facing || (RequiresCornerRecovery && from.Equals(failedCorner)
                && to.Equals(failedDirection))) return false;
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
                    if (grid.NearBoundary(pose, Footprint) || grid.IsRampPose(pose, Footprint)
                        || grid.PassageAt(pose, pose, Footprint, out _, out _, out _)) return false;
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
