using System;
using System.Collections.Generic;
using Unity.Profiling;
using UnityEngine;

namespace GameDev.RtsRpg
{
    /// <summary>
    /// Continuous route following with local avoidance (Stage 06.1). A unit is
    /// never committed to a grid edge: on any fixed tick it may change
    /// velocity, accept a new route from its actual position, or step aside.
    /// The grid supplies routes and static clearance; bodies never overlap.
    /// </summary>
    public sealed class UnitMovement : MonoBehaviour
    {
        private static readonly ProfilerMarker Marker = new("Navigation.UnitMovement");
        private const int LookAhead = 5;
        public enum Motion { Moving, Yielding, Waiting, RepathPending, Blocked, Arrived, Invalidated }
        // Traffic: allied or predicted contact. Enemy: hostile body on the next sweep (never pushed).
        internal enum Hold { None, Planning, Geometry, Traffic, Passage, Enemy }

        // Tunables in cells and seconds. Local to this stage, not a global contract.
        private const float StallSeconds = 2f;           // no waypoint progress -> drop route, dispatcher replans
        private const float SettleSeconds = 1f, SettleRadius = 4f;   // arrival by contact near the goal
        private const float SettleFar = 12f;                         // ... or behind an arrived ally
        private const float StallProgress = 0.1f;
        private const float PushSeconds = 0.45f;
        private const float AvoidCommitSeconds = 0.6f;
        private const float TurnRate = 270f;             // degrees per second for the 1x2 body
        private const float Horizon = 0.3f;              // steering prediction horizon
        private const float PersonalSpace = 0.1f;        // predicted gap kept to neighbours

        private static readonly float[] SteeringAngles = { 0f, 20f, -20f, 40f, -40f, 60f, -60f, 85f, -85f };
        private static readonly float[] SteeringSpeeds = { 1f, 0.6f, 0.3f };
        // Escape directions used only after the forward set failed for a while:
        // a body touching a stopped ally head-on has no forward candidate at all.
        private static readonly float[] EscapeAngles = { 90f, -90f, 130f, -130f };
        private const float EscapeAfter = 0.3f;
        private static int nextSerial;

        private TerrainNavigation navigation;
        private NavigationGrid grid;
        private NavigationPath path;
        private int next, revision;
        private bool provisional, revisionHold, watchEnemy;
        private NavigationPose restingPose;
        private Vector2 centre, previousCentre, velocity;
        private float facing, previousFacing, speed, cellSize, height, yieldUntil;
        private Quaternion terrainYaw;
        private UnitMovement enemyContact;

        // Steering result for this tick (computed on the shared snapshot).
        private Vector2 steerVelocity;
        private float steerSpeed;
        private bool steerReady, turning, turnReserved;
        private Hold steerHold;
        private int avoidSide;
        private float avoidUntil;

        // Stall watchdog and push-aside request.
        private float bestDistance = float.PositiveInfinity, stallTime;
        private Vector2 pushDirection, pushFrom;
        private float pushUntil;
        // Body this unit is currently waiting on (cycle detection); null while moving.
        private UnitMovement waitingOn;
        private Vector2 waitHeading;
        internal int CycleBreaks { get; private set; }

        // Passage arbitration inputs consumed by TerrainNavigation.
        private NavigationGrid.Passage passage, previousPassage;
        private Vector2Int passageMouth;
        private int passageDirection, previousPassageDirection, passageWidth, passageSince;
        private bool passagePending, passageAdmitted;
        private float passageWaitRun;

        private NavigationPose snapPose;
        private bool snapValid;
        private int snapTick = -1, tickCounter;

        internal int Serial { get; private set; }
        internal Vector2Int Bucket { get; set; }
        public Motion State { get; private set; }
        public NavigationStatus Status { get; private set; }
        internal Hold HoldReason { get; private set; }
        internal NavigationFootprint Footprint { get; private set; }
        internal int Team { get; private set; }
        internal bool Planning { get; set; }
        internal Rect Occupancy { get; private set; }
        internal Vector2 PredictionVelocity { get; private set; }
        internal Vector2 Position => centre;
        internal Vector2 TravelVelocity => PredictionVelocity;

        // Development counters (monotonic; consumers diff them against an order baseline).
        internal int MotionTicks { get; private set; }
        internal int TurnCount { get; private set; }
        internal int StopCount { get; private set; }
        internal int SideFlips { get; private set; }
        internal int ClearanceRejects { get; private set; }
        internal int CornerRetries { get; private set; }   // stall-triggered replans
        internal float PassageWaitMax { get; private set; }
        internal float PassageWaitTotal { get; private set; }

        internal UnitMovement EnemyContact => enemyContact != null && enemyContact.isActiveAndEnabled
            && enemyContact.Team != Team && SharesGrid(enemyContact) ? enemyContact : null;
        internal NavigationGrid.Passage Passage => passage;
        internal Vector2Int PassageMouth => passageMouth;
        internal int PassageDirection => passageDirection;
        internal int PassageWidth => passageWidth;
        internal int PassageSince => passageSince;

        internal bool CurrentGrid => navigation != null && grid == navigation.Grid && grid.IsAlive;
        internal bool IsRevisionHeld => revisionHold;
        internal bool IsRecoveringRevision => CurrentGrid && (revisionHold || revision != grid.Revision);
        internal bool HasPlanningAnchor => CurrentGrid && revision == grid.Revision && !revisionHold && Snap();
        internal bool AtAnchor => !revisionHold;
        internal bool HasRoute => path != null && path.IsCurrent;
        internal bool StopsAfterEdge => false;
        internal bool RequiresCornerRecovery => false;
        internal bool CanPlan => HasPlanningAnchor && Time.time >= yieldUntil;
        internal bool CanLaunch => CanPlan && !HasRoute;
        /// <summary>Arrived by contact on its current pose instead of the planned destination.</summary>
        internal bool Settled { get; private set; }
        internal bool Recovering => State == Motion.RepathPending || State == Motion.Blocked || State == Motion.Waiting;

        /// <summary>Exact pose when arrived; otherwise the nearest valid grid pose to the body.</summary>
        internal NavigationPose Pose => State == Motion.Arrived && path == null ? restingPose : Snap() ? snapPose : restingPose;
        internal NavigationPose PlanningPose => provisional && HasRoute ? path.Points[path.Points.Count - 1].Pose : Pose;

        internal bool ReservesPassage(NavigationGrid.Passage zone) => !revisionHold && zone.Overlaps(Occupancy);
        internal bool Reserves(Rect area) => Overlaps(Occupancy, area)
            || (HasRoute && Overlaps(TranslationSweep(centre, centre + PredictionVelocity * 0.25f, facing), area));

        public bool Begin(TerrainNavigation owner, NavigationPath route, NavigationFootprint size, int faction = 0)
        {
            if (navigation != null || owner == null || owner.Grid == null || route == null
                || !route.IsCurrent || !route.BelongsTo(owner.Grid, size) || route.Points.Count == 0
                || !owner.Grid.CanOccupy(route.Points[0].Pose, size)) return false;
            navigation = owner;
            grid = owner.Grid;
            if (Serial == 0) Serial = ++nextSerial;
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
            if (Blocker(Occupancy) != null) { navigation = null; return false; }
            if (route.Points.Count == 1 && route.Status == NavigationStatus.Success) State = Motion.Arrived;
            else if (!Assign(route)) { navigation = null; return false; }
            navigation.Track(this, centre);
            transform.localScale = new Vector3(size.Width * cellSize, height, size.Length * cellSize);
            Render(centre, facing);
            return true;
        }

        // ---------------------------------------------------------------- routes

        internal bool Assign(NavigationPath route, bool preview = false)
        {
            if (!CurrentGrid || revisionHold || revision != grid.Revision || route == null
                || !route.IsCurrent || !route.BelongsTo(grid, Footprint) || route.Points.Count == 0) return false;
            int join = FindJoin(route);
            if (join < 0) return false;
            path = route;
            next = join;
            turnReserved = false;
            provisional = preview;
            Settled = false;
            Status = route.Status;
            State = Motion.Moving;
            HoldReason = Hold.None;
            ResetStall();
            return true;
        }

        // The farthest early point that is directly reachable from the actual
        // body. A 1x2 only joins at a point with its current body facing.
        private int FindJoin(NavigationPath route)
        {
            int end = Mathf.Min(route.Points.Count - 1, LookAhead);
            var half = HalfExtent(facing);
            for (int i = end; i >= 0; i--)
            {
                var pose = route.Points[i].Pose;
                Vector2 point = Centre(pose);
                if ((point - centre).sqrMagnitude > 3.2f * 3.2f) continue;
                if (Footprint.NeedsTurns)
                {
                    if (Mathf.Abs(Mathf.DeltaAngle(facing, pose.Yaw)) > 1f) continue;
                    if ((point - centre).sqrMagnitude > 0.0001f && !AlongFacing(point - centre, 12f)) continue;
                }
                if (grid.CanSweep(centre, point, half)) return i;
            }
            // A 1x2 stopped between orientations (a stalled turn) joins at the
            // first point and completes the rotation with the swept-arc check.
            if (Footprint.NeedsTurns)
            {
                var first = route.Points[0].Pose;
                Vector2 point = Centre(first);
                if (Mathf.Abs(Mathf.DeltaAngle(facing, first.Yaw)) <= 45.5f && (point - centre).sqrMagnitude <= 2.25f
                    && grid.CanSweep(centre, point, TurnSliceExtent(facing, facing + Mathf.DeltaAngle(facing, first.Yaw)))) return 0;
            }
            return -1;
        }

        // After being displaced, continue from the farthest reachable waypoint
        // ahead; never drive back to a point that is now behind. Without one,
        // drop the route so the dispatcher replans from the actual body.
        private void Rejoin()
        {
            var half = HalfExtent(facing);
            int end = Mathf.Min(path.Points.Count - 1, next + LookAhead);
            for (int i = end; i >= next; i--)
            {
                var pose = path.Points[i].Pose;
                Vector2 point = Centre(pose);
                if (Footprint.NeedsTurns && (Mathf.Abs(Mathf.DeltaAngle(facing, pose.Yaw)) > 1f
                    || ((point - centre).sqrMagnitude > 0.0001f && !AlongFacing(point - centre, 12f)))) continue;
                if (!grid.CanSweep(centre, point, half)) continue;
                next = i;
                return;
            }
            Halt();
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
            if (provisional) Halt();
        }

        internal void CancelPending() { }

        internal void StopAfterCurrentEdge()
        {
            if (!IsRecoveringRevision) Halt();
        }

        private void Halt()
        {
            path = null;
            turnReserved = false;
            provisional = false;
            State = Motion.RepathPending;
            ResetStall();
        }

        internal void WatchEnemy(bool watch)
        {
            watchEnemy = watch;
            if (!watch) enemyContact = null;
        }

        internal void ReportFailure(NavigationStatus status)
        {
            if (HasRoute || revisionHold) return;
            provisional = false;
            Status = status;
            State = Motion.Blocked;
        }

        internal void ConfigureFormation(float lane) { }

        // ---------------------------------------------------------------- tick

        internal void CaptureTick()
        {
            previousCentre = centre;
            previousFacing = facing;
            PredictionVelocity = velocity;
            previousPassage = passage;
            previousPassageDirection = passageDirection;
            passage = null;
            passageDirection = 0;
            passagePending = passageAdmitted = false;
            tickCounter++;
        }

        internal void PrepareTick(int tick)
        {
            if (!CurrentGrid) return;
            if (grid.Revision != revision) RecoverRevision();
            if (revisionHold) return;
            if (path != null && !path.IsCurrent) Halt();
            if (path == null) return;
            AdvanceWaypoint();
            if (path == null) return;
            DetectPassage(tick);
        }

        private void AdvanceWaypoint()
        {
            int count = path.Points.Count;
            // Reached waypoints are consumed; one visibility shortcut per tick
            // keeps the cost bounded.
            while (next < count)
            {
                var pose = path.Points[next].Pose;
                bool reached = (Centre(pose) - centre).sqrMagnitude < 0.0004f
                    && (!Footprint.NeedsTurns || Mathf.Abs(Mathf.DeltaAngle(facing, pose.Yaw)) < 0.5f);
                if (!reached) break;
                if (next == count - 1)
                {
                    Arrive(pose);
                    return;
                }
                next++;
                ResetStall();
            }
            if (next + 1 >= count || (tickCounter + Serial) % 4 != 0) return;
            // Line of sight (string pulling): head straight for the farthest
            // visible route point instead of tracing the grid staircase, so a
            // formation moves in parallel straight lines. Never across a ramp
            // boundary or into passage cells (those keep their arbitration).
            bool onRamp = grid.IsRampPose(path.Points[next].Pose, Footprint);
            var half = HalfExtent(facing);
            int farthest = Mathf.Min(count - 1, next + LongSight);
            for (int k = farthest; k > next; k--)
            {
                var candidate = path.Points[k].Pose;
                Vector2 target = Centre(candidate);
                if ((target - centre).sqrMagnitude > LongSight * LongSight) continue;
                bool consistent = true;
                for (int j = next + 1; j <= k && consistent; j++)
                    consistent = grid.IsRampPose(path.Points[j].Pose, Footprint) == onRamp
                        && (!Footprint.NeedsTurns || Mathf.Abs(Mathf.DeltaAngle(facing, path.Points[j].Pose.Yaw)) <= 0.5f);
                if (!consistent) continue;
                // A 1x2 only shortcuts straight ahead along its body axis.
                if (Footprint.NeedsTurns && (Mathf.Abs(Mathf.DeltaAngle(facing, path.Points[next].Pose.Yaw)) > 0.5f
                    || !AlongFacing(target - centre, 6f))) continue;
                if (!grid.CanSweep(centre, target, half)
                    || !grid.CanSteerAcrossPassage(TranslationSweep(centre, target, facing), Occupancy, null)) continue;
                next = k;
                ResetStall();
                return;
            }
        }

        private const int LongSight = 12;

        private void Arrive(NavigationPose pose)
        {
            restingPose = pose;
            bool partial = path.Status == NavigationStatus.Partial;
            bool wasProvisional = provisional;
            path = null;
            provisional = false;
            speed = 0f;
            ResetStall();
            State = wasProvisional ? Motion.RepathPending : partial ? Motion.Blocked : Motion.Arrived;
            HoldReason = wasProvisional && Planning ? Hold.Planning : Hold.None;
        }

        private void DetectPassage(int tick)
        {
            int count = path.Points.Count;
            int first = Mathf.Max(0, next - 1), last = Mathf.Min(count - 2, next + 2);
            for (int i = first; i <= last; i++)
            {
                var from = path.Points[i].Pose;
                var to = path.Points[i + 1].Pose;
                if ((Centre(to) - centre).sqrMagnitude > 9f) break;
                if (!grid.PassageAt(from, to, Footprint, out passageMouth, out passageDirection, out passageWidth)) continue;
                passage = grid.RequestPassage(passageMouth);
                passagePending = passage == null || !passage.Ready;
                StickPassageSince(tick);
                return;
            }
            // Already standing in a passage (e.g. after a line-of-sight
            // shortcut past its route segments): keep that passage, so its
            // cells stay allowed and the reservation is held until the body leaves.
            if (Snap() && grid.PassageAt(snapPose, path.Points[Mathf.Min(next, count - 1)].Pose, Footprint,
                    out var mouth, out int direction, out int width)
                && grid.PassageAt(snapPose, snapPose, Footprint, out _, out _, out _))
            {
                passageMouth = mouth;
                passageDirection = direction;
                passageWidth = width;
                passage = grid.RequestPassage(passageMouth);
                passagePending = passage == null || !passage.Ready;
                StickPassageSince(tick);
            }
        }

        // Passage priority belongs to the request, not to one tick's detection:
        // it resets only when the unit asks for a different passage or direction.
        private NavigationGrid.Passage requestedPassage;
        private int requestedDirection;

        private void StickPassageSince(int tick)
        {
            if (ReferenceEquals(requestedPassage, passage) && requestedDirection == passageDirection) return;
            requestedPassage = passage;
            requestedDirection = passageDirection;
            passageSince = tick;
        }

        private void RecoverRevision()
        {
            // The body keeps its actual pose. Without static clearance there it
            // is held until the next revision; otherwise it simply replans.
            revision = grid.Revision;
            path = null;
            provisional = false;
            speed = 0f;
            turning = turnReserved = false;
            Status = NavigationStatus.Stale;
            snapTick = -1;
            revisionHold = !grid.CanSweep(centre, centre, HalfExtent(facing));
            State = revisionHold ? Motion.Invalidated : Motion.RepathPending;
            HoldReason = revisionHold ? Hold.Geometry : Hold.None;
            ResetStall();
        }

        // ------------------------------------------------------------- steering

        internal void PrepareSteering(float dt)
        {
            steerReady = turning = false;
            steerHold = Hold.None;
            steerVelocity = Vector2.zero;
            if (!CurrentGrid || revisionHold || revision != grid.Revision) return;

            if (Time.fixedTime < pushUntil)
            {
                SteerPush(dt);
                return;
            }
            if (path == null) return;
            if (passage != null)
            {
                passageAdmitted = !passagePending && (passage.Overlaps(Occupancy) || navigation.CanEnterPassage(this));
            }

            var pose = path.Points[next].Pose;
            Vector2 target = Centre(pose);
            Vector2 toTarget = target - centre;
            float remaining = toTarget.magnitude;
            float maxSpeed = 4f / cellSize, acceleration = 12f / cellSize;

            if (Footprint.NeedsTurns && Mathf.Abs(Mathf.DeltaAngle(facing, pose.Yaw)) >= 0.5f)
            {
                turning = true;   // MoveTick rotates with a swept-arc check
                return;
            }
            if (remaining < 0.0001f) { steerReady = true; return; }

            Vector2 direct = toTarget / remaining;
            bool final = next == path.Points.Count - 1;
            float limit = final ? Mathf.Min(maxSpeed, Mathf.Sqrt(2f * acceleration * remaining)) : maxSpeed;
            float targetSpeed = Mathf.MoveTowards(speed, limit, acceleration * dt);
            Rect body = Occupancy;
            Vector2 half = HalfExtent(facing);
            var allowed = passage != null && passageAdmitted ? passage : null;
            navigation.CollectSteeringNeighbors(this, centre, 3f + maxSpeed * Horizon);

            // Exact final step onto the waypoint when it fits this tick.
            if (remaining <= Mathf.Max(targetSpeed, 0.5f * acceleration * dt) * dt)
            {
                Vector2 snap = toTarget / dt;
                if (StepClear(target, body, half, allowed) && navigation.TimeToContact(body, snap, dt) >= dt)
                {
                    steerVelocity = snap;
                    // Passing through an intermediate waypoint keeps momentum.
                    steerSpeed = final ? snap.magnitude : targetSpeed;
                    steerReady = true;
                    return;
                }
            }

            bool committed = avoidSide != 0 && Time.fixedTime < avoidUntil;
            int angles = Footprint.NeedsTurns ? 1 : SteeringAngles.Length;
            float best = float.NegativeInfinity;
            int bestSide = 0;
            bool geometricallyClear = false, passageClosed = false;
            float baseHeading = Heading(direct);
            for (int i = 0; i < angles; i++)
            {
                float radians = (baseHeading + SteeringAngles[i]) * Mathf.Deg2Rad;
                Vector2 direction = new(Mathf.Sin(radians), Mathf.Cos(radians));
                float forward = Vector2.Dot(direct, direction);
                for (int j = 0; j < SteeringSpeeds.Length; j++)
                {
                    float candidateSpeed = Mathf.Max(targetSpeed * SteeringSpeeds[j], 0.5f * acceleration * dt * SteeringSpeeds[j]);
                    float amount = Mathf.Min(remaining, candidateSpeed * dt);
                    if (amount <= 0.000001f) continue;
                    Vector2 point = centre + direction * amount;
                    if (!grid.CanSweep(centre, point, half)) continue;
                    if (!grid.CanSteerAcrossPassage(TranslationSweep(centre, point, facing), body, allowed))
                    {
                        passageClosed = true;
                        continue;
                    }
                    geometricallyClear = true;
                    Vector2 candidate = (point - centre) / dt;
                    float safe = navigation.TimeToContact(body, candidate, Horizon, PersonalSpace);
                    if (safe <= dt) continue;
                    int side = i == 0 ? 0 : Math.Sign(SteeringAngles[i]);
                    float sideTerm = side == 0 ? 0f
                        : committed ? (side == avoidSide ? 0.4f : -1.5f)
                        : side > 0 ? 0.05f : 0f;
                    float score = forward * SteeringSpeeds[j] * 1.6f + safe / Horizon * 1.2f + sideTerm
                        - Mathf.Abs(SteeringAngles[i]) / 400f;
                    if (score <= best) continue;
                    best = score;
                    bestSide = side;
                    steerVelocity = candidate;
                    steerSpeed = candidateSpeed;
                    steerReady = true;
                    if (i == 0 && j == 0 && safe >= Horizon) return;
                }
            }
            if (!steerReady && geometricallyClear && stallTime >= EscapeAfter)
                Escape(direct, body, half, allowed, Mathf.Max(targetSpeed, 0.25f * maxSpeed), dt, ref bestSide);
            if (steerReady && bestSide != 0)
            {
                if (committed && bestSide != avoidSide) SideFlips++;
                avoidSide = bestSide;
                avoidUntil = Time.fixedTime + AvoidCommitSeconds;
            }
            if (!steerReady)
                steerHold = geometricallyClear ? Hold.Traffic : passageClosed || (passage != null && !passageAdmitted) ? Hold.Passage : Hold.Geometry;
        }

        // Slide around a stopped body: sideways or diagonally back, preferring
        // the committed side. A 1x2 only backs off along its body axis (the
        // special recovery step); it never slides sideways.
        private void Escape(Vector2 direct, Rect body, Vector2 half, NavigationGrid.Passage allowed, float escapeSpeed,
            float dt, ref int bestSide)
        {
            float best = float.NegativeInfinity;
            int options = Footprint.NeedsTurns ? 1 : EscapeAngles.Length;
            float baseHeading = Heading(direct);
            for (int i = 0; i < options; i++)
            {
                Vector2 direction;
                int side;
                if (Footprint.NeedsTurns)
                {
                    float axis = facing * Mathf.Deg2Rad;
                    direction = -new Vector2(Mathf.Sin(axis), Mathf.Cos(axis));
                    side = 0;
                }
                else
                {
                    float radians = (baseHeading + EscapeAngles[i]) * Mathf.Deg2Rad;
                    direction = new Vector2(Mathf.Sin(radians), Mathf.Cos(radians));
                    side = Math.Sign(EscapeAngles[i]);
                }
                Vector2 point = centre + direction * escapeSpeed * 0.5f * dt;
                if (!StepClear(point, body, half, allowed)) continue;
                Vector2 candidate = (point - centre) / dt;
                float safe = navigation.TimeToContact(body, candidate, Horizon);
                if (safe <= dt) continue;
                float score = Vector2.Dot(direct, direction) + safe / Horizon
                    + (avoidSide != 0 && side == avoidSide ? 0.5f : side > 0 ? 0.05f : 0f);
                if (score <= best) continue;
                best = score;
                bestSide = side;
                steerVelocity = candidate;
                steerSpeed = 0f;
                steerReady = true;
            }
        }

        // Step aside for an ally: prefer the requested side, keep static and
        // live clearance, never through a passage that is not ours.
        private void SteerPush(float dt)
        {
            float pushSpeed = 2.4f / cellSize;
            Rect body = Occupancy;
            Vector2 half = HalfExtent(facing);
            navigation.CollectSteeringNeighbors(this, centre, 3f);
            float baseHeading = Heading(pushDirection);
            float best = float.NegativeInfinity;
            Vector2 away = centre - pushFrom;

            int options = Footprint.NeedsTurns ? 2 : 16;
            for (int i = 0; i < options; i++)
            {
                // A 1x2 is pushed only along its body axis (forward or back).
                // Others try 16 directions: the requested one is preferred,
                // any is acceptable that does not close in on the requester.
                Vector2 direction;
                if (Footprint.NeedsTurns)
                {
                    float axis = facing * Mathf.Deg2Rad;
                    direction = new Vector2(Mathf.Sin(axis), Mathf.Cos(axis)) * (i == 0 ? 1f : -1f);
                }
                else
                {
                    float radians = (baseHeading + i * 22.5f) * Mathf.Deg2Rad;
                    direction = new Vector2(Mathf.Sin(radians), Mathf.Cos(radians));
                }
                float along = Vector2.Dot(direction, pushDirection);
                if (away.sqrMagnitude > 0.0001f && Vector2.Dot(direction, away.normalized) < -0.1f) continue;
                Vector2 point = centre + direction * pushSpeed * dt;
                // A short push ignores passage arbitration (a side bay is a valid
                // refuge); static clearance and live occupancy still apply.
                if (!grid.CanSweep(centre, point, half)) continue;
                Vector2 candidate = (point - centre) / dt;
                if (navigation.TimeToContact(body, candidate, Horizon) <= dt) continue;
                if (along <= best) continue;
                best = along;
                steerVelocity = candidate;
                steerReady = true;
            }
            // Nowhere to go: pass the request on to the ally in the way
            // (a bounded cascade, e.g. a chain of idle units in a corridor).
            if (!steerReady && pushDepth < MaxPushDepth && pushAlong != Vector2.zero)
            {
                var next = Blocker(TranslationSweep(centre, centre + pushAlong * 0.5f, facing));
                if (next != null && next.Team == Team) next.RequestPush(this, pushAlong, pushDepth + 1);
            }
        }

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
            if (path == null || Time.fixedTime < pushUntil)
            {
                speed = 0f;
                if (steerReady && Time.fixedTime < pushUntil)
                {
                    Vector2 step = centre + steerVelocity * dt;
                    if (Blocker(TranslationSweep(centre, step, facing)) == null)
                    {
                        Commit(step, facing, dt);
                        State = Motion.Yielding;
                    }
                }
                else if (pushUntil > 0f && Time.fixedTime >= pushUntil)
                {
                    pushUntil = 0f;
                    yieldUntil = Time.time + 0.35f;
                    // Not displaced: still on the arrived pose. Displaced: the
                    // dispatcher routes it back to its own destination.
                    bool home = (Centre(restingPose) - centre).sqrMagnitude < 0.0004f;
                    if (path != null) Rejoin();
                    // An idle unit pushed off its pose stands on the new pose; the
                    // dispatcher routes a unit with its own destination back there.
                    if (State == Motion.Yielding && path == null && !home && Snap())
                    {
                        restingPose = snapPose;
                        Settled = false;
                    }
                    if (State == Motion.Yielding) State = path != null ? Motion.Moving : Motion.Arrived;
                    ResetStall();
                }
                if (path != null) return;
                if (Planning) HoldReason = Hold.Planning;
                else if (!Recovering) HoldReason = Hold.None;
                return;
            }

            var pose = path.Points[next].Pose;
            Vector2 target = Centre(pose);
            Vector2 heading = turning ? Vector2.zero : (target - centre).normalized;
            if (watchEnemy && !turning && heading != Vector2.zero)
            {
                Rect probe = TranslationSweep(centre, centre + heading * 0.35f, facing);
                if (grid.CanSweep(centre, centre + heading * 0.35f, HalfExtent(facing)))
                {
                    var enemy = navigation.FindContact(this, probe, enemyOnly: true);
                    if (enemy != null)
                    {
                        enemyContact = enemy;
                        Halt();
                        HoldReason = Hold.Enemy;
                        State = Motion.Blocked;
                        return;
                    }
                }
            }
            if (turning)
            {
                TurnStep(pose, target, dt);
                return;
            }
            if (!steerReady && steerHold == Hold.Geometry)
            {
                // The waypoint is no longer reachable from the actual body
                // (after a dodge or push): replan now rather than wait.
                StallReplan();
                return;
            }
            if (!steerReady)
            {
                // Legitimate waits are not stalls: an unadmitted passage mouth,
                // or a queue behind an ally that is moving or itself waiting
                // for a passage. Everything else counts toward the watchdog.
                var ahead = steerHold == Hold.Passage ? null : Ahead(heading);
                // Queuing behind an ally waiting for a passage is legitimate only
                // when this unit is not itself admitted to that passage; an
                // admitted unit asks a non-admitted one parked in the mouth to
                // step aside (otherwise both wait for each other).
                bool blocksAdmitted = ahead != null && passage != null && ahead.HoldReason == Hold.Passage
                    && (passageAdmitted || passage.Overlaps(Occupancy));
                bool queue = ahead != null && ahead.Team == Team && !blocksAdmitted
                    && (ahead.PredictionVelocity.sqrMagnitude > 0.0001f || ahead.HoldReason == Hold.Passage);
                waitingOn = ahead;
                waitHeading = heading;
                Wait(ahead != null && ahead.Team != Team ? Hold.Enemy : steerHold, dt,
                    legitimate: steerHold == Hold.Passage || queue);
                if (ahead != null && ahead.Team == Team && !queue) ahead.RequestPush(this, heading);
                return;
            }

            Vector2 proposed = centre + steerVelocity * dt;
            var blocker = Blocker(TranslationSweep(centre, proposed, facing));
            if (blocker != null)
            {
                // Prediction accepted the step, live occupancy did not: try shorter.
                ClearanceRejects++;
                Vector2 shorter = centre + steerVelocity * dt * 0.4f;
                if (Blocker(TranslationSweep(centre, shorter, facing)) == null) proposed = shorter;
                else
                {
                    waitingOn = blocker;
                    waitHeading = heading;
                    Wait(blocker.Team == Team ? Hold.Traffic : Hold.Enemy, dt, legitimate: false);
                    if (blocker.Team == Team) blocker.RequestPush(this, heading);
                    return;
                }
            }
            if (passage != null && passageAdmitted) navigation.EnterPassage(this);
            bool fullStep = blocker == null;
            Commit(proposed, facing, dt);
            speed = fullStep ? steerSpeed : velocity.magnitude;
            State = Motion.Moving;
            HoldReason = Hold.None;
            TrackProgress(target, dt);
        }

        private void TurnStep(NavigationPose pose, Vector2 target, float dt)
        {
            float delta = Mathf.DeltaAngle(facing, pose.Yaw);
            // Start a rotation only when the whole remaining arc is free, then
            // hold that arc as occupancy so nobody closes in mid-turn.
            if (!turnReserved)
            {
                Vector2 arc = TurnSliceExtent(facing, facing + delta);
                Rect whole = Union(new Rect(centre - arc, arc * 2f), new Rect(target - arc, arc * 2f));
                if (!grid.CanSweep(centre, target, arc))
                {
                    Wait(Hold.Geometry, dt, legitimate: false);
                    return;
                }
                var occupant = Blocker(whole);
                if (occupant != null)
                {
                    waitingOn = occupant;
                    waitHeading = (target - centre).sqrMagnitude > 0.0001f ? (target - centre).normalized
                        : new Vector2(Mathf.Sin(facing * Mathf.Deg2Rad), Mathf.Cos(facing * Mathf.Deg2Rad));
                    Wait(occupant.Team == Team ? Hold.Traffic : Hold.Enemy, dt, legitimate: false);
                    if (occupant.Team == Team) occupant.RequestPush(this, (occupant.centre - centre).normalized);
                    return;
                }
                turnReserved = true;
                UpdateOccupancy(whole);
                navigation.Register(this, centre);
            }
            float step = Mathf.Min(Mathf.Abs(delta), TurnRate * dt);
            float nextYaw = facing + Mathf.Sign(delta) * step;
            // The centre of a turn edge moves with the rotation (the cell envelope changes).
            Vector2 nextCentre = Vector2.MoveTowards(centre, target, (target - centre).magnitude * step / Mathf.Abs(delta));
            Vector2 extent = TurnSliceExtent(facing, nextYaw);
            if (!grid.CanSweep(centre, nextCentre, extent))
            {
                Wait(Hold.Geometry, dt, legitimate: false);
                return;
            }
            bool done = Mathf.Abs(delta) - step < 0.5f;
            if (done) { nextYaw = pose.Yaw; nextCentre = target; TurnCount++; turnReserved = false; }
            Rect held = Occupancy;
            Commit(nextCentre, nextYaw, dt);
            if (!done) UpdateOccupancy(held);
            speed = 0f;
            State = Motion.Moving;
            HoldReason = Hold.None;
            ResetStall();
        }

        private void Commit(Vector2 point, float yaw, float dt)
        {
            waitingOn = null;
            velocity = (point - centre) / dt;
            if (velocity.sqrMagnitude > 0.000001f) MotionTicks++;
            centre = point;
            facing = yaw;
            passageWaitRun = 0f;
            snapTick = -1;
            UpdateOccupancy(BoundsAt(centre, facing));
            navigation.Register(this, centre);
        }

        // Stall watchdog: the guarantee that plain movement never freezes.
        // Without measurable progress toward the waypoint the route is dropped
        // and the dispatcher replans from the actual body, avoiding allies.
        private void TrackProgress(Vector2 target, float dt)
        {
            float distance = (target - centre).magnitude;
            if (distance < bestDistance - StallProgress) { bestDistance = distance; stallTime = 0f; }
            else stallTime += dt;
            if (stallTime >= StallLimit) StallReplan();
        }

        // Arrival by contact: close to the goal, or stuck behind a stationary
        // ally (arrived or itself blocked) near the goal. The stop cascades
        // back through the crowd, as in SC2 group arrival; the dispatcher
        // keeps destination footprints unique by exchanging slots.
        private bool NearGoal
        {
            get
            {
                if (path == null || provisional) return false;
                float distance = (Centre(path.Points[path.Points.Count - 1].Pose) - centre).sqrMagnitude;
                return distance <= SettleRadius * SettleRadius
                    || (distance <= SettleFar * SettleFar && waitingOn != null && waitingOn.Team == Team
                        && ((waitingOn.path == null && waitingOn.State == Motion.Arrived)
                            || waitingOn.PredictionVelocity.sqrMagnitude < 0.0001f));
            }
        }
        private float StallLimit => NearGoal ? SettleSeconds : StallSeconds;

        private void Wait(Hold reason, float dt, bool legitimate)
        {
            if (HoldReason == Hold.None && MotionTicks > 0) StopCount++;
            if (reason == Hold.Passage)
            {
                passageWaitRun += dt;
                PassageWaitTotal += dt;
                PassageWaitMax = Mathf.Max(PassageWaitMax, passageWaitRun);
            }
            speed = 0f;
            HoldReason = reason;
            State = Motion.Waiting;
            if (legitimate) return;
            stallTime += dt;
            if (stallTime >= 0.6f && TryBreakCycle()) return;
            if (stallTime >= StallLimit) StallReplan();
        }

        // Waiting cycles (A waits on B waits on C waits on A) occur in tight
        // packing even when every unit is individually right to wait. Exactly
        // one member backs off briefly: chosen by a hash that changes every
        // two seconds, so a member that cannot move hands the role on. It
        // keeps its route and rejoins it after the step.
        private bool TryBreakCycle()
        {
            if (waitingOn == null || waitingOn.Team != Team || waitHeading == Vector2.zero) return false;
            int window = Mathf.FloorToInt(Time.fixedTime / 2f);
            int mine = CycleKey(this, window), highest = mine;
            var current = waitingOn;
            for (int length = 0; length < 8; length++)
            {
                if (current == null || current.Team != Team || current.State != Motion.Waiting) return false;
                if (current == this) break;
                highest = Math.Max(highest, CycleKey(current, window));
                current = current.waitingOn;
                if (length == 7) return false;
            }
            if (current != this || mine != highest) return false;
            pushDirection = -waitHeading;
            pushFrom = waitingOn.centre;
            pushUntil = Time.fixedTime + PushSeconds;
            State = Motion.Yielding;
            CycleBreaks++;
            return true;

            // The side without passage admission (or with the later request)
            // gives way first: honest single-lane queuing. Otherwise a hash
            // that rotates every two seconds.
            static int CycleKey(UnitMovement unit, int window)
            {
                int hash = (unchecked((unit.Serial * 73856093) ^ (window * 19349663)) & 0xFFFFF);
                int priority = unit.passage == null ? 1 : unit.passageAdmitted ? 0 : 2;
                return (priority << 28) | (Math.Min(unit.passageSince & 0xFF, 0xFF) << 20) | hash;
            }
        }

        private void StallReplan()
        {
            if (turnReserved) { turnReserved = false; UpdateOccupancy(BoundsAt(centre, facing)); }
            // Arrival by contact: blocked by the crowd close to the goal, stop
            // on the current valid pose (no turn left to finish) instead of
            // reshuffling into the exact slot. The dispatcher moves the
            // destination reservation to this pose.
            if (NearGoal && Snap() && Mathf.Abs(Mathf.DeltaAngle(facing, snapPose.Yaw)) < 0.5f)
            {
                restingPose = snapPose;
                path = null;
                provisional = false;
                Settled = true;
                speed = 0f;
                State = Motion.Arrived;
                HoldReason = Hold.None;
                ResetStall();
                return;
            }
            CornerRetries++;
            path = null;
            provisional = false;
            State = Motion.RepathPending;
            HoldReason = Hold.Traffic;
            ResetStall();
        }

        private void ResetStall()
        {
            bestDistance = float.PositiveInfinity;
            stallTime = 0f;
        }

        private UnitMovement Ahead(Vector2 heading) =>
            heading == Vector2.zero ? null : Blocker(TranslationSweep(centre, centre + heading * 0.5f, facing));

        // An idle ally, or one that has itself been stopped for a while, steps
        // aside, preferring the side away from the requester's line of travel;
        // a stopped ally keeps its route and resumes after the push. In a
        // mutual block both step sideways, which resolves head-on pairs.
        // Moving allies are not pushed; enemies never are.
        private const int MaxPushDepth = 6;
        private int pushDepth;
        private Vector2 pushAlong;

        private void RequestPush(UnitMovement requester, Vector2 along, int depth = 0)
        {
            if (requester.Team != Team || revisionHold || along == Vector2.zero || Time.fixedTime < pushUntil
                || (HasRoute && !(State == Motion.Waiting && (stallTime >= 0.4f || HoldReason == Hold.Passage)))) return;
            Vector2 side = new(-along.y, along.x);
            if (Vector2.Dot(centre - requester.centre, side) < 0f) side = -side;
            pushDirection = (side * 0.8f + along * 0.6f).normalized;
            pushFrom = requester.centre;
            pushAlong = along.normalized;
            pushDepth = depth;
            pushUntil = Time.fixedTime + PushSeconds;
            if (State == Motion.Arrived) State = Motion.Yielding;
        }

        // --------------------------------------------------------------- launch

        internal bool TryLaunch(Vector2 target)
        {
            if (!CanLaunch) return false;
            var from = Pose;
            if (!grid.CanOccupy(from, Footprint)) return false;
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
                    float score = (Centre(option) - target).magnitude + (turn ? 0.3f : 0f)
                        + (grid.NearBoundary(option, Footprint) ? 0.6f : 0f);
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

        // ------------------------------------------------------------- planning

        // Nearest statically valid grid pose that the body can reach in a
        // straight checked sweep. This is the planning start: any tick, any
        // position, no edge has to finish first.
        private bool Snap()
        {
            if (snapTick == tickCounter) return snapValid;
            snapTick = tickCounter;
            snapValid = false;
            if (!CurrentGrid) return false;
            // A 1x2 between orientations may snap to either neighbouring
            // orientation, but only one whose remaining rotation is clear.
            float sector = Mathf.Repeat(facing, 360f) / 45f;
            int lower = Mathf.FloorToInt(sector) & 7, upper = (lower + 1) & 7;
            bool between = Footprint.NeedsTurns && Mathf.Abs(sector - Mathf.Round(sector)) > 0.01f;
            float best = float.PositiveInfinity;
            for (int option = 0; option < (between ? 2 : 1); option++)
            {
                var orientation = !Footprint.NeedsTurns ? NavigationFacing.North
                    : between ? (NavigationFacing)(option == 0 ? lower : upper)
                    : (NavigationFacing)(Mathf.RoundToInt(sector) & 7);
                float yaw = (int)orientation * 45f;
                var half = between ? TurnSliceExtent(facing, facing + Mathf.DeltaAngle(facing, yaw)) : HalfExtent(facing);
                float turnCost = Mathf.Abs(Mathf.DeltaAngle(facing, yaw)) / 90f;
                Vector2 size = Footprint.Size(orientation);
                Vector2 corner = centre - size * 0.5f;
                var origin = new Vector2Int(Mathf.RoundToInt(corner.x), Mathf.RoundToInt(corner.y));
                for (int dz = -1; dz <= 1; dz++)
                for (int dx = -1; dx <= 1; dx++)
                {
                    var pose = new NavigationPose(origin + new Vector2Int(dx, dz), orientation);
                    float distance = (Centre(pose) - centre).sqrMagnitude + turnCost;
                    if (distance >= best || !grid.CanOccupy(pose, Footprint) || !grid.CanSweep(centre, Centre(pose), half)) continue;
                    best = distance;
                    snapPose = pose;
                    snapValid = true;
                }
            }
            return snapValid;
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

        // ----------------------------------------------------------- lifecycle

        private void OnDisable()
        {
            WatchEnemy(false);
            if (navigation != null) navigation.Untrack(this);
            navigation = null;
            grid = null;
            path = null;
            Planning = provisional = revisionHold = turning = turnReserved = false;
            speed = yieldUntil = pushUntil = passageWaitRun = 0f;
            velocity = PredictionVelocity = Vector2.zero;
            passage = previousPassage = null;
            passageDirection = previousPassageDirection = avoidSide = 0;
            snapTick = -1;
            ResetStall();
            State = Motion.Invalidated;
        }

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

        // ------------------------------------------------------------ geometry

        private bool StepClear(Vector2 point, Rect body, Vector2 half, NavigationGrid.Passage allowed) =>
            grid.CanSweep(centre, point, half) && grid.CanSteerAcrossPassage(TranslationSweep(centre, point, facing), body, allowed);

        private bool AlongFacing(Vector2 delta, float toleranceDegrees) =>
            Mathf.Abs(Mathf.DeltaAngle(facing, Heading(delta))) <= toleranceDegrees;

        private void UpdateOccupancy(Rect occupancy)
        {
            if (Occupancy != occupancy) navigation.NotifyTrafficChanged();
            Occupancy = occupancy;
        }

        private static float Heading(Vector2 direction) => Mathf.Atan2(direction.x, direction.y) * Mathf.Rad2Deg;
        private Vector2 Centre(NavigationPose pose) => pose.Centre(Footprint);
        private Vector2 TurnHalfExtent => Vector2.one * 1.118034f;
        private UnitMovement Blocker(Rect sweep) => navigation.FindContact(this, sweep);

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

        // Axis-aligned extent of a rotating 1x2 between two yaws, including
        // the extent maxima an arc can straddle.
        private Vector2 TurnSliceExtent(float first, float last)
        {
            var a = HalfExtent(first);
            var b = HalfExtent(last);
            float middle = (first + last) * 0.5f, span = Mathf.Abs(last - first) * 0.5f;
            return new Vector2(ContainsPeak(63.43495f) ? TurnHalfExtent.x : Mathf.Max(a.x, b.x),
                ContainsPeak(26.56505f) ? TurnHalfExtent.y : Mathf.Max(a.y, b.y));

            bool ContainsPeak(float peak) =>
                Mathf.Abs(Mathf.DeltaAngle(middle, peak)) <= span || Mathf.Abs(Mathf.DeltaAngle(middle, -peak)) <= span
                || Mathf.Abs(Mathf.DeltaAngle(middle, 180f + peak)) <= span
                || Mathf.Abs(Mathf.DeltaAngle(middle, 180f - peak)) <= span;
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
    }
}
