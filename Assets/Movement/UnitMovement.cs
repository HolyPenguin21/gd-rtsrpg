using UnityEngine;

namespace GameDev.RtsRpg
{
    /// <summary>Executes validated edges and local sidesteps on the current terrain snapshot.</summary>
    public sealed class UnitMovement : MonoBehaviour
    {
        public enum Motion { Moving, Yielding, Waiting, RepathPending, Blocked, Arrived, Invalidated }
        private TerrainNavigation navigation;
        private NavigationGrid grid;
        private NavigationPath path;
        private Vector2 centre;
        private float facing, progress, wait, yieldUntil;
        private int next, team, revision;
        private NavigationPose restingPose;
        private NavigationPose? yieldTarget;
        private bool retreating;
        private float cellSize;
        internal Vector2Int Bucket { get; set; }
        public Motion State { get; private set; }
        public NavigationStatus Status { get; private set; }
        internal NavigationFootprint Footprint { get; private set; }
        internal NavigationPose Pose => restingPose;
        internal bool AtAnchor => progress == 0f && !yieldTarget.HasValue && !retreating;
        internal bool HasRoute => path != null && path.IsCurrent;
        internal Rect Occupancy { get; private set; }
        // The coordinator holds at most one anchored participant while a search spans frames.
        internal bool Planning { get; set; }

        internal bool Reserves(Rect area)
        {
            if (Overlaps(Occupancy, area)) return true;
            if (progress == 0f) return false;
            var to = yieldTarget ?? path.Points[next].Pose;
            return Overlaps(Sweep(Centre(restingPose), (int)restingPose.Facing * 90f,
                Centre(to), (int)to.Facing * 90f, restingPose.Facing != to.Facing), area);
        }

        public bool Begin(TerrainNavigation owner, NavigationPath route, NavigationFootprint size, int faction = 0)
        {
            if (navigation != null || owner == null || owner.Grid == null || route == null
                || !route.IsCurrent || !route.BelongsTo(owner.Grid) || route.Points.Count == 0
                || !owner.Grid.CanOccupy(route.Points[0].Pose, size)) return false;
            navigation = owner;
            grid = owner.Grid;
            Footprint = size;
            team = faction;
            cellSize = grid.LocalToWorld.MultiplyVector(Vector3.right).magnitude * grid.Surface.BlockWorldSize / 4;
            restingPose = route.Points[0].Pose;
            centre = Centre(restingPose);
            revision = grid.Revision;
            facing = (int)restingPose.Facing * 90f;
            Occupancy = Envelope(centre, facing, false);
            if (Blocker(Occupancy) != null || !Assign(route)) { navigation = null; return false; }
            navigation.Register(this, centre);
            ApplyPose();
            return true;
        }

        internal bool Assign(NavigationPath route)
        {
            if (!AtAnchor || !route.IsCurrent || !route.BelongsTo(grid) || route.Points.Count == 0
                || !route.Points[0].Pose.Equals(restingPose)) return false;
            for (int i = 1; i < route.Points.Count; i++)
                if (!CanTraverse(route.Points[i - 1].Pose, route.Points[i].Pose)) return false;
            revision = grid.Revision;
            path = route;
            next = 1;
            wait = 0;
            Status = route.Status;
            State = Motion.Moving;
            return true;
        }

        internal void ReportFailure(NavigationStatus status)
        {
            if (HasRoute) return;
            Status = status;
            State = Motion.Blocked;
        }

        private void OnDisable()
        {
            if (navigation != null) navigation.Unregister(this);
            navigation = null;
            grid = null;
            path = null;
            yieldTarget = null;
            retreating = Planning = false;
            progress = wait = yieldUntil = 0f;
            State = Motion.Invalidated;
        }

        private Vector2 Centre(NavigationPose pose) => (Vector2)pose.Cell + (Vector2)Footprint.Size(pose.Facing) * 0.5f;
        private bool CanTraverse(NavigationPose from, NavigationPose to) => from.Facing == to.Facing
            ? grid.CanStep(from, to, Footprint) : grid.CanTurn(from, to, Footprint);

        private void Update()
        {
            if (navigation == null || Time.deltaTime <= 0f || !Application.isFocused) return;
            if (grid != navigation.Grid || !grid.IsAlive || !navigation.HasCurrentTransform)
            {
                State = Motion.Invalidated;
                return;
            }

            if (grid.Revision != revision)
            {
                // Finish retreat only along an edge that remains valid in the new revision.
                if (progress > 0 && !CanTraverse(restingPose, yieldTarget ?? path.Points[next].Pose))
                {
                    Status = NavigationStatus.Stale;
                    State = Motion.Invalidated;
                    return;
                }
                revision = grid.Revision;
                AbandonEdge();
            }

            if (Planning || (path == null && !yieldTarget.HasValue)) return;
            if (!yieldTarget.HasValue && next >= path.Points.Count)
            {
                State = path.Status == NavigationStatus.Partial ? Motion.Blocked : Motion.Arrived;
                path = null;
                return;
            }

            var to = yieldTarget ?? path.Points[next].Pose;
            bool turning = restingPose.Facing != to.Facing;
            Vector2 a = Centre(restingPose), b = Centre(to);
            float distance = Vector2.Distance(a, b);
            float step = Mathf.Min(0.2f, Time.deltaTime * 4f / cellSize);
            float t = Mathf.MoveTowards(progress, retreating ? 0f : 1f,
                step / Mathf.Max(distance, turning ? 1f : 0.01f));
            Vector2 proposed = Vector2.Lerp(a, b, t);
            float angle = Mathf.LerpAngle((int)restingPose.Facing * 90f, (int)to.Facing * 90f, t);
            // Reserve enough room for a full turn before entering it. During motion
            // the old anchor stays occupied so a retreat cannot be cut off by followers.
            Rect sweep = turning
                ? Sweep(a, facing, b, angle, true)
                : Sweep(centre, facing, proposed, angle, false);
            var obstruction = retreating ? null : Blocker(sweep);
            if (obstruction != null)
            {
                wait += Time.deltaTime;
                State = Motion.Waiting;
                if (obstruction.team == team && wait >= 0.15f)
                    obstruction.TryYield((retreating ? a : b) - centre, this);
                if (wait >= 0.85f && !retreating) AbandonEdge();
                return;
            }

            wait = 0f;
            State = retreating ? Motion.RepathPending : yieldTarget.HasValue ? Motion.Yielding : Motion.Moving;
            centre = proposed;
            facing = angle;
            progress = t;
            ApplyPose();
            if (retreating && t <= 0f) AbandonEdge();
            else if (t >= 1f)
            {
                progress = 0f;
                restingPose = to;
                navigation.NotifyTrafficChanged();
                if (yieldTarget.HasValue)
                {
                    AbandonEdge();
                    yieldUntil = Time.time + 0.85f;
                }
                else next++;
            }
            Occupancy = progress == 0f ? Envelope(centre, facing, false)
                : turning ? sweep : Sweep(a, (int)restingPose.Facing * 90f, centre, facing, false);
            navigation.Register(this, centre);
        }

        private void AbandonEdge()
        {
            State = Motion.RepathPending;
            retreating = progress > 0f;
            if (retreating) return;
            wait = 0f;
            path = null;
            yieldTarget = null;
        }

        internal bool CanPlan => AtAnchor && Time.time >= yieldUntil;
        internal bool Recovering => State == Motion.RepathPending || State == Motion.Blocked || State == Motion.Waiting;

        private void TryYield(Vector2 approach, UnitMovement requester)
        {
            if (Planning || !AtAnchor || grid.Revision != revision || Time.time < yieldUntil
                || (HasRoute && !Recovering)) return;
            Vector2Int forward = Mathf.Abs(approach.x) >= Mathf.Abs(approach.y)
                ? Vector2Int.right * (approach.x >= 0 ? 1 : -1)
                : Vector2Int.up * (approach.y >= 0 ? 1 : -1);
            var side = new Vector2Int(-forward.y, forward.x);
            NavigationPose? queued = null;
            for (int i = 0; i < 3; i++)
            {
                var delta = i == 0 ? side : i == 1 ? -side : forward;
                var target = new NavigationPose(restingPose.Cell + delta, restingPose.Facing);
                if (!CanTraverse(restingPose, target)
                    || Vector2.Dot(Centre(target) - requester.centre, (Vector2)delta) <= 0f) continue;
                var obstruction = Blocker(Sweep(centre, facing, Centre(target), facing, false));
                if (obstruction == null)
                {
                    yieldTarget = target;
                    wait = 0f;
                    return;
                }
                if (obstruction != requester && obstruction.team == team && obstruction.AtAnchor
                    && Vector2.Dot(obstruction.centre - centre, (Vector2)delta) > 0f)
                    queued ??= target;
            }
            // Queue one hop. Its own Update can ask the next ally to yield without recursion.
            if (queued.HasValue) { yieldTarget = queued; wait = 0f; }
        }

        internal bool AllowsSearchEdge(NavigationPose from, NavigationPose to, bool avoidAllies)
        {
            bool turn = from.Facing != to.Facing;
            Rect sweep = Sweep(Centre(from), (int)from.Facing * 90f, Centre(to), (int)to.Facing * 90f, turn);
            return navigation.FindContact(this, sweep, search: true, avoidAllies: avoidAllies) == null;
        }

        internal bool SharesGrid(UnitMovement other) => grid == other.grid;
        internal bool Avoids(UnitMovement other, bool avoidAllies) =>
            other.team != team || (avoidAllies && (other.Recovering || !other.HasRoute));

        private UnitMovement Blocker(Rect sweep) => navigation.FindContact(this, sweep);
        internal static bool Overlaps(Rect a, Rect b)
        {
            const float epsilon = 0.0001f;
            return a.xMin < b.xMax - epsilon && a.xMax > b.xMin + epsilon
                && a.yMin < b.yMax - epsilon && a.yMax > b.yMin + epsilon;
        }

        private Rect Sweep(Vector2 a, float angleA, Vector2 b, float angleB, bool turning)
        {
            Rect previous = Envelope(a, angleA, turning), current = Envelope(b, angleB, turning);
            return Rect.MinMaxRect(Mathf.Min(previous.xMin, current.xMin), Mathf.Min(previous.yMin, current.yMin),
                Mathf.Max(previous.xMax, current.xMax), Mathf.Max(previous.yMax, current.yMax));
        }

        private Rect Envelope(Vector2 at, float angle, bool turning)
        {
            float radians = angle * Mathf.Deg2Rad;
            float sin = Mathf.Abs(Mathf.Sin(radians)), cos = Mathf.Abs(Mathf.Cos(radians));
            float radius = Mathf.Sqrt(Footprint.Width * Footprint.Width + Footprint.Length * Footprint.Length);
            var size = turning ? Vector2.one * radius : new Vector2(
                Footprint.Width * cos + Footprint.Length * sin, Footprint.Width * sin + Footprint.Length * cos);
            return new Rect(at - size * 0.5f, size);
        }

        private void ApplyPose()
        {
            Vector3 surface = grid.WorldPoint(centre.x, centre.y);
            float height = cellSize * 0.75f;
            transform.localScale = new Vector3(Footprint.Width * cellSize, height, Footprint.Length * cellSize);
            Quaternion yaw = Quaternion.LookRotation(grid.LocalToWorld.MultiplyVector(Vector3.forward));
            transform.SetPositionAndRotation(surface + Vector3.up * height * 0.5f,
                yaw * Quaternion.Euler(0f, facing, 0f));
        }
    }
}
