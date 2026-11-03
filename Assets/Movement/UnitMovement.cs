using System.Collections.Generic;
using UnityEngine;

namespace GameDev.RtsRpg
{
    /// <summary>Executes one accepted path. No path searches or terrain ownership.</summary>
    public sealed class UnitMovement : MonoBehaviour
    {
        private static readonly List<UnitMovement> Units = new List<UnitMovement>();
        private TerrainNavigation navigation;
        private NavigationGrid grid;
        private NavigationPath path;
        private NavigationFootprint footprint;
        private Vector2 centre;
        private float facing, progress, wait;
        private int next;
        private int team;
        private int revision;
        private NavigationPose restingPose;
        private NavigationPose? yieldTarget;
        private bool canYield;
        private float cellSize;
        public string State { get; private set; }

        public bool Begin(TerrainNavigation owner, NavigationPath route, NavigationFootprint size, int faction = 0)
        {
            if (navigation != null || owner == null || owner.Grid == null || route == null
                || !route.IsCurrent || !route.BelongsTo(owner.Grid) || route.Points.Count == 0
                || !owner.Grid.CanOccupy(route.Points[0].Pose, size)) return false;
            for (int i = 1; i < route.Points.Count; i++)
            {
                var a = route.Points[i - 1].Pose;
                var b = route.Points[i].Pose;
                if (!(a.Facing == b.Facing ? owner.Grid.CanStep(a, b, size) : owner.Grid.CanTurn(a, b, size)))
                    return false;
            }
            progress = wait = 0f;
            navigation = owner;
            grid = owner.Grid;
            path = route;
            footprint = size;
            team = faction;
            cellSize = grid.LocalToWorld.MultiplyVector(Vector3.right).magnitude * grid.Surface.BlockWorldSize / 4;
            restingPose = route.Points[0].Pose;
            centre = Centre(restingPose);
            revision = grid.Revision;
            canYield = true;
            facing = (int)route.Points[0].Pose.Facing * 90f;
            if (Blocker(Envelope(centre, facing, false)) != null) { navigation = null; return false; }
            navigation.Changed += Invalidate;
            next = 1;
            State = "Moving";
            ApplyPose();
            return true;
        }

        private void OnEnable() => Units.Add(this);

        private void OnDisable()
        {
            Units.Remove(this);
            if (navigation != null) navigation.Changed -= Invalidate;
            navigation = null;
            grid = null;
            path = null;
        }

        private void Invalidate()
        {
            if (grid != navigation.Grid || !grid.IsAlive)
            {
                State = "Stopped: terrain snapshot retired";
                gameObject.SetActive(false);
            }
            else if (grid.Revision != revision)
            {
                path = null;
                yieldTarget = null;
                canYield = false;
                State = "Stopped: navigation changed; new route required";
            }
        }

        private Vector2 Centre(NavigationPose pose) => (Vector2)pose.Cell + (Vector2)footprint.Size(pose.Facing) * 0.5f;

        private void Update()
        {
            if ((path == null && !yieldTarget.HasValue) || Time.deltaTime <= 0f || !Application.isFocused) return;
            if (grid.Revision != revision || !navigation.HasCurrentTransform)
            {
                Invalidate();
                return;
            }
            if (!yieldTarget.HasValue && next >= path.Points.Count)
            {
                State = path.Status == NavigationStatus.Partial ? "Partial destination" : "Arrived";
                path = null;
                return;
            }

            var from = yieldTarget.HasValue ? restingPose : path.Points[next - 1].Pose;
            var to = yieldTarget ?? path.Points[next].Pose;
            bool turning = from.Facing != to.Facing;
            Vector2 a = Centre(from), b = Centre(to);
            float distance = Vector2.Distance(a, b);
            // Limit sweep length independently of render-frame hitches.
            float step = Mathf.Min(0.2f, Time.deltaTime * 4f / cellSize);
            float t = Mathf.Min(1f, progress + step / Mathf.Max(distance, turning ? 1f : 0.01f));
            Vector2 proposed = Vector2.Lerp(a, b, t);
            float angle = Mathf.LerpAngle((int)from.Facing * 90f, (int)to.Facing * 90f, t);
            Rect previous = Envelope(centre, facing, turning), current = Envelope(proposed, angle, turning);
            Rect sweep = Rect.MinMaxRect(Mathf.Min(previous.xMin, current.xMin), Mathf.Min(previous.yMin, current.yMin),
                Mathf.Max(previous.xMax, current.xMax), Mathf.Max(previous.yMax, current.yMax));
            UnitMovement obstruction = Blocker(sweep);
            if (obstruction != null)
            {
                wait += Time.deltaTime;
                if (obstruction.team == team && !yieldTarget.HasValue)
                    obstruction.TryYield(b - a, this);
                State = obstruction.team == team ? "Waiting for ally" : "Blocked by enemy";
                if (wait >= 2f)
                {
                    State += "; stopped";
                    path = null;
                    yieldTarget = null;
                    canYield = false;
                }
                return;
            }

            wait = 0f;
            State = "Moving";
            centre = proposed;
            facing = angle;
            progress = t;
            ApplyPose();
            if (t >= 1f)
            {
                progress = 0f;
                restingPose = to;
                if (yieldTarget.HasValue)
                {
                    yieldTarget = null;
                    State = "Yielded lane";
                }
                else next++;
            }
        }

        private void TryYield(Vector2 approach, UnitMovement requester)
        {
            if (!canYield || path != null || yieldTarget.HasValue || grid.Revision != revision
                || progress != 0f) return;
            canYield = false;
            Vector2Int side = Mathf.Abs(approach.x) >= Mathf.Abs(approach.y) ? Vector2Int.up : Vector2Int.right;
            for (int sign = 1; sign >= -1; sign -= 2)
            {
                var target = new NavigationPose(restingPose.Cell + side * sign, restingPose.Facing);
                if (!grid.CanStep(restingPose, target, footprint)) continue;
                Rect endpoint = Envelope(Centre(target), facing, false);
                if (Blocker(endpoint) != null) continue;
                // One local step away from the requester's route. No path search or retry loop.
                Vector2 separation = Centre(target) - requester.centre;
                if (Vector2.Dot(separation, side * sign) <= 0f) continue;
                yieldTarget = target;
                return;
            }
        }

        private UnitMovement Blocker(Rect sweep)
        {
            foreach (var unit in Units)
            {
                if (unit == this || unit.grid != grid || unit.navigation == null) continue;
                Rect other = unit.Envelope(unit.centre, unit.facing, false);
                const float epsilon = 0.0001f;
                if (sweep.xMin < other.xMax - epsilon && sweep.xMax > other.xMin + epsilon
                    && sweep.yMin < other.yMax - epsilon && sweep.yMax > other.yMin + epsilon)
                    return unit;
            }
            return null;
        }

        private Rect Envelope(Vector2 at, float angle, bool turning)
        {
            float radians = angle * Mathf.Deg2Rad;
            float sin = Mathf.Abs(Mathf.Sin(radians)), cos = Mathf.Abs(Mathf.Cos(radians));
            float radius = Mathf.Sqrt(footprint.Width * footprint.Width + footprint.Length * footprint.Length);
            var size = turning ? Vector2.one * radius : new Vector2(
                footprint.Width * cos + footprint.Length * sin, footprint.Width * sin + footprint.Length * cos);
            return new Rect(at - size * 0.5f, size);
        }

        private void ApplyPose()
        {
            Vector3 surface = grid.WorldPoint(centre.x, centre.y);
            float height = cellSize * 0.75f;
            transform.localScale = new Vector3(footprint.Width * cellSize, height, footprint.Length * cellSize);
            Quaternion yaw = Quaternion.LookRotation(grid.LocalToWorld.MultiplyVector(Vector3.forward));
            transform.SetPositionAndRotation(surface + Vector3.up * height * 0.5f,
                yaw * Quaternion.Euler(0f, facing, 0f));
        }
    }
}
