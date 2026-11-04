using UnityEngine;

namespace GameDev.RtsRpg
{
    /// <summary>Owns the latest player order and its presentation. Navigation owns dispatch.</summary>
    internal sealed class RtsOrders
    {
        internal const int PlayerTeam = 0;
        internal enum Kind { Move, AttackMove }

        private readonly NavigationDemo routes;
        private NavigationDemo.GroupRouteRequest latest;
        private int orderedCount;
        private Kind kind;
        internal Vector3 Destination { get; private set; }
        internal bool HasOrder => latest != null;
        internal string Feedback { get; private set; }

        internal RtsOrders(NavigationDemo routes) => this.routes = routes;

        internal bool Issue(Kind type, Ray ray, NavigationGrid grid, RtsSelection selection)
        {
            if (selection.Count == 0)
            {
                Feedback = "Select a friendly unit first.";
                return false;
            }
            if (!grid.TryPickSurface(ray, out var target, out var point, out _))
            {
                Feedback = "Rejected · no terrain top under cursor";
                return false;
            }
            var request = new NavigationDemo.GroupRouteRequest(target, selection.Count == 1,
                type == Kind.AttackMove, PlayerTeam);
            if (!routes.QueueOrder(selection.Members, request))
            {
                Feedback = "Rejected · invalid or unavailable destination";
                return false;
            }
            latest = request;
            orderedCount = selection.Count;
            kind = type;
            Destination = point;
            Feedback = null;
            return true;
        }

        internal void Reset()
        {
            latest = null;
            Feedback = null;
        }

        internal string Describe()
        {
            if (latest == null) return "RMB Move · F then LMB Attack-Move";
            var progress = new NavigationDemo.GroupRouteProgress();
            routes.ReadOrderProgress(latest, ref progress);
            int stale = progress.Stale + orderedCount - progress.Active;
            string label = kind == Kind.Move ? "Move" : "Attack-Move";
            if (progress.Active == 0) return label + " · Stale";
            string result = label;
            if (progress.Accepted > 0) result += $" · Accepted {progress.Accepted}";
            if (progress.Planning > 0) result += $" · Planning {progress.Planning}";
            if (progress.Moving > 0) result += $" · Moving {progress.Moving}";
            if (progress.Arrived > 0) result += $" · Arrived {progress.Arrived}";
            if (progress.Blocked > 0) result += $" · Blocked {progress.Blocked}";
            if (progress.EnemyBlocked > 0) result += $" · EnemyBlocked {progress.EnemyBlocked}";
            if (progress.NoPath > 0) result += $" · NoPath {progress.NoPath}";
            if (progress.BudgetExceeded > 0) result += $" · BudgetExceeded {progress.BudgetExceeded}";
            if (stale > 0) result += $" · Stale {stale}";
            if (progress.EngagementPending > 0) result += $" · EngagementPending {progress.EngagementPending}";
            return result;
        }
    }
}
