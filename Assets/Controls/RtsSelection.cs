using System;
using System.Collections.Generic;
using UnityEngine;

namespace GameDev.RtsRpg
{
    /// <summary>Selection uses the displayed, interpolated demo unit bounds.</summary>
    internal sealed class RtsSelection
    {
        private readonly HashSet<UnitMovement> members = new();
        private readonly Vector2[] corners = new Vector2[8];
        internal HashSet<UnitMovement> Members => members;
        internal int Count => members.Count;
        internal void Clear() => members.Clear();
        internal void Prune() => members.RemoveWhere(IsUnavailable);

        private static bool IsUnavailable(UnitMovement unit) => !Visible(unit) || unit.Team != RtsOrders.PlayerTeam;
        private static bool Visible(UnitMovement unit) => unit != null && unit.isActiveAndEnabled && unit.CurrentGrid;

        internal void Click(Ray ray, Camera view, TerrainNavigation navigation, bool additive)
        {
            navigation.Grid.TryPickSurface(ray, out _, out _, out float terrain);
            float closest = Mathf.Min(terrain, view.farClipPlane - view.nearClipPlane);
            UnitMovement hit = null;
            foreach (var unit in navigation.LiveUnits)
            {
                if (!Visible(unit) || !unit.TryGetComponent<Renderer>(out var renderer) || !renderer.enabled) continue;
                if (!Intersects(renderer, ray, out float distance) || distance >= closest) continue;
                closest = distance;
                hit = unit;
            }
            if (!additive) members.Clear();
            if (hit == null || hit.Team != RtsOrders.PlayerTeam) return;
            if (!members.Add(hit) && additive) members.Remove(hit);
        }

        internal void Box(Rect box, Camera view, TerrainNavigation navigation, bool additive,
            Predicate<Vector2> blocksPointer)
        {
            if (!additive) members.Clear();
            foreach (var unit in navigation.LiveUnits)
            {
                if (IsUnavailable(unit) || !unit.TryGetComponent<Renderer>(out var renderer) || !renderer.enabled) continue;
                var projected = Project(renderer, view);
                if (!box.Overlaps(projected)) continue;
                var overlap = Rect.MinMaxRect(Mathf.Max(box.xMin, projected.xMin), Mathf.Max(box.yMin, projected.yMin),
                    Mathf.Min(box.xMax, projected.xMax), Mathf.Min(box.yMax, projected.yMax));
                if (Sample(overlap.center) || Sample(overlap.min) || Sample(overlap.max)
                    || Sample(new Vector2(overlap.xMin, overlap.yMax)) || Sample(new Vector2(overlap.xMax, overlap.yMin))
                    || VisibleEdge())
                    members.Add(unit);

                bool Sample(Vector2 pixel)
                {
                    var ray = view.ScreenPointToRay(pixel);
                    if (!Intersects(renderer, ray, out float distance) || blocksPointer(pixel)) return false;
                    navigation.Grid.TryPickSurface(ray, out _, out _, out float terrain);
                    return distance < terrain;
                }

                // A rotated box can cross the marquee without containing its
                // centre or corners. Test the portions of its 12 edges inside it.
                bool VisibleEdge()
                {
                    for (int i = 0; i < corners.Length; i++)
                    for (int bit = 1; bit <= 4; bit <<= 1)
                    {
                        if ((i & bit) != 0) continue;
                        var origin = corners[i];
                        var delta = corners[i | bit] - origin;
                        float enter = 0f, leave = 1f;
                        if (Clip(origin.x, delta.x, overlap.xMin, overlap.xMax, ref enter, ref leave)
                            && Clip(origin.y, delta.y, overlap.yMin, overlap.yMax, ref enter, ref leave)
                            && (Sample(origin + delta * (0.5f * (enter + leave)))
                                || Sample(origin + delta * enter) || Sample(origin + delta * leave))) return true;
                    }
                    return false;
                }
            }
        }

        private Rect Project(Renderer renderer, Camera view)
        {
            var bounds = renderer.localBounds;
            var low = bounds.min;
            var high = bounds.max;
            var matrix = renderer.localToWorldMatrix;
            Vector2 min = new(float.PositiveInfinity, float.PositiveInfinity);
            Vector2 max = new(float.NegativeInfinity, float.NegativeInfinity);
            for (int corner = 0; corner < corners.Length; corner++)
            {
                var point = new Vector3((corner & 1) == 0 ? low.x : high.x,
                    (corner & 2) == 0 ? low.y : high.y, (corner & 4) == 0 ? low.z : high.z);
                Vector3 screen = view.WorldToScreenPoint(matrix.MultiplyPoint3x4(point));
                if (screen.z < view.nearClipPlane || screen.z > view.farClipPlane) return default;
                corners[corner] = screen;
                min = Vector2.Min(min, screen);
                max = Vector2.Max(max, screen);
            }
            return Rect.MinMaxRect(min.x, min.y, max.x, max.y);
        }

        private static bool Clip(float origin, float direction, float min, float max, ref float enter, ref float leave)
        {
            if (direction == 0f) return origin >= min && origin <= max;
            float a = (min - origin) / direction, b = (max - origin) / direction;
            enter = Mathf.Max(enter, Mathf.Min(a, b));
            leave = Mathf.Min(leave, Mathf.Max(a, b));
            return enter <= leave;
        }

        private static bool Intersects(Renderer renderer, Ray ray, out float distance)
        {
            var matrix = renderer.worldToLocalMatrix;
            Vector3 direction = matrix.MultiplyVector(ray.direction);
            var local = new Ray(matrix.MultiplyPoint3x4(ray.origin), direction.normalized);
            var bounds = renderer.localBounds;
            bounds.Expand(0.0001f); // Roundoff at a projected edge.
            bool hit = bounds.IntersectRay(local, out distance);
            distance /= direction.magnitude;
            return hit;
        }
    }
}
