using UnityEngine;

namespace GameDev.RtsRpg
{
    /// <summary>Mathematical reference plane picking, independent of input and terrain colliders.</summary>
    public static class CursorWorldProjection
    {
        public static bool TryGetCursorWorldPoint(Camera camera, Vector2 screenPosition,
            float planeHeight, out Vector3 worldPoint)
        {
            worldPoint = default;
            if (camera == null || !Finite(screenPosition.x) || !Finite(screenPosition.y)
                || !Finite(planeHeight) || !camera.pixelRect.Contains(screenPosition))
                return false;

            Ray ray = camera.ScreenPointToRay(screenPosition);
            if (!Finite(ray.direction.y) || Mathf.Abs(ray.direction.y) < 0.000001f)
                return false;

            var plane = new Plane(Vector3.up, new Vector3(0f, planeHeight, 0f));
            if (!plane.Raycast(ray, out float distance) || distance < 0f || !Finite(distance))
                return false;

            Vector3 hit = ray.GetPoint(distance);
            if (!Finite(hit.x) || !Finite(hit.y) || !Finite(hit.z))
                return false;

            worldPoint = hit;
            return true;
        }

        private static bool Finite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);
    }
}
