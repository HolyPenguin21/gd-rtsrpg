using UnityEngine;
using UnityEngine.InputSystem;

namespace GameDev.RtsRpg
{
    /// <summary>Owns only the RTS camera pose, input lifetime and optional picking preview.</summary>
    [DisallowMultipleComponent]
    [RequireComponent(typeof(Camera))]
    public sealed class RtsCameraController : MonoBehaviour
    {
        [Header("Input (Camera map in the existing project asset)")]
        [SerializeField] private InputActionAsset inputActions;

        [Header("Pan")]
        [SerializeField, Min(0f)] private float moveSpeed = 15f;
        [Tooltip("Velocity response time in unscaled seconds. Zero means immediate movement.")]
        [SerializeField, Min(0f)] private float moveSmoothTime = 0.12f;
        [Tooltip("Initial ground reference: X = world X, Y = world Z.")]
        [SerializeField] private Vector2 initialPosition = Vector2.zero;
        [SerializeField] private bool useBounds = true;
        [Tooltip("Limits the ground reference, not the edges of the camera frustum.")]
        [SerializeField] private Rect panBounds = new Rect(-25f, -25f, 50f, 50f);

        [Header("Projection")]
        [SerializeField, Range(5f, 85f)] private float tilt = 45f;
        [SerializeField, Min(0.1f)] private float cameraHeight = 25f;
        [SerializeField] private float planeHeight;

        [Header("Zoom")]
        [SerializeField, Min(0.1f)] private float minZoom = 4f;
        [SerializeField, Min(0.1f)] private float maxZoom = 24f;
        [SerializeField, Min(0.1f)] private float initialZoom = 12f;
        [Tooltip("Orthographic size units per raw Input System scroll unit; editable for each mouse/OS.")]
        [SerializeField, Min(0f)] private float zoomSpeed = 0.02f;
        [SerializeField, Min(0f)] private float zoomSmoothTime = 0.12f;

        [Header("Optional manual picking preview")]
        [SerializeField] private bool showCursorMarker = true;
        [SerializeField] private Transform cursorMarker;
        [SerializeField, Min(0f)] private float markerHeightOffset = 0.03f;

        private Camera viewCamera;
        private InputActionMap cameraInput;
        private InputAction pan;
        private InputAction zoom;
        private InputAction drag;
        private InputAction point;
        private Vector2 groundPosition;
        private Vector2 panVelocity;
        private Vector2 previousPointer;
        private float targetZoom;
        private bool wasDragging;

        private void Awake()
        {
            viewCamera = GetComponent<Camera>();
            groundPosition = ClampPosition(initialPosition);
            targetZoom = Mathf.Clamp(initialZoom, minZoom, maxZoom);
            viewCamera.orthographicSize = targetZoom;
            ApplyPose();
            SetMarkerVisible(false);

            var sourceMap = inputActions != null ? inputActions.FindActionMap("Camera") : null;
            if (sourceMap == null)
            {
                Debug.LogError("RTS camera needs the Camera map from InputSystem_Actions.", this);
                enabled = false;
                return;
            }

            // A local copy of this map owns its enable/disable state. Never enable/disable
            // the shared project asset or its template Player/UI maps on behalf of other consumers.
            cameraInput = sourceMap.Clone();
            pan = cameraInput.FindAction("Pan", true);
            zoom = cameraInput.FindAction("Zoom", true);
            drag = cameraInput.FindAction("Drag", true);
            point = cameraInput.FindAction("Point", true);
        }

        private void OnEnable()
        {
            cameraInput?.Enable();
            ResetMotion();
        }

        private void OnDisable()
        {
            cameraInput?.Disable();
            ResetMotion();
            SetMarkerVisible(false);
        }

        private void OnDestroy()
        {
            cameraInput?.Dispose();
        }

        private void OnApplicationFocus(bool hasFocus)
        {
            ResetMotion();
            if (!hasFocus)
                SetMarkerVisible(false);
        }

        private void LateUpdate()
        {
            if (cameraInput == null)
                return;

            float dt = Time.unscaledDeltaTime;
            bool focused = Application.isFocused;
            // Position (0,0) is a valid pixel even when a Value action has no activeControl.
            bool pointerValid = focused && point.controls.Count > 0;
            Vector2 pointerPosition = point.ReadValue<Vector2>();
            pointerValid &= viewCamera.pixelRect.Contains(pointerPosition);

            targetZoom = Mathf.Clamp(targetZoom - (pointerValid ? zoom.ReadValue<float>() : 0f)
                * zoomSpeed, minZoom, maxZoom);
            viewCamera.orthographicSize = Mathf.Clamp(Mathf.Lerp(
                Mathf.Clamp(viewCamera.orthographicSize, minZoom, maxZoom), targetZoom,
                Response(dt, zoomSmoothTime)), minZoom, maxZoom);
            groundPosition = ClampPosition(groundPosition);
            ApplyPose();

            bool dragging = pointerValid && drag.IsPressed();
            if (dragging)
            {
                panVelocity = Vector2.zero;
                // First held frame only establishes a baseline; no accumulated pre-click delta.
                // Both rays use the SAME current camera pose, so zoom/keyboard motion cannot
                // be fed back as pointer movement on the next frame.
                if (wasDragging && CursorWorldProjection.TryGetCursorWorldPoint(
                        viewCamera, previousPointer, planeHeight, out var previousWorld)
                    && CursorWorldProjection.TryGetCursorWorldPoint(
                        viewCamera, pointerPosition, planeHeight, out var currentWorld))
                {
                    Vector3 displacement = previousWorld - currentWorld;
                    groundPosition += new Vector2(displacement.x, displacement.z);
                }
                previousPointer = pointerPosition;
            }
            else
            {
                Vector2 input = focused ? Vector2.ClampMagnitude(pan.ReadValue<Vector2>(), 1f) : Vector2.zero;
                if (input.sqrMagnitude == 0f)
                {
                    // Stop on release rather than letting a smoothed target drift past the field.
                    panVelocity = Vector2.zero;
                }
                else
                {
                    // Fixed yaw = 0: screen right projects onto +X, screen up onto +Z.
                    Vector2 desiredVelocity = input * moveSpeed;
                    float response = Response(dt, moveSmoothTime);
                    // Exact integral of exponential velocity response for this held input.
                    groundPosition += desiredVelocity * dt
                        + (panVelocity - desiredVelocity) * moveSmoothTime * response;
                    panVelocity = Vector2.Lerp(panVelocity, desiredVelocity, response);
                }
            }

            wasDragging = dragging;
            Vector2 clamped = ClampPosition(groundPosition);
            if (clamped.x != groundPosition.x) panVelocity.x = 0f;
            if (clamped.y != groundPosition.y) panVelocity.y = 0f;
            groundPosition = clamped;
            ApplyPose();

            bool validHit = TryGetCursorWorldPoint(out var markerPoint);
            bool show = showCursorMarker && pointerValid && validHit;
            if (show && cursorMarker != null)
                cursorMarker.position = markerPoint + Vector3.up * markerHeightOffset;
            SetMarkerVisible(show);
        }

        /// <summary>Fresh intersection only. False clears the output; no cached hit is exposed.</summary>
        public bool TryGetCursorWorldPoint(out Vector3 worldPoint)
        {
            worldPoint = default;
            return isActiveAndEnabled && Application.isFocused && point != null && point.enabled
                && point.controls.Count > 0 && CursorWorldProjection.TryGetCursorWorldPoint(
                    viewCamera, point.ReadValue<Vector2>(), planeHeight, out worldPoint);
        }

        public void FrameBounds(Bounds bounds)
        {
            if (viewCamera == null) return;
            panBounds = Rect.MinMaxRect(bounds.min.x, bounds.min.z, bounds.max.x, bounds.max.z);
            groundPosition = new Vector2(bounds.center.x, bounds.center.z);
            planeHeight = bounds.center.y;
            float radians = tilt * Mathf.Deg2Rad;
            float projectedHalfHeight = bounds.extents.z * Mathf.Sin(radians)
                + bounds.extents.y * Mathf.Cos(radians);
            targetZoom = maxZoom = Mathf.Max(minZoom, 1.08f * Mathf.Max(projectedHalfHeight,
                bounds.extents.x / Mathf.Max(0.1f, viewCamera.aspect)));
            cameraHeight = Mathf.Max(25f, bounds.extents.y + (bounds.extents.z * 2f + 2f)
                * Mathf.Sin(radians) * Mathf.Cos(radians) + 2f);
            viewCamera.farClipPlane = Mathf.Max(viewCamera.farClipPlane,
                cameraHeight / Mathf.Sin(radians) + bounds.size.magnitude);
            viewCamera.orthographicSize = targetZoom;
            ResetMotion();
            ApplyPose();
        }

        private void ApplyPose()
        {
            viewCamera.orthographic = true;
            transform.SetPositionAndRotation(new Vector3(groundPosition.x, planeHeight + cameraHeight,
                groundPosition.y - cameraHeight / Mathf.Tan(tilt * Mathf.Deg2Rad)),
                Quaternion.Euler(tilt, 0f, 0f));
        }

        private Vector2 ClampPosition(Vector2 position)
        {
            if (!useBounds) return position;
            return new Vector2(Mathf.Clamp(position.x, panBounds.xMin, panBounds.xMax),
                Mathf.Clamp(position.y, panBounds.yMin, panBounds.yMax));
        }

        private static float Response(float dt, float smoothTime)
        {
            return smoothTime <= 0f ? 1f : 1f - Mathf.Exp(-dt / smoothTime);
        }

        private void ResetMotion()
        {
            panVelocity = Vector2.zero;
            wasDragging = false;
        }

        private void SetMarkerVisible(bool visible)
        {
            if (cursorMarker != null && cursorMarker.gameObject.activeSelf != visible)
                cursorMarker.gameObject.SetActive(visible);
        }

        private void OnValidate()
        {
            moveSpeed = Mathf.Max(0f, moveSpeed);
            moveSmoothTime = Mathf.Max(0f, moveSmoothTime);
            tilt = Mathf.Clamp(tilt, 5f, 85f);
            cameraHeight = Mathf.Max(0.1f, cameraHeight);
            minZoom = Mathf.Max(0.1f, minZoom);
            maxZoom = Mathf.Max(minZoom, maxZoom);
            initialZoom = Mathf.Clamp(initialZoom, minZoom, maxZoom);
            zoomSpeed = Mathf.Max(0f, zoomSpeed);
            zoomSmoothTime = Mathf.Max(0f, zoomSmoothTime);
            markerHeightOffset = Mathf.Max(0f, markerHeightOffset);
            panBounds = Rect.MinMaxRect(Mathf.Min(panBounds.xMin, panBounds.xMax),
                Mathf.Min(panBounds.yMin, panBounds.yMax), Mathf.Max(panBounds.xMin, panBounds.xMax),
                Mathf.Max(panBounds.yMin, panBounds.yMax));
        }
    }
}
