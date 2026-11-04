using System.Collections.Generic;
using Unity.Profiling;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.InputSystem;
using UnityEngine.Rendering;

namespace GameDev.RtsRpg
{
    [DisallowMultipleComponent]
    [RequireComponent(typeof(TerrainNavigation), typeof(NavigationDemo))]
    [DefaultExecutionOrder(100)]
    public sealed class RtsControls : MonoBehaviour
    {
        private static readonly ProfilerMarker InputMarker = new("RTS.Input");
        private static readonly ProfilerMarker IndicatorsMarker = new("RTS.Indicators");
        private static readonly int BaseColor = Shader.PropertyToID("_BaseColor");
        private const float DragThreshold = 6f;

        [SerializeField] private RtsCameraController terrainCamera;
        [SerializeField] private InputActionAsset inputActions;
        [SerializeField] private Material indicatorMaterial;

        private readonly RtsSelection selection = new();
        private readonly List<RaycastResult> uiHits = new();
        private TerrainNavigation navigation;
        private NavigationDemo demo;
        private RtsOrders orders;
        private Camera view;
        private NavigationGrid grid;
        private int gridRevision, displayedSelectionCount = -1;
        private InputActionMap map;
        private InputAction select, additive, move, attackMove, cancel, pause;
        private EventSystem uiSystem;
        private PointerEventData uiPointer;
        private Mesh ring;
        private MaterialPropertyBlock properties;
        private Vector2 press, pointer;
        private bool captured, rightCaptured, dragging, additivePress, attackArmed, ownsPause;
        private float savedTimeScale, nextReport;
        private string orderReport, selectionReport;

        private Rect Hud => new(Mathf.Max(0, Screen.width - 362), Mathf.Max(0, Screen.height - 102),
            Mathf.Min(350, Screen.width), Mathf.Min(90, Screen.height));

        private void Awake()
        {
            navigation = GetComponent<TerrainNavigation>();
            demo = GetComponent<NavigationDemo>();
            orders = new RtsOrders(demo);
            view = terrainCamera != null ? terrainCamera.GetComponent<Camera>() : null;
            var source = inputActions != null ? inputActions.FindActionMap("RTS") : null;
            if (view == null || source == null)
            {
                Debug.LogError("RTS controls need a scene camera and the RTS input map.", this);
                enabled = false;
                return;
            }

            map = source.Clone();
            select = map.FindAction("Select", true);
            additive = map.FindAction("Additive", true);
            move = map.FindAction("Move", true);
            attackMove = map.FindAction("AttackMove", true);
            cancel = map.FindAction("Cancel", true);
            pause = map.FindAction("Pause", true);
            properties = new MaterialPropertyBlock();
            ring = CreateRing();
        }

        /// <summary>Exposes the pending contact for a future combat consumer.</summary>
        public bool TryGetEngagementIntent(UnitMovement unit, out UnitMovement enemy)
        {
            enemy = null;
            return isActiveAndEnabled && demo != null && demo.TryGetEngagementIntent(unit, out enemy);
        }

        private void OnEnable() => map?.Enable();

        private void OnDisable()
        {
            map?.Disable();
            ResetGesture();
            attackArmed = false;
            selection.Clear();
            orders?.Reset();
            grid = null;
            if (ownsPause) TogglePause();
        }

        private void OnDestroy()
        {
            map?.Dispose();
            if (ring != null) Destroy(ring);
        }

        private void OnApplicationFocus(bool focused)
        {
            ResetGesture();
            attackArmed = false;
        }

        private void LateUpdate()
        {
            if (map == null) return;
            using (InputMarker.Auto())
            {
                if (grid != navigation.Grid)
                {
                    grid = navigation.Grid;
                    ResetGesture();
                    selection.Clear();
                    orders.Reset();
                    attackArmed = false;
                    nextReport = 0f;
                }
                int revision = grid?.Revision ?? 0;
                if (gridRevision != revision)
                {
                    gridRevision = revision;
                    ResetGesture();
                }

                selection.Prune();
                if (Application.isFocused)
                {
                    if (pause.WasPressedThisFrame()) TogglePause();
                    if (cancel.WasPressedThisFrame())
                    {
                        if (attackArmed) attackArmed = false;
                        else selection.Clear();
                        ResetGesture();
                    }
                    else if (grid != null && grid.IsAlive && navigation.HasCurrentTransform)
                        ReadPointer();
                }

                if (displayedSelectionCount != selection.Count)
                {
                    displayedSelectionCount = selection.Count;
                    selectionReport = $"Selected {selection.Count}";
                }
                if (Time.unscaledTime >= nextReport)
                {
                    nextReport = Time.unscaledTime + 0.15f;
                    orderReport = orders.Describe();
                }
            }
            DrawIndicators();
        }

        private void ReadPointer()
        {
            if (!terrainCamera.TryGetCursorRay(out var ray, out pointer))
            {
                ResetGesture();
                return;
            }

            // Idle cursor motion needs no UI raycast and no selection work.
            bool leftPressed = select.WasPressedThisFrame();
            bool rightPressed = move.WasPressedThisFrame();
            bool armPressed = attackMove.WasPressedThisFrame();
            if (!captured && !rightCaptured && !leftPressed && !rightPressed && !armPressed) return;
            if (BlocksPointer(pointer))
            {
                ResetGesture();
                return;
            }

            if (armPressed && selection.Count > 0)
            {
                attackArmed = true;
                ResetGesture();
            }

            if (rightPressed)
            {
                ResetGesture();
                rightCaptured = true;
            }
            if (rightCaptured)
            {
                if (move.WasReleasedThisFrame())
                {
                    if (orders.Issue(RtsOrders.Kind.Move, ray, grid, selection)) attackArmed = false;
                    nextReport = 0f;
                    ResetGesture();
                }
                else if (!move.IsPressed()) ResetGesture();
                return;
            }

            if (leftPressed)
            {
                captured = true;
                dragging = false;
                press = pointer;
                additivePress = additive.IsPressed();
            }
            if (!captured) return;
            dragging |= (pointer - press).sqrMagnitude >= DragThreshold * DragThreshold;
            if (!select.WasReleasedThisFrame())
            {
                if (!select.IsPressed()) ResetGesture();
                return;
            }

            if (attackArmed)
            {
                if (!dragging && orders.Issue(RtsOrders.Kind.AttackMove, ray, grid, selection)) attackArmed = false;
                nextReport = 0f;
            }
            else if (dragging) selection.Box(SelectionRect(), view, navigation, additivePress, BlocksPointer);
            else selection.Click(ray, view, navigation, additivePress);
            ResetGesture();
        }

        internal bool BlocksPointer(Vector2 screen)
        {
            if (!isActiveAndEnabled || map == null) return false;
            if (demo.CoversPointer(screen) || Hud.Contains(ToGui(screen))) return true;
            var system = EventSystem.current;
            if (system == null || !system.isActiveAndEnabled) return false;
            if (uiPointer == null || uiSystem != system)
            {
                uiSystem = system;
                uiPointer = new PointerEventData(system);
            }
            uiPointer.position = screen;
            system.RaycastAll(uiPointer, uiHits);
            foreach (var hit in uiHits)
                if (hit.module is UnityEngine.UI.GraphicRaycaster) return true;
            return false;
        }

        private void TogglePause()
        {
            if (ownsPause)
            {
                if (Time.timeScale == 0f) Time.timeScale = savedTimeScale;
                ownsPause = false;
            }
            else if (Time.timeScale > 0f)
            {
                savedTimeScale = Time.timeScale;
                Time.timeScale = 0f;
                ownsPause = true;
            }
            nextReport = 0f;
        }

        private void ResetGesture() => captured = rightCaptured = dragging = false;

        private Rect SelectionRect() => Rect.MinMaxRect(Mathf.Min(press.x, pointer.x), Mathf.Min(press.y, pointer.y),
            Mathf.Max(press.x, pointer.x), Mathf.Max(press.y, pointer.y));

        private static Vector2 ToGui(Vector2 screen) => new(screen.x, Screen.height - screen.y);

        private void OnGUI()
        {
            if (map == null || Event.current.type != EventType.Repaint) return;
            int depth = GUI.depth;
            GUI.depth = 5; // Draw the marquee behind the diagnostic panel.
            if (captured && dragging && !attackArmed)
            {
                var rect = SelectionRect();
                rect.y = Screen.height - rect.yMax;
                var color = GUI.color;
                GUI.color = new Color(0.30f, 1f, 0.63f, 0.16f);
                GUI.DrawTexture(rect, Texture2D.whiteTexture);
                GUI.color = new Color(0.30f, 1f, 0.63f, 0.95f);
                GUI.DrawTexture(new Rect(rect.x, rect.y, rect.width, 1f), Texture2D.whiteTexture);
                GUI.DrawTexture(new Rect(rect.x, rect.yMax - 1f, rect.width, 1f), Texture2D.whiteTexture);
                GUI.DrawTexture(new Rect(rect.x, rect.y, 1f, rect.height), Texture2D.whiteTexture);
                GUI.DrawTexture(new Rect(rect.xMax - 1f, rect.y, 1f, rect.height), Texture2D.whiteTexture);
                GUI.color = color;
            }
            var hud = Hud;
            GUI.Box(hud, GUIContent.none);
            var style = GUI.skin.label;
            GUI.Label(new Rect(hud.x + 8, hud.y + 3, hud.width - 16, 18),
                attackArmed ? selectionReport + " · Attack-Move armed" : selectionReport, style);
            GUI.Label(new Rect(hud.x + 8, hud.y + 22, hud.width - 16, 41), orderReport,
                GUI.skin.textArea);
            GUI.Label(new Rect(hud.x + 8, hud.y + 66, hud.width - 16, 18),
                orders.Feedback ?? (attackArmed ? "LMB to confirm · Esc cancels"
                    : Time.timeScale == 0f ? (ownsPause ? "Paused · P resumes demo pause" : "Paused")
                    : "LMB / drag · Shift adds · P pauses"), style);
            GUI.depth = depth;
        }

        private void DrawIndicators()
        {
            if (indicatorMaterial == null || grid == null) return;
            using var marker = IndicatorsMarker.Auto();
            properties.SetColor(BaseColor, new Color(0.25f, 1f, 0.65f));
            foreach (var unit in selection.Members)
            {
                var pose = unit.transform;
                var size = pose.lossyScale;
                Vector3 foot = pose.position - Vector3.up * size.y * 0.5f;
                DrawRing(foot, pose.rotation, new Vector3(size.x * 1.4f, 1f, size.z * 1.4f));
            }
            if (!orders.HasOrder) return;
            properties.SetColor(BaseColor, new Color(1f, 0.69f, 0.20f));
            float width = grid.LocalToWorld.MultiplyVector(Vector3.right).magnitude * grid.Surface.BlockWorldSize / 4f;
            DrawRing(orders.Destination, Quaternion.identity, new Vector3(width * 1.8f, 1f, width * 1.8f));
        }

        private void DrawRing(Vector3 position, Quaternion rotation, Vector3 size)
        {
            // Align the shared marker to the same logical surface as movement.
            var normal = grid.SurfaceNormal(position);
            rotation = Quaternion.FromToRotation(Vector3.up, normal) * rotation;
            var matrix = Matrix4x4.TRS(position + normal * 0.10f, rotation, size);
            Graphics.DrawMesh(ring, matrix, indicatorMaterial, gameObject.layer, view, 0, properties,
                ShadowCastingMode.Off, false, null, LightProbeUsage.Off);
        }

        private static Mesh CreateRing()
        {
            const int segments = 32;
            var vertices = new Vector3[segments * 2];
            var indices = new int[segments * 6];
            for (int i = 0; i < segments; i++)
            {
                float angle = i * 2f * Mathf.PI / segments;
                var direction = new Vector3(Mathf.Sin(angle), 0f, Mathf.Cos(angle));
                int inner = i * 2, next = (inner + 2) % vertices.Length;
                vertices[inner] = direction * 0.43f;
                vertices[inner + 1] = direction * 0.50f;
                int triangle = i * 6;
                indices[triangle] = inner;
                indices[triangle + 1] = inner + 1;
                indices[triangle + 2] = next + 1;
                indices[triangle + 3] = inner;
                indices[triangle + 4] = next + 1;
                indices[triangle + 5] = next;
            }
            return new Mesh { name = "RTS order and selection ring", vertices = vertices, triangles = indices };
        }
    }
}
