using System;
using System.Collections.Generic;
using Unity.Profiling;
using Clock = System.Diagnostics.Stopwatch;
using UnityEngine;

namespace GameDev.RtsRpg
{
    /// <summary>Manual terrain, navigation and movement scenarios. No RTS commands.</summary>
    [DisallowMultipleComponent]
    [RequireComponent(typeof(TerrainNavigation))]
    public sealed class NavigationDemo : MonoBehaviour
    {
        [SerializeField] private Material lineMaterial;
        [SerializeField] private Vector2Int start = new Vector2Int(1, 1);
        [SerializeField] private Vector2Int goal = new Vector2Int(20, 10);
        [SerializeField] private Vector2Int footprint = Vector2Int.one;
        [SerializeField] private TerrainDirection startFacing, goalFacing;
        [SerializeField] private bool allowPartial;
        [SerializeField] private int maxExpanded = 20000;
        [SerializeField, Range(0, 32)] private int groupRadius = 12;
        [SerializeField] private bool showBoundaries;
        [SerializeField] private RtsCameraController terrainCamera;
        [SerializeField, Range(256, 24000)] private int expansionsPerFrame = 6000;
        [SerializeField, Range(1, 12)] private int requestsPerFrame = 5;
        [SerializeField, Min(0.2f)] private float followInterval = 0.28f;
        [SerializeField, Min(1)] private int followDistance = 2;

        [SerializeField, Range(0.25f, 8f)] private float planningMilliseconds = 1.5f;
        private static readonly ProfilerMarker PlanningMarker = new("Navigation.PlanningFrame");
        private static readonly ProfilerMarker HoverMarker = new("Navigation.SampleFollowTarget");
        private static readonly ProfilerMarker GuideMarker = new("Navigation.AdvanceGuide");
        private static readonly ProfilerMarker ParticipantMarker = new("Navigation.AdvanceParticipant");
        private static readonly ProfilerMarker GuiMarker = new("Navigation.OnGUI");

        private sealed class Participant
        {
            public UnitMovement Unit;
            public NavigationFootprint Footprint;
            public NavigationPose? Destination;
            public NavigationStatus? LastAttempt;
            public int Failures, LastTrafficRevision, NextCandidate;
            public float RetryAt, StartDelay = -1f;
            public int MotionAtOrder, TurnsAtOrder, StopsAtOrder;
            public Vector2 SlotBias;
            public NavigationPath SlotRoute;
            public bool Reserved, Assigned, LaunchChecked;
        }

        private sealed class ParticipantPlan
        {
            public Participant Entry;
            public NavigationPathfinder.Search Search;
            public NavigationPathfinder.ApproachField Field;
            public List<NavigationPoint> Prefix;
            public NavigationPose Start, Approach, Goal;
            public NavigationGroupPlanner.TargetFlood Area;
            public Vector2Int SlotCentre, TargetAtStart;
            public Vector2 PreferredSlot;
            public readonly List<NavigationPose> Candidates = new(16);
            public NavigationPath Suffix, ReuseRoute;
            public int ReuseIndex, ReuseScan, ReuseEnd = -1;
            public int ScoringIndex, BestCandidate = -1;
            public float BestScore = float.PositiveInfinity;
            public bool Scoring;
            public Func<NavigationPose, NavigationPose, bool> AllowsEdge;
            public int Tried, Remaining, TrafficRevision, CandidateIndex, PrefixCost, SuffixIndex;
            public int FieldLimit = 8192;
            public int ApproachPointCount;
            public bool NeedsSlot, HasCandidate, LocalSearch, StartsInside, ApproachReady;
        }

        private readonly List<Participant> units = new List<Participant>();
        private readonly HashSet<Vector2Int> reservations = new();
        private readonly NavigationPathfinder.ApproachField[] fields = new NavigationPathfinder.ApproachField[3];
        private readonly NavigationBudget frameWork = new(0);
        private NavigationGroupPlanner.TargetFlood targetFlood, acceptedArea, fieldArea;
        private List<Vector2Int> targetArea;
        private NavigationGrid sessionGrid;
        private NavigationGroupPlanner planner;
        private NavigationPathfinder finder;
        private Vector2Int? acceptedTarget, requestedTarget, lastRejectedTarget;
        private Vector2Int? hoverTarget;
        private Vector2Int guideTarget;
        private bool launchInitialOrder;
        private bool followMouse, guideFromHover;
        private NavigationPathfinder.Search guideSearch;
        private const int ConcurrentPlans = 3;
        private readonly List<ParticipantPlan> participantPlans = new(ConcurrentPlans);
        private ParticipantPlan participantPlan;
        private int guideExpanded, nextHoverGuideFrame;
        private float nextFollow;
        private int planIndex, targetExpanded, frameExpanded, launchCursor;
        private float acceptedAt, orderAt, guideStartedAt;
        private bool guideLocal;
        private readonly List<float> startDelays = new();
        private readonly float[] frameTimes = new float[300], sortedFrameTimes = new float[300];
        private int frameTimeCursor, frameTimeCount;
        private long totalExpanded;
        private double lastPlanningMs, sumPlanningMs, peakPlanningMs;
        private long lastPlanningBytes, sumPlanningBytes, peakPlanningBytes;
        private int planningFrames, assignments, sharedAssignments, fullSearches;
        private string trafficReport, planningReport, requestReport;
        private float nextReport, nextRequestReport;
        private string targetReport;
        private Rect Panel => new Rect(12, 12, 420, Math.Min(Screen.height - 24, 560));
        private bool movementDemo, armyDemo;
        private bool blockerEnabled;
        private NavigationGrid blockerGrid;
        [SerializeField] private bool showPanel = true;
        [SerializeField] private RectInt blocker = new RectInt(12, 4, 4, 4);
        private TerrainNavigation navigation;
        private TerrainBuilder builder;
        private readonly List<NavigationPath> paths = new List<NavigationPath>();
        private readonly List<GameObject> lines = new List<GameObject>();
        private Material ownedMaterial;
        private Mesh mountedPreview;
        private int blockerHandle;
        private NavigationGrid laneGrid;
        private UnitMovement sealedLaneGuard;
        private readonly List<int> laneHandles = new();
        private bool groupMode;
        private string report = "Choose a demo or edit the request in Inspector, then Replan.";
        private Vector2 scroll;
        private NavigationGrid Grid => navigation != null ? navigation.Grid : null;

        private void OnEnable()
        {
            navigation = GetComponent<TerrainNavigation>(); builder = GetComponent<TerrainBuilder>();
            navigation.Changed += Invalidate;
            builder.SurfaceChanged += FrameTerrain;
            if (lineMaterial != null) ownedMaterial = new Material(lineMaterial) { hideFlags = HideFlags.DontSave };
        }
        private void Start() => FrameTerrain();
        private void FrameTerrain()
        {
            if (builder.Surface != null && terrainCamera != null)
                terrainCamera.FrameBounds(GetComponent<MeshRenderer>().bounds);
        }
        private void OnDisable()
        {
            if (navigation != null) navigation.Changed -= Invalidate;
            if (builder != null) builder.SurfaceChanged -= FrameTerrain;
            ClearUnits(); ClearLane(); ClearLines(); paths.Clear();
            if (blockerGrid != null && blockerHandle != 0) blockerGrid.RemoveBlocker(blockerHandle);
            blockerHandle = 0;
            blockerGrid = null;
            if (ownedMaterial != null) Release(ownedMaterial);
            ownedMaterial = null;
            if (mountedPreview != null) Release(mountedPreview);
            mountedPreview = null;
        }
        private static void Release(UnityEngine.Object obj)
        { if (Application.isPlaying) Destroy(obj); else DestroyImmediate(obj); }
        private void ClearLines() { foreach (var line in lines) if (line != null) Release(line); lines.Clear(); }
        private void Invalidate()
        {
            paths.Clear(); ClearLines();
            followMouse = false;
            requestedTarget = hoverTarget = lastRejectedTarget = null;
            CancelGuide();
            CancelParticipantPlans();
            if (sessionGrid != null && sessionGrid != Grid)
            {
                ClearUnits();
                movementDemo = armyDemo = groupMode = false;
            }
            if (laneGrid != null && laneGrid != Grid) ClearLane();
            if (sessionGrid == Grid)
                foreach (var entry in units) { entry.RetryAt = 0; entry.Failures = entry.NextCandidate = 0; entry.LastAttempt = null; }
            if (acceptedTarget.HasValue && sessionGrid == Grid && Grid != null)
            {
                requestedTarget = acceptedTarget;
                ClearFields();
                targetArea = null;
                acceptedArea = fieldArea = null;
            }
            // Handles belong to the old grid only. Never remove an old handle from a new map.
            if (Grid == null || !Grid.IsAlive) { blockerHandle = 0; blockerGrid = null; }
            else if (blockerEnabled && blockerGrid != Grid)
            {
                blockerGrid = Grid;
                if (!Grid.TryAddBlocker(blocker, out blockerHandle, out string error))
                    Debug.LogWarning(error, this);
            }
            report = "Navigation changed. Old paths invalidated.";
        }
        private void Line(IReadOnlyList<Vector3> points, Color color, float width = 0.07f)
        {
            if (ownedMaterial == null || points.Count < 2) return;
            var go = new GameObject("Navigation debug line") { hideFlags = HideFlags.DontSave };
            go.transform.SetParent(transform, false); lines.Add(go);
            var renderer = go.AddComponent<LineRenderer>();
            renderer.sharedMaterial = ownedMaterial; renderer.useWorldSpace = true;
            renderer.widthMultiplier = width; renderer.positionCount = points.Count;
            renderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            renderer.receiveShadows = false;
            var properties = new MaterialPropertyBlock(); properties.SetColor("_BaseColor", color);
            renderer.SetPropertyBlock(properties);
            for (int i = 0; i < points.Count; i++) renderer.SetPosition(i, points[i] + Vector3.up * 0.12f);
        }
        private void Outline(NavigationPose pose, NavigationFootprint f, Color color)
        {
            var s = f.Size(pose.Facing);
            // At a cliff, sampling the exact upper/right boundary selects the OUTSIDE cell.
            // Keep the debug outline just inside its own footprint, on its own surface.
            Vector3 Sample(float x, float z) => Grid.WorldPoint(
                Mathf.Clamp(x, pose.Cell.x + 0.002f, pose.Cell.x + s.x - 0.002f),
                Mathf.Clamp(z, pose.Cell.y + 0.002f, pose.Cell.y + s.y - 0.002f));
            // Each side sampled at half-cell intervals to follow ramps rather than draw through them.
            var points = new List<Vector3>();
            for (int i = 0; i <= s.x * 2; i++) points.Add(Sample(pose.Cell.x + i * 0.5f, pose.Cell.y));
            for (int i = 1; i <= s.y * 2; i++) points.Add(Sample(pose.Cell.x + s.x, pose.Cell.y + i * 0.5f));
            for (int i = s.x * 2 - 1; i >= 0; i--) points.Add(Sample(pose.Cell.x + i * 0.5f, pose.Cell.y + s.y));
            for (int i = s.y * 2 - 1; i >= 0; i--) points.Add(Sample(pose.Cell.x, pose.Cell.y + i * 0.5f));
            Line(points, color);
        }
        private void ClearUnits()
        {
            CancelGuide();
            CancelParticipantPlans();
            foreach (var entry in units)
            {
                if (entry.Unit == null) continue;
                entry.Unit.gameObject.SetActive(false);
                Release(entry.Unit.gameObject);
            }
            units.Clear();
            followMouse = false;
            nextFollow = 0;
            planIndex = targetExpanded = frameExpanded = nextHoverGuideFrame = launchCursor = 0;
            acceptedAt = orderAt = 0f;
            frameTimeCursor = frameTimeCount = 0;
            totalExpanded = 0;
            lastPlanningMs = sumPlanningMs = peakPlanningMs = 0;
            lastPlanningBytes = sumPlanningBytes = peakPlanningBytes = 0;
            planningFrames = assignments = sharedAssignments = fullSearches = 0;
            targetReport = trafficReport = planningReport = null;
            nextReport = 0;
            reservations.Clear();
            ClearFields();
            acceptedTarget = requestedTarget = hoverTarget = lastRejectedTarget = null;
            targetArea = null;
            acceptedArea = fieldArea = null;
            launchInitialOrder = false;
            sessionGrid = null;
            planner = null;
            finder = null;
        }

        private void DrawPath(NavigationPath path, NavigationFootprint f, Color color)
        {
            paths.Add(path);
            if (movementDemo)
            {
                Spawn(path, f, color);
                return;
            }
            if (!path.IsCurrent || path.Points.Count == 0) return;
            var points = new List<Vector3>();
            for (int i = 0; i < path.Points.Count; i++)
            {
                var p = path.Points[i];
                // Sample each centre segment so slope changes remain on the logical top.
                if (i > 0)
                {
                    Vector2Int a = f.Size(path.Points[i - 1].Pose.Facing), b = f.Size(p.Pose.Facing);
                    Vector2 prev = (Vector2)path.Points[i - 1].Pose.Cell + (Vector2)a * 0.5f;
                    Vector2 next = (Vector2)p.Pose.Cell + (Vector2)b * 0.5f;
                    for (int j = 1; j < 4; j++)
                    {
                        Vector2 q = Vector2.Lerp(prev, next, j * 0.25f);
                        points.Add(Grid.WorldPoint(q.x, q.y));
                    }
                }
                points.Add(p.World);
                if (i == 0 || i == path.Points.Count - 1 || p.Pose.Facing != path.Points[i - 1].Pose.Facing)
                    Outline(p.Pose, f, color);
            }
            Line(points, color, 0.11f);
        }
        private UnitMovement Spawn(NavigationPath path, NavigationFootprint footprint, Color color,
            int team = 0, bool hold = false)
        {
            if (!path.IsCurrent || path.Points.Count == 0) return null;
            var visual = GameObject.CreatePrimitive(PrimitiveType.Cube);
            visual.hideFlags = HideFlags.DontSave;
            visual.name = footprint.NeedsTurns ? "Demo mounted unit (1x2)" : "Demo unit";
            visual.GetComponent<Collider>().enabled = false;
            if (footprint.NeedsTurns)
                visual.GetComponent<MeshFilter>().sharedMesh = mountedPreview ??= BuildMountedPreview();
            var renderer = visual.GetComponent<Renderer>();
            if (ownedMaterial != null) renderer.sharedMaterial = ownedMaterial;
            var properties = new MaterialPropertyBlock();
            properties.SetColor("_BaseColor", color);
            renderer.SetPropertyBlock(properties);
            var unit = visual.AddComponent<UnitMovement>();
            if (unit.Begin(navigation, path, footprint, team))
            {
                if (sessionGrid != Grid)
                {
                    sessionGrid = Grid;
                    finder = new NavigationPathfinder(Grid);
                    planner = new NavigationGroupPlanner(Grid);
                }
                units.Add(new Participant { Unit = unit, Footprint = footprint,
                    Destination = hold || path.Status == NavigationStatus.Partial
                        ? null : path.Points[path.Points.Count - 1].Pose });
                return unit;
            }
            visual.SetActive(false);
            Release(visual);
            return null;
        }

        private static Mesh BuildMountedPreview()
        {
            // Shared debug silhouette with a visible front on local +Z.
            var mesh = new Mesh { name = "Mounted unit preview", hideFlags = HideFlags.DontSave };
            mesh.vertices = new[]
            {
                new Vector3(-0.5f, -0.5f, -0.5f), new Vector3(0.5f, -0.5f, -0.5f),
                new Vector3(-0.5f, 0.5f, -0.5f), new Vector3(0.5f, 0.5f, -0.5f),
                new Vector3(-0.275f, -0.5f, 0.5f), new Vector3(0.275f, -0.5f, 0.5f),
                new Vector3(-0.275f, 0.5f, 0.5f), new Vector3(0.275f, 0.5f, 0.5f)
            };
            mesh.triangles = new[]
            {
                0, 2, 3, 0, 3, 1, 4, 5, 7, 4, 7, 6,
                0, 4, 6, 0, 6, 2, 1, 3, 7, 1, 7, 5,
                0, 1, 5, 0, 5, 4, 2, 6, 7, 2, 7, 3
            };
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
            return mesh;
        }

        private void DrawGeometry()
        {
            if (Grid == null) return;
            var surface = Grid.Surface;
            int displayed = 0;
            if (showBoundaries)
            for (int z = 0; z < surface.CellsZ && displayed < 192; z++)
            for (int x = 0; x < surface.CellsX && displayed < 192; x++)
            for (int d = 0; d < 4; d++)
            {
                if (displayed >= 192) break;
                var side = (TerrainDirection)d;
                // Show discontinuities on BOTH heights, so upper/lower routes remain inspectable.
                if (!surface.IsBoundaryBlocked(x, z, side)) continue;
                surface.GetEdge(x, z, side, out int a, out int b);
                int ax = x, az = z, bx = x, bz = z;
                switch (side)
                {
                    case TerrainDirection.North: az++; bx++; bz++; break;
                    case TerrainDirection.East: ax++; bx++; bz++; break;
                    case TerrainDirection.South: bx++; break;
                    case TerrainDirection.West: bz++; break;
                }
                Line(new[] { Grid.LocalToWorld.MultiplyPoint3x4(surface.Position(ax, az, a)),
                    Grid.LocalToWorld.MultiplyPoint3x4(surface.Position(bx, bz, b)) }, new Color(1, 0.22f, 0.12f), 0.035f);
                displayed++;
            }
            foreach (var r in Grid.Blockers)
            {
                // A visible cross per blocked cell; demo only, no collider or building object.
                for (int z = r.y; z < r.yMax; z++)
                for (int x = r.x; x < r.xMax; x++)
                {
                    Line(new[] { Grid.WorldPoint(x + 0.15f, z + 0.15f), Grid.WorldPoint(x + 0.85f, z + 0.85f) }, Color.magenta);
                    Line(new[] { Grid.WorldPoint(x + 0.85f, z + 0.15f), Grid.WorldPoint(x + 0.15f, z + 0.85f) }, Color.magenta);
                }
            }
        }

        [ContextMenu("Replan Demo Request")]
        public void Replan()
        {
            if (movementDemo && armyDemo && Grid != null)
            {
                if (sessionGrid == Grid && units.Count > 0) requestedTarget = goal;
                else Army();
                return;
            }
            ClearUnits(); ClearLane(); ClearLines(); paths.Clear();
            if (Grid == null) { report = navigation != null ? navigation.Error : "Navigation unavailable."; return; }
            DrawGeometry();
            if (groupMode) { PlanGroup(); return; }
            var f = new NavigationFootprint(footprint.x, footprint.y);
            var clock = System.Diagnostics.Stopwatch.StartNew();
            int stateCount = Grid.Surface.CellsX * Grid.Surface.CellsZ * f.Orientations;
            var path = new NavigationPathfinder(Grid).Find(new NavigationRequest(new NavigationPose(start, startFacing),
                new NavigationPose(goal, goalFacing), f, movementDemo ? stateCount : maxExpanded, allowPartial));
            double searchMs = clock.Elapsed.TotalMilliseconds;
            DrawPath(path, f, Color.cyan);
            report = $"{path.Status}: {path.Reason}\nExpanded {path.Expanded}; cost {path.Cost}; points {path.Points.Count}; revision {path.Revision}; {searchMs:F1} ms";
        }
        private void PlanGroup()
        {
            var small = new NavigationFootprint(1, 1); var large = new NavigationFootprint(2, 2);
            var members = new[] {
                new NavigationMember(new NavigationPose(start), small),
                new NavigationMember(new NavigationPose(start + Vector2Int.right), small),
                new NavigationMember(new NavigationPose(start + Vector2Int.up), small),
                new NavigationMember(new NavigationPose(start + new Vector2Int(2, 0)), large) };
            var clock = System.Diagnostics.Stopwatch.StartNew();
            var result = new NavigationGroupPlanner(Grid).Plan(members, goal, groupRadius,
                80000, 8000);
            report = $"Group {result.Status}: {result.Accepted}/{members.Length} assigned; {result.Expanded} expansions; shared guide {result.Corridor.Count} cells; {clock.Elapsed.TotalMilliseconds:F1} ms";
            for (int i = 0; i < members.Length; i++)
            {
                report += $"\n{i}: {result.Paths[i].Status} — {result.Paths[i].Reason}";
                DrawPath(result.Paths[i], members[i].Footprint, Color.HSVToRGB(i * 0.21f, 0.85f, 1));
            }
        }
        private void ExplicitScenario(int mode)
        {
            builder.ShowExplicit();
            if (Grid == null) return; movementDemo = armyDemo = groupMode = false; groupRadius = 12;
            startFacing = goalFacing = TerrainDirection.North; footprint = Vector2Int.one;
            if (mode == 0) { start = new Vector2Int(1, 1); goal = new Vector2Int(20, 10); }
            if (mode == 1)
            {
                start = new Vector2Int(5, 1); goal = new Vector2Int(5, 9);
                // Close the OTHER ramp; otherwise a large agent correctly takes the wide detour.
                Grid.TryAddBlocker(new RectInt(12, 4, 4, 4), out _, out _);
            }
            if (mode == 2) { start = new Vector2Int(12, 1); goal = new Vector2Int(14, 8); groupMode = true; }
            if (mode == 3)
            {
                start = new Vector2Int(1, 1); goal = new Vector2Int(2, 2);
                Grid.TryAddBlocker(new RectInt(2, 1, 1, 1), out _, out _);
                Grid.TryAddBlocker(new RectInt(1, 2, 1, 1), out _, out _);
            }
            if (mode == 4)
            {
                start = new Vector2Int(9, 1); goal = start; footprint = new Vector2Int(1, 2);
                goalFacing = TerrainDirection.East;
                movementDemo = true;
            }
            Replan();
        }
        private void ToggleBlocker()
        {
            if (Grid == null) return;
            if (blockerHandle != 0)
            {
                blockerEnabled = false; Grid.RemoveBlocker(blockerHandle); blockerHandle = 0;
                DrawGeometry(); report = "Blocker removed; old paths invalidated. Press Replan.";
            }
            else
            {
                var area = new Rect(blocker.x, blocker.y, blocker.width, blocker.height);
                foreach (var entry in units)
                    if (entry.Unit != null && entry.Unit.Reserves(area))
                    {
                        report = "Wait for units to clear the blocker area.";
                        return;
                    }
                if (!Grid.TryAddBlocker(blocker, out int handle, out string error)) { report = error; return; }
                blockerHandle = handle;
                blockerEnabled = true; blockerGrid = Grid;
                DrawGeometry(); report = "Blocker added; old paths invalidated. Press Replan.";
            }
        }
        private void Seed(int seed)
        {
            ClearUnits(); ClearLane();
            movementDemo = armyDemo = groupMode = false;
            builder.ShowLandformSeed(seed);
            report = builder.BuildReport ?? "Terrain generation failed. See Console.";
        }

        private void Landmark(int index)
        {
            if (Grid == null || Grid.Surface.Landmarks.Count != 5) return;
            movementDemo = true;
            armyDemo = groupMode = false;
            start = new Vector2Int(5, 5);
            goal = Grid.Surface.Landmarks[index] * 4 + Vector2Int.one;
            Replan();
        }

        private void Army(int count = 64)
        {
            if (Grid == null || Grid.Surface.Landmarks.Count != 5) return;
            ClearUnits(); ClearLane(); ClearLines(); paths.Clear();
            movementDemo = armyDemo = groupMode = true;
            start = new Vector2Int(5, 5);
            goal = new Vector2Int(Grid.Surface.CellsX / 2, 16);
            groupRadius = 20;
            var spawnFinder = new NavigationPathfinder(Grid);
            // Keep the entire mixed spawn on level zero, including its footprint.
            // Skip local cliffs/blocked cells and extend the formation across the plain.
            for (int z = 5; z < 29 && units.Count < count; z += 3)
            for (int x = 5; x < 65 && units.Count < count; x += 3)
            {
                int i = units.Count;
                var anchor = new NavigationPose(new Vector2Int(x, z));
                var size = i % 3 == 0 ? new NavigationFootprint(2, 2)
                    : i % 3 == 1 ? new NavigationFootprint(1, 2) : new NavigationFootprint(1, 1);
                if (!Grid.CanOccupy(anchor, size)) continue;
                var span = size.Size(anchor.Facing);
                bool onPlain = true;
                for (int dz = 0; dz < span.y; dz++)
                for (int dx = 0; dx < span.x; dx++)
                {
                    var cell = Grid.Surface.GetCell(x + dx, z + dz);
                    onPlain &= cell.Level == 0 && !cell.IsRamp;
                }
                if (!onPlain) continue;
                Spawn(spawnFinder.Find(new NavigationRequest(anchor, anchor, size, 1)), size,
                    Color.HSVToRGB(i * 0.21f, 0.85f, 1), hold: true);
            }
            requestedTarget = goal;
            launchInitialOrder = true;
            orderAt = Time.time;
            report = $"Army spawned: {units.Count}/{count}. Enable Follow Mouse to steer.";
        }

        private void ClearLane()
        {
            if (laneGrid != null && laneGrid.IsAlive)
                foreach (int handle in laneHandles) laneGrid.RemoveBlocker(handle);
            laneHandles.Clear();
            laneGrid = null;
            sealedLaneGuard = null;
        }

        private void Traffic(int scenario)
        {
            if (Grid == null || Grid.Surface.Landmarks.Count != 5) return;
            ClearUnits(); ClearLane(); ClearLines(); paths.Clear();
            movementDemo = true;
            armyDemo = groupMode = false;
            if (scenario == 6)
            {
                laneGrid = Grid;
                AddWall(new RectInt(6, 0, 14, 5));
                AddWall(new RectInt(6, 6, 14, 25));
                DrawGeometry();
            }
            else if (scenario == 7)
            {
                laneGrid = Grid;
                AddWall(new RectInt(6, 0, 10, 6));
                AddWall(new RectInt(6, 7, 9, 21));
                AddWall(new RectInt(15, 17, 7, 11));
                AddWall(new RectInt(16, 0, 6, 16));
                DrawGeometry();
            }
            else if (scenario == 8)
            {
                laneGrid = Grid;
                AddWall(new RectInt(6, 4, 8, 6));
                AddWall(new RectInt(15, 4, 7, 6));
                AddWall(new RectInt(6, 11, 16, 19));
                DrawGeometry();
            }
            else if (scenario >= 2 && scenario <= 4)
            {
                laneGrid = Grid;
                AddWall(new RectInt(2, 4, 22, 1));
                // A gap in the north wall provides a passing bay and a possible bypass.
                AddWall(new RectInt(2, 6, scenario == 4 ? 22 : 9, 1));
                if (scenario == 4) AddWall(new RectInt(2, 5, 1, 1));
                else AddWall(new RectInt(14, 6, 10, 1));
                DrawGeometry();
            }
            var size = new NavigationFootprint(1, 1);
            var laneFinder = new NavigationPathfinder(Grid);
            if (scenario >= 5)
            {
                int west = scenario == 7 ? 6 : scenario == 8 ? 10 : 5;
                int east = scenario == 7 ? 16 : west;
                for (int i = 0; i < 3; i++)
                {
                    AddUnit(4 - i, 24 + i, Color.cyan, startZ: west + i - 1, endZ: east + 2 * i - 2);
                    AddUnit(23 + i, 1 + i, Color.yellow, startZ: east + i - 1, endZ: west + 2 * i - 2);
                }
                if (scenario == 8)
                {
                    AddUnit(14, 27, Color.green, startZ: 1, endZ: 10);
                    AddUnit(13, 2, Color.green, startZ: 2, endZ: 12);
                }
            }
            else if (scenario == 2)
                for (int x = 9; x <= 12; x++) AddUnit(x, x, Color.green, hold: true);
            else if (scenario == 3)
                for (int i = 0; i < 3; i++)
                {
                    AddUnit(5 + 2 * i, 26 + i, Color.cyan);
                    AddUnit(22 - 2 * i, 1, Color.yellow, endZ: 3 + 2 * i);
                }
            else
            {
                var guard = AddUnit(13, 13, scenario == 0 ? Color.green : Color.red, scenario == 0 ? 0 : 1, true);
                if (scenario == 4) sealedLaneGuard = guard;
            }
            if (scenario < 5 && scenario != 3) AddUnit(5, 26, Color.cyan);
            report = scenario switch
            {
                0 => "Stopped ally: watch it yield.",
                1 => "Enemy holds the lane: watch for a detour.",
                2 => "Allied chain: watch the yielding move propagate.",
                3 => "Opposing traffic: watch both flows clear the corridor.",
                4 => "Enemy seals the only exit. Release lane to remove the obstruction.",
                5 => "Open crossing: both groups have room to pass on either side.",
                6 => "One-lane crossing: groups start outside the passage.",
                7 => "Bent passage: wait for the opposing stream to clear both bends.",
                _ => "Junction: three streams share the crossing."
            };

            void AddWall(RectInt wall)
            {
                if (Grid.TryAddBlocker(wall, out int handle, out _)) laneHandles.Add(handle);
            }
            UnitMovement AddUnit(int from, int to, Color color, int team = 0, bool hold = false, int endZ = 5, int startZ = 5)
            {
                return Spawn(laneFinder.Find(new NavigationRequest(new NavigationPose(new Vector2Int(from, startZ)),
                    new NavigationPose(new Vector2Int(to, endZ)), size)), size, color, team, hold);
            }
        }

        private void MountedRamp(bool angled)
        {
            if (Grid == null || Grid.Surface.Landmarks.Count != 5) return;
            var hill = Grid.Surface.Landmarks[2] * 4;
            var size = new NavigationFootprint(1, 2);
            for (int z = Mathf.Max(0, hill.y - 32); z < Mathf.Min(Grid.Surface.CellsZ, hill.y + 33); z++)
            for (int x = Mathf.Max(0, hill.x - 32); x < Mathf.Min(Grid.Surface.CellsX, hill.x + 33); x++)
            {
                var ramp = Grid.Surface.GetCell(x, z);
                if (!ramp.IsRamp || ramp.Level != 0) continue;
                var entry = new Vector2Int(x, z);
                var forward = TerrainSurface.Step(ramp.Uphill);
                var below = entry - forward;
                if (Grid.Surface.Contains(below.x, below.y) && Grid.Surface.GetCell(below.x, below.y).RampIndex == ramp.RampIndex) continue;
                var aligned = entry - forward * 4;
                var exit = entry + forward * 7;
                var from = new NavigationPose(aligned, ramp.Uphill);
                bool clear = true;
                for (int i = 1; i <= 11; i++)
                {
                    var to = new NavigationPose(aligned + forward * i, ramp.Uphill);
                    if (!Grid.CanStep(from, to, size)) { clear = false; break; }
                    from = to;
                }
                if (!clear) continue;

                var spawn = aligned;
                var heading = ramp.Uphill;
                if (angled)
                {
                    var side = new Vector2Int(forward.y, -forward.x);
                    heading = (TerrainDirection)(((int)ramp.Uphill + 1) & 3);
                    spawn += side * 3;
                    if (!Grid.CanOccupy(new NavigationPose(spawn, heading), size)) continue;
                }
                movementDemo = true;
                groupMode = armyDemo = false;
                footprint = new Vector2Int(1, 2);
                start = spawn; goal = exit;
                startFacing = heading; goalFacing = ramp.Uphill;
                Replan();
                return;
            }
            report = "No mounted approach on this hill fits the current terrain or blockers.";
        }

        private void ReleaseLane()
        {
            if (sealedLaneGuard == null) return;
            var guard = sealedLaneGuard.gameObject;
            sealedLaneGuard = null;
            guard.SetActive(false);
            Release(guard);
            report = "Guard removed; the same corridor is open.";
        }

        private void Update()
        {
            if (units.Count == 0 || Grid != sessionGrid || !navigation.HasCurrentTransform) return;
            if (!Application.isFocused || Time.deltaTime <= 0f)
            {
                CancelHoverTarget();
                if (GuideActive) { requestedTarget = guideTarget; CancelGuide(); }
                CancelParticipantPlans();
                nextFollow = 0f;
                return;
            }
            for (int i = units.Count - 1; i >= 0; i--)
            {
                var entry = units[i];
                if (entry.Unit != null && entry.Unit.isActiveAndEnabled) continue;
                for (int j = participantPlans.Count - 1; j >= 0; j--)
                    if (participantPlans[j].Entry == entry)
                    {
                        CancelParticipantPlan(participantPlans[j]);
                        participantPlans.RemoveAt(j);
                    }
                if (entry.Reserved) NavigationGroupPlanner.Reserve(entry.Destination.Value, entry.Footprint, reservations, false);
                units.RemoveAt(i);
                if (i < launchCursor) launchCursor--;
            }
            if (units.Count == 0) { ClearUnits(); return; }
            planIndex %= units.Count;

            float now = Time.time;
            frameTimes[frameTimeCursor] = Time.unscaledDeltaTime * 1000f;
            frameTimeCursor = (frameTimeCursor + 1) % frameTimes.Length;
            frameTimeCount = Mathf.Min(frameTimes.Length, frameTimeCount + 1);
            int frameBudget = Mathf.Clamp(expansionsPerFrame, 256, 24000);
            int requests = Mathf.Clamp(requestsPerFrame, 1, 12);
            long started = Clock.GetTimestamp();
            long allocated = GC.GetAllocatedBytesForCurrentThread();
            long duration = (long)(Mathf.Clamp(planningMilliseconds, 0.25f, 8f) * Clock.Frequency / 1000.0);
            frameWork.Reset(frameBudget, started + duration);
            using (PlanningMarker.Auto())
            {
                if (followMouse) SampleFollowTarget(now);
                if (launchInitialOrder && launchCursor < units.Count)
                    LaunchParticipants(frameWork, goal);
                AdvanceGuide(frameWork);
                if (acceptedTarget.HasValue) LaunchParticipants(frameWork, acceptedTarget.Value);
                while (frameWork.CanContinue && requests > 0 && launchCursor >= (targetArea == null ? 0 : units.Count))
                {
                    if (participantPlans.Count < ConcurrentPlans && StartParticipantPlan(now))
                    {
                        participantPlans.Add(participantPlan);
                        participantPlan = null;
                    }
                    if (participantPlans.Count == 0) break;
                    participantPlan = participantPlans[0];
                    participantPlans.RemoveAt(0);
                    frameWork.BeginSlice(duration / ConcurrentPlans);
                    bool done = AdvanceParticipantPlan(frameWork, ref requests);
                    frameWork.EndSlice();
                    if (!done) participantPlans.Add(participantPlan);
                    participantPlan = null;
                }
            }
            if (armyDemo && (acceptedTarget.HasValue || launchInitialOrder))
                foreach (var entry in units)
                    if (entry.StartDelay < 0f && entry.Unit.MotionTicks > entry.MotionAtOrder)
                        entry.StartDelay = now - orderAt;
            frameExpanded = frameBudget - frameWork.Remaining;
            totalExpanded += frameExpanded;
            targetExpanded += frameExpanded;
            lastPlanningMs = (Clock.GetTimestamp() - started) * 1000.0 / Clock.Frequency;
            lastPlanningBytes = GC.GetAllocatedBytesForCurrentThread() - allocated;
            sumPlanningMs += lastPlanningMs;
            sumPlanningBytes += lastPlanningBytes;
            peakPlanningMs = Math.Max(peakPlanningMs, lastPlanningMs);
            peakPlanningBytes = Math.Max(peakPlanningBytes, lastPlanningBytes);
            planningFrames++;
            if (Time.unscaledTime >= nextReport)
            {
                nextReport = Time.unscaledTime + 0.25f;
                UpdateTrafficReport();
            }
        }

        private void SampleFollowTarget(float now)
        {
            using var marker = HoverMarker.Auto();
            if (terrainCamera == null || !terrainCamera.TryGetCursorRay(out Ray ray, out Vector2 screen)
                || (showPanel && Panel.Contains(new Vector2(screen.x, Screen.height - screen.y))))
            {
                CancelHoverTarget();
                nextFollow = 0f;
                return;
            }
            if (now < nextFollow) return;
            nextFollow = now + Mathf.Max(0.2f, followInterval);
            if (!Grid.TryPickSurface(ray, out Vector2Int picked)
                || !Grid.CanOccupy(new NavigationPose(picked), new NavigationFootprint(1, 1)))
            {
                CancelHoverTarget();
                targetReport = "No valid terrain top under cursor.";
                return;
            }

            if (IsNearAcceptedTarget(picked))
            {
                CancelHoverTarget();
                targetReport = "Accepted";
                return;
            }
            if (picked == lastRejectedTarget)
            {
                CancelHoverTarget();
                return;
            }
            // Coalesce queued input without restarting an already running guide.
            hoverTarget = guideFromHover && picked == guideTarget ? null : picked;
            if (hoverTarget.HasValue && !GuideActive) targetReport = $"Queued {picked}";
        }

        private bool IsNearAcceptedTarget(Vector2Int cell)
        {
            int distance = Mathf.Max(1, followDistance);
            return acceptedTarget.HasValue && (cell - acceptedTarget.Value).sqrMagnitude < (float)distance * distance;
        }

        private void CancelHoverTarget()
        {
            bool hadHover = hoverTarget.HasValue || guideFromHover;
            hoverTarget = null;
            if (guideFromHover) CancelGuide();
            if (hadHover && !GuideActive)
                targetReport = acceptedTarget.HasValue ? "Accepted" : "No accepted target";
        }

        private bool GuideActive => guideSearch != null || targetFlood != null;

        private void CancelGuide()
        {
            guideSearch?.Dispose();
            guideSearch = null;
            targetFlood = null;
            guideFromHover = guideLocal = false;
            guideExpanded = 0;
        }

        private void ClearFields()
        {
            for (int i = 0; i < fields.Length; i++)
            {
                fields[i]?.Dispose();
                fields[i] = null;
            }
        }

        private bool DispatchPending() => targetArea != null && units.Exists(entry => !entry.LastAttempt.HasValue);

        private int SearchLimit(NavigationFootprint size) => Grid.Surface.CellsX * Grid.Surface.CellsZ * size.Orientations;

        private void AdvanceGuide(NavigationBudget budget)
        {
            using var marker = GuideMarker.Auto();
            if (!armyDemo || !budget.CanContinue) return;
            if (requestedTarget.HasValue && guideFromHover) CancelGuide();
            if (!GuideActive)
            {
                bool fromHover = !requestedTarget.HasValue;
                var target = fromHover ? hoverTarget : requestedTarget;
                if (!target.HasValue || (fromHover && (Time.frameCount < nextHoverGuideFrame
                    || (DispatchPending() && Time.time - acceptedAt < 0.65f)))) return;
                if (fromHover) hoverTarget = null; else requestedTarget = null;
                if (!StartGuide(target.Value, fromHover)) return;
            }

            if (!targetFlood.Advance(budget)) return;
            if (guideFromHover && targetFlood.JoinsPrevious) guideLocal = true;
            else
            {
                if (guideSearch == null && !BeginConnectivityCheck())
                {
                    RejectTarget(guideTarget, NavigationStatus.InvalidStart);
                    CancelGuide();
                    return;
                }
                int before = budget.Remaining;
                var path = guideSearch.Advance(budget);
                guideExpanded += before - budget.Remaining;
                if (path == null) return;
                guideSearch = null;
                if (path.Status != NavigationStatus.Success)
                {
                    RejectTarget(guideTarget, path.Status);
                    if (guideFromHover) nextHoverGuideFrame = Time.frameCount + 2;
                    CancelGuide();
                    return;
                }
            }
            AcceptTarget(guideTarget, targetFlood, guideLocal);
            if (hoverTarget.HasValue && IsNearAcceptedTarget(hoverTarget.Value)) hoverTarget = null;
            CancelGuide();
        }

        private bool StartGuide(Vector2Int target, bool fromHover)
        {
            guideStartedAt = Time.time;
            var small = new NavigationFootprint(1, 1);
            if (!Grid.CanOccupy(new NavigationPose(target), small))
            {
                RejectTarget(target, NavigationStatus.InvalidGoal);
                return false;
            }
            guideTarget = target;
            guideFromHover = fromHover;
            targetFlood = new NavigationGroupPlanner.TargetFlood(Grid, target, Mathf.Clamp(groupRadius, 0, 32),
                fromHover ? acceptedArea : null, fromHover ? fieldArea : null);
            targetReport = $"Checking destination {target}";
            return true;
        }

        private bool BeginConnectivityCheck()
        {
            var small = new NavigationFootprint(1, 1);
            var anchors = new HashSet<int>();
            var min = new Vector2Int(Grid.Surface.CellsX, Grid.Surface.CellsZ);
            var max = Vector2Int.zero;
            foreach (var entry in units)
            {
                var pose = new NavigationPose(entry.Unit.Pose.Cell);
                if (!Grid.CanOccupy(pose, small)) continue;
                anchors.Add(Grid.StateId(pose));
                min = Vector2Int.Min(min, pose.Cell);
                max = Vector2Int.Max(max, pose.Cell);
            }
            if (anchors.Count != 0)
            {
                guideSearch = finder.BeginGuide(guideTarget, anchors, new RectInt(min, max - min + Vector2Int.one));
                return true;
            }
            return false;
        }

        private void RejectTarget(Vector2Int target, NavigationStatus status)
        {
            lastRejectedTarget = target;
            if (launchInitialOrder)
            {
                launchInitialOrder = false;
                foreach (var entry in units) entry.Unit.EndProvisionalRoute();
            }
            if (hoverTarget == target) hoverTarget = null;
            targetReport = $"Target {target}: {status}.";
        }

        private void AcceptTarget(Vector2Int target, NavigationGroupPlanner.TargetFlood area, bool local)
        {
            var previous = acceptedTarget;
            bool hadInitialLaunch = launchInitialOrder;
            acceptedTarget = goal = target;
            acceptedAt = Time.time;
            if (!hadInitialLaunch) orderAt = guideStartedAt;
            lastRejectedTarget = null;
            targetReport = local ? "Destination adjusted" : "Accepted";
            launchInitialOrder = false;
            // Reuse the field while its reachable area still connects to the
            // new destination. Distance by itself does not invalidate a route.
            if (!local || !area.JoinsField)
            {
                CancelParticipantPlans();
                ClearFields();
                fieldArea = area;
            }
            acceptedArea = area;
            targetArea = area.Cells;
            launchCursor = 0;
            Vector2 groupCentre = Vector2.zero;
            foreach (var entry in units) groupCentre += entry.Unit.PlanningPose.Centre(entry.Footprint);
            groupCentre /= units.Count;
            Vector2 forward = ((Vector2)target + Vector2.one * 0.5f - groupCentre).normalized;
            Vector2 across = new Vector2(-forward.y, forward.x);
            foreach (var entry in units)
            {
                float lateral = Vector2.Dot(entry.Unit.PlanningPose.Centre(entry.Footprint) - groupCentre, across);
                entry.SlotBias = across * Mathf.Clamp(lateral, -groupRadius * 0.7f, groupRadius * 0.7f);
                entry.Unit.ConfigureFormation(lateral / Mathf.Max(1f, groupRadius * 0.7f));
                bool keep = false;
                if (local && previous.HasValue && entry.Reserved && entry.Destination.HasValue)
                {
                    var destination = entry.Destination.Value;
                    int distance = (destination.Cell - target).sqrMagnitude;
                    keep = distance <= (destination.Cell - previous.Value).sqrMagnitude
                        && area.Contains(destination.Cell)
                        && (entry.Unit.HasRoute || (entry.Unit.State == UnitMovement.Motion.Arrived
                            && entry.Unit.Pose.Equals(destination)));
                }
                if (!keep) entry.Unit.CancelPending();
                entry.Assigned = keep;
                entry.Failures = entry.NextCandidate = 0;
                entry.RetryAt = 0;
                entry.LastAttempt = keep ? NavigationStatus.Success : null;
                entry.LaunchChecked = keep || entry.Unit.HasRoute || (hadInitialLaunch && entry.LaunchChecked);
                if (!hadInitialLaunch)
                {
                    entry.MotionAtOrder = entry.Unit.MotionTicks;
                    entry.TurnsAtOrder = entry.Unit.TurnCount;
                    entry.StopsAtOrder = entry.Unit.StopCount;
                    entry.StartDelay = -1f;
                }
            }
            targetExpanded = 0;
        }

        private void LaunchParticipants(NavigationBudget budget, Vector2Int target)
        {
            Vector2 groupCentre = Vector2.zero;
            if (launchInitialOrder)
            {
                foreach (var entry in units) groupCentre += entry.Unit.Pose.Centre(entry.Footprint);
                groupCentre /= units.Count;
            }
            Vector2 forward = ((Vector2)target + Vector2.one * 0.5f - groupCentre).normalized;
            Vector2 across = new Vector2(-forward.y, forward.x);
            while (launchCursor < units.Count)
            {
                if (!budget.Take()) return;
                var entry = units[launchCursor++];
                if (entry.LaunchChecked || entry.Unit.Planning || !entry.Unit.CanLaunch) continue;
                entry.LaunchChecked = true;
                if (launchInitialOrder)
                {
                    float lateral = Vector2.Dot(entry.Unit.Pose.Centre(entry.Footprint) - groupCentre, across);
                    entry.SlotBias = across * Mathf.Clamp(lateral, -groupRadius * 0.7f, groupRadius * 0.7f);
                    entry.Unit.ConfigureFormation(lateral / Mathf.Max(1f, groupRadius * 0.7f));
                }
                entry.Unit.TryLaunch((Vector2)target + Vector2.one * 0.5f + entry.SlotBias);
            }
        }

        private bool StartParticipantPlan(float now)
        {
            bool dispatch = DispatchPending();
            for (int i = 0; i < units.Count; i++)
            {
                var entry = units[planIndex];
                planIndex = (planIndex + 1) % units.Count;
                if (dispatch && entry.LastAttempt.HasValue) continue;
                if (entry.Unit.Planning || !entry.Unit.CanPlan || entry.RetryAt > now) continue;
                if (entry.Failures >= 3 && entry.LastAttempt == NavigationStatus.NoPath
                    && entry.LastTrafficRevision == navigation.TrafficRevision) continue;
                bool needsSlot = targetArea != null && !entry.Assigned;
                if (!needsSlot && (!entry.Destination.HasValue || entry.Unit.HasRoute
                    || (entry.Unit.State == UnitMovement.Motion.Arrived
                        && entry.Unit.Pose.Equals(entry.Destination.Value)))) continue;

                var unit = entry.Unit;
                bool avoidAllies = unit.Recovering && entry.Failures % 3 != 2;
                bool recovery = unit.RequiresCornerRecovery || (unit.Recovering && (!needsSlot || entry.LastAttempt.HasValue));
                var planStart = unit.PlanningPose;
                bool startsInside = needsSlot && !recovery && acceptedArea.Contains(planStart.Cell);
                var field = needsSlot && !recovery && !startsInside
                    ? fields[unit.Footprint.Graph] ??= new NavigationPathfinder.ApproachField(finder, unit.Footprint, fieldArea.EntryCells)
                    : null;
                participantPlan = new ParticipantPlan
                {
                    Entry = entry,
                    NeedsSlot = needsSlot,
                    StartsInside = startsInside,
                    CandidateIndex = needsSlot ? entry.NextCandidate : 0,
                    Area = acceptedArea,
                    TargetAtStart = acceptedTarget ?? planStart.Cell,
                    SlotCentre = (acceptedTarget ?? planStart.Cell) + new Vector2Int(
                        Mathf.RoundToInt(entry.SlotBias.x), Mathf.RoundToInt(entry.SlotBias.y)),
                    PreferredSlot = (Vector2)(acceptedTarget ?? planStart.Cell) + Vector2.one * 0.5f + entry.SlotBias,
                    Field = field,
                    Start = planStart,
                    Approach = planStart,
                    Remaining = SearchLimit(unit.Footprint),
                    TrafficRevision = navigation.TrafficRevision,
                    AllowsEdge = recovery ? (from, to) => unit.AllowsSearchEdge(from, to, avoidAllies) : null
                };
                if (needsSlot && !recovery && !startsInside
                    && unit.GetReusableRoute(planStart, out var reuseRoute, out int reuseIndex))
                {
                    participantPlan.ReuseRoute = reuseRoute;
                    participantPlan.ReuseIndex = reuseIndex;
                    participantPlan.ReuseScan = reuseIndex;
                }
                unit.Planning = true;
                return true;
            }
            return false;
        }

        private void CancelParticipantPlans()
        {
            foreach (var plan in participantPlans) CancelParticipantPlan(plan);
            participantPlans.Clear();
        }

        private static void CancelParticipantPlan(ParticipantPlan plan)
        {
            if (plan.Entry.Unit != null) plan.Entry.Unit.Planning = false;
            plan.Search?.Dispose();
        }

        private bool AdvanceParticipantPlan(NavigationBudget budget, ref int requests)
        {
            using var marker = ParticipantMarker.Auto();
            const int candidateLimit = 48;
            var plan = participantPlan;
            var unit = plan.Entry.Unit;
            if (plan.ReuseRoute != null && !plan.ApproachReady && !AdvanceExistingApproach(plan, unit, budget)) return false;
            if (plan.Field != null && !plan.ApproachReady)
            {
                if (plan.Prefix == null)
                {
                    if (!plan.Field.AdvanceTo(plan.Start, budget, ref plan.FieldLimit, out bool reached)) return false;
                    if (!reached) plan.Field = null;
                    else plan.Prefix = new List<NavigationPoint> { Grid.Point(plan.Start, unit.Footprint) };
                }
                if (plan.Field != null)
                {
                    while (!plan.Area.Contains(plan.Approach.Cell)
                        && plan.Field.TryNext(plan.Approach, plan.PreferredSlot, out var next))
                    {
                        if (!budget.HasTime) return false;
                        Vector2Int delta = next.Cell - plan.Approach.Cell;
                        int edge = delta == Vector2Int.zero ? 8 : Array.IndexOf(NavigationPose.Steps, delta);
                        plan.PrefixCost += Grid.EdgeCost(plan.Approach, next, unit.Footprint, edge);
                        plan.Approach = next;
                        plan.Prefix.Add(Grid.Point(next, unit.Footprint));
                    }
                }
                plan.ApproachPointCount = plan.Prefix?.Count ?? 0;
                plan.ApproachReady = true;
            }

            while (requests > 0 && budget.CanContinue)
            {
                if (plan.Suffix != null) return FinishSharedRoute(plan, unit, budget);
                if (plan.Search == null)
                {
                    if (plan.Remaining == 0) return FinishParticipantPlan(NavigationStatus.BudgetExceeded);
                    if (!plan.HasCandidate)
                    {
                        if (plan.Tried == candidateLimit) return FinishParticipantPlan(NavigationStatus.BudgetExceeded);
                        if (!plan.NeedsSlot) plan.Goal = plan.Entry.Destination.Value;
                        else
                        {
                            if (!ChooseSlot(plan, unit, budget))
                            {
                                if (!budget.CanContinue) return false;
                                return FinishParticipantPlan(plan.Entry.NextCandidate != 0 ? NavigationStatus.BudgetExceeded
                                    : plan.Tried == 0 ? NavigationStatus.InvalidGoal : NavigationStatus.NoPath);
                            }
                        }
                        plan.Tried++;
                        plan.HasCandidate = true;
                        plan.LocalSearch = plan.StartsInside || plan.Prefix != null;
                    }

                    var from = plan.LocalSearch ? plan.Approach : plan.Start;
                    int limitForSearch = plan.LocalSearch ? Math.Min(512, plan.Remaining) : plan.Remaining;
                    requests--;
                    if (!plan.LocalSearch) fullSearches++;
                    plan.Search = finder.BeginSearch(new NavigationRequest(from, plan.Goal,
                        unit.Footprint, limitForSearch), allowsEdge: plan.AllowsEdge);
                }

                int before = budget.Remaining;
                var path = plan.Search.Advance(budget);
                if (!plan.LocalSearch) plan.Remaining -= before - budget.Remaining;
                if (path == null) return false;
                plan.Search = null;
                if (path.Status == NavigationStatus.Success)
                {
                    if (!plan.LocalSearch || plan.Prefix == null) return AssignParticipant(plan, unit, path);
                    plan.Suffix = path;
                    plan.SuffixIndex = 1;
                    return FinishSharedRoute(plan, unit, budget);
                }
                if (plan.LocalSearch)
                {
                    // The bounded local pass is an optimisation. Try the same
                    // slot with unrestricted A* before rejecting it.
                    plan.LocalSearch = false;
                    continue;
                }
                if (path.Status != NavigationStatus.NoPath || !plan.NeedsSlot)
                    return FinishParticipantPlan(path.Status);
                plan.HasCandidate = false;
            }
            return false;
        }

        private bool AdvanceExistingApproach(ParticipantPlan plan, UnitMovement unit, NavigationBudget budget)
        {
            var route = plan.ReuseRoute;
            if (plan.ReuseEnd < 0)
            {
                while (plan.ReuseScan < route.Points.Count)
                {
                    if (!budget.Take()) return false;
                    if (plan.Area.Contains(route.Points[plan.ReuseScan].Pose.Cell))
                    {
                        plan.ReuseEnd = plan.ReuseScan;
                        break;
                    }
                    plan.ReuseScan++;
                }
                if (plan.ReuseEnd < 0)
                {
                    plan.ReuseRoute = null;
                    return true;
                }
                plan.Prefix = new List<NavigationPoint>();
            }
            while (plan.ReuseIndex <= plan.ReuseEnd)
            {
                if (!budget.Take()) return false;
                var point = route.Points[plan.ReuseIndex++];
                if (plan.Prefix.Count > 0)
                {
                    Vector2Int delta = point.Pose.Cell - plan.Approach.Cell;
                    int edge = delta == Vector2Int.zero ? 8 : Array.IndexOf(NavigationPose.Steps, delta);
                    plan.PrefixCost += edge >= 0 ? Grid.EdgeCost(plan.Approach, point.Pose, unit.Footprint, edge)
                        : Mathf.CeilToInt(((Vector2)delta).magnitude * 10f);
                }
                plan.Approach = point.Pose;
                plan.Prefix.Add(point);
            }
            plan.ApproachPointCount = plan.Prefix.Count;
            plan.ApproachReady = true;
            return true;
        }

        private bool ChooseSlot(ParticipantPlan plan, UnitMovement unit, NavigationBudget budget)
        {
            var own = plan.Entry.Reserved ? plan.Entry.Destination : null;
            int orientations = unit.Footprint.Orientations;
            int diameter = 4 * plan.Area.Radius + 1;
            int limit = diameter * diameter * orientations;
            while (true)
            {
                // Keep alternatives until tried. Resume both collection and
                // crossing evaluation if the frame budget expires.
                while (!plan.Scoring && plan.Candidates.Count < 16 && plan.CandidateIndex < limit)
                {
                    if (!budget.HasTime) return false;
                    int candidate = plan.CandidateIndex++;
                    var cell = plan.SlotCentre + RingCell(candidate / orientations);
                    if (plan.Area.Contains(cell) && planner.TryCandidate(unit.Footprint, cell, candidate % orientations,
                        reservations, own, plan.Approach.Facing, out var pose)) plan.Candidates.Add(pose);
                }
                plan.Scoring = true;
                while (plan.ScoringIndex < plan.Candidates.Count)
                {
                    if (!budget.Take()) return false;
                    int i = plan.ScoringIndex++;
                    var pose = plan.Candidates[i];
                    if (!FitsCurrentTarget(plan, pose) || !NavigationGroupPlanner.CanReserve(pose, unit.Footprint, reservations, own))
                        continue;
                    var centre = pose.Centre(unit.Footprint);
                    float score = (centre - plan.PreferredSlot).sqrMagnitude
                        + (centre - plan.Approach.Centre(unit.Footprint)).sqrMagnitude * 0.08f
                        + Mathf.Abs(Mathf.DeltaAngle(plan.Approach.Yaw, pose.Yaw)) / 45f
                        + CountSlotCrossings(plan, centre) * 6f;
                    if (score >= plan.BestScore) continue;
                    plan.BestScore = score;
                    plan.BestCandidate = i;
                }
                int best = plan.BestCandidate;
                plan.Scoring = false;
                plan.ScoringIndex = 0;
                plan.BestCandidate = -1;
                plan.BestScore = float.PositiveInfinity;
                if (best >= 0)
                {
                    plan.Goal = plan.Candidates[best];
                    plan.Candidates.RemoveAt(best);
                    if (FitsCurrentTarget(plan, plan.Goal)
                        && NavigationGroupPlanner.CanReserve(plan.Goal, unit.Footprint, reservations, own)) return true;
                    continue;
                }
                plan.Candidates.Clear();
                if (plan.CandidateIndex >= limit) return false;
            }
        }

        private int CountSlotCrossings(ParticipantPlan plan, Vector2 candidate)
        {
            Vector2 from = plan.Approach.Centre(plan.Entry.Footprint);
            int count = 0;
            foreach (var other in units)
            {
                if (other == plan.Entry || !other.Assigned || !other.Destination.HasValue) continue;
                var destination = other.Destination.Value.Centre(other.Footprint);
                if ((destination - candidate).sqrMagnitude > 100f) continue;
                var route = other.SlotRoute;
                if (route == null || !route.IsCurrent) continue;
                int end = route.Points.Count - 1;
                for (int i = Mathf.Max(0, end - 6); i < end; i++)
                {
                    var a = route.Points[i].Pose.Centre(other.Footprint);
                    var b = route.Points[i + 1].Pose.Centre(other.Footprint);
                    if (!Crosses(from, candidate, a, b)) continue;
                    count++;
                    break;
                }
            }
            foreach (var other in participantPlans)
                if (other.Entry != plan.Entry && other.HasCandidate && (other.Goal.Centre(other.Entry.Footprint) - candidate).sqrMagnitude <= 100f
                    && Crosses(from, candidate, other.Approach.Centre(other.Entry.Footprint), other.Goal.Centre(other.Entry.Footprint)))
                    count++;
            return count;

            static bool Crosses(Vector2 a, Vector2 b, Vector2 c, Vector2 d)
            {
                Vector2 along = b - a, other = d - c;
                float first = Cross(along, c - a), second = Cross(along, d - a);
                float third = Cross(other, a - c), fourth = Cross(other, b - c);
                if (first * second < -0.0001f && third * fourth < -0.0001f) return true;
                // Count opposing use of the same segment. Shared movement
                // in the same direction does not penalize a formation slot.
                if (Mathf.Abs(first) > 0.0001f || Mathf.Abs(second) > 0.0001f || Vector2.Dot(along, other) >= 0f)
                    return false;
                float start = Vector2.Dot(along, c - a), end = Vector2.Dot(along, d - a);
                return Mathf.Min(along.sqrMagnitude, Mathf.Max(start, end))
                    > Mathf.Max(0f, Mathf.Min(start, end)) + 0.0001f;
            }
            static float Cross(Vector2 a, Vector2 b) => a.x * b.y - a.y * b.x;
        }

        private bool FitsCurrentTarget(ParticipantPlan plan, NavigationPose slot)
        {
            if (!acceptedTarget.HasValue || acceptedArea == null || !acceptedArea.Contains(slot.Cell)) return false;
            var target = acceptedTarget.Value;
            return ReferenceEquals(plan.Area, acceptedArea)
                || (slot.Cell - target).sqrMagnitude <= (slot.Cell - plan.TargetAtStart).sqrMagnitude;
        }

        // Square rings around the participant's own preferred slot. The cursor
        // can resume after any cell without rebuilding or sorting the area.
        private static Vector2Int RingCell(int index)
        {
            if (index == 0) return Vector2Int.zero;
            int radius = (Mathf.FloorToInt(Mathf.Sqrt(index)) + 1) / 2;
            int side = radius * 2;
            int offset = index - (side - 1) * (side - 1);
            if (offset < side) return new Vector2Int(-radius + offset, -radius);
            if (offset < side * 2) return new Vector2Int(radius, -radius + offset - side);
            if (offset < side * 3) return new Vector2Int(radius - (offset - side * 2), radius);
            return new Vector2Int(-radius, radius - (offset - side * 3));
        }

        private bool FinishSharedRoute(ParticipantPlan plan, UnitMovement unit, NavigationBudget budget)
        {
            while (plan.SuffixIndex < plan.Suffix.Points.Count)
            {
                if (!budget.HasTime) return false;
                plan.Prefix.Add(plan.Suffix.Points[plan.SuffixIndex++]);
            }
            var path = new NavigationPath(Grid, NavigationStatus.Success, "Shared approach and exact destination.",
                plan.Prefix, plan.Suffix.Expanded, plan.PrefixCost + plan.Suffix.Cost, unit.Footprint);
            return AssignParticipant(plan, unit, path);
        }

        private bool AssignParticipant(ParticipantPlan plan, UnitMovement unit, NavigationPath path)
        {
            var entry = plan.Entry;
            if (!plan.NeedsSlot && targetArea != null && !entry.Assigned)
                return FinishParticipantPlan(NavigationStatus.Stale);
            if (plan.NeedsSlot && (!FitsCurrentTarget(plan, plan.Goal)
                || !NavigationGroupPlanner.CanReserve(plan.Goal, unit.Footprint, reservations,
                    entry.Reserved ? entry.Destination : null)))
            {
                // A different sliced plan confirmed this slot while this one was suspended.
                plan.HasCandidate = false;
                plan.Suffix = null;
                if (plan.Prefix != null)
                    plan.Prefix.RemoveRange(plan.ApproachPointCount, plan.Prefix.Count - plan.ApproachPointCount);
                return false;
            }
            if (!unit.Assign(path)) return FinishParticipantPlan(NavigationStatus.Stale);
            assignments++;
            if (plan.Suffix != null && plan.ApproachPointCount > 1) sharedAssignments++;
            if (plan.NeedsSlot)
            {
                if (entry.Reserved)
                    NavigationGroupPlanner.Reserve(entry.Destination.Value, unit.Footprint, reservations, false);
                entry.Destination = path.Points[path.Points.Count - 1].Pose;
                NavigationGroupPlanner.Reserve(entry.Destination.Value, unit.Footprint, reservations);
                entry.SlotRoute = path;
                entry.Reserved = entry.Assigned = true;
            }
            return FinishParticipantPlan(NavigationStatus.Success);
        }

        private bool FinishParticipantPlan(NavigationStatus status)
        {
            var plan = participantPlan;
            var entry = plan.Entry;
            if (status == NavigationStatus.NoPath && plan.AllowsEdge != null
                && plan.TrafficRevision != navigation.TrafficRevision)
                status = NavigationStatus.Stale;
            entry.LastAttempt = status;
            entry.NextCandidate = plan.NeedsSlot && ReferenceEquals(plan.Area, acceptedArea)
                && status == NavigationStatus.BudgetExceeded && plan.Tried >= 48
                && plan.CandidateIndex < (4 * plan.Area.Radius + 1) * (4 * plan.Area.Radius + 1) * entry.Footprint.Orientations
                ? plan.CandidateIndex : 0;
            // Live traffic can change while a recovery search is suspended.
            entry.LastTrafficRevision = plan.TrafficRevision;
            entry.Failures = status == NavigationStatus.Success ? 0 : Math.Min(entry.Failures + 1, 8);
            if (!plan.NeedsSlot && targetArea != null && entry.Failures == 3)
            {
                entry.Assigned = false;
                entry.Failures = 0;
            }
            entry.RetryAt = Time.time + (status == NavigationStatus.Stale ? 0.08f : 0.18f * (1 << Math.Min(4, entry.Failures)));
            if (status != NavigationStatus.Success) entry.Unit.ReportFailure(status);
            CancelParticipantPlan(plan);
            return true;
        }

        private void UpdateTrafficReport()
        {
            int routes = 0, waiting = 0, blocked = 0, arrived = 0, capped = 0, noPath = 0, accepted = 0, attempted = 0;
            int planning = 0, geometry = 0, traffic = 0, passage = 0, turns = 0, stops = 0;
            startDelays.Clear();
            foreach (var entry in units)
            {
                var unit = entry.Unit;
                if (unit == null || !unit.isActiveAndEnabled) continue;
                if (unit.HasRoute) routes++;
                if (entry.StartDelay >= 0f) startDelays.Add(entry.StartDelay);
                turns += unit.TurnCount - entry.TurnsAtOrder;
                stops += unit.StopCount - entry.StopsAtOrder;
                switch (unit.HoldReason)
                {
                    case UnitMovement.Hold.Planning: planning++; break;
                    case UnitMovement.Hold.Geometry: geometry++; break;
                    case UnitMovement.Hold.Traffic: traffic++; break;
                    case UnitMovement.Hold.Passage: passage++; break;
                }
                if (entry.Destination.HasValue && unit.State == UnitMovement.Motion.Arrived
                    && unit.Pose.Equals(entry.Destination.Value)) arrived++;
                if (unit.Recovering || unit.State == UnitMovement.Motion.Yielding || unit.Planning
                    || (targetArea != null && !entry.LastAttempt.HasValue)) waiting++;
                if (unit.State == UnitMovement.Motion.Blocked || unit.State == UnitMovement.Motion.Invalidated) blocked++;
                if (entry.LastAttempt == NavigationStatus.BudgetExceeded) capped++;
                if (entry.LastAttempt == NavigationStatus.NoPath || entry.LastAttempt == NavigationStatus.InvalidGoal
                    || entry.LastAttempt == NavigationStatus.InvalidStart) noPath++;
                if (entry.Assigned) accepted++;
                if (entry.LastAttempt.HasValue) attempted++;
            }
            startDelays.Sort();
            int start90 = Mathf.Max(0, Mathf.CeilToInt(units.Count * 0.9f) - 1);
            string startTime = startDelays.Count > start90 ? $"{startDelays[start90]:F2}s" : "pending";
            Array.Copy(frameTimes, sortedFrameTimes, frameTimeCount);
            Array.Sort(sortedFrameTimes, 0, frameTimeCount);
            float frameP95 = sortedFrameTimes[Mathf.Max(0, Mathf.CeilToInt(frameTimeCount * 0.95f) - 1)];
            string target = acceptedTarget.HasValue
                ? $"{acceptedTarget.Value}, level {Grid.Surface.GetCell(acceptedTarget.Value.x, acceptedTarget.Value.y).Level}"
                : "none";
            trafficReport = $"{units.Count} units · routes {routes} · waiting/recovery {waiting} · arrived {arrived} · blocked {blocked}\n"
                + (armyDemo ? $"Target {target} · {targetReport} · queued {hoverTarget}\nDispatch {attempted}/{units.Count} · assigned {accepted} · " : "")
                + $"NoPath/invalid {noPath} · BudgetExceeded {capped} · active {(GuideActive ? 1 : 0) + participantPlans.Count}"
                + (GuideActive ? $" · guide expansions {guideExpanded}" : "")
                + $"\nHolds: plan {planning} · terrain {geometry} · units {traffic} · lane {passage} · turns {turns} · stops {stops}"
                + (armyDemo ? $"\nStarted {startDelays.Count}/{units.Count} · 90% start {startTime}" : "");
            planningReport = $"Planning ms last/mean/max {lastPlanningMs:F2}/{sumPlanningMs / planningFrames:F2}/{peakPlanningMs:F2} · "
                + $"B {lastPlanningBytes}/{sumPlanningBytes / planningFrames}/{peakPlanningBytes}\n"
                + $"Expansions {frameExpanded}/{targetExpanded}/{totalExpanded} · assignments {assignments} (shared {sharedAssignments}) · A* {fullSearches}\n"
                + $"Frame p95 {frameP95:F1} ms · last {frameTimeCount} frames";
        }

        private void OnGUI()
        {
            if (!showPanel || !Application.isPlaying) return;
            using var marker = GuiMarker.Auto();
            GUILayout.BeginArea(Panel, GUI.skin.box);
            scroll = GUILayout.BeginScrollView(scroll);
            GUILayout.Label("Stage 05.3 — Terrain and group traffic");
            GUILayout.Label(builder.BuildReport ?? "Terrain unavailable.");
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Basin")) Landmark(0);
            if (GUILayout.Button("Sealed basin")) Landmark(1);
            if (GUILayout.Button("Hill")) Landmark(2);
            if (GUILayout.Button("Sealed hill")) Landmark(3);
            if (GUILayout.Button("Summit")) Landmark(4);
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Army (64)")) Army();
            if (GUILayout.Button("Army (100)")) Army(100);
            if (GUILayout.Button("Allied lane")) Traffic(0);
            if (GUILayout.Button("Enemy lane")) Traffic(1);
            if (GUILayout.Button("Frame map")) FrameTerrain();
            if (GUILayout.Button("Reverse route") && movementDemo && !groupMode)
            {
                (start, goal) = (goal, start);
                (startFacing, goalFacing) = (goalFacing, startFacing);
                Replan();
            }
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Ally chain")) Traffic(2);
            if (GUILayout.Button("Two groups")) Traffic(3);
            if (GUILayout.Button("Sealed lane")) Traffic(4);
            if (sealedLaneGuard != null && GUILayout.Button("Release lane")) ReleaseLane();
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Open crossing")) Traffic(5);
            if (GUILayout.Button("One lane")) Traffic(6);
            if (GUILayout.Button("Ramp straight")) MountedRamp(false);
            if (GUILayout.Button("Ramp angled")) MountedRamp(true);
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Bent passage")) Traffic(7);
            if (GUILayout.Button("Junction")) Traffic(8);
            GUILayout.EndHorizontal();
            if (units.Count > 0)
            {
                if (armyDemo)
                {
                    bool follow = GUILayout.Toggle(followMouse, "Follow Mouse");
                    if (follow != followMouse)
                    {
                        followMouse = follow;
                        nextFollow = 0;
                        lastRejectedTarget = null;
                        if (!followMouse)
                        {
                            CancelHoverTarget();
                            targetReport = acceptedTarget.HasValue ? "Accepted" : "No accepted target";
                        }
                    }
                }
                GUILayout.Label(trafficReport ?? "Waiting for planning.");
                GUILayout.Label(planningReport ?? "");
            }
            GUILayout.Label("Cyan/colours: paths + footprints; red: cliffs/edges; magenta: static blockers.");
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Grass + ramps")) ExplicitScenario(0);
            if (GUILayout.Button("Narrow ramp")) ExplicitScenario(1);
            if (GUILayout.Button("Narrow group")) { ExplicitScenario(1); groupMode = true; groupRadius = 3; Replan(); }
            GUILayout.EndHorizontal(); GUILayout.BeginHorizontal();
            if (GUILayout.Button("Wide mixed group")) ExplicitScenario(2);
            if (GUILayout.Button("Diagonal corner")) ExplicitScenario(3);
            if (GUILayout.Button("Turn 1x2")) ExplicitScenario(4);
            GUILayout.EndHorizontal(); GUILayout.BeginHorizontal();
            if (GUILayout.Button("Seed 104729")) Seed(104729);
            if (GUILayout.Button("Seed 130363")) Seed(130363);
            if (GUILayout.Button("Seed 20261103")) Seed(20261103);
            if (GUILayout.Button("N/E/S/W map")) { builder.ShowDirections(); ClearUnits(); movementDemo = armyDemo = groupMode = false; report = "Edit start/goal in Inspector; Replan for each ramp."; DrawGeometry(); }
            GUILayout.EndHorizontal(); GUILayout.BeginHorizontal();
            if (GUILayout.Button("Request 1x1")) { footprint = Vector2Int.one; armyDemo = groupMode = false; Replan(); }
            if (GUILayout.Button("Request 1x2 N")) { footprint = new Vector2Int(1, 2); startFacing = goalFacing = TerrainDirection.North; armyDemo = groupMode = false; Replan(); }
            if (GUILayout.Button("Request 1x2 E")) { footprint = new Vector2Int(1, 2); startFacing = goalFacing = TerrainDirection.East; armyDemo = groupMode = false; Replan(); }
            if (GUILayout.Button("Request 2x2")) { footprint = new Vector2Int(2, 2); armyDemo = groupMode = false; Replan(); }
            GUILayout.EndHorizontal(); GUILayout.BeginHorizontal();
            if (GUILayout.Button("Add/remove blocker")) ToggleBlocker();
            if (GUILayout.Button("Replan")) Replan();
            bool execute = GUILayout.Toggle(movementDemo, "Move units");
            if (execute != movementDemo) { ClearUnits(); movementDemo = execute; armyDemo = false; Replan(); }
            GUILayout.EndHorizontal();
            bool visible = GUILayout.Toggle(showBoundaries, "Show impassable geometry (display capped at 192 segments)");
            if (visible != showBoundaries) { showBoundaries = visible; ClearLines(); DrawGeometry(); paths.Clear(); report = "Display changed; press Replan to redraw routes."; }
            if (Event.current.type == EventType.Layout && Time.unscaledTime >= nextRequestReport)
            {
                requestReport = $"Request: {start} {startFacing} → {goal} {goalFacing}; footprint {footprint}";
                nextRequestReport = Time.unscaledTime + 0.25f;
            }
            GUILayout.Label(requestReport ?? "");
            GUILayout.Label(report ?? "No navigation snapshot.");
            GUILayout.EndScrollView(); GUILayout.EndArea();
        }
    }
}
