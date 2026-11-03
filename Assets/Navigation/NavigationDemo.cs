using System;
using System.Collections.Generic;
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

        private sealed class Participant
        {
            public UnitMovement Unit;
            public NavigationFootprint Footprint;
            public NavigationPose? Destination;
            public NavigationStatus? LastAttempt;
            public int Failures, LastTrafficRevision;
            public float RetryAt;
            public bool Reserved, Assigned;
        }

        private readonly List<Participant> units = new List<Participant>();
        private readonly HashSet<Vector2Int> reservations = new();
        private readonly HashSet<Vector2Int> sharedCorridor = new();
        private readonly NavigationMember[] memberRequest = new NavigationMember[1];
        private readonly System.Diagnostics.Stopwatch planningClock = new();
        private List<Vector2Int> targetArea;
        private NavigationGrid sessionGrid;
        private NavigationGroupPlanner planner;
        private NavigationPathfinder finder;
        private Vector2Int? acceptedTarget, requestedTarget, lastRejectedTarget;
        private bool followMouse;
        private float nextFollow;
        private int planIndex, targetExpanded, frameExpanded;
        private long totalExpanded;
        private double targetMs;
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
        private int blockerHandle;
        private NavigationGrid laneGrid;
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
        }
        private static void Release(UnityEngine.Object obj)
        { if (Application.isPlaying) Destroy(obj); else DestroyImmediate(obj); }
        private void ClearLines() { foreach (var line in lines) if (line != null) Release(line); lines.Clear(); }
        private void Invalidate()
        {
            paths.Clear(); ClearLines();
            followMouse = false;
            requestedTarget = lastRejectedTarget = null;
            if (sessionGrid != null && sessionGrid != Grid)
            {
                ClearUnits();
                movementDemo = armyDemo = groupMode = false;
            }
            if (laneGrid != null && laneGrid != Grid) ClearLane();
            if (sessionGrid == Grid)
                foreach (var entry in units) { entry.RetryAt = 0; entry.Failures = 0; }
            if (acceptedTarget.HasValue && sessionGrid == Grid && Grid != null)
            {
                requestedTarget = acceptedTarget;
                sharedCorridor.Clear();
                targetArea = null;
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
            foreach (var entry in units)
            {
                if (entry.Unit == null) continue;
                entry.Unit.gameObject.SetActive(false);
                Release(entry.Unit.gameObject);
            }
            units.Clear();
            followMouse = false;
            nextFollow = 0;
            planIndex = targetExpanded = frameExpanded = 0;
            totalExpanded = 0;
            targetMs = 0;
            targetReport = null;
            reservations.Clear();
            sharedCorridor.Clear();
            acceptedTarget = requestedTarget = lastRejectedTarget = null;
            targetArea = null;
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
        private void Spawn(NavigationPath path, NavigationFootprint footprint, Color color,
            int team = 0, bool hold = false)
        {
            if (!path.IsCurrent || path.Points.Count == 0) return;
            var visual = GameObject.CreatePrimitive(PrimitiveType.Cube);
            visual.hideFlags = HideFlags.DontSave;
            visual.name = footprint.NeedsTurns ? "Demo mounted unit (1x2)" : "Demo unit";
            visual.GetComponent<Collider>().enabled = false;
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
            }
            else { visual.SetActive(false); Release(visual); }
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
            int stateCount = Grid.Surface.CellsX * Grid.Surface.CellsZ * (f.NeedsTurns ? 4 : 1);
            var path = new NavigationPathfinder(Grid).Find(new NavigationRequest(new NavigationPose(start, startFacing),
                new NavigationPose(goal, goalFacing), f, movementDemo ? Math.Min(1000000, stateCount) : maxExpanded, allowPartial));
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

        private void Army()
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
            for (int z = 5; z < 29 && units.Count < 64; z += 3)
            for (int x = 5; x < 65 && units.Count < 64; x += 3)
            {
                int i = units.Count;
                var anchor = new NavigationPose(new Vector2Int(x, z));
                var size = i % 3 == 0 ? new NavigationFootprint(2, 2)
                    : i % 3 == 1 ? new NavigationFootprint(1, 2) : new NavigationFootprint(1, 1);
                if (!Grid.CanOccupy(anchor, size) || Grid.Surface.GetCell(x, z).Level != 0
                    || Grid.Surface.GetCell(x, z).IsRamp) continue;
                Spawn(spawnFinder.Find(new NavigationRequest(anchor, anchor, size, 1)), size,
                    Color.HSVToRGB(i * 0.21f, 0.85f, 1), hold: true);
            }
            requestedTarget = goal;
            report = $"Army spawned: {units.Count}/64. Enable Follow Mouse to steer.";
        }

        private void ClearLane()
        {
            if (laneGrid != null && laneGrid.IsAlive)
                foreach (int handle in laneHandles) laneGrid.RemoveBlocker(handle);
            laneHandles.Clear();
            laneGrid = null;
        }

        private void Traffic(int scenario)
        {
            if (Grid == null || Grid.Surface.Landmarks.Count != 5) return;
            ClearUnits(); ClearLane(); ClearLines(); paths.Clear();
            movementDemo = true;
            armyDemo = groupMode = false;
            if (scenario >= 2)
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
            if (scenario == 2)
                for (int x = 9; x <= 12; x++) AddUnit(x, x, Color.green, hold: true);
            else if (scenario == 3)
                for (int i = 0; i < 3; i++)
                {
                    AddUnit(5 + 2 * i, 26 + i, Color.cyan);
                    AddUnit(22 - 2 * i, 1, Color.yellow, endZ: 3 + 2 * i);
                }
            else AddUnit(13, 13, scenario == 0 ? Color.green : Color.red, scenario == 0 ? 0 : 1, true);
            if (scenario != 3) AddUnit(5, 26, Color.cyan);
            report = scenario switch
            {
                0 => "Stopped ally: watch it yield.",
                1 => "Enemy holds the lane: watch for a detour.",
                2 => "Allied chain: watch the yielding move propagate.",
                3 => "Opposing traffic: watch both flows clear the corridor.",
                _ => "Enemy seals the only exit. Release lane to remove the obstruction."
            };

            void AddWall(RectInt wall)
            {
                if (Grid.TryAddBlocker(wall, out int handle, out _)) laneHandles.Add(handle);
            }
            void AddUnit(int from, int to, Color color, int team = 0, bool hold = false, int endZ = 5)
            {
                Spawn(laneFinder.Find(new NavigationRequest(new NavigationPose(new Vector2Int(from, 5)),
                    new NavigationPose(new Vector2Int(to, endZ)), size)), size, color, team, hold);
            }
        }

        private void ReleaseLane()
        {
            ClearLane();
            for (int i = units.Count - 1; i >= 0; i--)
            {
                var entry = units[i];
                if (entry.Destination.HasValue || entry.Unit == null) continue;
                entry.Unit.gameObject.SetActive(false);
                Release(entry.Unit.gameObject);
            }
            ClearLines();
            DrawGeometry();
            report = "Lane released; waiting units can recover.";
        }

        private void Update()
        {
            if (units.Count == 0 || Grid != sessionGrid || !navigation.HasCurrentTransform
                || !Application.isFocused || Time.deltaTime <= 0f) return;
            for (int i = units.Count - 1; i >= 0; i--)
            {
                var entry = units[i];
                if (entry.Unit != null && entry.Unit.isActiveAndEnabled) continue;
                if (entry.Reserved) NavigationGroupPlanner.Reserve(entry.Destination.Value, entry.Footprint, reservations, false);
                units.RemoveAt(i);
            }
            if (units.Count == 0) { ClearUnits(); return; }
            planIndex %= units.Count;
            float now = Time.time;
            if (followMouse && now >= nextFollow)
            {
                nextFollow = now + followInterval;
                if (terrainCamera != null && terrainCamera.TryGetCursorRay(out Ray ray, out Vector2 screen)
                    && (!showPanel || !Panel.Contains(new Vector2(screen.x, Screen.height - screen.y))))
                {
                    if (!Grid.TryPickSurface(ray, out Vector2Int picked))
                    {
                        if (targetReport != "No terrain top under cursor.") targetReport = "No terrain top under cursor.";
                        lastRejectedTarget = null;
                    }
                    else if (!lastRejectedTarget.HasValue || lastRejectedTarget.Value != picked)
                    {
                        if (!acceptedTarget.HasValue || (picked - acceptedTarget.Value).sqrMagnitude
                            >= (float)followDistance * followDistance) requestedTarget = picked;
                        else targetReport = "Accepted";
                        lastRejectedTarget = null;
                    }
                }
            }

            int frameBudget = Mathf.Clamp(expansionsPerFrame, 256, 24000);
            int remaining = frameBudget;
            planningClock.Restart();
            if (requestedTarget.HasValue && armyDemo)
            {
                remaining -= AcceptTarget(requestedTarget.Value, remaining);
                requestedTarget = null;
            }
            int requests = 0;
            int budgetAfterTarget = remaining;
            for (int i = 0; i < units.Count && remaining > 0 && requests < requestsPerFrame; i++)
            {
                var entry = units[planIndex];
                planIndex = (planIndex + 1) % units.Count;
                if (!entry.Unit.CanPlan || entry.RetryAt > now) continue;
                if (entry.Failures >= 3 && entry.LastAttempt == NavigationStatus.NoPath
                    && entry.LastTrafficRevision == navigation.TrafficRevision) continue;
                bool needsSlot = targetArea != null && !entry.Assigned;
                if (!needsSlot && (!entry.Destination.HasValue || entry.Unit.HasRoute
                    || (entry.Unit.State == UnitMovement.Motion.Arrived
                        && entry.Unit.Pose.Equals(entry.Destination.Value)))) continue;
                requests++;
                int expanded = PlanParticipant(entry, needsSlot, remaining);
                remaining -= expanded;
            }
            frameExpanded = frameBudget - remaining;
            totalExpanded += frameExpanded;
            targetExpanded += budgetAfterTarget - remaining;
            if (frameExpanded > 0 || requests > 0) targetMs += planningClock.Elapsed.TotalMilliseconds;
        }

        private int AcceptTarget(Vector2Int target, int budget)
        {
            var small = new NavigationFootprint(1, 1);
            if (!Grid.CanOccupy(new NavigationPose(target), small))
            {
                lastRejectedTarget = target;
                targetReport = $"Target {target}: blocked.";
                return 0;
            }

            // The centre must be reachable from the army's side of the map. Probe
            // a rotating live member so one stuck or disabled scout cannot veto it.
            NavigationPath guide = null;
            for (int i = 0; i < units.Count; i++)
            {
                var unit = units[(planIndex + i) % units.Count].Unit;
                if (unit == null || !unit.isActiveAndEnabled || !Grid.CanOccupy(new NavigationPose(unit.Pose.Cell), small))
                    continue;
                guide = finder.Find(new NavigationRequest(new NavigationPose(target),
                    new NavigationPose(unit.Pose.Cell), small, budget));
                break;
            }
            if (guide == null || guide.Status != NavigationStatus.Success)
            {
                if (guide?.Status != NavigationStatus.BudgetExceeded) lastRejectedTarget = target;
                targetReport = $"Target {target}: {guide?.Status ?? NavigationStatus.InvalidStart}.";
                return guide?.Expanded ?? 0;
            }

            acceptedTarget = goal = target;
            targetReport = "Accepted";
            targetArea = planner.BuildTargetArea(target, Mathf.Clamp(groupRadius, 0, 32));
            sharedCorridor.Clear();
            foreach (var point in guide.Points)
            for (int z = -2; z <= 2; z++)
            for (int x = -2; x <= 2; x++) sharedCorridor.Add(point.Pose.Cell + new Vector2Int(x, z));
            foreach (var entry in units)
            {
                entry.Assigned = false;
                entry.Failures = 0;
                entry.RetryAt = 0;
                entry.LastAttempt = null;
            }
            targetExpanded = guide.Expanded;
            targetMs = 0;
            return guide.Expanded;
        }

        private int PlanParticipant(Participant entry, bool needsSlot, int budget)
        {
            var unit = entry.Unit;
            bool avoidAllies = unit.Recovering && entry.Failures % 3 != 2;
            bool AllowsEdge(NavigationPose from, NavigationPose to) => unit.AllowsSearchEdge(from, to, avoidAllies);
            NavigationPath path;
            int expanded;
            if (needsSlot)
            {
                if (entry.Reserved) NavigationGroupPlanner.Reserve(entry.Destination.Value, unit.Footprint, reservations, false);
                memberRequest[0] = new NavigationMember(unit.Pose, unit.Footprint);
                var result = planner.Plan(memberRequest, acceptedTarget.Value, groupRadius,
                    budget, budget, 48, reservations, targetArea, sharedCorridor, AllowsEdge);
                path = result.Paths[0];
                expanded = result.Expanded;
                if (path.Status == NavigationStatus.Success)
                {
                    var destination = path.Points[path.Points.Count - 1].Pose;
                    if (unit.Assign(path))
                    {
                        entry.Destination = destination;
                        entry.Reserved = true;
                        entry.Assigned = true;
                    }
                    else NavigationGroupPlanner.Reserve(destination, unit.Footprint, reservations, false);
                }
                if (!entry.Assigned && entry.Reserved)
                    NavigationGroupPlanner.Reserve(entry.Destination.Value, unit.Footprint, reservations);
            }
            else
            {
                path = finder.Find(new NavigationRequest(unit.Pose, entry.Destination.Value, unit.Footprint, budget),
                    null, null, allowsEdge: AllowsEdge);
                expanded = path.Expanded;
                if (path.Status == NavigationStatus.Success) unit.Assign(path);
            }

            entry.LastAttempt = path.Status;
            entry.LastTrafficRevision = navigation.TrafficRevision;
            entry.Failures = path.Status == NavigationStatus.Success ? 0 : Math.Min(entry.Failures + 1, 8);
            if (!needsSlot && targetArea != null && entry.Failures == 3)
            {
                // A destination can remain boxed in after traffic has moved. Offer
                // another non-overlapping slot in the same command area.
                entry.Assigned = false;
                entry.Failures = 0;
            }
            entry.RetryAt = Time.time + 0.18f * (1 << Math.Min(4, entry.Failures));
            if (path.Status != NavigationStatus.Success) unit.ReportFailure(path.Status);
            return expanded;
        }

        private void DrawTrafficReport()
        {
            int routes = 0, waiting = 0, blocked = 0, arrived = 0, budget = 0, noPath = 0, accepted = 0;
            foreach (var entry in units)
            {
                var unit = entry.Unit;
                if (unit == null || !unit.isActiveAndEnabled) continue;
                if (unit.HasRoute) routes++;
                if (unit.State == UnitMovement.Motion.Arrived) arrived++;
                if (unit.State == UnitMovement.Motion.Waiting || unit.State == UnitMovement.Motion.Yielding
                    || unit.State == UnitMovement.Motion.RepathPending || (!entry.Assigned && targetArea != null
                    && !entry.LastAttempt.HasValue)) waiting++;
                if (unit.State == UnitMovement.Motion.Blocked || unit.State == UnitMovement.Motion.Invalidated) blocked++;
                if (entry.LastAttempt == NavigationStatus.BudgetExceeded) budget++;
                if (entry.LastAttempt == NavigationStatus.NoPath || entry.LastAttempt == NavigationStatus.InvalidGoal
                    || entry.LastAttempt == NavigationStatus.InvalidStart) noPath++;
                if (entry.Assigned) accepted++;
            }
            GUILayout.Label($"{units.Count} units · routes {routes} · waiting/recovery {waiting} · arrived {arrived} · blocked {blocked}");
            if (!armyDemo)
            {
                GUILayout.Label($"NoPath/invalid {noPath} · BudgetExceeded {budget} · expansions {frameExpanded}/{totalExpanded}");
                return;
            }
            string target = acceptedTarget.HasValue
                ? $"{acceptedTarget.Value}, level {Grid.Surface.GetCell(acceptedTarget.Value.x, acceptedTarget.Value.y).Level}"
                : "none";
            GUILayout.Label($"Follow Mouse {(followMouse ? "ON" : "OFF")} · target {target} · {targetReport}");
            GUILayout.Label($"Accepted {accepted} · NoPath/invalid {noPath} · BudgetExceeded {budget}");
            GUILayout.Label($"Planning {targetMs:F1} ms · expansions frame {frameExpanded}, target {targetExpanded}, total {totalExpanded}");
        }

        private void OnGUI()
        {
            if (!showPanel || !Application.isPlaying) return;
            GUILayout.BeginArea(Panel, GUI.skin.box);
            scroll = GUILayout.BeginScrollView(scroll);
            GUILayout.Label("Stage 05.1 — Terrain and group traffic");
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
            if (laneGrid != null && GUILayout.Button("Release lane")) ReleaseLane();
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
                    }
                }
                DrawTrafficReport();
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
            GUILayout.Label($"Request: {start} {startFacing} → {goal} {goalFacing}; footprint {footprint}");
            GUILayout.Label(report ?? "No navigation snapshot.");
            GUILayout.EndScrollView(); GUILayout.EndArea();
        }
    }
}
