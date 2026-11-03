using System;
using System.Collections.Generic;
using UnityEngine;

namespace GameDev.RtsRpg
{
    /// <summary>Stage 04 debug display only. No units, click picking, RTS input or gameplay commands.</summary>
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
        [SerializeField] private bool showBoundaries = true;
        [SerializeField] private bool showPanel = true;
        [SerializeField] private RectInt blocker = new RectInt(12, 4, 4, 4);
        private TerrainNavigation navigation;
        private TerrainBuilder builder;
        private readonly List<NavigationPath> paths = new List<NavigationPath>();
        private readonly List<GameObject> lines = new List<GameObject>();
        private Material ownedMaterial;
        private int blockerHandle;
        private bool groupMode;
        private string report = "Choose a demo or edit the request in Inspector, then Replan.";
        private Vector2 scroll;
        private NavigationGrid Grid => navigation != null ? navigation.Grid : null;

        private void OnEnable()
        {
            navigation = GetComponent<TerrainNavigation>(); builder = GetComponent<TerrainBuilder>();
            navigation.Changed += Invalidate;
            if (lineMaterial != null) ownedMaterial = new Material(lineMaterial) { hideFlags = HideFlags.DontSave };
        }
        private void Start() => Replan();
        private void OnDisable()
        {
            if (navigation != null) navigation.Changed -= Invalidate;
            ClearLines(); paths.Clear();
            if (ownedMaterial != null) Release(ownedMaterial);
            ownedMaterial = null;
        }
        private static void Release(UnityEngine.Object obj)
        { if (Application.isPlaying) Destroy(obj); else DestroyImmediate(obj); }
        private void ClearLines() { foreach (var line in lines) if (line != null) Release(line); lines.Clear(); }
        private void Invalidate()
        {
            paths.Clear(); ClearLines();
            // Handles belong to the old grid only. Never remove an old handle from a new map.
            if (Grid == null || !Grid.IsAlive) blockerHandle = 0;
            report = "Navigation changed. Previous results invalidated. Press Replan explicitly.";
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
        private void DrawPath(NavigationPath path, NavigationFootprint f, Color color)
        {
            paths.Add(path);
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
        private void DrawGeometry()
        {
            if (Grid == null) return;
            var surface = Grid.Surface;
            int displayed = 0;
            if (showBoundaries)
            for (int z = 0; z < surface.CellsZ && displayed < 5000; z++)
            for (int x = 0; x < surface.CellsX && displayed < 5000; x++)
            for (int d = 0; d < 4; d++)
            {
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
            ClearLines(); paths.Clear();
            if (Grid == null) { report = navigation != null ? navigation.Error : "Navigation unavailable."; return; }
            DrawGeometry();
            if (groupMode) { PlanGroup(); return; }
            var f = new NavigationFootprint(footprint.x, footprint.y);
            var path = new NavigationPathfinder(Grid).Find(new NavigationRequest(new NavigationPose(start, startFacing),
                new NavigationPose(goal, goalFacing), f, maxExpanded, allowPartial));
            DrawPath(path, f, Color.cyan);
            report = $"{path.Status}: {path.Reason}\nExpanded {path.Expanded}; cost {path.Cost}; points {path.Points.Count}; revision {path.Revision}";
        }
        private void PlanGroup()
        {
            var small = new NavigationFootprint(1, 1); var large = new NavigationFootprint(2, 2);
            var members = new[] {
                new NavigationMember(new NavigationPose(start), small),
                new NavigationMember(new NavigationPose(start + Vector2Int.right), small),
                new NavigationMember(new NavigationPose(start + Vector2Int.up), small),
                new NavigationMember(new NavigationPose(start + new Vector2Int(2, 0)), large) };
            var result = new NavigationGroupPlanner(Grid).Plan(members, goal, groupRadius);
            report = $"Group {result.Status}: {result.Accepted}/{members.Length} assigned; {result.Expanded} expansions; shared guide {result.Corridor.Count} cells";
            for (int i = 0; i < members.Length; i++)
            {
                report += $"\n{i}: {result.Paths[i].Status} — {result.Paths[i].Reason}";
                DrawPath(result.Paths[i], members[i].Footprint, Color.HSVToRGB(i * 0.21f, 0.85f, 1));
            }
        }
        private void ExplicitScenario(int mode)
        {
            builder.ShowExplicit(); groupMode = false; groupRadius = 12;
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
            }
            Replan();
        }
        private void ToggleBlocker()
        {
            if (Grid == null) return;
            if (blockerHandle != 0)
            {
                Grid.RemoveBlocker(blockerHandle); blockerHandle = 0;
                DrawGeometry(); report = "Blocker removed; old paths invalidated. Press Replan.";
            }
            else
            {
                if (!Grid.TryAddBlocker(blocker, out int handle, out string error)) { report = error; return; }
                blockerHandle = handle;
                DrawGeometry(); report = "Blocker added; old paths invalidated. Press Replan.";
            }
        }
        private void Seed(int seed)
        {
            builder.ShowProceduralSeed(seed); groupMode = true; groupRadius = 12;
            start = new Vector2Int(5, 5); goal = new Vector2Int(34, 34); Replan();
        }
        private void OnGUI()
        {
            if (!showPanel || !Application.isPlaying) return;
            GUILayout.BeginArea(new Rect(12, 12, 420, Math.Min(Screen.height - 24, 560)), GUI.skin.box);
            scroll = GUILayout.BeginScrollView(scroll);
            GUILayout.Label("Stage 04 — Navigation debug (no unit movement)");
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
            if (GUILayout.Button("N/E/S/W map")) { builder.ShowDirections(); groupMode = false; report = "Edit start/goal in Inspector; Replan for each ramp."; DrawGeometry(); }
            GUILayout.EndHorizontal(); GUILayout.BeginHorizontal();
            if (GUILayout.Button("Request 1x1")) { footprint = Vector2Int.one; groupMode = false; Replan(); }
            if (GUILayout.Button("Request 1x2 N")) { footprint = new Vector2Int(1, 2); startFacing = goalFacing = TerrainDirection.North; groupMode = false; Replan(); }
            if (GUILayout.Button("Request 1x2 E")) { footprint = new Vector2Int(1, 2); startFacing = goalFacing = TerrainDirection.East; groupMode = false; Replan(); }
            if (GUILayout.Button("Request 2x2")) { footprint = new Vector2Int(2, 2); groupMode = false; Replan(); }
            GUILayout.EndHorizontal(); GUILayout.BeginHorizontal();
            if (GUILayout.Button("Add/remove blocker")) ToggleBlocker();
            if (GUILayout.Button("Replan")) Replan();
            GUILayout.EndHorizontal();
            bool visible = GUILayout.Toggle(showBoundaries, "Show impassable geometry (display capped at 5000 segments)");
            if (visible != showBoundaries) { showBoundaries = visible; ClearLines(); DrawGeometry(); paths.Clear(); report = "Display changed; press Replan to redraw routes."; }
            GUILayout.Label($"Request: {start} {startFacing} → {goal} {goalFacing}; footprint {footprint}");
            GUILayout.Label(report ?? "No navigation snapshot.");
            GUILayout.EndScrollView(); GUILayout.EndArea();
        }
    }
}
