using System;
using System.Collections.Generic;
using UnityEngine;

namespace GameDev.RtsRpg
{
    public enum NavigationStatus
    {
        Success, Partial, NoPath, InvalidRequest, InvalidStart, InvalidGoal, BudgetExceeded, Stale
    }

    /// <summary>Occupied cells start at Pose.Cell (the lower XZ corner), not a point agent.</summary>
    public readonly struct NavigationFootprint
    {
        public int Width { get; }
        public int Length { get; }
        public bool IsValid => (Width == 1 && (Length == 1 || Length == 2)) || (Width == 2 && Length == 2);
        public NavigationFootprint(int width, int length) { Width = width; Length = length; }
        public Vector2Int Size(TerrainDirection facing) => ((int)facing & 1) == 0
            ? new Vector2Int(Width, Length) : new Vector2Int(Length, Width);
        public bool NeedsTurns => Width != Length;
    }

    public readonly struct NavigationPose : IEquatable<NavigationPose>
    {
        public Vector2Int Cell { get; }
        public TerrainDirection Facing { get; }
        public NavigationPose(Vector2Int cell, TerrainDirection facing = TerrainDirection.North)
        { Cell = cell; Facing = facing; }
        public bool Equals(NavigationPose other) => Cell == other.Cell && Facing == other.Facing;
        public override bool Equals(object obj) => obj is NavigationPose other && Equals(other);
        public override int GetHashCode() => unchecked(Cell.GetHashCode() * 397 ^ (int)Facing);
    }

    public readonly struct NavigationPoint
    {
        public NavigationPose Pose { get; }
        public Vector3 World { get; }
        public NavigationPoint(NavigationPose pose, Vector3 world) { Pose = pose; World = world; }
    }

    public readonly struct NavigationRequest
    {
        public NavigationPose Start { get; }
        public NavigationPose Goal { get; }
        public NavigationFootprint Footprint { get; }
        public int MaxExpanded { get; }
        public bool AllowPartial { get; }
        public NavigationRequest(NavigationPose start, NavigationPose goal, NavigationFootprint footprint,
            int maxExpanded = 20000, bool allowPartial = false)
        { Start = start; Goal = goal; Footprint = footprint; MaxExpanded = maxExpanded; AllowPartial = allowPartial; }
    }

    public sealed class NavigationPath
    {
        private readonly NavigationGrid owner;
        internal bool BelongsTo(NavigationGrid grid) => ReferenceEquals(owner, grid);
        public int Revision { get; }
        public bool IsCurrent => owner.IsAlive && Revision == owner.Revision;
        public NavigationStatus Status { get; }
        public string Reason { get; }
        public IReadOnlyList<NavigationPoint> Points { get; }
        public int Expanded { get; }
        public int Cost { get; }
        internal NavigationPath(NavigationGrid grid, NavigationStatus status, string reason,
            List<NavigationPoint> points = null, int expanded = 0, int cost = 0)
        {
            owner = grid; Revision = grid.Revision; Status = status; Reason = reason;
            Points = (points ?? new List<NavigationPoint>()).AsReadOnly(); Expanded = expanded; Cost = cost;
        }
    }

    /// <summary>One shared, consumable cap for a group, including failed destination candidates.</summary>
    public sealed class NavigationBudget
    {
        public int Remaining { get; private set; }
        public NavigationBudget(int expanded) { Remaining = Math.Max(0, Math.Min(1000000, expanded)); }
        internal bool Take() { if (Remaining == 0) return false; Remaining--; return true; }
    }
}
