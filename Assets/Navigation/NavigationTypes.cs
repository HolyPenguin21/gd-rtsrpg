using System;
using System.Collections.Generic;
using System.Diagnostics;
using UnityEngine;

namespace GameDev.RtsRpg
{
    public enum NavigationStatus
    {
        Success, Partial, NoPath, InvalidRequest, InvalidStart, InvalidGoal, BudgetExceeded, Stale
    }

    public enum NavigationFacing { North, NorthEast, East, SouthEast, South, SouthWest, West, NorthWest }

    /// <summary>Occupied cells start at Pose.Cell (the lower XZ corner), not a point agent.</summary>
    public readonly struct NavigationFootprint
    {
        public int Width { get; }
        public int Length { get; }
        public bool IsValid => (Width == 1 && (Length == 1 || Length == 2)) || (Width == 2 && Length == 2);
        public NavigationFootprint(int width, int length) { Width = width; Length = length; }
        public Vector2Int Size(NavigationFacing facing)
        {
            if (!NeedsTurns) return new Vector2Int(Width, Length);
            // The rotated 1×2 body fits inside this conservative cell envelope.
            if (((int)facing & 1) != 0) return new Vector2Int(3, 3);
            return ((int)facing & 2) == 0 ? new Vector2Int(Width, Length) : new Vector2Int(Length, Width);
        }
        public Vector2Int Size(TerrainDirection facing) => Size((NavigationFacing)((int)facing * 2));
        internal int Orientations => NeedsTurns ? 8 : 1;
        internal int Graph => NeedsTurns ? 2 : Width - 1;
        public bool NeedsTurns => Width != Length;
    }

    public readonly struct NavigationPose : IEquatable<NavigationPose>
    {
        public Vector2Int Cell { get; }
        public NavigationFacing Facing { get; }
        internal float Yaw => (int)Facing * 45f;
        internal static readonly Vector2Int[] Steps = {
            new(0, 1), new(1, 1), new(1, 0), new(1, -1),
            new(0, -1), new(-1, -1), new(-1, 0), new(-1, 1)
        };
        public NavigationPose(Vector2Int cell, NavigationFacing facing = NavigationFacing.North)
        { Cell = cell; Facing = facing; }
        public NavigationPose(Vector2Int cell, TerrainDirection facing) : this(cell, (NavigationFacing)((int)facing * 2)) { }
        internal NavigationPose Turn(int direction) => new(Cell, (NavigationFacing)(((int)Facing + direction + 8) & 7));
        internal Vector2 Centre(NavigationFootprint footprint) => (Vector2)Cell + (Vector2)footprint.Size(Facing) * 0.5f;
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
        private readonly NavigationFootprint footprint;
        internal bool BelongsTo(NavigationGrid grid, NavigationFootprint size) =>
            ReferenceEquals(owner, grid) && footprint.Width == size.Width && footprint.Length == size.Length;
        public int Revision { get; }
        public bool IsCurrent => owner.IsAlive && Revision == owner.Revision;
        public NavigationStatus Status { get; }
        public string Reason { get; }
        public IReadOnlyList<NavigationPoint> Points { get; }
        public int Expanded { get; }
        public int Cost { get; }
        internal NavigationPath(NavigationGrid grid, NavigationStatus status, string reason,
            List<NavigationPoint> points = null, int expanded = 0, int cost = 0, NavigationFootprint footprint = default)
        {
            this.footprint = footprint;
            owner = grid; Revision = grid.Revision; Status = status; Reason = reason;
            Points = points == null ? Array.Empty<NavigationPoint>() : points.AsReadOnly(); Expanded = expanded; Cost = cost;
        }
    }

    /// <summary>One shared, consumable cap for a group, including failed destination candidates.</summary>
    public sealed class NavigationBudget
    {
        public int Remaining { get; private set; }
        private long deadline, slice;
        public NavigationBudget(int expanded) => Reset(expanded);
        internal bool HasTime => deadline == 0 || Stopwatch.GetTimestamp() < slice;
        internal void BeginSlice(long ticks) => slice = Math.Min(deadline, Stopwatch.GetTimestamp() + ticks);
        internal void EndSlice() => slice = deadline;
        internal bool CanContinue => Remaining > 0 && HasTime;
        internal void Reset(int expanded, long until = 0)
        {
            Remaining = Math.Max(0, Math.Min(4000000, expanded));
            deadline = slice = until;
        }
        internal bool Take()
        {
            if (Remaining == 0 || !HasTime) return false;
            Remaining--;
            return true;
        }
    }
}
