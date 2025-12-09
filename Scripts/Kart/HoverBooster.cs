using Godot;
using System;

public partial class HoverBooster : Node3D
{
    [ExportCategory("Functional")]
    [Export] public bool IsFront;
    [Export] public ShapeCast3D ShapeCast;
    [Export] public MeshInstance3D Mesh;

    [ExportCategory("Visual")]
    [Export] public float BobbingSpeed = 8f;
    [Export] public float BobbingMax = 0.25f;
    [Export] public float RotationSpeed = 10f;
    [Export] public float RotationAngleMax = 45f; // In degrees

    // Store for smoothing and for fallback if not colliding
    private float _currentZOffset = 0f;
    private Vector3 _currentNormal = Vector3.Up;
    private Vector3 _lastValidNormal = Vector3.Up;

    public override void _Process(double delta)
    {
        if (Mesh == null || ShapeCast == null)
            return;

        // TODO: Hover booster animation.
        // Bob up/down at BobbingSpeed, max height BobbingMax
        // Rotate to hit normal at RotationSpeed, max angle RotationAngleMax
    }
}