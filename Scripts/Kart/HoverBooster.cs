using Godot;
using System;

public partial class HoverBooster : Node3D
{
    [ExportCategory("Functional")]
    [Export] public bool IsFront;
    [Export] public RayCast3D Ray;
    [Export] public MeshInstance3D Mesh;
    
    [ExportCategory("Visual")]
    [Export] public float DistanceBobbingSpeed = 2f;
    [Export] public float NormalRotationSpeed = 2f;
    [Export] public float DistanceBobbingMax = 0.25f;
    [Export] public float NormalRotationAngleMax = 45f; // In degrees

    public override void _Process(double delta)
    {
        
    }
}
