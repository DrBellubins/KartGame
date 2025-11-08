using Godot;
using System;

public partial class HoverBooster : Node3D
{
    [Export] public RayCast3D Ray;
    [Export] public MeshInstance3D Mesh;
}
