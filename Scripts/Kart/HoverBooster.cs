using Godot;
using System;

public partial class HoverBooster : Node3D
{
    [Export] public RayCast3D Ray;

    public override void _Ready()
    {
        
    }

    public override void _Process(double delta)
    {
        // TODO: Move the booster up and down depending on hit distance with min-max to be spring-like
    }
}
