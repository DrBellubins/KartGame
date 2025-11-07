using Godot;
using System;

public partial class HoverKart : RigidBody3D
{
    [ExportCategory("Kart specs")]
    [Export] public float Weight = 125f;
    [Export] public float MaxSpeed = 10f;
    [Export] public float MaxAcceleration = 10f;

    [ExportCategory("Kart physics")]
    [Export] public float Gravity = -9.81f;
    [Export] public float BoosterRayLength = 0.5f;
    
    [ExportCategory("Kart objects")]
    [Export] public HoverBooster FrontLeftBooster;
    [Export] public HoverBooster FrontRightBooster;
    [Export] public HoverBooster BackLeftBooster;
    [Export] public HoverBooster BackRightBooster;

    public override void _Ready()
    {
        Mass = Weight;
        
        FrontLeftBooster.Ray.TargetPosition = new Vector3(0f, -BoosterRayLength, 0f);
        FrontRightBooster.Ray.TargetPosition = new Vector3(0f, -BoosterRayLength, 0f);
        BackLeftBooster.Ray.TargetPosition = new Vector3(0f, -BoosterRayLength, 0f);
        BackRightBooster.Ray.TargetPosition = new Vector3(0f, -BoosterRayLength, 0f);
    }

    public override void _Process(double delta)
    {
        
    }
}
