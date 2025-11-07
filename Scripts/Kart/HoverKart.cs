using Godot;
using System;

public partial class HoverKart : RigidBody3D
{
    [ExportCategory("Kart specs")]
    [Export] public float Speed = 10f;
    [Export] public float Acceleration = 10f;
    [Export] public float Weight = 125f;
    [Export] public float Handling = 5f;
    [Export] public float Traction = 5f;

    [ExportCategory("Kart physics")]
    [Export] public float GravityStrength = 9.81f;
    [Export] public Vector3 GravityDirection = new Vector3(0f, -1f, 0f);
    [Export] public float BoosterRayLength = 0.5f;
    [Export] public float BoosterSpringStrength = 400f;
    [Export] public float BoosterSpringDamp = 30f;
    [Export] public float LinearDamp = 2.5f;
    [Export] public float AngularDamp = 2.5f;

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

        LinearDamp = this.LinearDamp;
        AngularDamp = this.AngularDamp;
    }

    public override void _Process(double delta)
    {
        // Update booster positions up and down depending on hit distance with min-max to be spring-like
        // This is just visual, so not implemented yet. Core physics is in _IntegrateForces.
    }

    public override void _IntegrateForces(PhysicsDirectBodyState3D state)
    {
        // 1. Custom omni-directional gravity (easily change direction later)
        Vector3 gravity = GravityDirection.Normalized() * GravityStrength * Mass;
        state.AddConstantCentralForce(gravity);

        // 2. Simple dampening (already set by LinearDamp/AngularDamp, but you can override here if needed)
        //state.SetLinearVelocity(LinearVelocity * LinearDamp);
        //state.SetAngularVelocity(AngularVelocity * AngularDamp);

        // 3. Apply hover spring forces at booster positions
        ApplyBoosterForce(state, FrontLeftBooster);
        ApplyBoosterForce(state, FrontRightBooster);
        ApplyBoosterForce(state, BackLeftBooster);
        ApplyBoosterForce(state, BackRightBooster);
    }

    private void ApplyBoosterForce(PhysicsDirectBodyState3D state, HoverBooster booster)
    {
        // Make sure the booster and its Ray exist
        if (booster == null || booster.Ray == null)
        {
            return;
        }

        // Only apply force if the ray is colliding
        if (booster.Ray.IsColliding())
        {
            // Get hit distance
            float hitDistance = booster.Ray.GetCollisionPoint().DistanceTo(booster.GlobalTransform.Origin);

            // Desired hover height = BoosterRayLength
            float displacement = BoosterRayLength - hitDistance;

            // If within range, apply spring force
            Vector3 springDir = booster.Ray.TargetPosition.Normalized();
            Vector3 boosterPos = booster.GlobalTransform.Origin;

            // Velocity at booster position
            Vector3 boosterVelocity = state.GetVelocityAtLocalPosition(boosterPos - GlobalTransform.Origin);

            // Spring force (Hooke's law)
            float springForce = displacement * BoosterSpringStrength;

            // Damping force (to prevent oscillation)
            float dampForce = -boosterVelocity.Dot(springDir) * BoosterSpringDamp;

            // Total force
            Vector3 force = springDir * (springForce + dampForce);

            // Apply force at booster position
            state.ApplyForce(force, boosterPos - GlobalTransform.Origin);
        }
    }
}