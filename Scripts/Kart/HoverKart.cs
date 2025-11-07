using Godot;
using System;
using System.Collections.Generic;

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
    [Export] public float m_LinearDamp = 2.5f;
    [Export] public float m_AngularDamp = 2.5f;

    [ExportCategory("Kart objects")]
    [Export] public Godot.Collections.Array<HoverBooster> Boosters;

    public override void _Ready()
    {
        Mass = Weight;

        foreach (var booster in Boosters)
            booster.Ray.TargetPosition = new Vector3(0f, -BoosterRayLength, 0f);
        
        LinearDamp = m_LinearDamp;
        AngularDamp = m_AngularDamp;

        DebugDraw3D.ScopedConfig().SetThickness(0f).SetCenterBrightness(0.1f);
    }

    public override void _Process(double delta)
    {
        // Update booster positions up and down depending on hit distance with min-max to be spring-like
        // This is just visual, so not implemented yet. Core physics is in _IntegrateForces.

        foreach (var booster in Boosters)
        {
            Color color = booster.Ray.IsColliding() ? new Color(255, 0, 0) : new Color(0, 0, 255);
            DebugDraw3D.DrawLine(booster.GlobalPosition, booster.GlobalPosition + booster.Ray.TargetPosition, color);
        }
    }

    public override void _IntegrateForces(PhysicsDirectBodyState3D state)
    {
        // 1. Gravity (omni-directional)
        Vector3 gravity = GravityDirection.Normalized() * GravityStrength;
        state.LinearVelocity += gravity * (float)state.Step;

        // 2. Damping (manual; Godot does NOT handle this for custom integrator)
        Vector3 curVel = state.LinearVelocity;
        float linDamp = LinearDamp * (float)state.Step;
        curVel -= curVel * linDamp;
        state.LinearVelocity = curVel;

        Vector3 curAngVel = state.AngularVelocity;
        float angDamp = AngularDamp * (float)state.Step;
        curAngVel -= curAngVel * angDamp;
        state.AngularVelocity = curAngVel;

        // 3. Boosters (spring + damper)
        foreach (var booster in Boosters)
            ApplyBoosterForce(state, booster);
    }

    private void ApplyBoosterForce(PhysicsDirectBodyState3D state, HoverBooster booster)
    {
        // Null check
        if (booster == null || booster.Ray == null)
        {
            return;
        }

        // Only boost when ray hits something
        if (booster.Ray.IsColliding())
        {
            // Get booster world position
            Vector3 boosterWorldPos = booster.GlobalTransform.Origin;

            // Get surface hit position
            Vector3 hitPoint = booster.Ray.GetCollisionPoint();

            // Calculate distance from booster to surface
            float hitDistance = boosterWorldPos.DistanceTo(hitPoint);

            // Spring displacement: positive means we need to push upward
            float displacement = BoosterRayLength - hitDistance;

            // Correct spring direction: local TargetPosition in world space
            //Vector3 springDir = booster.GlobalTransform.Basis * booster.Ray.TargetPosition.Normalized();
            
            Vector3 springDir = GravityDirection.Normalized() * -1.0f; // Always up, opposite of gravity
            
            // Ensure direction is normalized; flip if needed (should point up, away from surface)
            springDir = springDir.Normalized();

            // The local position of the booster, from kart center
            Vector3 localBoosterPos = boosterWorldPos - GlobalTransform.Origin;

            // Get the velocity (linear) at the booster location
            Vector3 boosterVelocity = state.GetVelocityAtLocalPosition(localBoosterPos);

            // Project velocity onto spring direction to get vertical component
            float verticalSpeed = boosterVelocity.Dot(springDir);

            // Spring force using Hooke's law
            float springForce = displacement * BoosterSpringStrength;

            // Damping force to prevent oscillation
            float dampForce = -verticalSpeed * BoosterSpringDamp;

            // Total force to apply: up toward desired hover point
            Vector3 totalForce = springDir * (springForce + dampForce);
            
            // Apply the force at the location of the booster
            //state.ApplyForce(totalForce, localBoosterPos);
            
            Vector3 up = -GravityDirection.Normalized(); 
            state.ApplyForce(up * 10000f, boosterWorldPos - GlobalTransform.Origin);
        }
    }
}