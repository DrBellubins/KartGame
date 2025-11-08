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

    private Vector2 inputAxis = Vector2.Zero;
    
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
        inputAxis = new Vector2(Input.GetAxis("backward", "forward"),
            Input.GetAxis("left", "right"));
        
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
        float step = (float)state.Step;

        // 1. Gravity (omni-directional): Manually add to velocity
        Vector3 gravity = GravityDirection.Normalized() * GravityStrength;
        state.LinearVelocity += gravity * step;

        // 2. Damping (manual; Godot does NOT handle this for custom integrator)
        Vector3 curVel = state.LinearVelocity;
        float linDamp = LinearDamp * step;
        curVel -= curVel * linDamp;
        state.LinearVelocity = curVel;

        Vector3 curAngVel = state.AngularVelocity;
        float angDamp = AngularDamp * step;
        curAngVel -= curAngVel * angDamp;
        state.AngularVelocity = curAngVel;

        // 3. Boosters (spring + damper): Manually integrate forces into velocity
        // (Accumulate total booster force/torque for simplicity; apply at end)
        Vector3 totalBoosterForce = Vector3.Zero;
        Vector3 totalBoosterTorque = Vector3.Zero;

        foreach (var booster in Boosters)
        {
            // Null check
            if (booster == null || booster.Ray == null)
            {
                continue;
            }

            // Only boost when ray hits something
            if (booster.Ray.IsColliding())
            {
                // Get booster world position
                Vector3 boosterWorldPos = booster.GlobalTransform.Origin;

                // Get surface hit position and normal (use normal for curved surfaces; fallback to -gravity)
                Vector3 hitPoint = booster.Ray.GetCollisionPoint();
                Vector3 surfaceNormal = booster.Ray.GetCollisionNormal();
                Vector3 springDir = surfaceNormal.Length() > 0 ? surfaceNormal.Normalized() : (-GravityDirection.Normalized());

                // Calculate distance from booster to surface
                float hitDistance = boosterWorldPos.DistanceTo(hitPoint);

                // Spring displacement: positive means we need to push away from surface
                float displacement = BoosterRayLength - hitDistance;
                displacement = Mathf.Clamp(displacement, 0f, BoosterRayLength);  // Prevent pulling down if ray misses slightly

                // The local position of the booster, from kart center (for torque)
                Vector3 localBoosterPos = boosterWorldPos - GlobalTransform.Origin;

                // Get the velocity (linear) at the booster location
                Vector3 boosterVelocity = state.GetVelocityAtLocalPosition(localBoosterPos);

                // Project velocity onto spring direction to get component away from surface
                float relativeSpeed = boosterVelocity.Dot(springDir);

                // Spring force using Hooke's law
                float springForce = displacement * BoosterSpringStrength;

                // Damping force to prevent oscillation
                float dampForce = -relativeSpeed * BoosterSpringDamp;

                // Total force to apply: away from surface toward desired hover point
                Vector3 boosterForce = springDir * (springForce + dampForce);

                // Accumulate central force
                totalBoosterForce += boosterForce;

                // Accumulate torque (force cross local position)
                totalBoosterTorque += localBoosterPos.Cross(boosterForce);
            }
        }

        // Manually integrate booster forces into velocity (like gravity)
        // Force / mass * step for linear; torque / inertia for angular (simplified; assumes diagonal inertia)
        state.LinearVelocity += (totalBoosterForce / Mass) * step;
        
        // Note: For full angular integration, you'd need state.PrincipalInertiaAxes and tensor math.
        // For now, add torque directly to angular velocity (tune multiplier as needed for stability)
        state.AngularVelocity += (totalBoosterTorque / Mass) * step * 0.1f;  // Scaled down to avoid over-rotation
    }
}