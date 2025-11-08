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

    private bool drifting = false;
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
        inputAxis = new Vector2(Input.GetAxis("right", "left"),
            Input.GetAxis("backward", "forward"));

        drifting = Input.IsActionPressed("drift");

        // Update booster positions up and down depending on hit distance with min-max to be spring-like
        // This is just visual, so not implemented yet. Core physics is in _IntegrateForces.

        /*foreach (var booster in Boosters)
        {
            Color color = booster.Ray.IsColliding() ? new Color(255, 0, 0) : new Color(0, 0, 255);
            DebugDraw3D.DrawLine(booster.GlobalPosition, booster.GlobalPosition + booster.Ray.TargetPosition, color);
        }*/
    }

    Vector3 kartVelocity = Vector3.Zero;
    Vector3 kartTorque = Vector3.Zero;
    
    public override void _IntegrateForces(PhysicsDirectBodyState3D state)
    {
        float step = (float)state.Step;
        
        // 1. Gravity (omni-directional): Manually add to velocity
        Vector3 gravity = GravityDirection.Normalized() * GravityStrength;
        kartVelocity += gravity * step;

        // 2. Damping (manual; Godot does NOT handle this for custom integrator)
        Vector3 curVel = kartVelocity;
        float linDamp = LinearDamp * step;
        curVel -= curVel * linDamp;
        kartVelocity = curVel;

        Vector3 curAngVel = kartTorque;
        float angDamp = AngularDamp * step;
        curAngVel -= curAngVel * angDamp;
        kartTorque = curAngVel;

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

            applyBosterForce(state, booster, ref totalBoosterForce, ref totalBoosterTorque);
        }
        
        // 4. Movement (add kart driving force based on inputAxis)
        // inputAxis.X = left/right (steering)
        // inputAxis.Y = forward/back
        if (inputAxis.LengthSquared() > 0.0001f)
        {
            Vector3 forward = -GlobalTransform.Basis.Z.Normalized();
            Vector3 right = GlobalTransform.Basis.X.Normalized();
            Vector3 up = GlobalTransform.Basis.Y.Normalized();

            // Steering: apply torque to turn (around local up)
            float steerInput = inputAxis.X;
            
            if (Mathf.Abs(steerInput) > 0.01f)
            {
                float steerAmount = steerInput * Handling; // Scale by handling stat
                
                // Apply torque for steering (add a "spin" to the kart around up axis)
                // You can tweak the divisor for tuning how quick it turns.
                kartTorque += up * steerAmount * step;
            }
            
            // Forward/backward movement along kart's local "forward" (usually -Z)
            // Calculate drive force
            float driveInput = inputAxis.Y; // Godot: Y is forward
            Vector3 driveForce = forward * driveInput * Acceleration * Mass; // F = ma

            if (!drifting)
            {
                // Dampen sideways movement
            }
            
            // Apply drive force (adds to velocity like boosters do)
            kartVelocity += (driveForce / Mass) * step;
        }

        kartVelocity +=  (totalBoosterForce / Mass) * step;
        kartTorque += (totalBoosterTorque / Mass) * step * 0.1f;
        
        state.LinearVelocity = kartVelocity;
        state.AngularVelocity = kartTorque;
    }

    private void applyBosterForce(PhysicsDirectBodyState3D state, HoverBooster booster, ref Vector3 totalBoosterForce, ref Vector3 totalBoosterTorque)
    {
        // Get booster world position
        Vector3 boosterWorldPos = booster.GlobalTransform.Origin;

        // Get surface hit position and normal (use normal for curved surfaces; fallback to -gravity)
        Vector3 hitPoint = booster.Ray.GetCollisionPoint();
        Vector3 surfaceNormal = booster.Ray.GetCollisionNormal();
        Vector3 springDir = Vector3.Zero;
        
        // If ray hitting push up, else pull down.
        if (booster.Ray.IsColliding())
            springDir = surfaceNormal.Normalized();
        else
            springDir = -surfaceNormal.Normalized();

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