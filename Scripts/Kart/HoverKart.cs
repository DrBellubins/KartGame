using Godot;
using System;
using System.Collections.Generic;

public partial class HoverKart : RigidBody3D
{
    [ExportCategory("Kart specs")]
    [Export] public float Speed = 10f;
    [Export] public float MaxSpeed = 20f;
    [Export] public float Acceleration = 10f;
    [Export] public float Weight = 125f;
    [Export] public float Handling = 5f;
    [Export] public float Traction = 5f;

    [ExportCategory("Kart physics")]
    [Export] public float GravityStrength = 9.81f;
    [Export] public Vector3 GravityDirection = new Vector3(0f, -1f, 0f);
    [Export] public float BoosterRayLength = 0.5f;
    [Export] public float BoosterMidPointDist = 0.25f;  // Equilibrium distance for hover; ray checks against this midpoint
    [Export] public float BoosterSpringStrength = 400f;
    [Export] public float BoosterSpringDamp = 30f;
    [Export] public float m_LinearDamp = 2.5f;
    [Export] public float m_AngularDamp = 2.5f;

    [ExportCategory("Kart objects")]
    [Export] public Godot.Collections.Array<HoverBooster> Boosters;

    private bool drifting = false;
    private Vector2 inputAxis = Vector2.Zero;

    // Persistent velocity and angular velocity for custom integration
    private Vector3 kartVelocity = Vector3.Zero;
    private Vector3 kartTorque = Vector3.Zero;
    
    public override void _Ready()
    {
        Mass = Weight;

        foreach (var booster in Boosters)
        {
            if (booster?.ShapeCast != null)
            {
                booster.ShapeCast.TargetPosition = new Vector3(0f, -BoosterRayLength, 0f);
            }
        }

        // Note: Do not set LinearDamp/AngularDamp on the body, as we handle damping manually
        // and the engine skips built-in damping in custom _IntegrateForces

        DebugDraw3D.ScopedConfig().SetThickness(0f).SetCenterBrightness(0.1f);
    }

    public override void _Process(double delta)
    {
        inputAxis = new Vector2(Input.GetAxis("right", "left"),
            Input.GetAxis("backward", "forward"));

        drifting = Input.IsActionPressed("drift");
    }

    public override void _IntegrateForces(PhysicsDirectBodyState3D state)
    {
        // Sync with current physics state velocities at the start of this step
        // (Redundant if maintaining fields manually, but ensures sync if external changes occur)
        float step = (float)state.Step;
        kartVelocity = state.LinearVelocity;
        kartTorque = state.AngularVelocity;

        // Apply physics steps in sequence to match original order
        ApplyGravity(step);
        ApplyDamping(step);
        ApplyMovement(step);  // Apply movement before boosters to match original

        // Clamp after movement but before boosters (original clamps old state ineffectively;
        // this clamps the updated kartVelocity for intended behavior)
        ClampMaxSpeed();

        ApplyBoosterForces(state, step);  // Boosters last, as in original

        // Update state with integrated values
        state.LinearVelocity = kartVelocity;
        state.AngularVelocity = kartTorque;
    }

    // Applies omni-directional gravity to the kart's velocity
    private void ApplyGravity(float step)
    {
        Vector3 gravity = GravityDirection.Normalized() * GravityStrength;
        kartVelocity += gravity * step;
    }

    // Applies manual linear and angular damping to prevent excessive motion
    private void ApplyDamping(float step)
    {
        // Linear damping (exponential decay)
        float linDamp = m_LinearDamp * step;
        kartVelocity *= (1f - linDamp);

        // Angular damping
        float angDamp = m_AngularDamp * step;
        kartTorque *= (1f - angDamp);
    }

    // Handles input-based steering, driving, and traction (lateral damping unless drifting)
    private void ApplyMovement(float step)
    {
        if (inputAxis.LengthSquared() <= 0.0001f)
        {
            return; // No input
        }

        Vector3 forward = -GlobalTransform.Basis.Z.Normalized();
        Vector3 right = GlobalTransform.Basis.X.Normalized();
        Vector3 up = GlobalTransform.Basis.Y.Normalized();

        // Steering: add angular velocity increment around local up
        float steerInput = inputAxis.X;
        if (Mathf.Abs(steerInput) > 0.01f)
        {
            float steerAmount = steerInput * Handling;
            kartTorque += up * steerAmount * step;
        }

        // Driving: accelerate/decelerate along forward, respecting target speed
        float driveInput = inputAxis.Y;
        float currentForwardSpeed = kartVelocity.Dot(forward);
        float targetSpeed = Mathf.Min(Speed, MaxSpeed); // Cruise speed limit

        Vector3 driveForce = Vector3.Zero;
        if (driveInput > 0f && currentForwardSpeed < targetSpeed)
        {
            // Forward acceleration with speed ramping
            float speedFactor = Mathf.Clamp((targetSpeed - currentForwardSpeed) / targetSpeed, 0f, 1f);
            driveForce = forward * driveInput * Acceleration * Mass * speedFactor;
        }
        else if (driveInput < 0f && currentForwardSpeed > -targetSpeed)
        {
            // Reverse (symmetric for simplicity)
            float speedFactor = Mathf.Clamp((targetSpeed + currentForwardSpeed) / targetSpeed, 0f, 1f);
            driveForce = forward * driveInput * Acceleration * Mass * speedFactor;
        }

        if (driveForce != Vector3.Zero)
        {
            kartVelocity += (driveForce / Mass) * step;
        }

        // Traction: damp lateral (sideways) velocity unless drifting
        if (!drifting)
        {
            float lateralSpeed = kartVelocity.Dot(right);
            Vector3 lateralVel = right * lateralSpeed;
            float sideDamp = Traction * step;
            kartVelocity -= lateralVel * sideDamp;
        }
    }

    // Clamps overall linear speed to MaxSpeed (absolute cap)
    // Placed after movement to approximate original intent (clamps updated velocity)
    private void ClampMaxSpeed()
    {
        if (kartVelocity.Length() > MaxSpeed)
        {
            kartVelocity = kartVelocity.Normalized() * MaxSpeed;
        }
    }

    // Applies forces and torques from all hover boosters
    // Only applies if ray collides; skips non-colliding boosters (no force when ray misses)
    private void ApplyBoosterForces(PhysicsDirectBodyState3D state, float step)
    {
        Vector3 totalForce = Vector3.Zero;
        Vector3 totalTorque = Vector3.Zero;

        foreach (var booster in Boosters)
        {
            if (booster?.ShapeCast == null)
            {
                continue; // Skip if null
            }

            ApplySingleBoosterForce(state, booster, ref totalForce, ref totalTorque);
        }

        // Integrate forces (F * dt / mass) and torques (approximate with mass scaling)
        kartVelocity += totalForce / Mass * step;
        kartTorque += totalTorque / Mass * step * 0.1f; // 0.1f is a tuning factor for torque sensitivity
    }

    // Applies spring + damper force for a single booster and accumulates torque
    // Refactored logic:
    // - Only if ray collides (hit within BoosterRayLength): compute forces
    // - BoosterRayLength: max ray cast distance
    // - BoosterMidPointDist: equilibrium hover distance
    // - If hitDistance < BoosterMidPointDist: push away from surface (+spring along surface normal)
    // - If hitDistance > BoosterMidPointDist: pull toward surface (-spring along surface normal)
    // - Damping always opposes separation velocity along surface normal for stability
    // - No force if !colliding (ray misses entirely)
    private void ApplySingleBoosterForce(PhysicsDirectBodyState3D state, HoverBooster booster, ref Vector3 totalForce, ref Vector3 totalTorque)
    {
        bool colliding = booster.ShapeCast.IsColliding();
        if (!colliding)
        {
            return; // No booster force if ray doesn't hit anything
        }

        Vector3 boosterWorldPos = booster.GlobalPosition;
        //Vector3 hitPoint = booster.ShapeCast.GetCollisionPoint();
        //Vector3 surfaceNormal = booster.ShapeCast.GetCollisionNormal();
        Vector3 hitPoint = Vector3.Zero;
        Vector3 surfaceNormal = Vector3.Zero;

        // Calculate actual hit distance
        float hitDistance = boosterWorldPos.DistanceTo(hitPoint);

        // Local position for velocity at point of contact (relative to kart; assumes booster is child)
        Vector3 localBoosterPos = booster.Position;

        // Get the velocity at the booster location
        Vector3 boosterVelocity = state.GetVelocityAtLocalPosition(localBoosterPos);

        // Surface direction: always away from surface (for consistent damping reference)
        Vector3 surfaceDir = surfaceNormal.Normalized();

        // Separation speed: positive if moving away from surface
        float separationSpeed = boosterVelocity.Dot(surfaceDir);

        // Spring force scalar: positive for push away, negative for pull toward
        float springForce = 0f;
        if (hitDistance < BoosterMidPointDist)
        {
            // Too close: push away from surface
            float displacement = BoosterMidPointDist - hitDistance;
            springForce = displacement * BoosterSpringStrength;
        }
        else if (hitDistance > BoosterMidPointDist)
        {
            // Too far: pull toward surface
            float displacement = hitDistance - BoosterMidPointDist;
            springForce = -displacement * BoosterSpringStrength;
        }
        // Else: at equilibrium, zero spring force

        // Damping force: opposes separation (pushes away if penetrating, pulls back if separating)
        float dampForce = -separationSpeed * BoosterSpringDamp;

        // Total force along surface normal
        Vector3 boosterForce = surfaceDir * (springForce + dampForce);

        // Accumulate central force
        totalForce += boosterForce;

        // Torque: r cross F (world space)
        Vector3 rWorld = boosterWorldPos - GlobalPosition;
        totalTorque += rWorld.Cross(boosterForce);
    }
}