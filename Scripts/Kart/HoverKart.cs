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
    [Export] public float StopDrag = 20f; // Controls deceleration when no drive input

    [ExportCategory("Kart physics")]
    [Export] public float GravityStrength = 9.81f;
    [Export] public Vector3 GravityDirection = new Vector3(0f, -1f, 0f);
    [Export] public float BoosterSphereRadius = 1f; // Radius of sphere shape for overlap detection
    [Export] public float BoosterMidPointDist = 0.25f; // Desired hover distance (equilibrium)
    [Export] public float BoosterSpringStrength = 800f; // Increased for stability
    [Export] public float BoosterSpringDamp = 50f;
    [Export] public float m_LinearDamp = 1.5f;
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
                booster.ShapeCast.TargetPosition = Vector3.Zero; // Overlap mode
                // Assume SphereShape3D(radius = BoosterSphereRadius) set in editor
                // Set MaxResults = 32 in editor for multi-collision support
            }
        }

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
        float step = (float)state.Step;
        kartVelocity = state.LinearVelocity;
        kartTorque = state.AngularVelocity;

        ApplyGravity(step);
        ApplyDamping(step);
        ApplyMovement(step);
        ClampMaxSpeed();
        ApplyBoosterForces(state, step);

        state.LinearVelocity = kartVelocity;
        state.AngularVelocity = kartTorque;
    }

    private void ApplyGravity(float step)
    {
        Vector3 gravity = GravityDirection.Normalized() * GravityStrength;
        kartVelocity += gravity * step;
    }

    private void ApplyDamping(float step)
    {
        float linDamp = m_LinearDamp * step;
        kartVelocity *= (1f - linDamp);

        float angDamp = m_AngularDamp * step;
        kartTorque *= (1f - angDamp);
    }

    private void ApplyMovement(float step)
    {
        Vector3 forward = -GlobalTransform.Basis.Z.Normalized();
        Vector3 right = GlobalTransform.Basis.X.Normalized();
        Vector3 up = GlobalTransform.Basis.Y.Normalized();

        float steerInput = inputAxis.X;
        float driveInput = inputAxis.Y;
        float currentForwardSpeed = kartVelocity.Dot(forward);
        float targetSpeed = Mathf.Min(Speed, MaxSpeed);

        // Steering (always processed if input present)
        if (Mathf.Abs(steerInput) > 0.01f)
        {
            float steerAmount = steerInput * Handling;
            kartTorque += up * steerAmount * step;
        }

        // Driving or stop drag
        if (Mathf.Abs(driveInput) > 0.01f)
        {
            // Acceleration/deceleration
            Vector3 driveForce = Vector3.Zero;
            if (driveInput > 0f && currentForwardSpeed < targetSpeed)
            {
                float speedFactor = Mathf.Clamp((targetSpeed - currentForwardSpeed) / targetSpeed, 0f, 1f);
                driveForce = forward * driveInput * Acceleration * Mass * speedFactor;
            }
            else if (driveInput < 0f && currentForwardSpeed > -targetSpeed)
            {
                float speedFactor = Mathf.Clamp((targetSpeed + currentForwardSpeed) / targetSpeed, 0f, 1f);
                driveForce = forward * driveInput * Acceleration * Mass * speedFactor;
            }

            if (driveForce != Vector3.Zero)
            {
                kartVelocity += (driveForce / Mass) * step;
            }
        }
        else
        {
            // Stop drag: strong forward damping when no drive input
            if (Mathf.Abs(currentForwardSpeed) > 0.1f)
            {
                Vector3 brakeAccel = -forward * currentForwardSpeed * StopDrag * step;
                kartVelocity += brakeAccel;
            }
        }

        // Traction: damp lateral velocity unless drifting (always processed)
        if (!drifting)
        {
            float lateralSpeed = kartVelocity.Dot(right);
            if (Mathf.Abs(lateralSpeed) > 0.1f)
            {
                float sideDamp = Traction * step;
                kartVelocity -= right * lateralSpeed * sideDamp;
            }
        }
    }

    private void ClampMaxSpeed()
    {
        if (kartVelocity.Length() > MaxSpeed)
        {
            kartVelocity = kartVelocity.Normalized() * MaxSpeed;
        }
    }

    private void ApplyBoosterForces(PhysicsDirectBodyState3D state, float step)
    {
        Vector3 totalForce = Vector3.Zero;
        Vector3 totalTorque = Vector3.Zero;

        foreach (var booster in Boosters)
        {
            if (booster?.ShapeCast == null)
                continue;

            ApplySingleBoosterForce(state, booster, ref totalForce, ref totalTorque);
        }

        kartVelocity += totalForce / Mass * step;
        kartTorque += totalTorque / Mass * step * 0.1f;
    }

    private void ApplySingleBoosterForce(PhysicsDirectBodyState3D state, HoverBooster booster, ref Vector3 totalForce, ref Vector3 totalTorque)
    {
        booster.ShapeCast.ForceShapecastUpdate(); // Immediate overlap update

        int collisionCount = booster.ShapeCast.GetCollisionCount();
        if (collisionCount == 0)
        {
            ApplyVirtualBoosterForce(state, booster, ref totalForce, ref totalTorque);
            return;
        }

        Vector3 boosterWorldPos = booster.GlobalPosition;
        Vector3 localBoosterPos = booster.Position;
        Vector3 boosterVelocity = state.GetVelocityAtLocalPosition(localBoosterPos);
        Vector3 downDir = GravityDirection.Normalized();

        float minDist = float.MaxValue;
        Vector3 closestNormal = Vector3.Zero;
        bool hasRealCollision = false;

        for (int i = 0; i < collisionCount; i++)
        {
            Vector3 hitPoint = booster.ShapeCast.GetCollisionPoint(i);
            Vector3 surfaceNormal = booster.ShapeCast.GetCollisionNormal(i).Normalized();

            Vector3 relPos = hitPoint - boosterWorldPos;
            float hitDist = relPos.Length();
            if (hitDist >= BoosterSphereRadius)
                continue; // Outside sphere

            // Filter: only surfaces "below" (relPos projects downward)
            float projDown = relPos.Dot(downDir);
            if (projDown <= 0f)
                continue;

            // Closeness multiplier: linear falloff, strongest at center
            float multiplier = Mathf.Max(0f, 1f - (hitDist / BoosterSphereRadius));

            // Spring: negative displacement for attraction/repulsion
            float displacement = hitDist - BoosterMidPointDist;
            float springForce = -displacement * BoosterSpringStrength * multiplier;

            // Damping: oppose relative velocity along normal (normal points toward booster)
            float separationSpeed = boosterVelocity.Dot(surfaceNormal);
            float dampForce = -separationSpeed * BoosterSpringDamp * multiplier;

            Vector3 hitForce = surfaceNormal * (springForce + dampForce);

            totalForce += hitForce;

            // Accurate torque at hit point
            Vector3 rHit = hitPoint - GlobalPosition;
            totalTorque += rHit.Cross(hitForce);

            hasRealCollision = true;

            // Track closest for visuals
            if (hitDist < minDist)
            {
                minDist = hitDist;
                closestNormal = surfaceNormal;
            }
        }

        // Virtual ground if no real collisions (ensures "stick" behavior)
        if (!hasRealCollision)
        {
            ApplyVirtualBoosterForce(state, booster, ref totalForce, ref totalTorque);
            return;
        }

        // Update visuals with real closest hit
        booster.currentZOffset = minDist;
        booster.currentNormal = closestNormal;
    }

    private void ApplyVirtualBoosterForce(PhysicsDirectBodyState3D state, HoverBooster booster, ref Vector3 totalForce, ref Vector3 totalTorque)
    {
        Vector3 boosterWorldPos = booster.GlobalPosition;
        Vector3 localBoosterPos = booster.Position;
        Vector3 boosterVelocity = state.GetVelocityAtLocalPosition(localBoosterPos);
        Vector3 downDir = GravityDirection.Normalized();
        Vector3 surfaceNormal = -downDir; // Upward normal for virtual flat ground below

        float virtualDist = BoosterSphereRadius;
        float virtualMultiplier = 0.3f; // Softer for virtual to prevent oscillation

        float displacement = virtualDist - BoosterMidPointDist;
        float springForce = -displacement * BoosterSpringStrength * virtualMultiplier;
        float separationSpeed = boosterVelocity.Dot(surfaceNormal);
        float dampForce = -separationSpeed * BoosterSpringDamp * virtualMultiplier;

        Vector3 boosterForce = surfaceNormal * (springForce + dampForce);

        totalForce += boosterForce;

        Vector3 rWorld = boosterWorldPos - GlobalPosition;
        totalTorque += rWorld.Cross(boosterForce);

        // Visuals: simulate distant hover
        booster.currentZOffset = virtualDist;
        booster.currentNormal = surfaceNormal;
    }
}