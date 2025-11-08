using Godot;
using System;

/// <summary>
/// A smooth camera follow script supporting omni-directional/omni-gravity environments.
/// It follows a target (kart) in 3D space, keeping a desired distance/height/angle regardless of world up/down.
/// Avoids reliance on any hard-coded Y axis.
/// Attach this script to a Camera3D node. Set the Target property in the editor.
/// </summary>
public partial class SmoothFollowCamera : Camera3D
{
    // TODO: Up direction needs to be based off of the karts current gravity vector!
    [ExportGroup("Camera Follow Settings")]
    [Export] public Node3D Target;             // The kart to follow
    [Export] public float Distance = 6.0f;     // Desired follow distance
    [Export] public float Height = 2.2f;       // Height above target (in local target "up" direction)
    [Export] public float SmoothTime = 0.25f;  // Follow smoothing (seconds)
    [Export] public float LookDamp = 0.15f;    // Smooth damp for look direction

    private Vector3 currentVelocity = Vector3.Zero;
    private Vector3 lookVelocity = Vector3.Zero;
    private Vector3 desiredLookDir = Vector3.Forward;

    public override void _Process(double delta)
    {
        // Null check
        if (Target == null)
        {
            return;
        }

        // Assume the "up" direction for the camera matches the kart's up
        // (If you want a different ref vector, pass that here)
        Transform3D targetXf = Target.GlobalTransform;

        // The forward direction = -Z axis (as per Godot convention)
        Vector3 kartForward = -targetXf.Basis.Z.Normalized();
        Vector3 kartUp = targetXf.Basis.Y.Normalized();

        // Camera desired position: behind the target in its own local space
        Vector3 desiredPos = Target.GlobalTransform.Origin
                             - kartForward * Distance
                             + kartUp * Height;

        // Smoothly interpolate camera position (classic SmoothDamp emulation)
        Vector3 newPos = GlobalTransform.Origin;
        newPos = newPos.Lerp(desiredPos, 1.0f - Mathf.Exp(-((float)delta) / Mathf.Max(SmoothTime, 0.001f)));
        GlobalTransform = new Transform3D(GlobalTransform.Basis, newPos);

        // Get where to look: target position plus height offset (feels better)
        Vector3 lookTarget = Target.GlobalTransform.Origin + kartUp * (Height * 0.4f);

        // Optional: Smooth out look direction for less abrupt rotation (especially on flips)
        desiredLookDir = (lookTarget - GlobalTransform.Origin).Normalized();
        Vector3 curLook = -GlobalTransform.Basis.Z.Normalized();
        Vector3 smoothLook = curLook.Lerp(desiredLookDir, 1f - Mathf.Exp(-((float)delta) / Mathf.Max(LookDamp, 0.0001f)));

        // Create a new basis matching this view: Z = back, Y = world up, X = right
        // Careful to avoid gimbal locking; recalc X from up x z
        Vector3 newZ = -smoothLook.Normalized();
        Vector3 newY = kartUp; // preserve kart's up for gimbal-stable camera
        Vector3 newX = newY.Cross(newZ).Normalized();
        newY = newZ.Cross(newX).Normalized();

        Basis basis = new Basis(newX, newY, newZ);
        GlobalTransform = new Transform3D(basis, newPos);
    }
}