using Godot;
using System;

public partial class HoverBooster : Node3D
{
    [ExportCategory("Functional")]
    [Export] public bool IsFront;
    [Export] public RayCast3D Ray;
    [Export] public MeshInstance3D Mesh;

    [ExportCategory("Visual")]
    [Export] public float DistanceBobbingSpeed = 8f;
    [Export] public float NormalRotationSpeed = 10f;
    [Export] public float DistanceBobbingMax = 0.25f;
    [Export] public float NormalRotationAngleMax = 45f; // In degrees

    // For smoothing movement/rotation
    private float _currentZOffset = 0f;
    private Vector3 _currentNormal = Vector3.Up;

    public override void _Process(double delta)
    {
        if (Mesh == null || Ray == null)
        {
            return;
        }

        // Determine desired distance offset
        float hitDist = 0f;

        if (Ray.Enabled && Ray.IsColliding())
        {
            hitDist = GlobalTransform.Origin.DistanceTo(Ray.GetCollisionPoint());
        }

        // Bobbing: offset along local Y axis based on how close the booster is to the surface
        float targetYOffset = 0f;

        if (Ray.Enabled && Ray.IsColliding())
        {
            float rayLen = Ray.TargetPosition.Length();
            float ratio = Mathf.Clamp((rayLen - hitDist) / rayLen, 0.0f, 1.0f);
            targetYOffset = ratio * DistanceBobbingMax;
        }

        // Smooth interpolation for bobbing
        _currentZOffset = Mathf.Lerp(_currentZOffset, targetYOffset, 1.0f - Mathf.Exp(-DistanceBobbingSpeed * (float)delta));

        Vector3 meshOrigin = Vector3.Zero + (Vector3.Up * _currentZOffset);

        // Get the new normal from the raycast, or just Up if no hit
        Vector3 targetNormal = Vector3.Up;
        if (Ray.Enabled && Ray.IsColliding())
        {
            targetNormal = Ray.GetCollisionNormal();
        }

        // Defensive: avoid nearly zero-length vectors!
        if (targetNormal.LengthSquared() < 0.0001f)
        {
            targetNormal = Vector3.Up;
        }
        else
        {
            targetNormal = targetNormal.Normalized();
        }

        if (_currentNormal.LengthSquared() < 0.0001f)
        {
            _currentNormal = Vector3.Up;
        }
        else
        {
            _currentNormal = _currentNormal.Normalized();
        }

        // Clamp max alignment, but only if both are normalized and nonzero
        float angle = Mathf.RadToDeg(_currentNormal.AngleTo(targetNormal));
        if (angle > NormalRotationAngleMax)
        {
            float t = NormalRotationAngleMax / angle;
            if (_currentNormal.LengthSquared() < 0.99f || _currentNormal.LengthSquared() > 1.01f)
            {
                _currentNormal = _currentNormal.Normalized();
            }
            if (targetNormal.LengthSquared() < 0.99f || targetNormal.LengthSquared() > 1.01f)
            {
                targetNormal = targetNormal.Normalized();
            }
            
            // Only slerp if both are valid, else hard snap.
            if (_currentNormal.LengthSquared() > 0.999f && targetNormal.LengthSquared() > 0.999f)
            {
                // Godot C# wants mathematically normalized
                targetNormal = _currentNormal.Slerp(targetNormal, t).Normalized();
            }
            else
            {
                targetNormal = Vector3.Up;
            }
        }

        // Now smooth the normal toward the target one, again with full guards
        if (_currentNormal.LengthSquared() > 0.999f && targetNormal.LengthSquared() > 0.999f)
        {
            _currentNormal = _currentNormal.Slerp(targetNormal, 1.0f - Mathf.Exp(-NormalRotationSpeed * (float)delta));
            _currentNormal = _currentNormal.Normalized();
        }
        else
        {
            _currentNormal = targetNormal.Normalized();
        }

        // Visual basis calculation as before
        Vector3 forward = -GlobalTransform.Basis.Z;
        Vector3 right = _currentNormal.Cross(forward).Normalized();

        // Handle degenerate cross for perfectly vertical
        if (right.LengthSquared() < 0.001f)
        {
            right = GlobalTransform.Basis.X;
        }

        Vector3 newForward = right.Cross(_currentNormal).Normalized();
        Basis meshBasis = new Basis(right, _currentNormal, newForward);

        // Apply transform to mesh (local visual effect only)
        Mesh.Transform = new Transform3D(meshBasis, meshOrigin);
    }
}