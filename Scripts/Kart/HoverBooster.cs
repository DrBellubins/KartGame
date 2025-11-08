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
            return;

        // Determine desired distance offset
        float hitDist = Ray.Enabled && Ray.IsColliding()
            ? GlobalTransform.Origin.DistanceTo(Ray.GetCollisionPoint())
            : 0f;
        
        // Bobbing: offset along local Y axis based on how close the booster is to the surface
        // Closer = lower offset, further = higher offset (max out at DistanceBobbingMax)
        float targetYOffset = 0f;

        if (Ray.Enabled && Ray.IsColliding())
        {
            // Linearly map 0..Ray.TargetPosition.Length() to 0..DistanceBobbingMax
            float rayLen = Ray.TargetPosition.Length();
            float ratio = Mathf.Clamp((rayLen - hitDist) / rayLen, 0.0f, 1.0f);
            
            targetYOffset = ratio * DistanceBobbingMax;
        }

        // Smooth interpolation for bobbing
        _currentZOffset = Mathf.Lerp(_currentZOffset, targetYOffset, 1.0f - Mathf.Exp(-DistanceBobbingSpeed * (float)delta));

        // Animation: Move the mesh locally along its up axis (local Y) by _currentZOffset
        Vector3 meshOrigin = Vector3.Zero + (Vector3.Up * _currentZOffset);

        // Determine desired normal (for mesh alignment)
        Vector3 targetNormal = Vector3.Up;

        if (Ray.Enabled && Ray.IsColliding())
        {
            targetNormal = Ray.GetCollisionNormal().Normalized();
        }

        // Clamp the max alignment angle for visuals
        float angle = Mathf.RadToDeg(_currentNormal.AngleTo(targetNormal));
        
        if (angle > NormalRotationAngleMax)
        {
            // Limit excessive snapping/tilting
            targetNormal = _currentNormal.Slerp(targetNormal, NormalRotationAngleMax / angle);
        }

        // Smooth the normal for smooth rotation
        _currentNormal = _currentNormal.Slerp(targetNormal, 1.0f - Mathf.Exp(-NormalRotationSpeed * (float)delta));
        _currentNormal = _currentNormal.Normalized();

        // Build the mesh's local basis: Y = up/normal, X = right, Z = forward
        Vector3 forward = -GlobalTransform.Basis.Z;
        Vector3 right = _currentNormal.Cross(forward).Normalized();

        // If right vector is degenerate (booster perfectly vertical), choose fallback
        if (right.LengthSquared() < 0.001f)
        {
            right = GlobalTransform.Basis.X;
        }

        Vector3 newForward = right.Cross(_currentNormal).Normalized();
        Basis meshBasis = new Basis(right, _currentNormal, newForward);

        // Apply transform to mesh (keeping it local within booster node)
        Mesh.Transform = new Transform3D(meshBasis, meshOrigin);
    }
}
