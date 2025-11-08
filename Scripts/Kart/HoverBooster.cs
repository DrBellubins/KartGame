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
    [Export] public float NormalRotationAngleMax = 45f; // Max angular speed in degrees per second

    // Store for smoothing and for fallback if not colliding
    private float _currentZOffset = 0f;
    private Vector3 _currentNormal = Vector3.Up;
    private Vector3 _lastValidNormal = Vector3.Up;

    public override void _Process(double delta)
    {
        if (Mesh == null || Ray == null)
        {
            return;
        }

        // Get kart reference for consistent forward/right directions
        HoverKart kart = GetParent<HoverKart>();
        if (kart == null)
        {
            return;
        }
        Vector3 kartForward = -kart.GlobalTransform.Basis.Z.Normalized();
        Vector3 kartRight = kart.GlobalTransform.Basis.X.Normalized();

        // Determine desired distance offset (bobbing based on compression)
        // When not colliding, assume full extension (no compression, no bobbing)
        float rayLen = Ray.TargetPosition.Length();
        float hitDist = rayLen; // Default to full length (no hit = no compression)
        if (Ray.Enabled && Ray.IsColliding())
        {
            hitDist = GlobalTransform.Origin.DistanceTo(Ray.GetCollisionPoint());
        }

        // Smooth bobbing: ratio 1.0 = fully compressed (bob max), 0.0 = extended (no bob)
        float ratio = Mathf.Clamp((rayLen - hitDist) / rayLen, 0.0f, 1.0f);
        float targetYOffset = ratio * DistanceBobbingMax;
        _currentZOffset = Mathf.Lerp(_currentZOffset, targetYOffset, 1.0f - Mathf.Exp(-DistanceBobbingSpeed * (float)delta));
        
        // Bobbing offset along the current normal (surface-relative, for consistency on slopes)
        Vector3 bobOffset = _currentNormal * _currentZOffset;
        Vector3 meshOrigin = Vector3.Zero + bobOffset;

        // --- Safe & Robust Normal Animation ---
        Vector3 targetNormal;

        if (Ray.Enabled && Ray.IsColliding())
        {
            Vector3 n = Ray.GetCollisionNormal();
            if (n.LengthSquared() > 0.1f)
            {
                targetNormal = n.Normalized();
                _lastValidNormal = targetNormal; // Cache for fallback
            }
            else
            {
                targetNormal = _lastValidNormal;
            }
        }
        else
        {
            // Off ground: use last valid normal, unless it's degenerate
            if (_lastValidNormal.LengthSquared() < 0.1f)
                _lastValidNormal = Vector3.Up;
            targetNormal = _lastValidNormal;
        }

        // Always ensure target is normalized
        targetNormal = targetNormal.Normalized();

        // Defensive: _currentNormal should always be valid
        if (_currentNormal.LengthSquared() < 0.1f)
        {
            _currentNormal = Vector3.Up;
        }
        _currentNormal = _currentNormal.Normalized();

        // --- Slerp with guards against anti-parallel (flipping), degenerate, or parallel cases ---
        float dp = _currentNormal.Dot(targetNormal);
        float currentAngleRad = _currentNormal.AngleTo(targetNormal);
        float currentAngleDeg = Mathf.RadToDeg(currentAngleRad);

        // Handle near-parallel (avoids zero-length axis in Slerp internal cross product)
        if (Mathf.Abs(dp) > 0.999f || currentAngleRad < Mathf.DegToRad(0.01f))
        {
            _currentNormal = targetNormal;
        }
        // If they're nearly opposite, snap instead of slerping (prevents wrong Lerp fallback)
        else if (dp < -0.95f)
        {
            _currentNormal = targetNormal;
        }
        else
        {
            // Exponential approach weight (smooth deceleration near target)
            float expoWeight = 1.0f - Mathf.Exp(-NormalRotationSpeed * (float)delta);

            // Angular speed limit: max rotation rate in rad/sec
            float maxAngularSpeedRadPerSec = NormalRotationAngleMax * Mathf.DegToRad(NormalRotationAngleMax);
            float maxDeltaRad = maxAngularSpeedRadPerSec * (float)delta;
            float maxWeight = (currentAngleRad > 0.001f) ? Mathf.Min(1.0f, maxDeltaRad / currentAngleRad) : 1.0f;

            // Use the minimum: respects both smooth approach and max speed
            float slerpWeight = Mathf.Min(expoWeight, maxWeight);

            _currentNormal = _currentNormal.Slerp(targetNormal, slerpWeight);
            _currentNormal = _currentNormal.Normalized(); // Always normalize after
        }

        // --- Basis construction: Align mesh Y to normal, Z to kart forward (projected) ---
        // Project kart forward onto plane perpendicular to normal (avoids gimbal issues)
        Vector3 projectedForward = kartForward - _currentNormal.Dot(kartForward) * _currentNormal;
        if (projectedForward.LengthSquared() > 0.001f)
        {
            projectedForward = projectedForward.Normalized();
        }
        else
        {
            // Fallback: choose a direction perpendicular to normal, preferring kart's right for consistency
            projectedForward = _currentNormal.Cross(kartRight).Normalized();
            if (projectedForward.LengthSquared() < 0.001f)
            {
                // Double fallback: arbitrary but stable
                projectedForward = _currentNormal.Cross(Vector3.Forward).Normalized();
            }
        }
        
        // Build orthonormal basis: Z = projected forward, Y = normal, X = Y cross Z (right-handed)
        Vector3 meshZ = projectedForward;
        Vector3 meshY = _currentNormal;
        Vector3 meshX = meshY.Cross(meshZ).Normalized();
        
        // Defensive check for degenerate X (should be rare after projection)
        if (meshX.LengthSquared() < 0.001f)
        {
            meshX = Vector3.Right; // Arbitrary stable fallback
            // Re-orthogonalize Z if needed: Z = X cross Y
            meshZ = meshX.Cross(meshY).Normalized();
        }
        
        // Ensure the basis Y aligns positively with _currentNormal (flip if inverted)
        if (meshY.Dot(_currentNormal) < 0.0f)
        {
            meshX = -meshX;
            meshZ = -meshZ;
            meshY = -meshY;
        }
        
        Basis meshBasis = new Basis(meshX, meshY, meshZ);

        Mesh.Transform = new Transform3D(meshBasis, meshOrigin);
    }
}