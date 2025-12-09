using Godot;
using System;

public partial class HoverBooster : Node3D
{
    [ExportCategory("Functional")]
    [Export] public bool IsFront;
    [Export] public ShapeCast3D ShapeCast;
    [Export] public MeshInstance3D Mesh;

    [ExportCategory("Visual")]
    [Export] public float BobbingSpeed = 8f;
    [Export] public float BobbingMax = 0.25f;
    [Export] public float RotationSpeed = 10f;
    [Export] public float RotationAngleMax = 45f; // In degrees (not directly used in advanced rotation)

    // Public for access from HoverKart
    public float currentZOffset = 0.5f;
    public Vector3 currentNormal = Vector3.Up;

    private float _bobTime = 0f;
    private Quaternion _meshRot = Quaternion.Identity;
    private const float HoverRefDist = 0.5f; // Reference for compression scaling

    public override void _Process(double delta)
    {
        if (Mesh == null)
            return;

        // Bobbing animation: sinusoidal vertical offset, reduced when close to surface
        _bobTime += (float)delta * BobbingSpeed;
        float bob = Mathf.Sin(_bobTime) * BobbingMax;
        float compression = Mathf.Clamp(1f - (currentZOffset / HoverRefDist), 0f, 1f);
        bob *= (1f - compression * 0.7f); // Less bobbing when compressed
        Mesh.Position = new Vector3(0f, bob, 0f);

        // Rotation: smooth alignment of mesh -Y axis towards surface (-currentNormal)
        Vector3 fromDir = Vector3.Down;
        Vector3 toDir = -currentNormal.Normalized();
        float dot = fromDir.Dot(toDir);
        
        if (Mathf.Abs(dot) < 0.999f)
        {
            Vector3 axis = fromDir.Cross(toDir).Normalized();
            float angle = Mathf.Acos(Mathf.Clamp(dot, -1f, 1f));
            Quaternion targetQuat = new Quaternion(axis, angle);
            _meshRot = _meshRot.Slerp(targetQuat, (float)delta * RotationSpeed);
        }
        
        Mesh.Quaternion = _meshRot;
    }
}