using PhysR;
using UnityEngine;

/// <summary>
/// Minimal FPS player on top of PhysRCharacter (uses the legacy Input Manager).
///
/// Shows the pattern for networked games: inputs are QUANTISED (sbyte axes,
/// centi-degree yaw/pitch) and rebuilt with exact fixed-point ratios, so the
/// value the server simulates is bit-identical to the one predicted locally.
/// </summary>
[RequireComponent(typeof(PhysRCharacter))]
public class SimpleFpsPlayer : MonoBehaviour
{
    public Transform cameraPivot;
    public float eyeHeight = 0.7f;
    public float mouseSensitivity = 2f;
    public float shotRange = 200f;
    public float shotImpulse = 20f;
    [Tooltip("Hits above this height (relative to the capsule centre) count as headshots.")]
    public float headshotHeight = 0.55f;

    PhysRCharacter character;
    float yawDeg, pitchDeg;
    bool fireQueued;

    // What would be sent to the server each tick.
    public struct NetInput
    {
        public sbyte moveX, moveZ;
        public short yawCentiDeg, pitchCentiDeg;
        public byte buttons;
    }

    void Awake()
    {
        character = GetComponent<PhysRCharacter>();
        Cursor.lockState = CursorLockMode.Locked;
    }

    void OnEnable()
    {
        if (PhysRWorld.Instance != null) PhysRWorld.Instance.BeforeTick += OnBeforeTick;
    }

    void OnDisable()
    {
        if (PhysRWorld.Instance != null) PhysRWorld.Instance.BeforeTick -= OnBeforeTick;
    }

    void Update()
    {
        yawDeg = Mathf.Repeat(yawDeg + Input.GetAxisRaw("Mouse X") * mouseSensitivity, 360f);
        pitchDeg = Mathf.Clamp(pitchDeg - Input.GetAxisRaw("Mouse Y") * mouseSensitivity, -89f, 89f);
        if (Input.GetButtonDown("Fire1")) fireQueued = true;
        if (cameraPivot != null)
        {
            // Transform = capsule centre; when crouched the centre drops by half
            // the height difference, so the eye drops by the same amount again.
            float crouchOffset = character.Handle.IsValid ? (character.height - character.Height.ToFloat()) * 0.5f : 0f;
            cameraPivot.position = transform.position + Vector3.up * (eyeHeight - crouchOffset);
            cameraPivot.rotation = Quaternion.Euler(pitchDeg, yawDeg, 0f);
        }
    }

    NetInput Sample()
    {
        NetInput n;
        n.moveX = (sbyte)Mathf.RoundToInt(Mathf.Clamp(Input.GetAxisRaw("Horizontal"), -1f, 1f) * 127f);
        n.moveZ = (sbyte)Mathf.RoundToInt(Mathf.Clamp(Input.GetAxisRaw("Vertical"), -1f, 1f) * 127f);
        n.yawCentiDeg = (short)Mathf.RoundToInt((yawDeg > 180f ? yawDeg - 360f : yawDeg) * 100f);
        n.pitchCentiDeg = (short)Mathf.RoundToInt(pitchDeg * 100f);
        n.buttons = (byte)((Input.GetButton("Jump") ? 1 : 0) | (Input.GetKey(KeyCode.LeftShift) ? 2 : 0) | (Input.GetKey(KeyCode.LeftControl) ? 4 : 0));
        return n;
    }

    /// <summary>Exactly what the server would do with a received NetInput.</summary>
    public static CharacterInput ToCharacterInput(NetInput n)
    {
        return new CharacterInput
        {
            MoveX = FP.FromRatio(n.moveX, 127),
            MoveZ = FP.FromRatio(n.moveZ, 127),
            Yaw = FP.FromRatio(n.yawCentiDeg, 100) * FP.Deg2Rad,
            Buttons = (CharacterButtons)n.buttons,
        };
    }

    void OnBeforeTick(PhysRWorld world)
    {
        NetInput n = Sample();
        character.NextInput = ToCharacterInput(n);
        if (fireQueued)
        {
            fireQueued = false;
            Fire(world, n);
        }
    }

    void Fire(PhysRWorld world, NetInput n)
    {
        FP yaw = FP.FromRatio(n.yawCentiDeg, 100) * FP.Deg2Rad;
        FP pitch = FP.FromRatio(n.pitchCentiDeg, 100) * FP.Deg2Rad;
        FPVector3 dir = FPQuaternion.Euler(pitch, yaw, FP.Zero) * FPVector3.Forward;
        var state = character.State;
        FP crouchOffset = (character.height.ToFP() - state.Height) * FP.Half;
        FPVector3 eye = state.Position + FPVector3.Up * (eyeHeight.ToFP() - crouchOffset);
        var filter = new QueryFilter(uint.MaxValue, character.Body);
        RayHit hit;
        // On a server, use RaycastRewind(clientRenderTick, alpha, ...) instead.
        if (world.World.Raycast(eye, dir, shotRange.ToFP(), filter, out hit))
        {
            Debug.DrawLine(eye.ToUnity(), hit.Point.ToUnity(), Color.red, 0.5f);
            CharacterHandle victim = world.World.CharacterOfBody(hit.Body);
            if (victim.IsValid)
            {
                bool headshot = hit.LocalPoint.Y > headshotHeight.ToFP();
                Debug.Log((headshot ? "HEADSHOT " : "hit ") + victim);
                return;
            }
            var body = world.FindBody(hit.Body);
            if (body != null && body.kind == BodyKind.Dynamic)
                body.AddImpulseAtPoint(dir * shotImpulse.ToFP(), hit.Point);
        }
    }
}
