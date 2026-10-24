// Minimal UnityEngine API surface used by the package, for compile checks only.
using System;
namespace UnityEngine {
  public class Object { public string name; public int GetInstanceID() { return 0; } }
  public class Component : Object { public Transform transform; public GameObject gameObject; public T GetComponent<T>() { return default(T); } public T[] GetComponentsInChildren<T>(bool inactive) { return new T[0]; } }
  public class Behaviour : Component { public bool enabled; }
  public class MonoBehaviour : Behaviour { }
  public class GameObject : Object { public int layer; }
  public class Transform : Component { public Vector3 localPosition, position, lossyScale, localScale; public Quaternion localRotation, rotation; public Transform parent; public int GetSiblingIndex() { return 0; } public void SetPositionAndRotation(Vector3 p, Quaternion r) {} }
  public struct Vector3 { public float x, y, z; public Vector3(float x, float y, float z) { this.x = x; this.y = y; this.z = z; }
    public static Vector3 zero, one, up, down;
    public static Vector3 operator +(Vector3 a, Vector3 b) { return a; } public static Vector3 operator -(Vector3 a, Vector3 b) { return a; } public static Vector3 operator -(Vector3 a) { return a; }
    public static Vector3 operator *(Vector3 a, float b) { return a; } public static Vector3 operator *(float b, Vector3 a) { return a; }
    public static Vector3 LerpUnclamped(Vector3 a, Vector3 b, float t) { return a; } public static Vector3 Scale(Vector3 a, Vector3 b) { return a; } }
  public struct Quaternion { public float x, y, z, w; public Quaternion(float x, float y, float z, float w) { this.x = x; this.y = y; this.z = z; this.w = w; }
    public static Quaternion SlerpUnclamped(Quaternion a, Quaternion b, float t) { return a; } public static Quaternion Euler(float x, float y, float z) { return default(Quaternion); } }
  public struct Color { public Color(float r, float g, float b) {} public static Color green, cyan, yellow, magenta, red; }
  public struct Matrix4x4 { public static Matrix4x4 TRS(Vector3 p, Quaternion q, Vector3 s) { return default(Matrix4x4); } }
  public struct LayerMask { public int value; public static implicit operator LayerMask(int v) { return new LayerMask { value = v }; } }
  public class Mesh : Object { public Vector3[] vertices; public int[] triangles; }
  public class Collider : Component { public bool isTrigger; public bool enabled; }
  public class BoxCollider : Collider { public Vector3 center, size; }
  public class SphereCollider : Collider { public Vector3 center; public float radius; }
  public class CapsuleCollider : Collider { public Vector3 center; public float radius, height; public int direction; }
  public class MeshFilter : Component { public Mesh sharedMesh; }
  public static class Mathf { public static float Max(params float[] v) { return 0; } public static int Max(int a, int b) { return a; } public static float Clamp01(float v) { return v; } public static float Clamp(float v, float a, float b) { return v; } public static int NextPowerOfTwo(int v) { return v; } public static float Repeat(float a, float b) { return a; } public static int RoundToInt(float v) { return 0; } public static float Abs(float v) { return v; } }
  public static class Debug { public static void Log(object o) {} public static void LogWarning(object o, Object c = null) {} public static void LogError(object o, Object c = null) {} public static void DrawLine(Vector3 a, Vector3 b, Color c, float d) {} }
  public static class Time { public static float deltaTime; }
  public static class Gizmos { public static Color color; public static Matrix4x4 matrix; public static void DrawWireCube(Vector3 c, Vector3 s) {} public static void DrawWireSphere(Vector3 c, float r) {} public static void DrawLine(Vector3 a, Vector3 b) {} public static void DrawWireMesh(Mesh m) {} }
  public enum KeyCode { LeftShift, LeftControl }
  public enum CursorLockMode { None, Locked }
  public static class Cursor { public static CursorLockMode lockState; }
  public static class Input { public static float GetAxisRaw(string s) { return 0; } public static bool GetButtonDown(string s) { return false; } public static bool GetButton(string s) { return false; } public static bool GetKey(KeyCode k) { return false; } }
  public class HeaderAttribute : Attribute { public HeaderAttribute(string s) {} }
  public class TooltipAttribute : Attribute { public TooltipAttribute(string s) {} }
  public class RangeAttribute : Attribute { public RangeAttribute(float a, float b) {} }
  public class SerializeField : Attribute {}
  public class DisallowMultipleComponent : Attribute {}
  public class DefaultExecutionOrder : Attribute { public DefaultExecutionOrder(int o) {} }
  public class RequireComponent : Attribute { public RequireComponent(Type t) {} }
}
