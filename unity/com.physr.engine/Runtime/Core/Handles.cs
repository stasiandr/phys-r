using System;
using System.Runtime.InteropServices;

namespace PhysR
{
    /// <summary>Generational body handle (0 = invalid). Safe to send over the network.</summary>
    [Serializable]
    [StructLayout(LayoutKind.Sequential)]
    public struct BodyHandle : IEquatable<BodyHandle>
    {
        public ulong Value;

        public BodyHandle(ulong v) { Value = v; }
        public bool IsValid { get { return Value != 0; } }
        public uint Index { get { return (uint)Value; } }
        public uint Generation { get { return (uint)(Value >> 32); } }

        public static readonly BodyHandle Invalid = new BodyHandle(0);
        public static bool operator ==(BodyHandle a, BodyHandle b) { return a.Value == b.Value; }
        public static bool operator !=(BodyHandle a, BodyHandle b) { return a.Value != b.Value; }
        public bool Equals(BodyHandle o) { return Value == o.Value; }
        public override bool Equals(object o) { return o is BodyHandle && ((BodyHandle)o).Value == Value; }
        public override int GetHashCode() { return Value.GetHashCode(); }
        public override string ToString() { return "Body(" + Index + "v" + Generation + ")"; }
    }

    /// <summary>Generational character handle (0 = invalid).</summary>
    [Serializable]
    [StructLayout(LayoutKind.Sequential)]
    public struct CharacterHandle : IEquatable<CharacterHandle>
    {
        public ulong Value;

        public CharacterHandle(ulong v) { Value = v; }
        public bool IsValid { get { return Value != 0; } }

        public static readonly CharacterHandle Invalid = new CharacterHandle(0);
        public static bool operator ==(CharacterHandle a, CharacterHandle b) { return a.Value == b.Value; }
        public static bool operator !=(CharacterHandle a, CharacterHandle b) { return a.Value != b.Value; }
        public bool Equals(CharacterHandle o) { return Value == o.Value; }
        public override bool Equals(object o) { return o is CharacterHandle && ((CharacterHandle)o).Value == Value; }
        public override int GetHashCode() { return Value.GetHashCode(); }
        public override string ToString() { return "Character(" + (uint)Value + "v" + (uint)(Value >> 32) + ")"; }
    }

    /// <summary>Index into the world's shape registry.</summary>
    [Serializable]
    [StructLayout(LayoutKind.Sequential)]
    public struct ShapeId : IEquatable<ShapeId>
    {
        public uint Value;

        public ShapeId(uint v) { Value = v; }
        public bool IsValid { get { return Value != uint.MaxValue; } }

        public static readonly ShapeId Invalid = new ShapeId(uint.MaxValue);
        public static bool operator ==(ShapeId a, ShapeId b) { return a.Value == b.Value; }
        public static bool operator !=(ShapeId a, ShapeId b) { return a.Value != b.Value; }
        public bool Equals(ShapeId o) { return Value == o.Value; }
        public override bool Equals(object o) { return o is ShapeId && ((ShapeId)o).Value == Value; }
        public override int GetHashCode() { return (int)Value; }
        public override string ToString() { return "Shape(" + Value + ")"; }
    }
}
