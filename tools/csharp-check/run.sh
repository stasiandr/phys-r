#!/usr/bin/env bash
# Compile-check the Unity package against a UnityEngine stub and run the
# engine-agnostic bindings test (FP math parity + full API round trip) on Mono
# against the native library. Requires `mcs` and `mono` (mono-complete).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
PKG="$ROOT/unity/com.physr.engine"
OUT="$ROOT/target/csharp-check"
mkdir -p "$OUT"
cd "$ROOT"
cargo build --release -p phys-r-ffi

echo "==> compile-check Unity package (UNITY_EDITOR)"
mcs -nologo -langversion:7.2 -warnaserror -define:UNITY_EDITOR -target:library -out:"$OUT/PhysR.UnityCheck.dll" \
  "$ROOT/tools/csharp-check/UnityStub.cs" $(find "$PKG" -name '*.cs')

echo "==> compile-check Unity package (player)"
mcs -nologo -langversion:7.2 -warnaserror -target:library -out:"$OUT/PhysR.UnityCheckPlayer.dll" \
  "$ROOT/tools/csharp-check/UnityStub.cs" $(find "$PKG" -name '*.cs')

echo "==> bindings test"
mcs -nologo -langversion:7.2 -r:System.Numerics.dll -out:"$OUT/BindingsTest.exe" \
  "$ROOT/tools/csharp-check/BindingsTest.cs" "$PKG"/Runtime/Math/*.cs "$PKG"/Runtime/Native/*.cs "$PKG"/Runtime/Core/*.cs
cp "$ROOT/target/release/libphysr.so" "$OUT/"
cd "$OUT" && LD_LIBRARY_PATH="$OUT" mono BindingsTest.exe
