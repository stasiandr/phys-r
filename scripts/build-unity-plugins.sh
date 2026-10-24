#!/usr/bin/env bash
# Build the native phys-r library for Unity platforms and install it into
# unity/com.physr.engine/Plugins (with Unity .meta import settings).
#
# Usage: scripts/build-unity-plugins.sh [android] [ios] [macos] [windows] [linux] [host]
#        (no arguments = host platform only)
#
# Requirements per platform:
#   android : Android NDK (ANDROID_NDK_HOME) + `cargo install cargo-ndk`
#             + rustup targets aarch64-linux-android armv7-linux-androideabi x86_64-linux-android
#   ios     : macOS + Xcode + rustup target aarch64-apple-ios
#   macos   : macOS + rustup targets aarch64-apple-darwin x86_64-apple-darwin (universal dylib)
#   windows : Windows host (MSVC) or `cargo install cargo-xwin` on other hosts
#   linux   : Linux host (x86_64)
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PKG="$ROOT/unity/com.physr.engine"
OUT="$PKG/Plugins"
CRATE=phys-r-ffi
cd "$ROOT"

targets=("$@")
if [ ${#targets[@]} -eq 0 ]; then targets=(host); fi

host_os() {
  case "$(uname -s)" in
    Darwin) echo macos ;;
    Linux) echo linux ;;
    MINGW*|MSYS*|CYGWIN*) echo windows ;;
    *) echo unknown ;;
  esac
}

build_android() {
  command -v cargo-ndk >/dev/null || { echo "cargo-ndk not found: cargo install cargo-ndk" >&2; exit 1; }
  rustup target add aarch64-linux-android armv7-linux-androideabi x86_64-linux-android
  mkdir -p "$OUT/Android"
  # cargo-ndk writes <abi>/libphysr.so under the output directory.
  cargo ndk -t arm64-v8a -t armeabi-v7a -t x86_64 -o "$OUT/Android" build --release -p "$CRATE"
}

build_ios() {
  rustup target add aarch64-apple-ios
  cargo build --release -p "$CRATE" --target aarch64-apple-ios
  mkdir -p "$OUT/iOS"
  cp "target/aarch64-apple-ios/release/libphysr.a" "$OUT/iOS/libphysr.a"
}

build_macos() {
  rustup target add aarch64-apple-darwin x86_64-apple-darwin
  cargo build --release -p "$CRATE" --target aarch64-apple-darwin
  cargo build --release -p "$CRATE" --target x86_64-apple-darwin
  mkdir -p "$OUT/macOS"
  lipo -create \
    "target/aarch64-apple-darwin/release/libphysr.dylib" \
    "target/x86_64-apple-darwin/release/libphysr.dylib" \
    -output "$OUT/macOS/libphysr.dylib"
}

build_windows() {
  mkdir -p "$OUT/Windows/x86_64"
  if [ "$(host_os)" = windows ]; then
    cargo build --release -p "$CRATE" --target x86_64-pc-windows-msvc
  else
    command -v cargo-xwin >/dev/null || { echo "cross-building Windows needs: cargo install cargo-xwin" >&2; exit 1; }
    rustup target add x86_64-pc-windows-msvc
    cargo xwin build --release -p "$CRATE" --target x86_64-pc-windows-msvc
  fi
  cp "target/x86_64-pc-windows-msvc/release/physr.dll" "$OUT/Windows/x86_64/physr.dll"
}

build_linux() {
  cargo build --release -p "$CRATE" --target x86_64-unknown-linux-gnu
  mkdir -p "$OUT/Linux/x86_64"
  cp "target/x86_64-unknown-linux-gnu/release/libphysr.so" "$OUT/Linux/x86_64/libphysr.so"
}

for t in "${targets[@]}"; do
  if [ "$t" = host ]; then t="$(host_os)"; fi
  echo "==> building $t"
  case "$t" in
    android) build_android ;;
    ios) build_ios ;;
    macos) build_macos ;;
    windows) build_windows ;;
    linux) build_linux ;;
    *) echo "unknown target '$t'" >&2; exit 1 ;;
  esac
done

python3 "$ROOT/scripts/gen_unity_meta.py" "$PKG"
echo "==> plugins installed in $OUT"
find "$OUT" -type f ! -name '*.meta' -exec ls -la {} \;
