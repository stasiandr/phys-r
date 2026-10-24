#!/usr/bin/env python3
"""Generate Unity .meta files for the phys-r package.

Packages installed from git are immutable and Unity ignores any asset without
a .meta file, so every file/folder in the package needs one. GUIDs are derived
from the package-relative path, so re-running never churns them.

Native plugins get explicit platform import settings based on their folder:
  Plugins/Android/<abi>/libphysr.so   -> Android, CPU from <abi>
  Plugins/iOS/libphysr.a              -> iOS
  Plugins/macOS/libphysr.dylib        -> Editor + Standalone macOS
  Plugins/Windows/x86_64/physr.dll    -> Editor + Standalone Win64
  Plugins/Linux/x86_64/libphysr.so    -> Editor + Standalone Linux64

Usage: gen_unity_meta.py [--force] <package_dir>
"""
import hashlib
import os
import sys

TAIL = "  userData: \n  assetBundleName: \n  assetBundleVariant: \n"


def guid(rel):
    return hashlib.md5(("phys-r:" + rel).encode()).hexdigest()


def folder_meta(g):
    return f"fileFormatVersion: 2\nguid: {g}\nfolderAsset: yes\nDefaultImporter:\n  externalObjects: {{}}\n" + TAIL


def simple(g, importer):
    return f"fileFormatVersion: 2\nguid: {g}\n{importer}:\n  externalObjects: {{}}\n" + TAIL


def mono(g):
    return (
        f"fileFormatVersion: 2\nguid: {g}\nMonoImporter:\n  externalObjects: {{}}\n  serializedVersion: 2\n"
        "  defaultReferences: []\n  executionOrder: 0\n  icon: {instanceID: 0}\n" + TAIL
    )


def platform(first_key, first_val, enabled, settings):
    s = f"  - first:\n      {first_key}: {first_val}\n    second:\n      enabled: {1 if enabled else 0}\n      settings:"
    if not settings:
        return s + " {}\n"
    s += "\n"
    for k, v in settings.items():
        s += f"        {k}: {v}\n"
    return s


def plugin(g, entries):
    s = (
        f"fileFormatVersion: 2\nguid: {g}\nPluginImporter:\n  externalObjects: {{}}\n  serializedVersion: 2\n"
        "  iconMap: {}\n  executionOrder: {}\n  defineConstraints: []\n  isPreloaded: 0\n  isOverridable: 1\n"
        "  isExplicitlyReferenced: 0\n  validateReferences: 1\n  platformData:\n"
    )
    s += platform("", "Any", False, {})
    for e in entries:
        s += platform(*e)
    return s + TAIL


ANDROID_CPU = {"arm64-v8a": "ARM64", "armeabi-v7a": "ARMv7", "x86_64": "X86_64", "x86": "X86"}


def plugin_entries(rel):
    parts = rel.split("/")
    off = {"CPU": "None"}
    if "Android" in parts:
        abi = parts[parts.index("Android") + 1]
        return [
            ("Android", "Android", True, {"CPU": ANDROID_CPU.get(abi, "ARM64")}),
            ("Editor", "Editor", False, {"CPU": "AnyCPU", "OS": "AnyOS"}),
        ]
    if "iOS" in parts:
        return [
            ("iPhone", "iOS", True, {"AddToEmbeddedBinaries": "false", "CPU": "AnyCPU", "CompileFlags": "", "FrameworkDependencies": ""}),
            ("Editor", "Editor", False, {"CPU": "AnyCPU", "OS": "AnyOS"}),
        ]
    if "macOS" in parts:
        return [
            ("Editor", "Editor", True, {"CPU": "AnyCPU", "OS": "OSX"}),
            ("Standalone", "OSXUniversal", True, {"CPU": "AnyCPU"}),
            ("Standalone", "Win64", False, off),
            ("Standalone", "Linux64", False, off),
        ]
    if "Windows" in parts:
        return [
            ("Editor", "Editor", True, {"CPU": "x86_64", "OS": "Windows"}),
            ("Standalone", "Win64", True, {"CPU": "x86_64"}),
            ("Standalone", "OSXUniversal", False, off),
            ("Standalone", "Linux64", False, off),
        ]
    if "Linux" in parts:
        return [
            ("Editor", "Editor", True, {"CPU": "x86_64", "OS": "Linux"}),
            ("Standalone", "Linux64", True, {"CPU": "x86_64"}),
            ("Standalone", "Win64", False, off),
            ("Standalone", "OSXUniversal", False, off),
        ]
    return []


def meta_for(path, rel):
    g = guid(rel)
    if os.path.isdir(path):
        return folder_meta(g)
    name = os.path.basename(path)
    ext = os.path.splitext(name)[1].lower()
    if name == "package.json":
        return simple(g, "PackageManifestImporter")
    if ext == ".cs":
        return mono(g)
    if ext == ".asmdef":
        return simple(g, "AssemblyDefinitionImporter")
    if ext in (".so", ".a", ".dylib", ".dll", ".bundle"):
        return plugin(g, plugin_entries(rel))
    if ext in (".md", ".txt", ".json", ".h", ".xml"):
        return simple(g, "TextScriptImporter")
    return simple(g, "DefaultImporter")


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    force = "--force" in sys.argv
    root = os.path.abspath(args[0] if args else os.path.join(os.path.dirname(__file__), "..", "unity", "com.physr.engine"))
    written = 0
    for dirpath, dirnames, filenames in os.walk(root):
        # Folders ending in '~' and hidden folders are ignored by Unity.
        dirnames[:] = sorted(d for d in dirnames if not d.endswith("~") and not d.startswith("."))
        entries = [os.path.join(dirpath, d) for d in dirnames] + [
            os.path.join(dirpath, f) for f in sorted(filenames) if not f.endswith(".meta") and not f.startswith(".")
        ]
        for p in entries:
            rel = os.path.relpath(p, root).replace(os.sep, "/")
            mp = p + ".meta"
            if os.path.exists(mp) and not force:
                continue
            with open(mp, "w", newline="\n") as fh:
                fh.write(meta_for(p, rel))
            written += 1
    print(f"wrote {written} meta files under {root}")


if __name__ == "__main__":
    main()
