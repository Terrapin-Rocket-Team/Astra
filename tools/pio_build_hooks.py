import json
import platform
import subprocess

Import("env")
from SCons.Script import DefaultEnvironment


def _get_git_sha() -> str:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "--short=7", "HEAD"],
            text=True,
        ).strip()
    except subprocess.CalledProcessError:
        return "unknown"


def _get_version() -> str:
    with open("library.json", "r", encoding="utf-8") as f:
        base_version = json.load(f)["version"]
    parts = base_version.split(".")
    major = int(parts[0]) if parts else 0
    minor_part = parts[1].split("-")[0] if len(parts) > 1 else "0"
    minor = int(minor_part)
    return f"{major}.{minor}-{_get_git_sha()}"


def _apply_version_define() -> None:
    version = _get_version()
    print(f"[Astra CPP DEFINE] ASTRA_VERSION: {version}")
    env.AppendUnique(CPPDEFINES=[f'ASTRA_VERSION=\\"{version}\\"'])


def _apply_native_platform_flags() -> None:
    if env.get("PIOENV", "") != "native":
        return

    env.AppendUnique(CPPDEFINES=["NATIVE=1", "ARDUINO=100", "UNITY_INCLUDE_DOUBLE"])
    env.AppendUnique(CPPPATH=["$PROJECT_DIR/src"])

    # Keep permissive mode scoped to C++ only for native-hosted vendor code.
    env.AppendUnique(CXXFLAGS=["-fpermissive"])

    if platform.system() == "Windows":
        env.AppendUnique(CPPDEFINES=["ENV_WINDOWS"])
        env.AppendUnique(LIBS=["ws2_32"])
        DefaultEnvironment().AppendUnique(LIBS=["ws2_32"])
    else:
        env.AppendUnique(CPPDEFINES=["ENV_UNIX"])


_apply_version_define()
_apply_native_platform_flags()
