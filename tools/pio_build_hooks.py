import json
import importlib.util
import subprocess
from pathlib import Path

Import("env")


def _get_git_sha() -> str:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "--short=7", "HEAD"],
            text=True,
        ).strip()
    except subprocess.CalledProcessError:
        return "unknown"


def _get_version() -> str:
    library_json_path = Path(__file__).resolve().parent.parent / "library.json"
    with open(library_json_path, "r", encoding="utf-8") as f:
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
    helper_path = Path(__file__).with_name("pio_auto_defines.py")
    spec = importlib.util.spec_from_file_location("astra_pio_auto_defines", helper_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load native auto-define helper: {helper_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.apply_native_auto_defines(env)


_apply_version_define()
_apply_native_platform_flags()
