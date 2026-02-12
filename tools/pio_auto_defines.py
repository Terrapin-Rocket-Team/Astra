from __future__ import annotations

import platform
import shutil
from typing import Any, Iterable

from SCons.Script import DefaultEnvironment

AUTO_DEFINE_DISABLE_KEY = "ASTRA_NO_AUTO_DEFINES"


def _define_name(define_item: Any) -> str:
    if isinstance(define_item, tuple) and define_item:
        return str(define_item[0])
    if isinstance(define_item, list) and define_item:
        return str(define_item[0])
    if isinstance(define_item, dict) and define_item:
        return str(next(iter(define_item)))
    if isinstance(define_item, str):
        return define_item.split("=", 1)[0].strip()
    return ""


def _has_define(defines: Iterable[Any], key: str) -> bool:
    key = key.strip()
    for define_item in defines:
        if _define_name(define_item) == key:
            return True
    return False


def _append_define_if_missing(target_env: Any, define_item: Any) -> None:
    define_key = _define_name(define_item)
    existing = target_env.get("CPPDEFINES", [])
    if define_key and not _has_define(existing, define_key):
        target_env.AppendUnique(CPPDEFINES=[define_item])


def _apply_common_native_defines(target_env: Any) -> None:
    _append_define_if_missing(target_env, ("NATIVE", 1))
    _append_define_if_missing(target_env, ("ARDUINO", 100))
    _append_define_if_missing(target_env, "UNITY_INCLUDE_DOUBLE")


def _ensure_windows_native_toolchain(target_env: Any) -> None:
    if platform.system() != "Windows":
        return
    if shutil.which("gcc") is None or shutil.which("g++") is None:
        return
    # Keep native C and C++ compilation on the same MinGW toolchain.
    # This avoids mixed ABI/symbol mangling when `c++.exe` points to
    # an older compiler than `g++.exe`.
    target_env.Replace(CC="gcc", CXX="g++", LINK="g++")


def _is_native_build(target_env: Any) -> bool:
    platform_name = str(target_env.get("PIOPLATFORM", "")).strip().lower()
    if platform_name:
        return platform_name == "native"
    env_name = str(target_env.get("PIOENV", "")).strip().lower()
    return env_name in {"native", "unix"}


def apply_native_auto_defines(env: Any) -> None:
    global_env = DefaultEnvironment()
    if not _is_native_build(global_env):
        return

    if _has_define(global_env.get("CPPDEFINES", []), AUTO_DEFINE_DISABLE_KEY):
        print("[Astra] Skipping native auto-defines (ASTRA_NO_AUTO_DEFINES is set)")
        return

    for target_env in (global_env, env):
        _apply_common_native_defines(target_env)
        _ensure_windows_native_toolchain(target_env)

    env.AppendUnique(CPPPATH=["$PROJECT_DIR/src"])
    env.AppendUnique(CXXFLAGS=["-fpermissive"])

    if platform.system() == "Windows":
        for target_env in (global_env, env):
            _append_define_if_missing(target_env, "ENV_WINDOWS")
            target_env.AppendUnique(LIBS=["ws2_32"])
    else:
        for target_env in (global_env, env):
            _append_define_if_missing(target_env, "ENV_UNIX")

    get_lib_builders = getattr(env, "GetLibBuilders", None)
    if not callable(get_lib_builders):
        return

    for lib_builder in get_lib_builders():
        lib_env = getattr(lib_builder, "env", None)
        if lib_env is None:
            continue
        _apply_common_native_defines(lib_env)
        _ensure_windows_native_toolchain(lib_env)
        if platform.system() == "Windows":
            _append_define_if_missing(lib_env, "ENV_WINDOWS")
        else:
            _append_define_if_missing(lib_env, "ENV_UNIX")
