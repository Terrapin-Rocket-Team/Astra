import platform

Import("env")


def _is_native_env(e):
    return str(e.get("PIOPLATFORM", "")).lower() == "native"


if platform.system() == "Windows" and _is_native_env(env):
    env.Append(LIBS=["ws2_32"])
    print("[Astra Native] Injected Windows native link lib: ws2_32")
