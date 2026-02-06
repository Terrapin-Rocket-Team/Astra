import json
import subprocess
Import("env")

def get_git_info():
    """Get current commit SHA"""
    try:
        # Get short SHA (7 chars)
        sha = subprocess.check_output(
            ["git", "rev-parse", "--short=7", "HEAD"],
            text=True
        ).strip()

        return sha
    except subprocess.CalledProcessError:
        return "unknown"

def get_version():
    """Generate version string: MAJOR.MINOR-SHA"""
    # Read base version from library.json
    with open("library.json", "r") as f:
        data = json.load(f)
        base_version = data["version"]

    # Parse major.minor from base version (handle both "X.Y" and full version strings)
    parts = base_version.split(".")
    major = int(parts[0])
    # Handle case where minor might have extra stuff after it
    minor_part = parts[1].split("-")[0] if len(parts) > 1 else "0"
    minor = int(minor_part)

    # Get git info
    sha = get_git_info()

    # Construct version string
    version = f"{major}.{minor}-{sha}"

    return version

# Call the Python script to get the version flag
version = get_version()
print(f"[Astra CPP DEFINE] ASTRA_VERSION: {version}")

# Append the version flag, making sure the version is passed as a string literal to C++
env.Append(CPPDEFINES=[f'ASTRA_VERSION=\\"{version}\\"'])
