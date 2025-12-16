import json
import subprocess
import os
Import("env")

VERSION_TRACK_FILE = ".version_build_track.json"

def get_git_info():
    """Get current commit SHA and check if working tree is clean"""
    try:
        # Get short SHA (7 chars)
        sha = subprocess.check_output(
            ["git", "rev-parse", "--short=7", "HEAD"],
            text=True
        ).strip()

        # Check if working tree has changes
        status = subprocess.check_output(
            ["git", "status", "--porcelain"],
            text=True
        ).strip()
        is_dirty = len(status) > 0

        return sha, is_dirty
    except subprocess.CalledProcessError:
        return "unknown", False

def count_commits_since_version(major, minor, track_data):
    """Count commits since the major.minor version was bumped"""
    current_version = f"{major}.{minor}"

    # Check if we have a stored base commit for this version
    if "version" in track_data and track_data["version"] == current_version:
        # Same version, count commits since base
        base_commit = track_data.get("base_commit", "")
        if base_commit:
            try:
                count = subprocess.check_output(
                    ["git", "rev-list", "--count", f"{base_commit}..HEAD"],
                    text=True
                ).strip()
                return int(count)
            except subprocess.CalledProcessError:
                return 0

    # New version or no tracking data - this is commit 0 of this version
    # Store current HEAD as the base commit for this version
    return 0

def update_version_tracking(major, minor, sha, is_dirty):
    """Update version tracking file and return build number"""
    current_version = f"{major}.{minor}"
    build_num = 0
    track_data = {}

    # Load existing tracking data
    if os.path.exists(VERSION_TRACK_FILE):
        with open(VERSION_TRACK_FILE, "r") as f:
            track_data = json.load(f)

    # Check if version changed
    if track_data.get("version") != current_version:
        # New version! Find the commit where library.json was last changed to this version
        try:
            # Get the commit where library.json was last modified
            base_commit = subprocess.check_output(
                ["git", "log", "-1", "--format=%H", "library.json"],
                text=True
            ).strip()
        except subprocess.CalledProcessError:
            # Fallback to current HEAD if git command fails
            try:
                base_commit = subprocess.check_output(
                    ["git", "rev-parse", "HEAD"],
                    text=True
                ).strip()
            except subprocess.CalledProcessError:
                base_commit = ""

        track_data = {
            "version": current_version,
            "base_commit": base_commit,
            "sha": sha,
            "build": 0
        }
        build_num = 0
    else:
        # Same version, check SHA for build number
        last_sha = track_data.get("sha", "")
        last_build = track_data.get("build", 0)

        if sha == last_sha:
            # Same commit, increment build if dirty or if last build wasn't 0
            if is_dirty or last_build > 0:
                build_num = last_build + 1
            else:
                build_num = 0
        else:
            # New commit, reset to 0
            build_num = 0

        # Update SHA and build, but preserve base_commit and version
        track_data["sha"] = sha
        track_data["build"] = build_num
        # Ensure base_commit and version are preserved
        if "base_commit" not in track_data:
            track_data["base_commit"] = ""
        if "version" not in track_data:
            track_data["version"] = current_version

    # Save updated tracking data
    with open(VERSION_TRACK_FILE, "w") as f:
        json.dump(track_data, f, indent=2)

    return build_num, track_data

def get_version():
    """Generate version string: MAJOR.MINOR.COMMITS-SHA-BUILD"""
    # Read base version from library.json
    with open("library.json", "r") as f:
        data = json.load(f)
        base_version = data["version"]

    # Parse major.minor from base version (handle both "X.Y" and full version strings)
    # If version is already in full format (X.Y.Z-SHA-BUILD), extract just X.Y
    parts = base_version.split(".")
    major = int(parts[0])
    # Handle case where minor might have extra stuff after it (e.g., "2.3-abc123-0")
    minor_part = parts[1].split("-")[0] if len(parts) > 1 else "0"
    minor = int(minor_part)

    # Get git info
    sha, is_dirty = get_git_info()

    # Update tracking and get build number (this also updates track_data for new versions)
    build, track_data = update_version_tracking(major, minor, sha, is_dirty)

    # Count commits since this major.minor version (using updated track_data)
    commits = count_commits_since_version(major, minor, track_data)

    # Construct version string
    version = f"{major}.{minor}.{commits}-{sha}-{build}"

    return version

# Call the Python script to get the version flag
version = get_version()
print(f"[Astra CPP DEFINE] ASTRA_VERSION: {version}")

# Append the version flag, making sure the version is passed as a string literal to C++
env.Append(CPPDEFINES=[f'ASTRA_VERSION=\\"{version}\\"'])
