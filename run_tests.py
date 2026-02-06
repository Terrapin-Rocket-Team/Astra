import subprocess
import os
import multiprocessing
import shutil
import time
import sys
import signal
import threading
import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass
from typing import Tuple, Optional, List, Dict

# --- CONFIGURATION ---
MAX_WORKERS = max(1, multiprocessing.cpu_count() - 2)
MAX_RETRIES = 3 
TEST_DIR = "test"
PARALLEL_BUILD_BASE = os.path.join(os.getcwd(), ".pio", "build_parallel")

# ANSI Colors
BS = "\033[1m"
R = "\033[91m"
G = "\033[92m"
Y = "\033[93m"
M = "\033[95m" 
C = "\033[96m" 
NC = "\033[0m"

# Status Categories
STATUS_PASS = "PASS"
STATUS_TEST_FAIL = "TEST_FAIL"    
STATUS_COMPILE_ERR = "COMPILE_ERR" 
STATUS_SYSTEM_ERR = "SYSTEM_ERR"   
STATUS_CANCELLED = "CANCELLED"

# Progress output control (overridden by CLI)
PROGRESS_ENABLED = True

@dataclass
class TestResult:
    name: str
    status: str
    code: int
    log: str
    duration: float
    test_count: Optional[int] = None
    passed_count: Optional[int] = None
    failed_count: Optional[int] = None

@dataclass
class BuildResult:
    name: str
    status: str
    code: int
    log: str
    duration: float

@dataclass
class PlatformInstallResult:
    name: str
    status: str
    code: int
    log: str
    duration: float

# --- UPDATED PROGRESS BAR ---
def draw_progress(done, total, start_time, bar_len=30):
    if not PROGRESS_ENABLED or total == 0:
        return
    
    elapsed = time.time() - start_time
    m, s = divmod(int(elapsed), 60)
    time_str = f"{m:02d}:{s:02d}"
    
    percent = float(done) / total
    fill_len = int(bar_len * percent)
    bar = '=' * fill_len + '-' * (bar_len - fill_len)
    remaining = total - done
    
    sys.stdout.write(f"\r[{bar}] {int(percent*100)}% ({done}/{total}) | {remaining} Left | Time: {time_str} ")
    sys.stdout.flush()

def clear_line():
    if not PROGRESS_ENABLED:
        return
    sys.stdout.write("\r" + " " * 90 + "\r")
    sys.stdout.flush()

def analyze_output(log_text: str, return_code: int) -> Tuple[str, str]:
    lines = log_text.split('\n')
    cleaned_lines = []
    
    found_assert_fail = False 
    found_syntax_error = False 
    found_system_lock = False 
    found_pio_error = False

    for line in lines:
        line_strip = line.strip()
        if ":FAIL:" in line:
            cleaned_lines.append(f"{R}  [ASSERT] {NC}{line_strip}")
            found_assert_fail = True
        elif (": error:" in line or "undefined reference" in line or "fatal error:" in line):
            cleaned_lines.append(f"{Y}  [COMPILER] {NC}{line_strip}")
            found_syntax_error = True
        elif ("Error:" in line or "ERROR:" in line):
            cleaned_lines.append(f"{M}  [PIO] {NC}{line_strip}")
            found_pio_error = True
        elif ("Permission denied" in line or "cannot open output file" in line or "Device or resource busy" in line):
            cleaned_lines.append(f"{M}  [OS LOCK] {NC}{line_strip}")
            found_system_lock = True

    if return_code == 0:
        if found_assert_fail: return STATUS_TEST_FAIL, "\n".join(cleaned_lines)
        return STATUS_PASS, ""

    if found_system_lock or found_pio_error: return STATUS_SYSTEM_ERR, "\n".join(cleaned_lines)
    if found_syntax_error: return STATUS_COMPILE_ERR, "\n".join(cleaned_lines)
    if found_assert_fail: return STATUS_TEST_FAIL, "\n".join(cleaned_lines)

    if not cleaned_lines: cleaned_lines = [f"{M}  [SYSTEM CRASH] {NC}No error output captured."]
    return STATUS_SYSTEM_ERR, "\n".join(cleaned_lines)

def parse_test_counts(log_text: str) -> Tuple[Optional[int], Optional[int], Optional[int]]:
    total = None
    passed = None
    failed = None
    collected = None

    for line in log_text.split('\n'):
        line_strip = line.strip()
        if line_strip.startswith("Collected ") and " tests" in line_strip:
            parts = line_strip.split()
            if len(parts) >= 2:
                try:
                    collected = int(parts[1])
                except ValueError:
                    pass
        if " test cases:" in line_strip and ("failed" in line_strip or "succeeded" in line_strip):
            # Example: "102 test cases: 1 failed, 101 succeeded in 00:00:12.171"
            left, right = line_strip.split(" test cases:", 1)
            # left might contain padding like "==== 151"
            num = ""
            for ch in left:
                if ch.isdigit():
                    num += ch
                elif num:
                    break
            if num:
                try:
                    total = int(num)
                except ValueError:
                    pass
            failed = 0
            passed = 0
            parts = right.split(',')
            for part in parts:
                part = part.strip()
                if part.endswith("failed") or " failed" in part:
                    try:
                        failed = int(part.split()[0])
                    except ValueError:
                        pass
                if part.endswith("succeeded") or " succeeded" in part:
                    try:
                        passed = int(part.split()[0])
                    except ValueError:
                        pass
            if total is not None:
                if passed is None:
                    passed = 0
                if failed is None:
                    failed = 0
            break
    if total is None:
        total = collected
    return total, passed, failed

def list_platformio_envs(config_path: str = "platformio.ini") -> List[str]:
    envs = []
    if not os.path.exists(config_path):
        return envs
    with open(config_path, "r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line or line.startswith(";") or line.startswith("#"):
                continue
            if line.startswith("[") and line.endswith("]"):
                header = line[1:-1].strip()
                if header.startswith("env:"):
                    env_name = header[4:].strip()
                    if env_name and env_name not in envs:
                        envs.append(env_name)
    return envs

def parse_env_platforms(config_path: str = "platformio.ini") -> Dict[str, str]:
    env_platforms: Dict[str, str] = {}
    if not os.path.exists(config_path):
        return env_platforms
    current_env = None
    with open(config_path, "r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line or line.startswith(";") or line.startswith("#"):
                continue
            if line.startswith("[") and line.endswith("]"):
                header = line[1:-1].strip()
                if header.startswith("env:"):
                    current_env = header[4:].strip()
                else:
                    current_env = None
                continue
            if current_env and line.lower().startswith("platform"):
                key, _, value = line.partition("=")
                if key.strip().lower() == "platform":
                    platform = value.strip().split()[0] if value.strip() else ""
                    if platform:
                        env_platforms[current_env] = platform
    return env_platforms

def select_build_envs(envs: List[str]) -> List[str]:
    if not envs:
        return []
    if sys.platform == "win32":
        return [e for e in envs if e != "unix"]
    return [e for e in envs if e != "native"]

def select_test_env(envs: List[str]) -> Optional[str]:
    if sys.platform != "win32" and "unix" in envs:
        return "unix"
    if "native" in envs:
        return "native"
    return envs[0] if envs else None

def select_platforms_for_envs(envs: List[str], env_platforms: Dict[str, str]) -> List[str]:
    platforms: List[str] = []
    for env_name in envs:
        platform = env_platforms.get(env_name)
        if platform and platform not in platforms:
            platforms.append(platform)
    return platforms

def run_platform_install(platform: str) -> PlatformInstallResult:
    cmd = ["pio", "platform", "install", platform]
    start_time = time.time()
    try:
        result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        duration = time.time() - start_time
        status, clean_log = analyze_output(result.stdout, result.returncode)
        return PlatformInstallResult(platform, status, result.returncode, clean_log, duration)
    except Exception as e:
        return PlatformInstallResult(platform, STATUS_SYSTEM_ERR, -1, str(e), 0)

def run_build_env(env_name: str) -> BuildResult:
    cmd = ["pio", "run", "-e", env_name]
    start_time = time.time()
    try:
        result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        duration = time.time() - start_time
        status, clean_log = analyze_output(result.stdout, result.returncode)
        return BuildResult(env_name, status, result.returncode, clean_log, duration)
    except Exception as e:
        return BuildResult(env_name, STATUS_SYSTEM_ERR, -1, str(e), 0)

def run_test_folder(folder_name, test_env: str):
    unique_build_path = os.path.join(PARALLEL_BUILD_BASE, folder_name)
    env = os.environ.copy()
    env["PLATFORMIO_BUILD_DIR"] = unique_build_path
    
    cmd = ["pio", "test", "-e", test_env, "-f", folder_name]
    
    start_time = time.time()
    try:
        # Use start_new_session to ensure we can kill process groups on Windows/Linux
        if sys.platform == 'win32':
             result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
        else:
             result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)

        duration = time.time() - start_time
        status, clean_log = analyze_output(result.stdout, result.returncode)
        test_count, passed_count, failed_count = parse_test_counts(result.stdout)
        return TestResult(folder_name, status, result.returncode, clean_log, duration, test_count, passed_count, failed_count)
    except Exception as e:
        return TestResult(folder_name, STATUS_SYSTEM_ERR, -1, str(e), 0)

def main():
    global PROGRESS_ENABLED
    parser = argparse.ArgumentParser(description="Run PlatformIO builds/tests with parallel execution.")
    parser.add_argument("--no-progress", action="store_true", help="Disable progress bar updates (CI friendly).")
    args = parser.parse_args()
    PROGRESS_ENABLED = not args.no_progress

    envs = list_platformio_envs()
    build_envs = select_build_envs(envs)
    env_platforms = parse_env_platforms()
    platforms_to_install = select_platforms_for_envs(build_envs, env_platforms)
    install_results: List[PlatformInstallResult] = []
    build_results: List[BuildResult] = []

    if platforms_to_install:
        print(f"{BS}📦 Installing {len(platforms_to_install)} PlatformIO platforms{NC}")
        print("---------------------------------------------------")
        install_start_time = time.time()
        for platform in platforms_to_install:
            res = run_platform_install(platform)
            install_results.append(res)
            if res.status == STATUS_PASS:
                print(f"{G}✅ PLATFORM OK: {res.name} ({res.duration:.1f}s){NC}")
            else:
                print(f"{M}☠️  PLATFORM FAIL: {res.name} ({res.duration:.1f}s){NC}")
                if res.log:
                    print(res.log)
        install_duration = time.time() - install_start_time
        print("---------------------------------------------------")
        print(f"{BS}Platform installs complete in {install_duration:.2f}s{NC}")

        failed_platforms = {r.name for r in install_results if r.status != STATUS_PASS}
        if failed_platforms:
            before_count = len(build_envs)
            build_envs = [e for e in build_envs if env_platforms.get(e) not in failed_platforms]
            skipped = before_count - len(build_envs)
            if skipped > 0:
                print(f"{Y}⚠️  Skipping {skipped} build env(s) due to failed platform install(s).{NC}")
    else:
        if envs:
            print(f"{Y}⚠️  No compatible build platforms found. Skipping installs.{NC}")
        else:
            print(f"{Y}⚠️  No PlatformIO environments found. Skipping installs.{NC}")

    if build_envs:
        print(f"{BS}🔨 Building {len(build_envs)} environments{NC}")
        print("---------------------------------------------------")
        build_start_time = time.time()
        try:
            with ProcessPoolExecutor(max_workers=MAX_WORKERS) as executor:
                future_to_env = {executor.submit(run_build_env, env_name): env_name for env_name in build_envs}
                for future in as_completed(future_to_env):
                    try:
                        res = future.result()
                    except Exception as exc:
                        res = BuildResult(future_to_env[future], STATUS_SYSTEM_ERR, -1, str(exc), 0)
                    build_results.append(res)
                    if res.status == STATUS_PASS:
                        print(f"{G}✅ BUILD OK: {res.name} ({res.duration:.1f}s){NC}")
                    elif res.status == STATUS_COMPILE_ERR:
                        print(f"{Y}💥 BUILD FAIL: {res.name} ({res.duration:.1f}s){NC}")
                        print(res.log)
                    else:
                        print(f"{M}☠️  BUILD CRASH: {res.name} ({res.duration:.1f}s){NC}")
                        print(res.log)
        except KeyboardInterrupt:
            print(f"\n{R}🛑 Build cancelled by user.{NC}")
            sys.exit(1)
        build_duration = time.time() - build_start_time
        print("---------------------------------------------------")
        print(f"{BS}Builds complete in {build_duration:.2f}s{NC}")
    else:
        if envs:
            print(f"{Y}⚠️  No compatible build environments for this platform. Skipping builds.{NC}")
        else:
            print(f"{Y}⚠️  No PlatformIO environments found. Skipping builds.{NC}")

    test_env = select_test_env(envs)
    if test_env:
        print(f"{C}🧪 Test env: {test_env}{NC}")
    else:
        print(f"{Y}⚠️  No compatible test environment found. Skipping tests.{NC}")

    test_results = {}
    test_duration = 0.0
    total_tests = 0
    tests_skipped_reason = None

    if not test_env:
        tests_skipped_reason = "No compatible test environment."
    elif not os.path.exists(TEST_DIR):
        tests_skipped_reason = f"Directory '{TEST_DIR}' not found."
    else:
        # Check if we need the primer (if build folder is missing or empty)
        needs_primer = True
        if os.path.exists(PARALLEL_BUILD_BASE) and len(os.listdir(PARALLEL_BUILD_BASE)) > 0:
            needs_primer = False
        else:
            # Clean ensures we start fresh if directory existed but was empty/corrupt
            if os.path.exists(PARALLEL_BUILD_BASE):
                shutil.rmtree(PARALLEL_BUILD_BASE)

        folders = [f for f in os.listdir(TEST_DIR) if os.path.isdir(os.path.join(TEST_DIR, f))]
        total_tests = len(folders)
        
        print(f"{BS}🚀 Queueing {total_tests} suites on {test_env} ({MAX_WORKERS} workers){NC}")
        print("---------------------------------------------------")

        completed_count = 0
        global_start_time = time.time()
        
        # --- STEP 1: PRIMER (CONDITIONAL) ---
        if folders and needs_primer:
            primer_folder = folders.pop(0)
            print(f"{C}🔧 Cache cold. Running PRIMER on '{primer_folder}'...{NC}")
            
            try:
                res = run_test_folder(primer_folder, test_env)
                test_results[primer_folder] = {'res': res}
                completed_count += 1
                
                if res.status == STATUS_PASS:
                    print(f"{G}✅ PRIMER PASSED ({res.duration:.1f}s). Starting parallel workers...{NC}")
                elif res.status == STATUS_COMPILE_ERR:
                    print(f"{Y}💥 PRIMER BUILD FAILED. Check code syntax.{NC}")
                    print(res.log)
                else:
                    print(f"{M}⚠️  PRIMER FLAKED/FAILED. Starting workers anyway...{NC}")
            except KeyboardInterrupt:
                print(f"\n{R}🛑 Cancelled during Primer.{NC}")
                sys.exit(1)
        elif not needs_primer:
            print(f"{G}⚡ Cache found. Skipping Primer.{NC}")

        # --- STEP 2: PARALLEL EXECUTION ---
        queue = folders[:] 
        draw_progress(completed_count, total_tests, global_start_time)

        # Periodic progress refresher so timer updates even when no suite completes
        stop_refresh = threading.Event()
        refresh_thread = None
        if PROGRESS_ENABLED:
            def progress_refresher():
                while not stop_refresh.is_set():
                    draw_progress(completed_count, total_tests, global_start_time)
                    time.sleep(0.5)
            refresh_thread = threading.Thread(target=progress_refresher, daemon=True)
            refresh_thread.start()

        # We use a try/except block around the Pool to handle Ctrl+C
        try:
            with ProcessPoolExecutor(max_workers=MAX_WORKERS) as executor:
                # Helper to manage the queue loop
                while queue:
                    # Submit all remaining items
                    future_to_folder = {executor.submit(run_test_folder, f, test_env): f for f in queue}
                    queue = [] 

                    for future in as_completed(future_to_folder):
                        folder = future_to_folder[future]
                        try:
                            res = future.result()
                        except KeyboardInterrupt:
                            # This catches if the worker itself signals interrupt (rare in this setup)
                            raise
                        except Exception:
                            continue 

                        # Retry Logic
                        current_retries = test_results.get(folder, {}).get('retries', 0)
                        if res.status == STATUS_SYSTEM_ERR and current_retries < MAX_RETRIES:
                            clear_line()
                            print(f"{M}⚠️  Retry {current_retries + 1}/{MAX_RETRIES} (System Flake): {folder}{NC}")
                            draw_progress(completed_count, total_tests, global_start_time)
                            
                            if folder not in test_results: test_results[folder] = {'retries': 0}
                            test_results[folder]['retries'] += 1
                            queue.append(folder) 
                            continue
                        
                        # Success/Failure processing
                        completed_count += 1
                        clear_line()
                        
                        if res.status == STATUS_PASS:
                            count_str = f" [{res.test_count} cases]" if res.test_count is not None else ""
                            print(f"{G}✅ PASS: {res.name}{count_str} ({res.duration:.1f}s){NC}")
                        elif res.status == STATUS_TEST_FAIL:
                            count_str = f" [{res.test_count} cases]" if res.test_count is not None else ""
                            print(f"{R}❌ FAIL: {res.name}{count_str}{NC}")
                            print(res.log)
                        elif res.status == STATUS_COMPILE_ERR:
                            count_str = f" [{res.test_count} cases]" if res.test_count is not None else ""
                            print(f"{Y}💥 ERR : {res.name}{count_str} (Build Failed){NC}")
                            print(res.log)
                        elif res.status == STATUS_SYSTEM_ERR:
                            count_str = f" [{res.test_count} cases]" if res.test_count is not None else ""
                            print(f"{M}☠️  CRASH: {res.name}{count_str} (System Error){NC}")
                            print(res.log)

                        test_results[folder] = {'res': res}
                        draw_progress(completed_count, total_tests, global_start_time)

        except KeyboardInterrupt:
            print(f"\n\n{R}🛑 EXECUTION CANCELLED BY USER.{NC}")
            print("Shutting down workers... (this may take a moment)")
            # ProcessPoolExecutor cleans up automatically on exit of the 'with' block,
            # but the KeyboardInterrupt breaks the loop instantly.
            sys.exit(1)

        # --- TEST SUMMARY ---
        stop_refresh.set()
        if refresh_thread is not None:
            refresh_thread.join(timeout=1.0)
        test_duration = time.time() - global_start_time

    # --- SUMMARY ---
    print("\n" + "="*50)
    print(f"{BS}RUN COMPLETE{NC}")
    print("="*50)

    install_failed = [r for r in install_results if r.status != STATUS_PASS]
    if install_results:
        print(f"{BS}Platform Install Results{NC}")
        if not install_failed:
            print(f"{G}All platforms installed successfully.{NC}")
        else:
            print(f"{M}Failed ({len(install_failed)}):{NC}")
            for r in install_failed: print(f"  ☠️  {r.name}")
        print("-"*50)

    build_passed = [r for r in build_results if r.status == STATUS_PASS]
    build_failed = [r for r in build_results if r.status != STATUS_PASS]
    build_broken = [r for r in build_results if r.status == STATUS_COMPILE_ERR]
    build_crashed = [r for r in build_results if r.status == STATUS_SYSTEM_ERR]

    if build_results:
        print(f"{BS}Build Results{NC}")
        if build_passed:
            print(f"{G}Passing ({len(build_passed)}):{NC}")
            for r in build_passed: print(f"  ✅ {r.name}")
        if build_broken:
            print(f"\n{Y}Build Errors ({len(build_broken)}) - [Syntax/Linker]:{NC}")
            for r in build_broken: print(f"  💥 {r.name}")
        if build_crashed:
            print(f"\n{M}System Crashes ({len(build_crashed)}) - [OS/Locking Issues]:{NC}")
            for r in build_crashed: print(f"  ☠️  {r.name}")
        if test_env:
            print("-"*50)
    else:
        print(f"{Y}No build results to report.{NC}")

    test_passed = [r['res'] for r in test_results.values() if r['res'].status == STATUS_PASS]
    test_failed = [r['res'] for r in test_results.values() if r['res'].status == STATUS_TEST_FAIL]
    test_broken = [r['res'] for r in test_results.values() if r['res'].status == STATUS_COMPILE_ERR]
    test_crashed = [r['res'] for r in test_results.values() if r['res'].status == STATUS_SYSTEM_ERR]

    total_test_cases = sum(r['res'].test_count or 0 for r in test_results.values())
    total_passed_cases = sum(r['res'].passed_count or 0 for r in test_results.values())
    total_failed_cases = sum(r['res'].failed_count or 0 for r in test_results.values())

    if tests_skipped_reason:
        print(f"{Y}Tests skipped: {tests_skipped_reason}{NC}")
    elif test_results:
        print(f"{BS}Test Results (env: {test_env}, {total_tests} suites, {test_duration:.2f}s){NC}")
        if test_passed:
            print(f"{G}Passing ({len(test_passed)}):{NC}")
            for r in test_passed: print(f"  ✅ {r.name}")
        if test_failed:
            print(f"\n{R}Test Failures ({len(test_failed)}) - [Logic/Assertions]:{NC}")
            for r in test_failed: print(f"  ❌ {r.name}")
        if test_broken:
            print(f"\n{Y}Build Errors ({len(test_broken)}) - [Syntax/Linker]:{NC}")
            for r in test_broken: print(f"  💥 {r.name}")
        if test_crashed:
            print(f"\n{M}System Crashes ({len(test_crashed)}) - [OS/Locking Issues]:{NC}")
            for r in test_crashed: print(f"  ☠️  {r.name}")

        print("\n" + "-"*50)
        print(f"{BS}Test Case Totals{NC}")
        print(f"  Total: {total_test_cases}")
        print(f"  Passed: {total_passed_cases}")
        print(f"  Failed: {total_failed_cases}")
    elif test_env:
        print(f"{Y}No test results to report.{NC}")

    print("="*50)
    exit_code = 0
    if install_failed:
        exit_code = 1
    if build_failed:
        exit_code = 1
    if len(test_failed) + len(test_broken) + len(test_crashed) > 0:
        exit_code = 1
    exit(exit_code)

if __name__ == "__main__":
    main()
