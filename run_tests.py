import subprocess
import os
import multiprocessing
import shutil
import time
import sys
import signal
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass
from typing import Tuple

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

@dataclass
class TestResult:
    name: str
    status: str
    code: int
    log: str
    duration: float

# --- UPDATED PROGRESS BAR ---
def draw_progress(done, total, start_time, bar_len=30):
    if total == 0: return
    
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
    sys.stdout.write("\r" + " " * 90 + "\r")
    sys.stdout.flush()

def analyze_output(log_text: str, return_code: int) -> Tuple[str, str]:
    lines = log_text.split('\n')
    cleaned_lines = []
    
    found_assert_fail = False 
    found_syntax_error = False 
    found_system_lock = False 

    for line in lines:
        line_strip = line.strip()
        if ":FAIL:" in line:
            cleaned_lines.append(f"{R}  [ASSERT] {NC}{line_strip}")
            found_assert_fail = True
        elif (": error:" in line or "undefined reference" in line or "fatal error:" in line):
            cleaned_lines.append(f"{Y}  [COMPILER] {NC}{line_strip}")
            found_syntax_error = True
        elif ("Permission denied" in line or "cannot open output file" in line or "Device or resource busy" in line):
            cleaned_lines.append(f"{M}  [OS LOCK] {NC}{line_strip}")
            found_system_lock = True

    if return_code == 0:
        if found_assert_fail: return STATUS_TEST_FAIL, "\n".join(cleaned_lines)
        return STATUS_PASS, ""

    if found_system_lock: return STATUS_SYSTEM_ERR, "\n".join(cleaned_lines)
    if found_syntax_error: return STATUS_COMPILE_ERR, "\n".join(cleaned_lines)
    if found_assert_fail: return STATUS_TEST_FAIL, "\n".join(cleaned_lines)

    if not cleaned_lines: cleaned_lines = [f"{M}  [SYSTEM CRASH] {NC}No error output captured."]
    return STATUS_SYSTEM_ERR, "\n".join(cleaned_lines)

def run_test_folder(folder_name):
    unique_build_path = os.path.join(PARALLEL_BUILD_BASE, folder_name)
    env = os.environ.copy()
    env["PLATFORMIO_BUILD_DIR"] = unique_build_path
    
    cmd = ["pio", "test", "-e", "native", "-f", folder_name]
    
    start_time = time.time()
    try:
        # Use start_new_session to ensure we can kill process groups on Windows/Linux
        if sys.platform == 'win32':
             result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
        else:
             result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)

        duration = time.time() - start_time
        status, clean_log = analyze_output(result.stdout, result.returncode)
        return TestResult(folder_name, status, result.returncode, clean_log, duration)
    except Exception as e:
        return TestResult(folder_name, STATUS_SYSTEM_ERR, -1, str(e), 0)

def main():
    if not os.path.exists(TEST_DIR):
        print(f"Error: Directory '{TEST_DIR}' not found.")
        return

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
    
    print(f"{BS}🚀 Queueing {total_tests} tests ({MAX_WORKERS} workers){NC}")
    print("---------------------------------------------------")

    results = {}
    completed_count = 0
    global_start_time = time.time()
    
    # --- STEP 1: PRIMER (CONDITIONAL) ---
    if folders and needs_primer:
        primer_folder = folders.pop(0)
        print(f"{C}🔧 Cache cold. Running PRIMER on '{primer_folder}'...{NC}")
        
        try:
            res = run_test_folder(primer_folder)
            results[primer_folder] = {'res': res}
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

    # We use a try/except block around the Pool to handle Ctrl+C
    try:
        with ProcessPoolExecutor(max_workers=MAX_WORKERS) as executor:
            # Helper to manage the queue loop
            while queue:
                # Submit all remaining items
                future_to_folder = {executor.submit(run_test_folder, f): f for f in queue}
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
                    current_retries = results.get(folder, {}).get('retries', 0)
                    if res.status == STATUS_SYSTEM_ERR and current_retries < MAX_RETRIES:
                        clear_line()
                        print(f"{M}⚠️  Retry {current_retries + 1}/{MAX_RETRIES} (System Flake): {folder}{NC}")
                        draw_progress(completed_count, total_tests, global_start_time)
                        
                        if folder not in results: results[folder] = {'retries': 0}
                        results[folder]['retries'] += 1
                        queue.append(folder) 
                        continue
                    
                    # Success/Failure processing
                    completed_count += 1
                    clear_line()
                    
                    if res.status == STATUS_PASS:
                        print(f"{G}✅ PASS: {res.name} ({res.duration:.1f}s){NC}")
                    elif res.status == STATUS_TEST_FAIL:
                        print(f"{R}❌ FAIL: {res.name}{NC}")
                        print(res.log)
                    elif res.status == STATUS_COMPILE_ERR:
                        print(f"{Y}💥 ERR : {res.name} (Build Failed){NC}")
                        print(res.log)
                    elif res.status == STATUS_SYSTEM_ERR:
                        print(f"{M}☠️  CRASH: {res.name} (System Error){NC}")
                        print(res.log)

                    results[folder] = {'res': res}
                    draw_progress(completed_count, total_tests, global_start_time)

    except KeyboardInterrupt:
        print(f"\n\n{R}🛑 EXECUTION CANCELLED BY USER.{NC}")
        print("Shutting down workers... (this may take a moment)")
        # ProcessPoolExecutor cleans up automatically on exit of the 'with' block,
        # but the KeyboardInterrupt breaks the loop instantly.
        sys.exit(1)

    # --- SUMMARY ---
    global_duration = time.time() - global_start_time
    print("\n" + "="*50)
    print(f"{BS}RUN COMPLETE in {global_duration:.2f}s{NC}")
    print("="*50)
    
    passed = [r['res'] for r in results.values() if r['res'].status == STATUS_PASS]
    failed = [r['res'] for r in results.values() if r['res'].status == STATUS_TEST_FAIL]
    broken = [r['res'] for r in results.values() if r['res'].status == STATUS_COMPILE_ERR]
    crashed = [r['res'] for r in results.values() if r['res'].status == STATUS_SYSTEM_ERR]

    if passed:
        print(f"{G}Passing ({len(passed)}):{NC}")
        for r in passed: print(f"  ✓ {r.name}")
    if failed:
        print(f"\n{R}Test Failures ({len(failed)}) - [Logic/Assertions]:{NC}")
        for r in failed: print(f"  ❌ {r.name}")
    if broken:
        print(f"\n{Y}Build Errors ({len(broken)}) - [Syntax/Linker]:{NC}")
        for r in broken: print(f"  💥 {r.name}")
    if crashed:
        print(f"\n{M}System Crashes ({len(crashed)}) - [OS/Locking Issues]:{NC}")
        for r in crashed: print(f"  ☠️  {r.name}")
    
    print("="*50)
    if len(failed) + len(broken) + len(crashed) > 0: exit(1)
    else: exit(0)

if __name__ == "__main__":
    main()