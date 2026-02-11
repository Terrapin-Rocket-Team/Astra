#!/usr/bin/env python3
import subprocess
import sys

cmd = [sys.executable, "-m", "astra_support", "test", "--project", "."] + sys.argv[1:]
raise SystemExit(subprocess.call(cmd))
