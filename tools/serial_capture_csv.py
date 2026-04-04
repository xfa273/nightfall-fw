#!/usr/bin/env python3

import os
import runpy
import sys

SCRIPT_DIR = os.path.dirname(__file__)
TARGET = os.path.join(SCRIPT_DIR, "logging", "serial_capture_csv.py")
sys.argv[0] = TARGET
runpy.run_path(TARGET, run_name="__main__")
