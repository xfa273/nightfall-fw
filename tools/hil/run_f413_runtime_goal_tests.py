#!/usr/bin/env python3
"""Run actual search/shortest code with distinct profiles, without editing tunes."""
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
with tempfile.TemporaryDirectory(prefix='f413-runtime-params-') as directory:
    tmp = Path(directory)
    profiles = []
    for name, rev in [('f413_preorder', 2), ('mini_r3_0', 3)]:
        target = tmp / name
        shutil.copytree(ROOT / 'params' / name, target)
        header = target / 'params.h'
        text = header.read_text()
        values = dict(GOAL_X=1 if rev == 2 else 0, GOAL_Y=0 if rev == 2 else 8,
                      START_X=2 if rev == 2 else 0, START_Y=3 if rev == 2 else 0,
                      GOAL9_X=15, GOAL9_Y=15, SENSOR_DIST_GAIN='1.25F' if rev == 2 else '2.0F')
        for key, value in values.items():
            text, count = re.subn(rf'(^#define\s+{key})\s+[^\n]*', rf'\g<1> {value}', text, flags=re.M)
            assert count == 1, key
        header.write_text(text)
        profiles.append(str(target / 'profile.c'))
    includes = ['board/f413/runtime', 'board/f413', 'tools/hil/nvm_stubs', 'tools/solver_host/include',
                'nvm', 'common/route', 'platform/trace',
                'platform/stm32f405/Core/Inc',
                'platform/stm32f413/HM_Nightfall_f413_preorder/Core/Inc']
    sources = ['tools/hil/f413_runtime_goal_tests.c', 'board/f413/f413_machine.c',
               'board/f413/f413_registry.c', 'nvm/nvm_identity.c', 'common/route/legacy_path_codec.c',
               *[f'platform/stm32f405/Core/Src/{s}.c' for s in
                 ['sensor_distance', 'solver', 'solver_params', 'path', 'maze_grid']]]
    binary = tmp / 'runtime_goal_tests'
    subprocess.run([os.environ.get('CC', 'cc'), '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-Wno-unused-function', '-Wno-array-bounds', '-Wno-sign-compare', '-O1', '-g',
                    '-DSTM32F413xx', '-DNIGHTFALL_F413_RUNTIME_CONFIG=1',
                    '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
                    '-ffunction-sections', '-fdata-sections',
                    *[f'-I{ROOT / d}' for d in includes], *[str(ROOT / s) for s in sources],
                    *profiles, '-Wl,-dead_strip' if sys.platform == 'darwin' else '-Wl,--gc-sections',
                    '-lm', '-o', str(binary)], check=True)
    env = dict(os.environ, ASAN_OPTIONS='detect_leaks=0:halt_on_error=1',
               UBSAN_OPTIONS='halt_on_error=1:print_stacktrace=1')
    for rev in [2, 3]:
        result = subprocess.run([str(binary), str(rev)], env=env, text=True, capture_output=True)
        if result.returncode:
            print(result.stdout, result.stderr)
            result.check_returncode()
        print(result.stdout.splitlines()[-1])
