#!/usr/bin/env python3
"""Reject missing F413 profile fields/aliases and optional compiled literal leaks."""
import argparse
import json
from pathlib import Path
import re
import shlex
import subprocess

ROOT = Path(__file__).resolve().parents[2]


def defines(text):
    return dict(re.findall(r'^\s*#define\s+(\w+)[ \t]+([^\n]*)', text, re.M))


def check(root, compile_commands=None):
    errors = []
    fields = re.findall(r'^X\(\w+, (\w+)\)',
                        (root / 'board/f413/f413_param_fields.def').read_text(), re.M)
    aliases = defines((root / 'board/f413/f413_runtime_aliases.h').read_text())
    profiles = sorted((root / 'params').glob('*/profile.c'))
    for profile in profiles:
        names = set(defines(profile.with_name('params.h').read_text()))
        names -= {'INC_PARAMS_H_', 'PARAMS_TUNE_VERSION'}
        if names != set(fields):
            errors.append(f'{profile.parent}: missing runtime fields {sorted(names - set(fields))}; '
                          f'undefined profile fields {sorted(set(fields) - names)}')
    if len(fields) != len(set(fields)):
        errors.append('duplicate runtime scalar fields')
    expected = {name: f'(f413_machine_params()->scalar.v_{name})' for name in fields}
    # Array layout and compact planner encoding are compile-time contracts.
    # f413_machine_resolve rejects a profile declaring a different MAZE_SIZE.
    expected['MAZE_SIZE'] = 'F413_COMPILED_MAZE_SIZE'
    expected['PARAMS_TUNE_VERSION'] = '(f413_machine_profile_name())'
    expected['searchRunParams'] = '(f413_machine_params()->search)'
    for mode in range(2, 8):
        expected[f'shortestRunModeParams{mode}'] = f'(f413_machine_params()->modes[{mode - 2}])'
        expected[f'shortestRunCaseParamsMode{mode}'] = f'(f413_machine_params()->cases[{mode - 2}])'
    for side, word in [('L', 'left'), ('R', 'right')]:
        expected[f'DIR_FWD_{side}'] = f'(f413_machine_hardware()->{word}_forward_in2_high ? 1U : 0U)'
        expected[f'DIR_BACK_{side}'] = f'(f413_machine_hardware()->{word}_forward_in2_high ? 0U : 1U)'
        expected[f'DIR_ENC_{side}'] = f'(f413_machine_hardware()->encoder_sign_{side.lower()})'
    for name, value in expected.items():
        if aliases.get(name) != value:
            errors.append(f'{name}: missing/incorrect runtime alias {aliases.get(name)!r}')
    compiled = 0
    if compile_commands:
        for entry in json.loads(compile_commands.read_text()):
            args = entry.get('arguments') or shlex.split(entry['command'])
            source = Path(entry['file'])
            if not any('NIGHTFALL_F413_RUNTIME_CONFIG' in arg for arg in args):
                continue
            if source.suffix != '.c' or source.name == 'profile.c':
                continue  # The immutable profile definitions must retain literals.
            flags = []
            skip = False
            for arg in args:
                if skip:
                    skip = False
                elif arg in ('-o', '-MF', '-MT', '-MQ'):
                    skip = True
                elif arg not in ('-c', '-MD', '-MMD', '-MP'):
                    flags.append(arg)
            result = subprocess.run(flags + ['-E', '-dM'], cwd=entry['directory'],
                                    check=True, text=True, capture_output=True)
            macros = defines(result.stdout)
            for name, value in expected.items():
                if name in macros and macros[name] != value:
                    errors.append(f'{source}: {name} bypasses selected profile: {macros[name]}')
            compiled += 1
        if compiled == 0:
            errors.append('no F413 runtime translation units found')
    if errors:
        raise SystemExit('\n'.join(errors))
    print(f'PASS: F413 parameter coverage ({len(profiles)} profiles, {len(fields)} scalars, '
          f'{compiled} preprocessed application sources)')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compile-commands', type=Path)
    options = parser.parse_args()
    check(ROOT, options.compile_commands)
