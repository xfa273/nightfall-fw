#!/usr/bin/env python3
"""Host-only checks for the production wall guard and shortest planner LED hooks."""
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src"
OUT = ROOT / "build/hil_host/status_led"


def function(source, signature):
    start = source.index(signature + "\n{")
    end = source.index("\n}", start) + 2
    return source[start:end]


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    guard = function((SRC / "main.c").read_text(),
                     "static bool nightfall_run_session_wall_sensor_ok(void)")
    planner = function((SRC / "f413_mode_shortest.c").read_text(),
                       "void f413_mode_shortest_run_config(const f413_shortest_case_config_t* config)")
    source = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "f413_wall_sensor.h"
#include "f413_mode_shortest.h"
#include "f413_trace_flags.h"
#define NIGHTFALL_F413_REAL_RUN_PATH_ENABLED 1
typedef f413_wall_sensor_snapshot_t nightfall_wall_sensor_snapshot_t;
static bool readable, path_ok;
static unsigned led_calls, led_bits, failures, runs, legacy, diagonal, resets;
static f413_wall_sensor_snapshot_t sample;
static bool nightfall_wall_sensor_read_snapshot(nightfall_wall_sensor_snapshot_t* out)
{ *out = sample; return readable; }
void f413_hw_show_wall_status_leds(bool valid, bool front, bool right, bool left)
{ led_calls++; led_bits = valid ? front + 2U*right + 4U*left : 0; }
void f413_hw_show_run_failure(void) { failures++; }
int trace_printf(const char* format, ...) { (void)format; return 0; }
void f413_run_features_set(const f413_run_features_t* features) { assert(features); }
void f413_run_features_reset(void) { resets++; }
bool solver_build_path(uint8_t mode, uint8_t op_case)
{ assert(mode == 3 && op_case == 1); legacy++; return path_ok; }
bool f413_route_build_mode2_path(uint8_t op_case)
{ assert(op_case == 6); diagonal++; return path_ok; }
void f413_path_run_print_preview(void) { assert(path_ok); }
void f413_path_run_session_once(uint8_t mode, uint8_t op_case, uint16_t flags, const char* label)
{ (void)mode; (void)op_case; (void)flags; (void)label; assert(path_ok); runs++; }
''' + guard + "\n" + planner + r'''
int main(void)
{
  for (unsigned read = 0; read <= 1; read++)
  for (unsigned sat = 0; sat <= 1; sat++)
  for (unsigned walls = 0; walls < 8; walls++)
  {
    readable = read; sample.saturated = sat;
    sample.front_wall = walls & 1; sample.right_wall = walls & 2; sample.left_wall = walls & 4;
    led_calls = 0;
    assert(nightfall_run_session_wall_sensor_ok() == (read && !sat));
    assert(led_calls == 1 && led_bits == ((read && !sat) ? walls : 0));
  }
  for (unsigned keri = 0; keri <= 1; keri++)
  for (unsigned ok = 0; ok <= 1; ok++)
  {
    const f413_shortest_case_config_t config = {
      .mode = keri ? 2 : 3, .op_case = keri ? 6 : 1, .diagonal_time_plan = keri};
    path_ok = ok; failures = runs = legacy = diagonal = resets = 0;
    f413_mode_shortest_run_config(&config);
    assert(runs == ok && failures == !ok && resets == 1);
    assert(legacy == !keri && diagonal == keri);
  }
  puts("status LED integration: guard validity unchanged; legacy/KERI no-path indication without run PASS");
}
'''
    test = OUT / "status_led.c"
    test.write_text(source)
    binary = OUT / "status_led"
    includes = ["tools/hil/nvm_stubs", "nvm", str(SRC.parent / "Inc")]
    subprocess.run([os.environ.get("CC", "cc"), "-std=c11", "-Wall", "-Wextra", "-Werror",
                    "-DSTM32F413xx", "-fsanitize=address,undefined", "-g",
                    *[f"-I{ROOT / item}" for item in includes], str(test), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True, env=dict(os.environ,
        ASAN_OPTIONS="detect_leaks=0:halt_on_error=1", UBSAN_OPTIONS="halt_on_error=1"))


if __name__ == "__main__":
    main()
