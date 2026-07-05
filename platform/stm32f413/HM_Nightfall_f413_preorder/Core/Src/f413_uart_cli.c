#include "f413_uart_cli.h"

#include <stddef.h>

#include "f413_hw_diag.h"
#include "f413_imu_diag.h"
#include "f413_nvm_diag.h"
#include "f413_op_ui.h"
#include "f413_search_step.h"
#include "f413_test_run.h"
#include "f413_trace_diag.h"
#include "f413_trace_sample.h"
#include "f413_wall_runtime.h"
#include "trace.h"

static f413_uart_cli_config_t g_uart_cli_config;

void f413_uart_cli_config(const f413_uart_cli_config_t* config)
{
  if (config == NULL)
  {
    g_uart_cli_config = (f413_uart_cli_config_t){0};
    return;
  }

  g_uart_cli_config = *config;
}

void f413_uart_cli_print_help(void)
{
  trace_printf("[NVM-TEST] commands: h=help, a=save+load all, A=load-only all\r\n");
  trace_printf("[NVM-TEST] d/s/m/t=save+load, D/S/M/T=load-only verify\r\n");
  trace_printf("[TRACE-LOG] q=format, r=append sample, R=dump latest, v/V=dump csv(256/all), </>=dump bin(256/all), k=selftest, u=run-start hook, U=run-stop hook\r\n");
  trace_printf("[RUN-TEST]  x=idle-run-session(1000ms), y=motor-run-session(short), z=search-entry(solver/fallback), j=shortest-entry(solver/fallback)\r\n");
  trace_printf("[HW-TEST]  w=wall, n=wall-distance, W=wall-end, O=search-map, G=search-preview, B=search-reset, N=search-step, [/]/@=state/clear/dump, p=switch, i=imu, I=imu-angle, c=imu-accel, b=buzzer, o/0=motor, e=encoder, l=led30s, g=smoke+trace\r\n");
  trace_printf("[TEST]     1=S3straight, 2=S6straight, 3=R90turn, 4=L90turn, 5=S3+R90+S3, F=arm for button; OP mode9/case0/sub0-9=control tune\r\n");
  trace_printf("[TEST]     OP mode2-7/case0/sub0-9=path-code tests\r\n");
  trace_printf("[TUNE]     !/\"/#/$/%%/^/&/*/(/)=OP mode9 case0 sub0..9 shortcut, then V=dump CSV\r\n");
  trace_printf("[HW-ENC]  6=L-motor-fwd, 7=R-motor-fwd, 8=L-motor-rev, 9=R-motor-rev (open-loop+enc)\r\n");
  trace_printf("[OP-UI]   F405-compatible select: PUSH increments 0..9 at each level, FR wall only=enter, mode9 case0=tune, case5=dump latest full log(bin), case8=side base save, case9=sensor offset save\r\n");
  trace_printf("[OP-UART] P=PUSH increment, E=FR enter; reset via ST-LINK software reset\r\n");
}

static void f413_uart_cli_run_tune_sub(uint8_t sub)
{
  if (g_uart_cli_config.run_tune_sub != NULL)
  {
    g_uart_cli_config.run_tune_sub(sub);
  }
}

void f413_uart_cli_handle_command(uint8_t cmd)
{
  switch (cmd)
  {
    case 'h':
    case 'H':
    case '?':
      f413_uart_cli_print_help();
      break;

    case 'a':
      trace_printf("[NVM-TEST] run all\r\n");
      f413_nvm_diag_run_all_tests();
      break;

    case 'A':
      trace_printf("[NVM-TEST] verify all (load_only)\r\n");
      f413_nvm_diag_verify_all_load_only();
      break;

    case 'd':
      trace_printf("[NVM-TEST] run distance\r\n");
      (void)f413_nvm_diag_run_distance_test();
      break;

    case 'D':
      trace_printf("[NVM-TEST] verify distance (load_only)\r\n");
      (void)f413_nvm_diag_verify_distance_load_only();
      break;

    case 's':
      trace_printf("[NVM-TEST] run sensor\r\n");
      (void)f413_nvm_diag_run_sensor_test();
      break;

    case 'S':
      trace_printf("[NVM-TEST] verify sensor (load_only)\r\n");
      (void)f413_nvm_diag_verify_sensor_load_only();
      break;

    case 'm':
      trace_printf("[NVM-TEST] run maze\r\n");
      (void)f413_nvm_diag_run_maze_test();
      break;

    case 't':
      trace_printf("[NVM-TEST] run trace\r\n");
      (void)f413_nvm_diag_run_trace_log_test();
      break;

    case 'M':
      trace_printf("[NVM-TEST] verify maze (load_only)\r\n");
      (void)f413_nvm_diag_verify_maze_load_only();
      break;

    case 'T':
      trace_printf("[NVM-TEST] verify trace (load_only)\r\n");
      (void)f413_nvm_diag_verify_trace_log_load_only();
      break;

    case 'w':
      if (g_uart_cli_config.run_wall_sensor_test != NULL)
      {
        g_uart_cli_config.run_wall_sensor_test();
      }
      break;

    case 'n':
      if (g_uart_cli_config.run_wall_distance_test != NULL)
      {
        g_uart_cli_config.run_wall_distance_test();
      }
      break;

    case 'W':
      f413_wall_runtime_run_end_monitor_once();
      break;

    case 'p':
      f413_hw_diag_run_switch_test_once();
      break;

    case 'P':
      f413_op_ui_uart_push_once();
      break;

    case 'i':
      f413_imu_diag_run_whoami_test_once();
      break;

    case 'I':
      f413_imu_diag_run_manual_turn_test_once();
      break;

    case 'c':
    case 'C':
      f413_imu_diag_run_accel_test_once();
      break;

    case 'b':
      f413_hw_diag_run_buzzer_test_once();
      break;

    case 'o':
    case '0':
      f413_hw_diag_run_motor_driver_test_once();
      break;

    case 'e':
      f413_hw_diag_run_encoder_test_once();
      break;

    case 'E':
      f413_op_ui_uart_enter_once();
      break;

    case '!':
      f413_uart_cli_run_tune_sub(0U);
      break;

    case '"':
      f413_uart_cli_run_tune_sub(1U);
      break;

    case '#':
      f413_uart_cli_run_tune_sub(2U);
      break;

    case '$':
      f413_uart_cli_run_tune_sub(3U);
      break;

    case '%':
      f413_uart_cli_run_tune_sub(4U);
      break;

    case '^':
      f413_uart_cli_run_tune_sub(5U);
      break;

    case '&':
      f413_uart_cli_run_tune_sub(6U);
      break;

    case '*':
      f413_uart_cli_run_tune_sub(7U);
      break;

    case '(':
      f413_uart_cli_run_tune_sub(8U);
      break;

    case ')':
      f413_uart_cli_run_tune_sub(9U);
      break;

    case 'G':
      f413_search_step_run_decision_preview_once();
      break;

    case 'B':
      f413_search_step_session_reset();
      trace_printf("[SEARCH-STEP] session reset\r\n");
      break;

    case 'N':
      f413_search_step_run_once();
      break;

    case '[':
      f413_search_step_run_status_once();
      break;

    case ']':
      f413_search_step_run_map_clear_once();
      break;

    case '@':
      f413_search_step_run_map_dump_once();
      break;

    case 'O':
      f413_search_step_run_map_probe_once();
      break;

    case 'q':
    case 'Q':
      f413_trace_diag_run_format_once();
      break;

    case 'r':
      f413_trace_diag_run_append_sample_once();
      break;

    case 'R':
      f413_trace_diag_run_dump_latest_once();
      break;

    case 'v':
      f413_trace_diag_run_dump_csv_once();
      break;

    case 'V':
      f413_trace_diag_run_dump_csv_all_once();
      break;

    case '<':
      f413_trace_diag_run_dump_bin_once();
      break;

    case '>':
      f413_trace_diag_run_dump_bin_all_once();
      break;

    case 'k':
    case 'K':
      f413_trace_diag_run_selftest_once();
      break;

    case 'u':
      if (g_uart_cli_config.trace_run_start != NULL)
      {
        g_uart_cli_config.trace_run_start();
      }
      break;

    case 'U':
      if (g_uart_cli_config.trace_run_stop != NULL)
      {
        g_uart_cli_config.trace_run_stop();
      }
      break;

    case 'l':
    case 'L':
      f413_hw_diag_run_led_test_once();
      break;

    case 'g':
      if (g_uart_cli_config.run_hardware_smoke_with_trace != NULL)
      {
        g_uart_cli_config.run_hardware_smoke_with_trace();
      }
      break;

    case 'x':
    case 'X':
      f413_trace_sample_set_context(0U, 0xFFU, 0xFFU, (uint8_t)'x');
      if (g_uart_cli_config.run_idle_trace != NULL)
      {
        g_uart_cli_config.run_idle_trace();
      }
      break;

    case 'y':
    case 'Y':
      f413_trace_sample_set_context(0U, 0xFFU, 0xFFU, (uint8_t)'y');
      if (g_uart_cli_config.run_motor_trace != NULL)
      {
        g_uart_cli_config.run_motor_trace();
      }
      break;

    case 'z':
    case 'Z':
      f413_trace_sample_set_context(1U, 4U, 0xFFU, (uint8_t)'z');
      if (g_uart_cli_config.run_search_trace_entry != NULL)
      {
        g_uart_cli_config.run_search_trace_entry();
      }
      break;

    case 'j':
    case 'J':
      f413_trace_sample_set_context(2U, 1U, 0xFFU, (uint8_t)'j');
      if (g_uart_cli_config.run_shortest_trace_entry != NULL)
      {
        g_uart_cli_config.run_shortest_trace_entry();
      }
      break;

    case '6':
    case '7':
    case '8':
    case '9':
      f413_trace_sample_set_context(8U, 0xFFU, 0xFFU, cmd);
      f413_test_run_run_now(cmd);
      break;

    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
      f413_test_run_arm_for_button(cmd);
      break;

    case 'f':
    case 'F':
      {
        uint8_t arm_id = f413_test_run_is_armed() ? f413_test_run_armed_id() : (uint8_t)'1';
        f413_test_run_arm_for_button(arm_id);
      }
      break;

    case '\r':
    case '\n':
      break;

    default:
      trace_printf("[NVM-TEST] unknown command '%c' (0x%02X)\r\n", (char)cmd, (unsigned int)cmd);
      f413_uart_cli_print_help();
      break;
  }
}
