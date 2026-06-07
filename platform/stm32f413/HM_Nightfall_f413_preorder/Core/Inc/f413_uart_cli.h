#ifndef F413_UART_CLI_H_
#define F413_UART_CLI_H_

#include <stdint.h>

typedef struct
{
  void (*run_wall_sensor_test)(void);
  void (*run_tune_sub)(uint8_t sub);
  void (*trace_run_start)(void);
  void (*trace_run_stop)(void);
  void (*run_hardware_smoke_with_trace)(void);
  void (*run_idle_trace)(void);
  void (*run_motor_trace)(void);
  void (*run_search_trace_entry)(void);
  void (*run_shortest_trace_entry)(void);
} f413_uart_cli_config_t;

void f413_uart_cli_config(const f413_uart_cli_config_t* config);
void f413_uart_cli_print_help(void);
void f413_uart_cli_handle_command(uint8_t cmd);

#endif
