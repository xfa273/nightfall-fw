#ifndef F413_MODE_SHORTEST_H_
#define F413_MODE_SHORTEST_H_

#include <stdint.h>

#include "f413_run_features.h"

typedef struct {
  uint8_t mode;
  uint8_t op_case;
  const char* label;
  f413_run_features_t features;
} f413_shortest_case_config_t;

void f413_mode_shortest_run_case(uint8_t mode, uint8_t op_case);
void f413_mode_shortest_run_config(const f413_shortest_case_config_t* config);
void f413_mode_shortest_run_case0_path(const char* label,
                                       uint8_t mode,
                                       uint8_t case_index,
                                       const uint16_t* codes,
                                       uint16_t code_count);

#endif
