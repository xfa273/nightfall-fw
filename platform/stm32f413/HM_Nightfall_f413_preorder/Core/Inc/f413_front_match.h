#ifndef F413_FRONT_MATCH_H_
#define F413_FRONT_MATCH_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  F413_FRONT_MATCH_PHASE_ALIGN_YAW = 0,
  F413_FRONT_MATCH_PHASE_SETTLE_YAW,
  F413_FRONT_MATCH_PHASE_ALIGN_POSITION,
  F413_FRONT_MATCH_PHASE_SETTLE_FINAL,
  F413_FRONT_MATCH_PHASE_HOLD,
  F413_FRONT_MATCH_PHASE_BACKOFF_TOO_CLOSE,
  F413_FRONT_MATCH_PHASE_PAUSED_WALL_LOST = 0xFE,
  F413_FRONT_MATCH_PHASE_PAUSED_TOO_CLOSE = 0xFF,
} f413_front_match_phase_t;

typedef struct
{
  f413_front_match_phase_t phase;
  uint16_t phase_elapsed_ms;
  uint16_t settle_gap_elapsed_ms;
  uint16_t release_elapsed_ms;
} f413_front_match_controller_t;

typedef struct
{
  float velocity_mm_s;
  float omega_deg_s;
  f413_front_match_phase_t phase;
  bool phase_changed;
  bool complete;
  bool holding;
} f413_front_match_output_t;

void f413_front_match_init(f413_front_match_controller_t* controller);
void f413_front_match_step(f413_front_match_controller_t* controller,
                           float position_error_mm,
                           float yaw_error_mm,
                           uint16_t elapsed_ms,
                           f413_front_match_output_t* output);
const char* f413_front_match_phase_name(f413_front_match_phase_t phase);

#endif
