#include "f413_exploration.h"
#include "trace.h"

/* Port seam only: this runtime execution graph is not supported by the pinned
 * oracle. No planner workspace is borrowed and no completion is certified.
 * A future validated adapter must borrow the existing trace lease through
 * f413_trace_log_try_borrow_idle_scratch/release_idle_scratch; do not introduce
 * a second lease or replace the current trace storage/phase restart code. */
void f413_exploration_begin(uint8_t op_case, uint8_t param_index)
{
  if (op_case == 1U && param_index == 0U)
    trace_printf("[EXPLORE] unsupported runtime route model; retaining Adachi\r\n");
}

void f413_exploration_end(void) {}
void f413_exploration_poll(void) {}
bool f413_exploration_observe_goal(void) { return true; }

void f413_exploration_prepare(uint8_t target, uint8_t next_relative)
{
  (void)target;
  (void)next_relative;
}

uint8_t f413_exploration_decide(uint8_t target, bool accelerated,
                              uint8_t *next_relative, bool *known_straight,
                              bool *next_is_turn90)
{
  (void)target;
  (void)accelerated;
  (void)next_relative;
  (void)known_straight;
  (void)next_is_turn90;
  return 0U;
}
