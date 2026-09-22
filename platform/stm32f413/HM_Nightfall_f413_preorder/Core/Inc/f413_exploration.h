#ifndef F413_EXPLORATION_H
#define F413_EXPLORATION_H
#include <stdbool.h>
#include <stdint.h>

/* Default-OFF integration. No motors, NVM writes or commands in this module. */
void f413_exploration_begin(uint8_t op_case, uint8_t param_index);
void f413_exploration_end(void);
/* Record real goal arrival before any phase stop/back-up/restart motion. */
bool f413_exploration_observe_goal(void);
/* Called from the existing guarded foreground motion wait, never an ISR. */
void f413_exploration_poll(void);
/* Queue a proposal for arrival after the chosen motion; no planner drain. */
void f413_exploration_prepare(uint8_t target, uint8_t next_relative);
/* 0 = retain Adachi, 1 = use all fields, 2 = certified done, 3 = abort. */
uint8_t f413_exploration_decide(uint8_t target, bool accelerated, uint8_t *next_relative,
                              bool *known_straight, bool *next_is_turn90);
#endif
