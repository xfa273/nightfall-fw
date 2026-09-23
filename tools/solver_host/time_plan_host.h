#ifndef NIGHTFALL_TOOLS_SOLVER_HOST_TIME_PLAN_HOST_H
#define NIGHTFALL_TOOLS_SOLVER_HOST_TIME_PLAN_HOST_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    NF_HOST_TURN_SET_PROFILE = 0,
    NF_HOST_TURN_SET_SMALL,
    NF_HOST_TURN_SET_ALL,
} NfHostTurnSet;

const char *nf_host_turn_set_name(NfHostTurnSet turn_set);

int nf_host_run_time_plan(const char *maze_path,
                          uint8_t mode,
                          uint8_t case_index,
                          NfHostTurnSet turn_set,
                          bool assert_valid);

#endif
