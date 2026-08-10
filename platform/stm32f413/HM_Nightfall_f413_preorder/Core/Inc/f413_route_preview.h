#ifndef F413_ROUTE_PREVIEW_H_
#define F413_ROUTE_PREVIEW_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * Read-only, non-motor shortest-route diagnostic for the saved FRAM maze and
 * preview-only 16x16 centre 2x2 goal.  The command never calls a path runner,
 * never changes the compiled exploration goal, and never writes NVM.
 */
void f413_route_preview_run_once(void);

/*
 * Build an executable, zero-terminated legacy path[] for a mode2 case using
 * the saved FRAM maze, compiled goal set, and KERI #1--#5 diagonal planner.
 * The planner only emits nominal-speed turns and a cardinal stopping tail so
 * the current F413 path runner can execute the result without a speed sidecar.
 * No motor, trace capture, or NVM write is started by this function.
 */
bool f413_route_build_mode2_path(uint8_t case_index);

#endif
