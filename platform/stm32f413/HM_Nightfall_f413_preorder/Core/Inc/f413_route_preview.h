#ifndef F413_ROUTE_PREVIEW_H_
#define F413_ROUTE_PREVIEW_H_

/*
 * Read-only, non-motor shortest-route diagnostic for the saved FRAM maze and
 * preview-only 16x16 centre 2x2 goal.  The command never calls a path runner,
 * never changes the compiled exploration goal, and never writes NVM.
 */
void f413_route_preview_run_once(void);

#endif
