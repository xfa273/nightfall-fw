#ifndef F405_TIME_PATH_H
#define F405_TIME_PATH_H
#include <stdbool.h>
#include <stdint.h>

/* Foreground only, before motors/fan start. Loads the saved maze, generates
 * canonical path[] and prints nominal timing. Any failure clears path[]. */
bool f405_time_build_path(uint8_t mode, uint8_t case_index);
#endif
