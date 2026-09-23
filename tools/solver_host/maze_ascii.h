#ifndef NIGHTFALL_TOOLS_SOLVER_HOST_MAZE_ASCII_H
#define NIGHTFALL_TOOLS_SOLVER_HOST_MAZE_ASCII_H

#include "orthogonal_time_planner.h"

#include <stddef.h>
#include <stdint.h>

typedef enum {
    NF_MAZE_ASCII_OK = 0,
    NF_MAZE_ASCII_IO_ERROR,
    NF_MAZE_ASCII_INVALID_FORMAT,
    NF_MAZE_ASCII_UNKNOWN_WALL,
    NF_MAZE_ASCII_INVALID_MARKERS,
} NfMazeAsciiStatus;

typedef struct {
    uint8_t start_x;
    uint8_t start_y;
    size_t goal_count;
} NfMazeAsciiInfo;

const char *nf_maze_ascii_status_name(NfMazeAsciiStatus status);

NfMazeAsciiStatus nf_maze_ascii_parse(const char *text,
                                      NfRouteMaze *maze,
                                      NfMazeAsciiInfo *info,
                                      char *error,
                                      size_t error_size);

NfMazeAsciiStatus nf_maze_ascii_load(const char *path,
                                     NfRouteMaze *maze,
                                     NfMazeAsciiInfo *info,
                                     char *error,
                                     size_t error_size);

#endif
