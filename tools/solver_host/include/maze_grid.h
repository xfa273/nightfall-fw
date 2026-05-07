#ifndef NIGHTFALL_SOLVER_HOST_MAZE_GRID_H_
#define NIGHTFALL_SOLVER_HOST_MAZE_GRID_H_

#include "global.h"
#include <stdbool.h>
#include <stdint.h>

#ifndef NORTH_WALL
#define NORTH_WALL 0x8
#endif
#ifndef EAST_WALL
#define EAST_WALL  0x4
#endif
#ifndef SOUTH_WALL
#define SOUTH_WALL 0x2
#endif
#ifndef WEST_WALL
#define WEST_WALL  0x1
#endif

extern uint8_t maze[MAZE_SIZE][MAZE_SIZE];
extern bool path_cell[MAZE_SIZE][MAZE_SIZE];

void initializeMaze(uint8_t maze_[MAZE_SIZE][MAZE_SIZE]);
void setMazeWalls(uint8_t walls[MAZE_SIZE][MAZE_SIZE]);
void correctWallInconsistencies(void);
void reverseArrayYAxis(uint8_t array[MAZE_SIZE][MAZE_SIZE]);
void printMaze(void);

#endif
