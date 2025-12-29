#ifndef INC_MAZE_GRID_H_
#define INC_MAZE_GRID_H_

#include "global.h"
#include <stdbool.h>
#include <stdint.h>

#ifndef NORTH_WALL
#define NORTH_WALL 0x8 // 北
#endif
#ifndef EAST_WALL
#define EAST_WALL  0x4 // 東
#endif
#ifndef SOUTH_WALL
#define SOUTH_WALL 0x2 // 南
#endif
#ifndef WEST_WALL
#define WEST_WALL  0x1 // 西
#endif

#ifdef __cplusplus
extern "C" {
#endif

extern uint8_t maze[MAZE_SIZE][MAZE_SIZE];
extern bool    path_cell[MAZE_SIZE][MAZE_SIZE];

void initializeMaze(uint8_t maze_[MAZE_SIZE][MAZE_SIZE]);
void setMazeWalls(uint8_t walls[MAZE_SIZE][MAZE_SIZE]);
void correctWallInconsistencies(void);
void reverseArrayYAxis(uint8_t array[MAZE_SIZE][MAZE_SIZE]);
void printMaze(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_MAZE_GRID_H_ */
