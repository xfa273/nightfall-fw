#include "global.h"
#include "maze_grid.h"

#include <stdio.h>

// 迷路のデータを格納する2次元配列（top-left 原点）
uint8_t maze[MAZE_SIZE][MAZE_SIZE];

// 経路オーバレイ（bottom-left 原点で格納）
bool path_cell[MAZE_SIZE][MAZE_SIZE] = {{false}};

void initializeMaze(uint8_t maze_[MAZE_SIZE][MAZE_SIZE]) {
    for (int y = 0; y < MAZE_SIZE; y++) {
        for (int x = 0; x < MAZE_SIZE; x++) {
            maze_[y][x] = 0; // 最初はすべての壁がないと仮定
        }
    }
}

void setMazeWalls(uint8_t walls[MAZE_SIZE][MAZE_SIZE]) {
    // 与えられた壁情報で迷路を更新
    for (int y = 0; y < MAZE_SIZE; y++) {
        for (int x = 0; x < MAZE_SIZE; x++) {
            maze[y][x] = walls[y][x];
        }
    }
}

void correctWallInconsistencies(void) {
    // 壁情報の矛盾を修正
    for (int y = 0; y < MAZE_SIZE; y++) {
        for (int x = 0; x < MAZE_SIZE; x++) {
            // 東の壁と隣接区画の西の壁をチェック
            if (x < MAZE_SIZE - 1) {
                if ((maze[y][x] & EAST_WALL) != (maze[y][x + 1] & WEST_WALL)) {
                    maze[y][x] |= EAST_WALL;
                    maze[y][x + 1] |= WEST_WALL;
                }
            }

            // 南の壁と隣接区画の北の壁をチェック
            if (y < MAZE_SIZE - 1) {
                if ((maze[y][x] & SOUTH_WALL) != (maze[y + 1][x] & NORTH_WALL)) {
                    maze[y][x] |= SOUTH_WALL;
                    maze[y + 1][x] |= NORTH_WALL;
                }
            }
        }
    }
}

void reverseArrayYAxis(uint8_t array[MAZE_SIZE][MAZE_SIZE]) {
    for (int i = 0; i < MAZE_SIZE / 2; i++) {
        for (int j = 0; j < MAZE_SIZE; j++) {
            // 一時変数を使用して、行の要素を交換
            uint8_t temp = array[i][j];
            array[i][j] = array[MAZE_SIZE - 1 - i][j];
            array[MAZE_SIZE - 1 - i][j] = temp;
        }
    }
}

void printMaze(void) {
    // 迷路の情報を表示
    for (int y = 0; y < MAZE_SIZE; y++) {
        int mazeY = MAZE_SIZE - 1 - y; // bottom-left 参照用にy反転
        // 区画の上部の壁と柱を表示
        for (int x = 0; x < MAZE_SIZE; x++) {
            printf(" ");
            if (maze[y][x] & NORTH_WALL) {
                printf("--- ---");
            } else {
                printf("       ");
            }
        }
        printf(" \n");

        // 左側の壁と中身（空白）
        for (int x = 0; x < MAZE_SIZE; x++) {
            if (maze[y][x] & WEST_WALL) {
                printf("|");
            } else {
                printf(" ");
            }
            printf("       ");
        }
        printf("|\n");

        // 経路表示行
        for (int x = 0; x < MAZE_SIZE; x++) {
            if (path_cell[mazeY][x]) {
                printf("    +   ");
            } else {
                printf("        ");
            }
        }
        printf(" \n");

        // 下側の行
        for (int x = 0; x < MAZE_SIZE; x++) {
            if (maze[y][x] & WEST_WALL) {
                printf("|       ");
            } else {
                printf("        ");
            }
        }
        printf("|\n");
    }

    // 最下段
    for (int x = 0; x < MAZE_SIZE; x++) {
        printf(" --- ---");
    }
    printf(" \n");
}
