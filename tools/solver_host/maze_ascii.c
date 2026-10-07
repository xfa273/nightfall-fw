#include "maze_ascii.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define NF_MAZE_ASCII_MAX_LINES ((2U * NF_ROUTE_MAZE_MAX_SIZE) + 1U)

static void nf_maze_ascii_error(char *error, size_t error_size, const char *message)
{
    if (error != NULL && error_size > 0U) {
        snprintf(error, error_size, "%s", message);
    }
}

static NfMazeAsciiStatus nf_maze_ascii_fail(char *error,
                                            size_t error_size,
                                            NfMazeAsciiStatus status,
                                            const char *message)
{
    nf_maze_ascii_error(error, error_size, message);
    return status;
}

const char *nf_maze_ascii_status_name(NfMazeAsciiStatus status)
{
    switch (status) {
    case NF_MAZE_ASCII_OK: return "ok";
    case NF_MAZE_ASCII_IO_ERROR: return "io-error";
    case NF_MAZE_ASCII_INVALID_FORMAT: return "invalid-format";
    case NF_MAZE_ASCII_UNKNOWN_WALL: return "unknown-wall";
    case NF_MAZE_ASCII_INVALID_MARKERS: return "invalid-markers";
    default: return "unknown";
    }
}

NfMazeAsciiStatus nf_maze_ascii_parse(const char *text,
                                      NfRouteMaze *maze,
                                      NfMazeAsciiInfo *info,
                                      char *error,
                                      size_t error_size)
{
    char *copy;
    char *lines[NF_MAZE_ASCII_MAX_LINES];
    size_t line_count = 0U;
    size_t width;
    size_t height;
    size_t first_line_length;
    size_t start_count = 0U;
    NfMazeAsciiInfo parsed_info = {0U, 0U, 0U};

    if (text == NULL || maze == NULL || info == NULL) {
        return nf_maze_ascii_fail(error, error_size,
                                  NF_MAZE_ASCII_INVALID_FORMAT,
                                  "null parser argument");
    }
    if (strchr(text, '.') != NULL) {
        return nf_maze_ascii_fail(error, error_size,
                                  NF_MAZE_ASCII_UNKNOWN_WALL,
                                  "maze contains unknown '.' walls");
    }

    copy = (char *)malloc(strlen(text) + 1U);
    if (copy == NULL) {
        return nf_maze_ascii_fail(error, error_size,
                                  NF_MAZE_ASCII_IO_ERROR,
                                  "out of memory");
    }
    strcpy(copy, text);

    for (char *cursor = copy; *cursor != '\0';) {
        char *line = cursor;
        char *end = strpbrk(cursor, "\r\n");
        if (end != NULL) {
            char separator = *end;
            *end = '\0';
            cursor = end + 1;
            if ((separator == '\r' && *cursor == '\n') ||
                (separator == '\n' && *cursor == '\r')) {
                cursor++;
            }
        } else {
            cursor += strlen(cursor);
        }
        if (*line == '\0') {
            continue;
        }
        if (line_count >= NF_MAZE_ASCII_MAX_LINES) {
            free(copy);
            return nf_maze_ascii_fail(error, error_size,
                                      NF_MAZE_ASCII_INVALID_FORMAT,
                                      "maze has too many lines");
        }
        lines[line_count++] = line;
    }

    if (line_count < 3U || (line_count & 1U) == 0U) {
        free(copy);
        return nf_maze_ascii_fail(error, error_size,
                                  NF_MAZE_ASCII_INVALID_FORMAT,
                                  "maze needs 2*h+1 non-empty lines");
    }
    first_line_length = strlen(lines[0]);
    if (first_line_length < 5U || ((first_line_length - 1U) % 4U) != 0U) {
        free(copy);
        return nf_maze_ascii_fail(error, error_size,
                                  NF_MAZE_ASCII_INVALID_FORMAT,
                                  "invalid first line width");
    }
    width = (first_line_length - 1U) / 4U;
    height = (line_count - 1U) / 2U;
    if (width != height || width > NF_ROUTE_MAZE_MAX_SIZE) {
        free(copy);
        return nf_maze_ascii_fail(error, error_size,
                                  NF_MAZE_ASCII_INVALID_FORMAT,
                                  "maze must be square and at most 32x32");
    }
    for (size_t line = 0U; line < line_count; line++) {
        if (strlen(lines[line]) != first_line_length) {
            free(copy);
            return nf_maze_ascii_fail(error, error_size,
                                      NF_MAZE_ASCII_INVALID_FORMAT,
                                      "maze lines have inconsistent widths");
        }
        if ((line & 1U) == 0U) {
            for (size_t junction = 0U; junction <= width; junction++) {
                if (lines[line][junction * 4U] != '+') {
                    free(copy);
                    return nf_maze_ascii_fail(error, error_size,
                                              NF_MAZE_ASCII_INVALID_FORMAT,
                                              "horizontal wall line needs '+' junctions");
                }
            }
        }
    }
    if (!nf_route_maze_init(maze, (uint8_t)width, (uint8_t)height)) {
        free(copy);
        return nf_maze_ascii_fail(error, error_size,
                                  NF_MAZE_ASCII_INVALID_FORMAT,
                                  "invalid maze dimensions");
    }

    for (size_t row_from_top = 0U; row_from_top < height; row_from_top++) {
        const char *north_line = lines[row_from_top * 2U];
        const char *cell_line = lines[row_from_top * 2U + 1U];
        const char *south_line = lines[row_from_top * 2U + 2U];
        const uint8_t y = (uint8_t)(height - 1U - row_from_top);

        for (size_t x = 0U; x < width; x++) {
            const size_t interior = x * 4U + 1U;
            const bool north_wall = north_line[interior] == '-' &&
                                    north_line[interior + 1U] == '-' &&
                                    north_line[interior + 2U] == '-';
            const bool north_open = north_line[interior] == ' ' &&
                                    north_line[interior + 1U] == ' ' &&
                                    north_line[interior + 2U] == ' ';
            const bool south_wall = south_line[interior] == '-' &&
                                    south_line[interior + 1U] == '-' &&
                                    south_line[interior + 2U] == '-';
            const bool south_open = south_line[interior] == ' ' &&
                                    south_line[interior + 1U] == ' ' &&
                                    south_line[interior + 2U] == ' ';
            const char west = cell_line[x * 4U];
            const char east = cell_line[(x + 1U) * 4U];
            size_t cell_start_count = 0U;
            size_t cell_goal_count = 0U;

            if ((!north_wall && !north_open) || (!south_wall && !south_open) ||
                (west != '|' && west != ' ') || (east != '|' && east != ' ')) {
                free(copy);
                return nf_maze_ascii_fail(error, error_size,
                                          NF_MAZE_ASCII_INVALID_FORMAT,
                                          "invalid wall glyph");
            }
            if (north_wall) {
                (void)nf_route_maze_set_wall(maze, (uint8_t)x, y,
                                             NF_ROUTE_DIR_NORTH);
            }
            if (south_wall) {
                (void)nf_route_maze_set_wall(maze, (uint8_t)x, y,
                                             NF_ROUTE_DIR_SOUTH);
            }
            if (west == '|') {
                (void)nf_route_maze_set_wall(maze, (uint8_t)x, y,
                                             NF_ROUTE_DIR_WEST);
            }
            if (east == '|') {
                (void)nf_route_maze_set_wall(maze, (uint8_t)x, y,
                                             NF_ROUTE_DIR_EAST);
            }
            for (size_t offset = 0U; offset < 3U; offset++) {
                const char marker = cell_line[interior + offset];
                if (marker == 'S') {
                    cell_start_count++;
                } else if (marker == 'G') {
                    cell_goal_count++;
                } else if (marker != ' ') {
                    free(copy);
                    return nf_maze_ascii_fail(error, error_size,
                                              NF_MAZE_ASCII_INVALID_FORMAT,
                                              "invalid cell glyph");
                }
            }
            if (cell_start_count > 1U || cell_goal_count > 1U ||
                (cell_start_count != 0U && cell_goal_count != 0U)) {
                free(copy);
                return nf_maze_ascii_fail(error, error_size,
                                          NF_MAZE_ASCII_INVALID_MARKERS,
                                          "cell has ambiguous S/G markers");
            }
            if (cell_start_count == 1U) {
                start_count += cell_start_count;
                parsed_info.start_x = (uint8_t)x;
                parsed_info.start_y = y;
            }
            if (cell_goal_count == 1U) {
                maze->goals[y][x] = true;
                parsed_info.goal_count += cell_goal_count;
            }
        }
    }
    free(copy);

    if (start_count != 1U || parsed_info.goal_count == 0U) {
        return nf_maze_ascii_fail(error, error_size,
                                  NF_MAZE_ASCII_INVALID_MARKERS,
                                  "maze needs exactly one S and at least one G");
    }
    *info = parsed_info;
    nf_maze_ascii_error(error, error_size, "ok");
    return NF_MAZE_ASCII_OK;
}

NfMazeAsciiStatus nf_maze_ascii_load(const char *path,
                                     NfRouteMaze *maze,
                                     NfMazeAsciiInfo *info,
                                     char *error,
                                     size_t error_size)
{
    FILE *file;
    long length;
    char *text;
    NfMazeAsciiStatus status;

    if (path == NULL) {
        return nf_maze_ascii_fail(error, error_size,
                                  NF_MAZE_ASCII_IO_ERROR,
                                  "null maze path");
    }
    file = fopen(path, "rb");
    if (file == NULL) {
        return nf_maze_ascii_fail(error, error_size,
                                  NF_MAZE_ASCII_IO_ERROR,
                                  "failed to open maze file");
    }
    if (fseek(file, 0L, SEEK_END) != 0 || (length = ftell(file)) < 0L ||
        fseek(file, 0L, SEEK_SET) != 0) {
        fclose(file);
        return nf_maze_ascii_fail(error, error_size,
                                  NF_MAZE_ASCII_IO_ERROR,
                                  "failed to inspect maze file");
    }
    text = (char *)malloc((size_t)length + 1U);
    if (text == NULL) {
        fclose(file);
        return nf_maze_ascii_fail(error, error_size,
                                  NF_MAZE_ASCII_IO_ERROR,
                                  "out of memory");
    }
    if (fread(text, 1U, (size_t)length, file) != (size_t)length) {
        free(text);
        fclose(file);
        return nf_maze_ascii_fail(error, error_size,
                                  NF_MAZE_ASCII_IO_ERROR,
                                  "failed to read maze file");
    }
    text[length] = '\0';
    fclose(file);
    status = nf_maze_ascii_parse(text, maze, info, error, error_size);
    free(text);
    return status;
}
