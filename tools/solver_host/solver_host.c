#define MAIN_C_
#include "global.h"

#include "maze_grid.h"
#include "solver.h"

#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <string.h>
#include <unistd.h>

static uint8_t s_walls_bl[MAZE_SIZE][MAZE_SIZE];
static unsigned int s_width = 16U;
static unsigned int s_height = 16U;

static void clear_sample_area(unsigned int width, unsigned int height)
{
    for (unsigned int y = 0U; y < MAZE_SIZE; y++) {
        for (unsigned int x = 0U; x < MAZE_SIZE; x++) {
            s_walls_bl[y][x] = 0x0FU;
        }
    }
    for (unsigned int y = 0U; y < height; y++) {
        for (unsigned int x = 0U; x < width; x++) {
            s_walls_bl[y][x] = 0U;
        }
    }
}

static void set_wall_pair(unsigned int x, unsigned int y, uint8_t wall)
{
    if (x >= s_width || y >= s_height) {
        return;
    }
    s_walls_bl[y][x] |= wall;
    if ((wall == NORTH_WALL) && ((y + 1U) < s_height)) {
        s_walls_bl[y + 1U][x] |= SOUTH_WALL;
    } else if ((wall == EAST_WALL) && ((x + 1U) < s_width)) {
        s_walls_bl[y][x + 1U] |= WEST_WALL;
    } else if ((wall == SOUTH_WALL) && (y > 0U)) {
        s_walls_bl[y - 1U][x] |= NORTH_WALL;
    } else if ((wall == WEST_WALL) && (x > 0U)) {
        s_walls_bl[y][x - 1U] |= EAST_WALL;
    }
}

static void add_sample_boundary(void)
{
    for (unsigned int x = 0U; x < s_width; x++) {
        set_wall_pair(x, 0U, SOUTH_WALL);
        set_wall_pair(x, s_height - 1U, NORTH_WALL);
    }
    for (unsigned int y = 0U; y < s_height; y++) {
        set_wall_pair(0U, y, WEST_WALL);
        set_wall_pair(s_width - 1U, y, EAST_WALL);
    }
}

static void build_internal_sample(void)
{
    s_width = 16U;
    s_height = 16U;
    clear_sample_area(s_width, s_height);
    add_sample_boundary();
    set_wall_pair(START_X, START_Y, EAST_WALL);
}

static bool read_file_bytes(const char *path_name, char **out_buf)
{
    FILE *fp = fopen(path_name, "rb");
    long size;
    char *buf;

    if (fp == NULL) {
        fprintf(stderr, "failed to open %s\n", path_name);
        return false;
    }
    if (fseek(fp, 0L, SEEK_END) != 0) {
        fclose(fp);
        return false;
    }
    size = ftell(fp);
    if (size < 0L) {
        fclose(fp);
        return false;
    }
    if (fseek(fp, 0L, SEEK_SET) != 0) {
        fclose(fp);
        return false;
    }
    buf = (char *)malloc((size_t)size + 1U);
    if (buf == NULL) {
        fclose(fp);
        return false;
    }
    if (fread(buf, 1U, (size_t)size, fp) != (size_t)size) {
        free(buf);
        fclose(fp);
        return false;
    }
    buf[size] = '\0';
    fclose(fp);
    *out_buf = buf;
    return true;
}

static size_t parse_integer_values(char *buf, long *values, size_t cap)
{
    char *p = strchr(buf, '=');
    size_t count = 0U;

    if (p != NULL) {
        p = strchr(p, '{');
    }
    if (p == NULL) {
        p = strchr(buf, '{');
    }
    if (p == NULL) {
        p = buf;
    }

    while (*p != '\0' && count < cap) {
        char *endp;
        while (*p != '\0' && *p != '-' && !isdigit((unsigned char)*p)) {
            p++;
        }
        if (*p == '\0') {
            break;
        }
        errno = 0;
        long v = strtol(p, &endp, 0);
        if (endp == p) {
            p++;
            continue;
        }
        if (errno == 0) {
            values[count++] = v;
        }
        p = endp;
    }
    return count;
}

static bool load_c_array_file(const char *path_name, bool top_left_origin)
{
    enum { MAX_VALUES = MAZE_SIZE * MAZE_SIZE };
    long values[MAX_VALUES];
    char *buf = NULL;
    size_t count;
    unsigned int src_size;

    if (!read_file_bytes(path_name, &buf)) {
        return false;
    }
    count = parse_integer_values(buf, values, MAX_VALUES);
    free(buf);

    if (count >= (MAZE_SIZE * MAZE_SIZE)) {
        src_size = MAZE_SIZE;
    } else if (count >= 256U) {
        src_size = 16U;
    } else {
        fprintf(stderr, "maze array needs at least 256 values, got %zu\n", count);
        return false;
    }

    s_width = src_size;
    s_height = src_size;
    clear_sample_area(s_width, s_height);

    for (unsigned int row = 0U; row < src_size; row++) {
        for (unsigned int x = 0U; x < src_size; x++) {
            unsigned int y = top_left_origin ? (src_size - 1U - row) : row;
            long v = values[row * src_size + x];
            if (v < 0L) {
                v = 0L;
            }
            s_walls_bl[y][x] = (uint8_t)v & 0x0FU;
        }
    }
    add_sample_boundary();
    set_wall_pair(START_X, START_Y, EAST_WALL);
    return true;
}

void load_map_from_eeprom(void)
{
    memset(map, 0, sizeof(map));
    for (unsigned int y = 0U; y < MAZE_SIZE; y++) {
        for (unsigned int x = 0U; x < MAZE_SIZE; x++) {
            map[y][x] = (uint16_t)((uint16_t)(s_walls_bl[y][x] & 0x0FU) << 4U);
        }
    }
}

static const char *path_code_name(uint16_t code, char *buf, size_t len)
{
    if ((code > 200U) && (code < 300U)) {
        snprintf(buf, len, "S%u", (unsigned int)(code - 200U));
    } else if (code == 300U) {
        snprintf(buf, len, "S-R90");
    } else if (code == 400U) {
        snprintf(buf, len, "S-L90");
    } else if (code == 501U || code == 502U) {
        snprintf(buf, len, "L-R%u", (code == 501U) ? 90U : 180U);
    } else if (code == 601U || code == 602U) {
        snprintf(buf, len, "L-L%u", (code == 601U) ? 90U : 180U);
    } else if ((code >= 701U) && (code <= 704U)) {
        snprintf(buf, len, "D45-%u", (unsigned int)(code - 700U));
    } else if ((code >= 801U) && (code <= 802U)) {
        snprintf(buf, len, "V90-%u", (unsigned int)(code - 800U));
    } else if ((code >= 901U) && (code <= 904U)) {
        snprintf(buf, len, "D135-%u", (unsigned int)(code - 900U));
    } else if (code > 1000U) {
        snprintf(buf, len, "DS%u", (unsigned int)(code - 1000U));
    } else {
        snprintf(buf, len, "%u", (unsigned int)code);
    }
    return buf;
}

static void print_path_summary(void)
{
    unsigned int count = 0U;
    bool has_diagonal = false;

    printf("[host] path_codes=");
    for (unsigned int i = 0U; i < ROUTE_MAX_LEN && path[i] != 0U; i++) {
        char name[16];
        if (i != 0U) {
            printf(",");
        }
        printf("%u:%s", (unsigned int)path[i], path_code_name(path[i], name, sizeof(name)));
        if (path[i] > 700U) {
            has_diagonal = true;
        }
        count++;
    }
    printf("\n");
    printf("[host] path_count=%u diagonal_or_special=%s\n", count, has_diagonal ? "yes" : "no");
}

static void print_usage(const char *argv0)
{
    printf("usage: %s [--maze-c-array FILE] [--origin top-left|bottom-left] [--mode N] [--case N] [--verbose-solver]\n", argv0);
}

static bool run_solver_quiet(uint8_t mode, uint8_t case_index)
{
    int saved_stdout = dup(STDOUT_FILENO);
    int null_fd = open("/dev/null", O_WRONLY);
    bool ok;

    if (saved_stdout < 0 || null_fd < 0) {
        if (saved_stdout >= 0) {
            close(saved_stdout);
        }
        if (null_fd >= 0) {
            close(null_fd);
        }
        return solver_build_path(mode, case_index);
    }

    fflush(stdout);
    dup2(null_fd, STDOUT_FILENO);
    ok = solver_build_path(mode, case_index);
    fflush(stdout);
    dup2(saved_stdout, STDOUT_FILENO);
    close(null_fd);
    close(saved_stdout);
    return ok;
}

int main(int argc, char **argv)
{
    const char *maze_file = NULL;
    bool top_left_origin = true;
    bool verbose_solver = false;
    uint8_t mode = 2U;
    uint8_t case_index = 1U;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--maze-c-array") == 0 && (i + 1) < argc) {
            maze_file = argv[++i];
        } else if (strcmp(argv[i], "--origin") == 0 && (i + 1) < argc) {
            const char *origin = argv[++i];
            if (strcmp(origin, "top-left") == 0) {
                top_left_origin = true;
            } else if (strcmp(origin, "bottom-left") == 0) {
                top_left_origin = false;
            } else {
                print_usage(argv[0]);
                return 2;
            }
        } else if (strcmp(argv[i], "--mode") == 0 && (i + 1) < argc) {
            mode = (uint8_t)strtoul(argv[++i], NULL, 0);
        } else if (strcmp(argv[i], "--case") == 0 && (i + 1) < argc) {
            case_index = (uint8_t)strtoul(argv[++i], NULL, 0);
        } else if (strcmp(argv[i], "--verbose-solver") == 0) {
            verbose_solver = true;
        } else if (strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        } else {
            print_usage(argv[0]);
            return 2;
        }
    }

    if (maze_file != NULL) {
        if (!load_c_array_file(maze_file, top_left_origin)) {
            return 1;
        }
        printf("[host] loaded maze=%s size=%ux%u origin=%s\n", maze_file, s_width, s_height,
               top_left_origin ? "top-left" : "bottom-left");
    } else {
        build_internal_sample();
        printf("[host] loaded internal open 16x16 sample\n");
    }

    printf("[host] solver_build_path mode=%u case=%u start=(%u,%u) goal=(%u,%u) firmware_maze_size=%u\n",
           (unsigned int)mode, (unsigned int)case_index,
           (unsigned int)START_X, (unsigned int)START_Y,
           (unsigned int)GOAL_X, (unsigned int)GOAL_Y,
           (unsigned int)MAZE_SIZE);

    bool ok = verbose_solver ? solver_build_path(mode, case_index) : run_solver_quiet(mode, case_index);
    printf("[host] result=%s\n", ok ? "ok" : "failed");
    if (!ok) {
        return 1;
    }
    print_path_summary();
    return 0;
}
