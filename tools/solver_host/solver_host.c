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

typedef struct {
    uint8_t x;
    uint8_t y;
    uint8_t dir;
} SimMouse;

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

static bool load_search_dump_file(const char *path_name)
{
    char *buf = NULL;
    bool seen[MAZE_SIZE] = {false};
    unsigned int row_count = 0U;
    unsigned int detected_size = 0U;

    if (!read_file_bytes(path_name, &buf)) {
        return false;
    }

    s_width = MAZE_SIZE;
    s_height = MAZE_SIZE;
    clear_sample_area(s_width, s_height);

    char *p = buf;
    while (*p != '\0') {
        char *line = p;
        char *end = strpbrk(p, "\r\n");
        if (end != NULL) {
            *end = '\0';
            p = end + 1;
            if ((*p == '\n' || *p == '\r') && *p != *end) {
                p++;
            }
        } else {
            p += strlen(p);
        }

        char *tag = strstr(line, "[SEARCH-DUMP]");
        if (tag == NULL) {
            continue;
        }

        unsigned int header_size = 0U;
        if (sscanf(tag, "[SEARCH-DUMP] source=%*s size=%u", &header_size) == 1) {
            if (header_size == 0U || header_size > MAZE_SIZE) {
                fprintf(stderr, "invalid SEARCH-DUMP size: %u\n", header_size);
                free(buf);
                return false;
            }
            detected_size = header_size;
            s_width = header_size;
            s_height = header_size;
            continue;
        }

        unsigned int y = 0U;
        char *colon = strchr(tag, ':');
        if (colon == NULL || sscanf(tag, "[SEARCH-DUMP] y=%u:", &y) != 1) {
            continue;
        }
        if (y >= MAZE_SIZE) {
            fprintf(stderr, "SEARCH-DUMP row y=%u is outside MAZE_SIZE=%u\n",
                    y, (unsigned int)MAZE_SIZE);
            free(buf);
            return false;
        }
        if (detected_size != 0U && y >= detected_size) {
            fprintf(stderr, "SEARCH-DUMP row y=%u is outside dump size=%u\n", y, detected_size);
            free(buf);
            return false;
        }

        char *hex = colon + 1;
        while (isspace((unsigned char)*hex)) {
            hex++;
        }
        unsigned int expected = (detected_size != 0U) ? detected_size : MAZE_SIZE;
        for (unsigned int x = 0U; x < expected; x++) {
            if (!isxdigit((unsigned char)hex[x * 2U]) || !isxdigit((unsigned char)hex[x * 2U + 1U])) {
                fprintf(stderr, "SEARCH-DUMP row y=%u is too short at x=%u\n", y, x);
                free(buf);
                return false;
            }
            char byte_text[3] = {hex[x * 2U], hex[x * 2U + 1U], '\0'};
            unsigned long cell = strtoul(byte_text, NULL, 16);
            s_walls_bl[y][x] = (uint8_t)((cell >> 4U) & 0x0FU);
        }
        seen[y] = true;
        row_count++;
    }

    if (detected_size == 0U) {
        detected_size = MAZE_SIZE;
        s_width = MAZE_SIZE;
        s_height = MAZE_SIZE;
    }
    if (row_count != detected_size) {
        fprintf(stderr, "SEARCH-DUMP expected %u rows, got %u\n", detected_size, row_count);
        free(buf);
        return false;
    }
    for (unsigned int y = 0U; y < detected_size; y++) {
        if (!seen[y]) {
            fprintf(stderr, "SEARCH-DUMP missing row y=%u\n", y);
            free(buf);
            return false;
        }
    }

    free(buf);
    return true;
}

static bool line_has_horizontal_wall(const char *line, unsigned int x)
{
    size_t pos = (size_t)x * 4U + 1U;
    return (line[pos] == '-') || (line[pos + 1U] == '-') || (line[pos + 2U] == '-') ||
           (line[pos] == '.') || (line[pos + 1U] == '.') || (line[pos + 2U] == '.');
}

static bool line_has_vertical_wall(const char *line, unsigned int x)
{
    size_t pos = (size_t)x * 4U;
    return (line[pos] == '|') || (line[pos] == '.');
}

static bool load_maze_text_file(const char *path_name)
{
    char *buf = NULL;
    char *lines[2U * MAZE_SIZE + 1U];
    size_t line_count = 0U;
    unsigned int src_size;

    if (!read_file_bytes(path_name, &buf)) {
        return false;
    }

    char *p = buf;
    while (*p != '\0' && line_count < (sizeof(lines) / sizeof(lines[0]))) {
        char *line = p;
        char *end = strpbrk(p, "\r\n");
        if (end != NULL) {
            *end = '\0';
            p = end + 1;
            if ((*p == '\n' || *p == '\r') && *p != *end) {
                p++;
            }
        } else {
            p += strlen(p);
        }
        if (line[0] != '\0') {
            lines[line_count++] = line;
        }
    }

    if ((line_count < 3U) || ((line_count % 2U) == 0U)) {
        fprintf(stderr, "invalid maze text line count: %zu\n", line_count);
        free(buf);
        return false;
    }

    src_size = (unsigned int)((line_count - 1U) / 2U);
    if (src_size > MAZE_SIZE) {
        fprintf(stderr, "maze size %u exceeds firmware MAZE_SIZE %u\n",
                src_size, (unsigned int)MAZE_SIZE);
        free(buf);
        return false;
    }

    s_width = src_size;
    s_height = src_size;
    clear_sample_area(s_width, s_height);

    for (unsigned int y = 0U; y < src_size; y++) {
        unsigned int row_from_top = src_size - 1U - y;
        const char *north_line = lines[row_from_top * 2U];
        const char *cell_line = lines[row_from_top * 2U + 1U];
        const char *south_line = lines[row_from_top * 2U + 2U];

        for (unsigned int x = 0U; x < src_size; x++) {
            if (line_has_horizontal_wall(north_line, x)) {
                set_wall_pair(x, y, NORTH_WALL);
            }
            if (line_has_horizontal_wall(south_line, x)) {
                set_wall_pair(x, y, SOUTH_WALL);
            }
            if (line_has_vertical_wall(cell_line, x)) {
                set_wall_pair(x, y, WEST_WALL);
            }
            if (line_has_vertical_wall(cell_line, x + 1U)) {
                set_wall_pair(x, y, EAST_WALL);
            }
        }
    }

    add_sample_boundary();
    set_wall_pair(START_X, START_Y, EAST_WALL);
    free(buf);
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

static uint8_t abs_wall_from_dir(uint8_t dir)
{
    static const uint8_t wall_by_dir[4] = {NORTH_WALL, EAST_WALL, SOUTH_WALL, WEST_WALL};
    return wall_by_dir[dir & 0x03U];
}

static bool sim_is_goal(uint8_t x, uint8_t y)
{
    const uint8_t goals[9][2] = {
        {GOAL1_X, GOAL1_Y}, {GOAL2_X, GOAL2_Y}, {GOAL3_X, GOAL3_Y},
        {GOAL4_X, GOAL4_Y}, {GOAL5_X, GOAL5_Y}, {GOAL6_X, GOAL6_Y},
        {GOAL7_X, GOAL7_Y}, {GOAL8_X, GOAL8_Y}, {GOAL9_X, GOAL9_Y},
    };

    for (unsigned int i = 0U; i < 9U; i++) {
        if (goals[i][0] == 0U && goals[i][1] == 0U) {
            continue;
        }
        if (x == goals[i][0] && y == goals[i][1]) {
            return true;
        }
    }
    return false;
}

static void sim_init_search_map(void)
{
    for (uint8_t y = 0U; y < MAZE_SIZE; y++) {
        for (uint8_t x = 0U; x < MAZE_SIZE; x++) {
            map[y][x] = 0xF0U;
            visited[y][x] = false;
        }
    }
    for (uint8_t y = 0U; y < MAZE_SIZE; y++) {
        map[y][0] |= 0xF1U;
        map[y][MAZE_SIZE - 1U] |= 0xF4U;
    }
    for (uint8_t x = 0U; x < MAZE_SIZE; x++) {
        map[0][x] |= 0xF2U;
        map[MAZE_SIZE - 1U][x] |= 0xF8U;
    }
    map[START_Y][START_X] |= 0x44U;
    if (START_X > 0U) {
        map[START_Y][START_X - 1U] |= 0x44U;
    }
    visited[START_Y][START_X] = true;
}

static uint16_t sim_relative_wall_info(uint8_t x, uint8_t y, uint8_t dir)
{
    uint8_t true_walls = s_walls_bl[y][x];
    uint16_t info = 0U;

    if ((true_walls & abs_wall_from_dir(dir)) != 0U) {
        info |= 0x88U;
    }
    if ((true_walls & abs_wall_from_dir((uint8_t)(dir + 1U))) != 0U) {
        info |= 0x44U;
    }
    if ((true_walls & abs_wall_from_dir((uint8_t)(dir + 3U))) != 0U) {
        info |= 0x11U;
    }
    return info;
}

static void sim_write_map_cell(uint8_t x, uint8_t y, uint8_t dir, uint16_t relative_wall_info)
{
    uint16_t m_temp;

    if ((x >= MAZE_SIZE) || (y >= MAZE_SIZE)) {
        return;
    }

    m_temp = (relative_wall_info >> (dir & 0x03U)) & 0x0FU;
    if ((x == START_X) && (y == START_Y)) {
        m_temp |= 0x07U;
    }
    m_temp |= (uint16_t)(m_temp << 4U);
    map[y][x] = m_temp;

    if (y != (MAZE_SIZE - 1U)) {
        if ((m_temp & 0x88U) != 0U) {
            map[y + 1U][x] |= 0x22U;
        } else {
            map[y + 1U][x] &= (uint16_t)~0x22U;
        }
    }
    if (x != (MAZE_SIZE - 1U)) {
        if ((m_temp & 0x44U) != 0U) {
            map[y][x + 1U] |= 0x11U;
        } else {
            map[y][x + 1U] &= (uint16_t)~0x11U;
        }
    }
    if (y != 0U) {
        if ((m_temp & 0x22U) != 0U) {
            map[y - 1U][x] |= 0x88U;
        } else {
            map[y - 1U][x] &= (uint16_t)~0x88U;
        }
    }
    if (x != 0U) {
        if ((m_temp & 0x11U) != 0U) {
            map[y][x - 1U] |= 0x44U;
        } else {
            map[y][x - 1U] &= (uint16_t)~0x44U;
        }
    }
    map[START_Y][START_X] |= 0x77U;
    if ((START_X + 1U) < MAZE_SIZE) {
        map[START_Y][START_X + 1U] |= 0x11U;
    }
}

static int sim_make_goal_smap(const SimMouse *m)
{
    uint16_t q[MAZE_SIZE * MAZE_SIZE];
    uint16_t head = 0U;
    uint16_t tail = 0U;

    for (uint8_t y = 0U; y < MAZE_SIZE; y++) {
        for (uint8_t x = 0U; x < MAZE_SIZE; x++) {
            smap[y][x] = 0xFFFFU;
        }
    }

    const uint8_t goals[9][2] = {
        {GOAL1_X, GOAL1_Y}, {GOAL2_X, GOAL2_Y}, {GOAL3_X, GOAL3_Y},
        {GOAL4_X, GOAL4_Y}, {GOAL5_X, GOAL5_Y}, {GOAL6_X, GOAL6_Y},
        {GOAL7_X, GOAL7_Y}, {GOAL8_X, GOAL8_Y}, {GOAL9_X, GOAL9_Y},
    };
    for (unsigned int i = 0U; i < 9U; i++) {
        uint8_t gx = goals[i][0];
        uint8_t gy = goals[i][1];
        if ((gx == 0U && gy == 0U) || gx >= MAZE_SIZE || gy >= MAZE_SIZE) {
            continue;
        }
        smap[gy][gx] = 0U;
        q[tail++] = (uint16_t)(gy * MAZE_SIZE + gx);
    }

    while ((head < tail) && (smap[m->y][m->x] == 0xFFFFU)) {
        uint16_t idx = q[head++];
        uint8_t cy = (uint8_t)(idx / MAZE_SIZE);
        uint8_t cx = (uint8_t)(idx - (uint16_t)(cy * MAZE_SIZE));
        uint16_t step = smap[cy][cx];
        uint8_t cell = (uint8_t)(map[cy][cx] & 0x0FU);

        if (((cell & NORTH_WALL) == 0U) && cy != (MAZE_SIZE - 1U) && smap[cy + 1U][cx] == 0xFFFFU) {
            smap[cy + 1U][cx] = (uint16_t)(step + 1U);
            q[tail++] = (uint16_t)((cy + 1U) * MAZE_SIZE + cx);
        }
        if (((cell & EAST_WALL) == 0U) && cx != (MAZE_SIZE - 1U) && smap[cy][cx + 1U] == 0xFFFFU) {
            smap[cy][cx + 1U] = (uint16_t)(step + 1U);
            q[tail++] = (uint16_t)(cy * MAZE_SIZE + (cx + 1U));
        }
        if (((cell & SOUTH_WALL) == 0U) && cy != 0U && smap[cy - 1U][cx] == 0xFFFFU) {
            smap[cy - 1U][cx] = (uint16_t)(step + 1U);
            q[tail++] = (uint16_t)((cy - 1U) * MAZE_SIZE + cx);
        }
        if (((cell & WEST_WALL) == 0U) && cx != 0U && smap[cy][cx - 1U] == 0xFFFFU) {
            smap[cy][cx - 1U] = (uint16_t)(step + 1U);
            q[tail++] = (uint16_t)(cy * MAZE_SIZE + (cx - 1U));
        }
    }

    return (smap[m->y][m->x] == 0xFFFFU) ? -1 : (int)smap[m->y][m->x];
}

static bool sim_next_move(const SimMouse *m, uint8_t *out_rel)
{
    static const uint8_t rel_priority[4] = {0U, 1U, 3U, 2U};
    static const int8_t dx[4] = {0, 1, 0, -1};
    static const int8_t dy[4] = {1, 0, -1, 0};
    uint16_t current_step = smap[m->y][m->x];

    for (unsigned int i = 0U; i < 4U; i++) {
        uint8_t rel = rel_priority[i];
        uint8_t abs_dir = (uint8_t)((m->dir + rel) & 0x03U);
        int nx = (int)m->x + dx[abs_dir];
        int ny = (int)m->y + dy[abs_dir];
        uint8_t cell = (uint8_t)(map[m->y][m->x] & 0x0FU);

        if ((cell & abs_wall_from_dir(abs_dir)) != 0U) {
            continue;
        }
        if (nx < 0 || ny < 0 || nx >= (int)MAZE_SIZE || ny >= (int)MAZE_SIZE) {
            continue;
        }
        if (smap[ny][nx] < current_step) {
            *out_rel = rel;
            return true;
        }
    }
    return false;
}

static bool sim_apply_move(SimMouse *m, uint8_t rel)
{
    static const int8_t dx[4] = {0, 1, 0, -1};
    static const int8_t dy[4] = {1, 0, -1, 0};
    uint8_t abs_dir = (uint8_t)((m->dir + rel) & 0x03U);
    int nx;
    int ny;

    if ((s_walls_bl[m->y][m->x] & abs_wall_from_dir(abs_dir)) != 0U) {
        return false;
    }

    nx = (int)m->x + dx[abs_dir];
    ny = (int)m->y + dy[abs_dir];
    if (nx < 0 || ny < 0 || nx >= (int)s_width || ny >= (int)s_height) {
        return false;
    }

    m->dir = abs_dir;
    m->x = (uint8_t)nx;
    m->y = (uint8_t)ny;
    return true;
}

static bool run_explore_sim(unsigned int max_steps, bool verbose)
{
    SimMouse m = {START_X, START_Y, 0U};
    unsigned int newly_visited = 0U;

    sim_init_search_map();

    for (unsigned int step = 0U; step <= max_steps; step++) {
        uint16_t rel_walls;
        int smap_step;
        uint8_t rel;

        if (m.x >= s_width || m.y >= s_height) {
            printf("[explore] result=failed reason=start_or_move_out_of_loaded_maze\n");
            return false;
        }

        rel_walls = sim_relative_wall_info(m.x, m.y, m.dir);
        sim_write_map_cell(m.x, m.y, m.dir, rel_walls);
        if (!visited[m.y][m.x]) {
            newly_visited++;
        }
        visited[m.y][m.x] = true;

        if (verbose) {
            printf("[explore] step=%u pos=(%u,%u,%u) rel_wall=0x%02X map=0x%04X\n",
                   step, (unsigned int)m.x, (unsigned int)m.y,
                   (unsigned int)m.dir, (unsigned int)rel_walls,
                   (unsigned int)map[m.y][m.x]);
        }

        if (sim_is_goal(m.x, m.y)) {
            printf("[explore] result=ok steps=%u pos=(%u,%u,%u) newly_visited=%u\n",
                   step, (unsigned int)m.x, (unsigned int)m.y,
                   (unsigned int)m.dir, newly_visited);
            return true;
        }

        smap_step = sim_make_goal_smap(&m);
        if (smap_step < 0 || !sim_next_move(&m, &rel)) {
            printf("[explore] result=failed reason=no_route pos=(%u,%u,%u)\n",
                   (unsigned int)m.x, (unsigned int)m.y, (unsigned int)m.dir);
            return false;
        }
        if (!sim_apply_move(&m, rel)) {
            printf("[explore] result=failed reason=hit_virtual_wall step=%u rel=%u pos=(%u,%u,%u)\n",
                   step, (unsigned int)rel, (unsigned int)m.x,
                   (unsigned int)m.y, (unsigned int)m.dir);
            return false;
        }
    }

    printf("[explore] result=failed reason=max_steps steps=%u pos=(%u,%u,%u)\n",
           max_steps, (unsigned int)m.x, (unsigned int)m.y, (unsigned int)m.dir);
    return false;
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
    printf("usage: %s [--maze FILE.maze] [--maze-c-array FILE] [--search-dump FILE] [--origin top-left|bottom-left] [--mode N] [--case N] [--verbose-solver] [--explore-sim] [--explore-verbose] [--max-steps N]\n", argv0);
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
    const char *maze_text_file = NULL;
    const char *search_dump_file = NULL;
    bool top_left_origin = true;
    bool verbose_solver = false;
    bool explore_sim = false;
    bool explore_verbose = false;
    unsigned int max_steps = 2048U;
    uint8_t mode = 2U;
    uint8_t case_index = 1U;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--maze-c-array") == 0 && (i + 1) < argc) {
            maze_file = argv[++i];
        } else if (strcmp(argv[i], "--maze") == 0 && (i + 1) < argc) {
            maze_text_file = argv[++i];
        } else if (strcmp(argv[i], "--search-dump") == 0 && (i + 1) < argc) {
            search_dump_file = argv[++i];
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
        } else if (strcmp(argv[i], "--explore-sim") == 0) {
            explore_sim = true;
        } else if (strcmp(argv[i], "--explore-verbose") == 0) {
            explore_verbose = true;
        } else if (strcmp(argv[i], "--max-steps") == 0 && (i + 1) < argc) {
            max_steps = (unsigned int)strtoul(argv[++i], NULL, 0);
        } else if (strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        } else {
            print_usage(argv[0]);
            return 2;
        }
    }

    if (search_dump_file != NULL) {
        if (maze_text_file != NULL || maze_file != NULL) {
            print_usage(argv[0]);
            return 2;
        }
        if (!load_search_dump_file(search_dump_file)) {
            return 1;
        }
        printf("[host] loaded search_dump=%s size=%ux%u format=f413-uart\n",
               search_dump_file, s_width, s_height);
    } else if (maze_text_file != NULL) {
        if (!load_maze_text_file(maze_text_file)) {
            return 1;
        }
        printf("[host] loaded maze=%s size=%ux%u format=text\n", maze_text_file, s_width, s_height);
    } else if (maze_file != NULL) {
        if (!load_c_array_file(maze_file, top_left_origin)) {
            return 1;
        }
        printf("[host] loaded maze=%s size=%ux%u origin=%s\n", maze_file, s_width, s_height,
               top_left_origin ? "top-left" : "bottom-left");
    } else {
        build_internal_sample();
        printf("[host] loaded internal open 16x16 sample\n");
    }

    if (explore_sim) {
        printf("[host] explore_sim start=(%u,%u) goal=(%u,%u) max_steps=%u size=%ux%u\n",
               (unsigned int)START_X, (unsigned int)START_Y,
               (unsigned int)GOAL_X, (unsigned int)GOAL_Y,
               max_steps, s_width, s_height);
        return run_explore_sim(max_steps, explore_verbose) ? 0 : 1;
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
