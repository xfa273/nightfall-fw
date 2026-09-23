/* Integer-only MCU oracle for the same mode2/case8 KERI motion graph as the
 * generic slalom planner. Topology helpers below are adapted from the compact
 * F413 preview at 4fb45ed; dimensions are dynamic and wall bits are NESW1<<d.
 * Immutable microsecond and geometric-crossing tables replace floating point.
 * A binary heap stores one minimum representative for each eight-state group;
 * predecessor edges are recovered from exact distances after search. */
#include "mcu_slalom_time_planner.h"
#include "mcu_slalom_tables.h"
#include <limits.h>
#include <string.h>

#define NONE UINT16_MAX
#define INF UINT32_MAX
#define F413_RP_WIDTH (maze->width)
#define F413_RP_HEIGHT (maze->height)
#define F413_RP_CELL_COUNT ((uint16_t)(maze->width * maze->height))
#define F413_RP_CENTER_POSES (F413_RP_CELL_COUNT * 4U)
#define F413_RP_INTERNAL_WALLS                                                                     \
    (((maze->width - 1U) * maze->height) + ((maze->height - 1U) * maze->width))
#define F413_RP_POSE_COUNT ((F413_RP_CELL_COUNT + F413_RP_INTERNAL_WALLS) * 4U)
#define F413_RP_SPEED_COUNT 3U
#define F413_RP_STATE_COUNT (F413_RP_POSE_COUNT * 3U)

typedef enum {
    F413_RP_HEADING_NORTH = 0,
    F413_RP_HEADING_NORTH_EAST,
    F413_RP_HEADING_EAST,
    F413_RP_HEADING_SOUTH_EAST,
    F413_RP_HEADING_SOUTH,
    F413_RP_HEADING_SOUTH_WEST,
    F413_RP_HEADING_WEST,
    F413_RP_HEADING_NORTH_WEST
} f413_rp_heading_t;
typedef enum { F413_RP_SPEED_NOMINAL = 0, F413_RP_SPEED_LOW, F413_RP_SPEED_CRAWL } f413_rp_speed_t;
typedef struct {
    int16_t half_x, half_y;
} f413_rp_anchor_t;
typedef struct {
    uint8_t width, height;
    uint8_t walls[1024];
    uint8_t goal_bits[128];
    uint8_t *required;
} f413_rp_maze_t;
static const int8_t g_f413_rp_dx[8] = {0, 1, 1, 1, 0, -1, -1, -1};
static const int8_t g_f413_rp_dy[8] = {1, 1, 0, -1, -1, -1, 0, 1};
static const uint8_t g_f413_rp_wall_mask[4] = {1, 2, 4, 8};
static bool f413_rp_is_cardinal(f413_rp_heading_t heading) {
    return (((unsigned int)heading & 1U) == 0U);
}

static f413_rp_heading_t f413_rp_heading_add(f413_rp_heading_t heading, int delta) {
    int result = ((int)heading + delta) % 8;
    if (result < 0) {
        result += 8;
    }
    return (f413_rp_heading_t)result;
}

static bool f413_rp_anchor_is_center(f413_rp_anchor_t anchor) {
    return ((anchor.half_x & 1) != 0) && ((anchor.half_y & 1) != 0);
}

static bool f413_rp_cell_in_bounds(const f413_rp_maze_t *maze, int x, int y) {
    return (x >= 0) && (y >= 0) && (x < (int)F413_RP_WIDTH) && (y < (int)F413_RP_HEIGHT);
}

static size_t f413_rp_cell_index(const f413_rp_maze_t *maze, int x, int y) {
    return ((size_t)y * F413_RP_WIDTH) + (size_t)x;
}

static bool f413_rp_goal_at(const f413_rp_maze_t *maze, int x, int y) {
    size_t index;
    if ((maze == NULL) || !f413_rp_cell_in_bounds(maze, x, y)) {
        return false;
    }
    index = f413_rp_cell_index(maze, x, y);
    return (maze->goal_bits[index >> 3U] & (uint8_t)(1U << (index & 7U))) != 0U;
}

static void f413_rp_goal_set(f413_rp_maze_t *maze, uint8_t x, uint8_t y) {
    const size_t index = f413_rp_cell_index(maze, x, y);
    maze->goal_bits[index >> 3U] |= (uint8_t)(1U << (index & 7U));
}

static bool f413_rp_cells_open(const f413_rp_maze_t *maze, int from_x, int from_y, int to_x,
                               int to_y) {
    unsigned int direction;
    unsigned int opposite;

    if ((maze == NULL) || !f413_rp_cell_in_bounds(maze, from_x, from_y) ||
        !f413_rp_cell_in_bounds(maze, to_x, to_y)) {
        return false;
    }
    if ((to_x == from_x) && (to_y == from_y + 1)) {
        direction = 0U;
    } else if ((to_x == from_x + 1) && (to_y == from_y)) {
        direction = 1U;
    } else if ((to_x == from_x) && (to_y == from_y - 1)) {
        direction = 2U;
    } else if ((to_x == from_x - 1) && (to_y == from_y)) {
        direction = 3U;
    } else {
        return false;
    }
    opposite = (direction + 2U) & 3U;
    bool open =
        (maze->walls[f413_rp_cell_index(maze, from_x, from_y)] & g_f413_rp_wall_mask[direction]) ==
            0U &&
        (maze->walls[f413_rp_cell_index(maze, to_x, to_y)] & g_f413_rp_wall_mask[opposite]) == 0U;
    if (open && maze->required != NULL) {
        maze->required[f413_rp_cell_index(maze, from_x, from_y)] |= g_f413_rp_wall_mask[direction];
        maze->required[f413_rp_cell_index(maze, to_x, to_y)] |= g_f413_rp_wall_mask[opposite];
    }
    return open;
}

static bool f413_rp_anchor_id(const f413_rp_maze_t *maze, f413_rp_anchor_t anchor,
                              uint16_t *out_id) {
    const int hx = anchor.half_x;
    const int hy = anchor.half_y;
    const uint16_t centers = F413_RP_CELL_COUNT;
    const uint16_t vertical = (F413_RP_WIDTH - 1U) * F413_RP_HEIGHT;

    if ((out_id == NULL) || (hx <= 0) || (hy <= 0) || (hx >= (int)(2U * F413_RP_WIDTH)) ||
        (hy >= (int)(2U * F413_RP_HEIGHT)) || (((hx & 1) == 0) && ((hy & 1) == 0))) {
        return false;
    }
    if (((hx & 1) != 0) && ((hy & 1) != 0)) {
        const uint16_t x = (uint16_t)(hx - 1) / 2U;
        const uint16_t y = (uint16_t)(hy - 1) / 2U;
        *out_id = (uint16_t)((y * F413_RP_WIDTH) + x);
        return true;
    }
    if ((hx & 1) == 0) {
        const uint16_t x_line = (uint16_t)hx / 2U;
        const uint16_t y = (uint16_t)(hy - 1) / 2U;
        if ((x_line == 0U) || (x_line >= F413_RP_WIDTH)) {
            return false;
        }
        *out_id = (uint16_t)(centers + (y * (F413_RP_WIDTH - 1U)) + (x_line - 1U));
        return true;
    }
    {
        const uint16_t x = (uint16_t)(hx - 1) / 2U;
        const uint16_t y_line = (uint16_t)hy / 2U;
        if ((y_line == 0U) || (y_line >= F413_RP_HEIGHT)) {
            return false;
        }
        *out_id = (uint16_t)(centers + vertical + ((y_line - 1U) * F413_RP_WIDTH) + x);
    }
    return true;
}

static bool f413_rp_anchor_from_id(const f413_rp_maze_t *maze, uint16_t id, f413_rp_anchor_t *out) {
    const uint16_t centers = F413_RP_CELL_COUNT;
    const uint16_t vertical = (F413_RP_WIDTH - 1U) * F413_RP_HEIGHT;

    if ((out == NULL) || (id >= (F413_RP_CELL_COUNT + F413_RP_INTERNAL_WALLS))) {
        return false;
    }
    if (id < centers) {
        const uint16_t x = id % F413_RP_WIDTH;
        const uint16_t y = id / F413_RP_WIDTH;
        out->half_x = (int16_t)((2U * x) + 1U);
        out->half_y = (int16_t)((2U * y) + 1U);
        return true;
    }
    id = (uint16_t)(id - centers);
    if (id < vertical) {
        const uint16_t x_line = (id % (F413_RP_WIDTH - 1U)) + 1U;
        const uint16_t y = id / (F413_RP_WIDTH - 1U);
        out->half_x = (int16_t)(2U * x_line);
        out->half_y = (int16_t)((2U * y) + 1U);
        return true;
    }
    id = (uint16_t)(id - vertical);
    out->half_x = (int16_t)((2U * (id % F413_RP_WIDTH)) + 1U);
    out->half_y = (int16_t)(2U * ((id / F413_RP_WIDTH) + 1U));
    return true;
}

static bool f413_rp_anchor_open(const f413_rp_maze_t *maze, f413_rp_anchor_t anchor) {
    if (f413_rp_anchor_is_center(anchor)) {
        return true;
    }
    if ((anchor.half_x & 1) == 0) {
        const int right_x = anchor.half_x / 2;
        const int y = (anchor.half_y - 1) / 2;
        return f413_rp_cells_open(maze, right_x - 1, y, right_x, y);
    }
    {
        const int x = (anchor.half_x - 1) / 2;
        const int upper_y = anchor.half_y / 2;
        return f413_rp_cells_open(maze, x, upper_y - 1, x, upper_y);
    }
}

static bool f413_rp_travel_pose_valid(const f413_rp_maze_t *maze, f413_rp_anchor_t anchor,
                                      f413_rp_heading_t heading) {
    uint16_t ignored;
    if (((unsigned int)heading >= 8U) || !f413_rp_anchor_id(maze, anchor, &ignored)) {
        return false;
    }
    if (f413_rp_anchor_is_center(anchor)) {
        return f413_rp_is_cardinal(heading);
    }
    if (!f413_rp_anchor_open(maze, anchor)) {
        return false;
    }
    if (!f413_rp_is_cardinal(heading)) {
        return true;
    }
    if ((anchor.half_x & 1) == 0) {
        return (heading == F413_RP_HEADING_EAST) || (heading == F413_RP_HEADING_WEST);
    }
    return (heading == F413_RP_HEADING_NORTH) || (heading == F413_RP_HEADING_SOUTH);
}

static bool f413_rp_state_pose_valid(const f413_rp_maze_t *maze, f413_rp_anchor_t anchor,
                                     f413_rp_heading_t heading) {
    if (!f413_rp_travel_pose_valid(maze, anchor, heading)) {
        return false;
    }
    return f413_rp_anchor_is_center(anchor) ? f413_rp_is_cardinal(heading)
                                            : !f413_rp_is_cardinal(heading);
}

static bool f413_rp_anchor_region(const f413_rp_maze_t *maze, f413_rp_anchor_t anchor,
                                  f413_rp_heading_t heading, int *out_x, int *out_y) {
    int x;
    int y;
    if ((out_x == NULL) || (out_y == NULL) || !f413_rp_travel_pose_valid(maze, anchor, heading)) {
        return false;
    }
    if (f413_rp_anchor_is_center(anchor)) {
        x = (anchor.half_x - 1) / 2;
        y = (anchor.half_y - 1) / 2;
    } else if ((anchor.half_x & 1) == 0) {
        const int line = anchor.half_x / 2;
        x = (g_f413_rp_dx[(unsigned int)heading] > 0) ? line : line - 1;
        y = (anchor.half_y - 1) / 2;
    } else {
        const int line = anchor.half_y / 2;
        x = (anchor.half_x - 1) / 2;
        y = (g_f413_rp_dy[(unsigned int)heading] > 0) ? line : line - 1;
    }
    if (!f413_rp_cell_in_bounds(maze, x, y)) {
        return false;
    }
    *out_x = x;
    *out_y = y;
    return true;
}

static bool f413_rp_advance_connector(const f413_rp_maze_t *maze, f413_rp_anchor_t current,
                                      f413_rp_heading_t heading, f413_rp_anchor_t *out) {
    f413_rp_anchor_t next;
    uint16_t ignored;

    if ((out == NULL) || !f413_rp_travel_pose_valid(maze, current, heading)) {
        return false;
    }
    next.half_x = (int16_t)(current.half_x + g_f413_rp_dx[(unsigned int)heading]);
    next.half_y = (int16_t)(current.half_y + g_f413_rp_dy[(unsigned int)heading]);
    if (!f413_rp_anchor_id(maze, next, &ignored) ||
        !f413_rp_travel_pose_valid(maze, next, heading)) {
        return false;
    }
    *out = next;
    return true;
}

static bool f413_rp_pose_index(const f413_rp_maze_t *maze, f413_rp_anchor_t anchor,
                               f413_rp_heading_t heading, uint16_t *out_pose) {
    uint16_t anchor_id;
    if ((out_pose == NULL) || !f413_rp_anchor_id(maze, anchor, &anchor_id)) {
        return false;
    }
    if (anchor_id < F413_RP_CELL_COUNT) {
        if (!f413_rp_is_cardinal(heading)) {
            return false;
        }
        *out_pose = (uint16_t)((anchor_id * 4U) + ((unsigned int)heading / 2U));
        return true;
    }
    if (f413_rp_is_cardinal(heading)) {
        return false;
    }
    *out_pose = (uint16_t)(F413_RP_CENTER_POSES + ((anchor_id - F413_RP_CELL_COUNT) * 4U) +
                           (((unsigned int)heading - 1U) / 2U));
    return true;
}

static bool f413_rp_pose_decode(const f413_rp_maze_t *maze, uint16_t pose,
                                f413_rp_anchor_t *out_anchor, f413_rp_heading_t *out_heading) {
    uint16_t anchor_id;
    if ((out_anchor == NULL) || (out_heading == NULL) || (pose >= F413_RP_POSE_COUNT)) {
        return false;
    }
    if (pose < F413_RP_CENTER_POSES) {
        anchor_id = pose / 4U;
        *out_heading = (f413_rp_heading_t)((pose % 4U) * 2U);
    } else {
        const uint16_t wall_pose = (uint16_t)(pose - F413_RP_CENTER_POSES);
        anchor_id = (uint16_t)(F413_RP_CELL_COUNT + (wall_pose / 4U));
        *out_heading = (f413_rp_heading_t)(((wall_pose % 4U) * 2U) + 1U);
    }
    return f413_rp_anchor_from_id(maze, anchor_id, out_anchor);
}

static uint16_t f413_rp_state_index(const f413_rp_maze_t *maze, f413_rp_anchor_t anchor,
                                    f413_rp_heading_t heading, f413_rp_speed_t speed) {
    uint16_t pose = 0U;
    (void)f413_rp_pose_index(maze, anchor, heading, &pose);
    return (uint16_t)((pose * F413_RP_SPEED_COUNT) + (uint16_t)speed);
}

static bool f413_rp_state_decode(const f413_rp_maze_t *maze, uint16_t state,
                                 f413_rp_anchor_t *out_anchor, f413_rp_heading_t *out_heading,
                                 f413_rp_speed_t *out_speed) {
    if ((state >= F413_RP_STATE_COUNT) || (out_speed == NULL)) {
        return false;
    }
    *out_speed = (f413_rp_speed_t)(state % F413_RP_SPEED_COUNT);
    return f413_rp_pose_decode(maze, (uint16_t)(state / F413_RP_SPEED_COUNT), out_anchor,
                               out_heading);
}

/* Each heap entry represents eight contiguous states. Decrease-key fixes at
 * most ceil(log2(4512)) levels; popping rescans at most eight states. This
 * preserves the full Dijkstra graph without per-state parent/heap arrays. */
#define GROUP_SHIFT 3U
#define GROUP_SIZE (1U << GROUP_SHIFT)
typedef enum {
    PH_INIT_DIST,
    PH_INIT_GROUP,
    PH_INIT_CACHE,
    PH_INIT_SETTLED,
    PH_POP,
    PH_SCAN,
    PH_DIRECT,
    PH_EDGE,
    PH_CACHE,
    PH_BRAKE,
    PH_RECON_INIT,
    PH_RECON_TEMPLATE,
    PH_RECON_EDGE,
    PH_VERIFY,
    PH_MARK,
    PH_DONE
} Phase;
typedef struct {
    bool valid, direct;
    uint16_t source, connector_steps, stop_steps;
    uint8_t kind, side, speed, goal_x, goal_y;
    uint32_t entry_us, stop_us;
} Goal;
typedef struct {
    uint16_t stop_steps, goal_step;
    bool has_goal;
    uint8_t goal_x, goal_y;
} Ray;
struct NfMcuSlalom {
    f413_rp_maze_t maze;
    uint8_t required[1024];
    uint32_t *dist;
    uint16_t *heap, *position, *best, *turn_mask;
    uint16_t poses, cache_pose, cache_mask, connector_mask;
    uint8_t cache_index;
    uint8_t *settled;
    uint16_t states, groups, start_state, current, heap_count, heap_peak;
    uint32_t last_key;
    Phase phase;
    NfMcuSlalomStatus status;
    Goal goal, pending_goal;
    uint16_t init_cursor, scan_step, brake_step;
    f413_rp_anchor_t scan_cursor, brake_cursor;
    f413_rp_heading_t brake_heading;
    uint32_t brake_best_us, pending_edge_end_us;
    uint16_t verify_source, verify_step;
    f413_rp_anchor_t verify_cursor;
    uint16_t mark_source, mark_step, mark_steps, mark_stop_steps;
    f413_rp_anchor_t mark_cursor;
    f413_rp_heading_t mark_heading;
    uint8_t mark_kind, mark_side, mark_stage;
    bool mark_direct;
    uint32_t expanded, relaxed, work;
    uint16_t actions;
    size_t workspace_used;
    bool goal_cross_reachable;
    f413_rp_anchor_t anchor, connector;
    f413_rp_heading_t heading;
    f413_rp_speed_t speed;
    uint16_t connector_steps, direct_step;
    Ray ray;
    uint8_t edge_index;
    uint16_t reconstruct_state, reconstruct_template, reconstruct_step;
    uint8_t reconstruct_speed;
    f413_rp_anchor_t reconstruct_end, reconstruct_source, reconstruct_turn;
    f413_rp_heading_t reconstruct_heading;
    f413_rp_speed_t reconstruct_exit_speed;
    const NfMcuTurnTemplate *reconstruct_shape;
    uint8_t reconstruct_kind, reconstruct_side;
};

static bool add_us(uint32_t a, uint32_t b, uint32_t *out) {
    if (a == INF || b == INF || UINT32_MAX - a <= b)
        return false;
    *out = a + b;
    return true;
}
static bool settled(const NfMcuSlalom *c, uint16_t state) {
    return (c->settled[state >> 3] & (1U << (state & 7U))) != 0;
}
static bool state_less(const NfMcuSlalom *c, uint16_t a, uint16_t b) {
    return c->dist[a] < c->dist[b] || (c->dist[a] == c->dist[b] && a < b);
}
static bool group_less(const NfMcuSlalom *c, uint16_t a, uint16_t b) {
    return state_less(c, c->best[a], c->best[b]);
}
static void heap_swap(NfMcuSlalom *c, uint16_t a, uint16_t b) {
    uint16_t ga = c->heap[a], gb = c->heap[b];
    c->heap[a] = gb;
    c->heap[b] = ga;
    c->position[ga] = b;
    c->position[gb] = a;
}
static void heap_up(NfMcuSlalom *c, uint16_t position) {
    while (position) {
        uint16_t parent = (uint16_t)((position - 1U) / 2U);
        if (!group_less(c, c->heap[position], c->heap[parent]))
            break;
        heap_swap(c, position, parent);
        position = parent;
    }
}
static void heap_down(NfMcuSlalom *c, uint16_t position) {
    for (;;) {
        unsigned left = 2U * position + 1U;
        if (left >= c->heap_count)
            break;
        unsigned right = left + 1U, smallest = left;
        if (right < c->heap_count && group_less(c, c->heap[right], c->heap[left]))
            smallest = right;
        if (!group_less(c, c->heap[smallest], c->heap[position]))
            break;
        heap_swap(c, position, (uint16_t)smallest);
        position = (uint16_t)smallest;
    }
}
static void queue_insert(NfMcuSlalom *c, uint16_t state) {
    uint16_t group = state >> GROUP_SHIFT, position = c->position[group];
    if (position == NONE) {
        c->best[group] = state;
        c->heap[c->heap_count] = group;
        c->position[group] = c->heap_count;
        position = c->heap_count++;
        if (c->heap_count > c->heap_peak)
            c->heap_peak = c->heap_count;
    } else if (!state_less(c, state, c->best[group]) && state != c->best[group])
        return;
    else
        c->best[group] = state;
    heap_up(c, position);
}
static void queue_remove_min(NfMcuSlalom *c) {
    uint16_t group = c->heap[0], best = NONE;
    unsigned first = (unsigned)group * GROUP_SIZE, end = first + GROUP_SIZE;
    if (end > c->states)
        end = c->states;
    for (unsigned state = first; state < end; state++) {
        if (!settled(c, (uint16_t)state) && c->dist[state] != INF &&
            (best == NONE || state_less(c, (uint16_t)state, best)))
            best = (uint16_t)state;
    }
    c->best[group] = best;
    if (best == NONE) {
        c->position[group] = NONE;
        c->heap_count--;
        if (c->heap_count) {
            c->heap[0] = c->heap[c->heap_count];
            c->position[c->heap[0]] = 0;
        }
    }
    if (c->heap_count)
        heap_down(c, 0);
}
static void fail(NfMcuSlalom *c, NfMcuSlalomStatus status) {
    c->status = status;
    c->phase = PH_DONE;
    c->maze.required = NULL;
}
static bool relax(NfMcuSlalom *c, uint16_t destination, uint32_t edge) {
    uint32_t value;
    if (destination >= c->states)
        return false;
    if (!add_us(c->dist[c->current], edge, &value))
        return false;
    if (value >= c->dist[destination])
        return true;
    if (settled(c, destination) || value < c->last_key)
        return false;
    c->relaxed++;
    c->dist[destination] = value;
    queue_insert(c, destination);
    return true;
}
static void scan_ray_step(NfMcuSlalom *c) {
    f413_rp_anchor_t next;
    int x, y;
    if (c->scan_step >= NF_MCU_SLALOM_MAX_STEPS ||
        !f413_rp_advance_connector(&c->maze, c->scan_cursor, c->heading, &next) ||
        !f413_rp_anchor_region(&c->maze, next, c->heading, &x, &y)) {
        c->goal_cross_reachable = c->goal_cross_reachable || c->ray.has_goal;
        c->phase = PH_DIRECT;
        return;
    }
    c->scan_cursor = next;
    c->scan_step++;
    if (!c->ray.has_goal && f413_rp_goal_at(&c->maze, x, y)) {
        c->ray.has_goal = true;
        c->ray.goal_step = c->scan_step;
        c->ray.goal_x = (uint8_t)x;
        c->ray.goal_y = (uint8_t)y;
    }
    if (!f413_rp_is_cardinal(c->heading) || f413_rp_anchor_is_center(next))
        c->ray.stop_steps = c->scan_step;
}
static unsigned anchor_class(f413_rp_anchor_t a) {
    return f413_rp_anchor_is_center(a) ? 0U : ((a.half_x & 1) ? 2U : 1U);
}
static const NfMcuTurnTemplate *shape_for(f413_rp_anchor_t a, f413_rp_heading_t heading,
                                          unsigned kind, unsigned side) {
    return &nf_mcu_turn_templates[anchor_class(a)][heading][kind][side];
}
static bool geometry_open(const f413_rp_maze_t *maze, f413_rp_anchor_t source,
                          const NfMcuTurnTemplate *shape) {
    if (shape->edge_count == NF_MCU_SLALOM_INVALID_TEMPLATE)
        return false;
    int bx = source.half_x / 2, by = source.half_y / 2;
    for (unsigned i = 0; i < shape->edge_count; i++) {
        const NfMcuCrossedEdge *e = &nf_mcu_required_turn_edges[shape->edge_offset + i];
        int x = bx + e->dx, y = by + e->dy;
        if (e->dir > 3 || !f413_rp_cells_open(maze, x, y, x + g_f413_rp_dx[2 * e->dir],
                                              y + g_f413_rp_dy[2 * e->dir]))
            return false;
    }
    return true;
}
static const NfMcuGoalCross *first_turn_goal(const f413_rp_maze_t *maze, f413_rp_anchor_t source,
                                             const NfMcuTurnTemplate *shape) {
    int bx = source.half_x / 2, by = source.half_y / 2;
    for (unsigned i = 0; i < shape->cross_count; i++) {
        const NfMcuGoalCross *p = &nf_mcu_goal_crosses[shape->cross_offset + i];
        if (f413_rp_goal_at(maze, bx + p->dx, by + p->dy))
            return p;
    }
    return NULL;
}
static bool turn_open(const f413_rp_maze_t *maze, f413_rp_anchor_t source,
                      f413_rp_heading_t heading, unsigned kind, unsigned side,
                      f413_rp_anchor_t *end, f413_rp_heading_t *end_heading,
                      const NfMcuTurnTemplate **shape) {
    *shape = shape_for(source, heading, kind, side);
    if ((*shape)->edge_count == NF_MCU_SLALOM_INVALID_TEMPLATE)
        return false;
    end->half_x = (int16_t)(source.half_x + (*shape)->delta_hx);
    end->half_y = (int16_t)(source.half_y + (*shape)->delta_hy);
    *end_heading = (f413_rp_heading_t)(*shape)->end_heading;
    return f413_rp_state_pose_valid(maze, source, heading) &&
           f413_rp_state_pose_valid(maze, *end, *end_heading) &&
           geometry_open(maze, source, *shape);
}
static bool better_goal(const Goal *a, const Goal *b) {
    if (!b->valid)
        return true;
    if (a->entry_us != b->entry_us)
        return a->entry_us < b->entry_us;
    if (a->stop_us != b->stop_us)
        return a->stop_us < b->stop_us;
    if (a->goal_y != b->goal_y)
        return a->goal_y < b->goal_y;
    if (a->goal_x != b->goal_x)
        return a->goal_x < b->goal_x;
    if (a->kind != b->kind)
        return a->kind < b->kind;
    if (a->side != b->side)
        return a->side < b->side;
    return a->source < b->source;
}
static void brake_tail_step(NfMcuSlalom *c) {
    f413_rp_anchor_t next;
    int x, y;
    if (c->brake_step >= NF_MCU_SLALOM_MAX_STEPS ||
        !f413_rp_advance_connector(&c->maze, c->brake_cursor, c->brake_heading, &next) ||
        !f413_rp_anchor_region(&c->maze, next, c->brake_heading, &x, &y)) {
        if (c->pending_goal.stop_steps) {
            if (!add_us(c->pending_edge_end_us, c->brake_best_us, &c->pending_goal.stop_us)) {
                fail(c, NF_MCU_SLALOM_OVERFLOW);
                return;
            }
            if (better_goal(&c->pending_goal, &c->goal))
                c->goal = c->pending_goal;
        }
        c->phase = PH_EDGE;
        return;
    }
    c->brake_cursor = next;
    c->brake_step++;
    unsigned diagonal = !f413_rp_is_cardinal(c->brake_heading);
    if (!diagonal && !f413_rp_anchor_is_center(next))
        return;
    uint32_t value = nf_mcu_connector_us[diagonal][c->pending_goal.speed][3][c->brake_step];
    if (value < c->brake_best_us) {
        c->brake_best_us = value;
        c->pending_goal.stop_steps = c->brake_step;
    }
}
static void consider_direct(NfMcuSlalom *c) {
    unsigned n = c->direct_step++, diagonal = !f413_rp_is_cardinal(c->heading);
    if (n > c->ray.stop_steps || n > NF_MCU_SLALOM_MAX_STEPS) {
        c->phase = PH_EDGE;
        return;
    }
    if ((unsigned)c->speed >= 3U) {
        fail(c, NF_MCU_SLALOM_INVALID);
        return;
    }
    if (!c->ray.has_goal || !c->ray.goal_step || n <= c->ray.goal_step)
        return;
    f413_rp_anchor_t end = {c->anchor.half_x + (int16_t)(n * g_f413_rp_dx[c->heading]),
                            c->anchor.half_y + (int16_t)(n * g_f413_rp_dy[c->heading])};
    if (!diagonal && !f413_rp_anchor_is_center(end))
        return;
    uint32_t cross = nf_mcu_stop_cross_time_us(diagonal, c->speed, n, c->ray.goal_step);
    uint32_t stop = nf_mcu_connector_us[diagonal][c->speed][3][n];
    if (cross == INF || stop == INF)
        return;
    Goal g = {0};
    g.valid = true;
    g.direct = true;
    g.source = c->current;
    g.connector_steps = (uint16_t)n;
    g.stop_steps = (uint16_t)n;
    g.speed = (uint8_t)c->speed;
    g.goal_x = c->ray.goal_x;
    g.goal_y = c->ray.goal_y;
    if (!add_us(c->dist[c->current], cross, &g.entry_us) ||
        !add_us(c->dist[c->current], stop, &g.stop_us)) {
        fail(c, NF_MCU_SLALOM_OVERFLOW);
        return;
    }
    if (better_goal(&g, &c->goal))
        c->goal = g;
}
/* Build one pose's entire 14-bit feasibility mask cooperatively. Bit15 marks
 * completion; an immutable input snapshot makes this cache valid for all
 * incoming speeds and all connector prefixes reaching the same pose. */
static void cache_turn_step(NfMcuSlalom *c) {
    if (c->cache_index >= 14U) {
        c->turn_mask[c->cache_pose] = (uint16_t)(c->cache_mask | 0x8000U);
        c->phase = PH_EDGE;
        return;
    }
    unsigned index = c->cache_index++, kind = index / 2U, side = index % 2U;
    f413_rp_anchor_t end;
    f413_rp_heading_t heading;
    const NfMcuTurnTemplate *shape;
    if (turn_open(&c->maze, c->connector, c->heading, kind, side, &end, &heading, &shape))
        c->cache_mask |= (uint16_t)(1U << index);
}
static void advance_expansion_connector(NfMcuSlalom *c) {
    f413_rp_anchor_t next;
    int x, y;
    if (c->connector_steps >= NF_MCU_SLALOM_MAX_STEPS ||
        !f413_rp_advance_connector(&c->maze, c->connector, c->heading, &next) ||
        !f413_rp_anchor_region(&c->maze, next, c->heading, &x, &y) ||
        f413_rp_goal_at(&c->maze, x, y)) {
        c->phase = PH_POP;
        return;
    }
    c->connector = next;
    c->connector_steps++;
    c->edge_index = 0;
}
static void expand_edge(NfMcuSlalom *c) {
    const f413_rp_maze_t *maze = &c->maze;
    if (c->edge_index >= 42U) {
        advance_expansion_connector(c);
        return;
    }
    if (c->edge_index == 0U) {
        uint16_t pose;
        if (!f413_rp_pose_index(maze, c->connector, c->heading, &pose)) {
            advance_expansion_connector(c);
            return;
        }
        uint16_t mask = c->turn_mask[pose];
        if (!(mask & 0x8000U)) {
            c->cache_pose = pose;
            c->cache_mask = 0;
            c->cache_index = 0;
            c->phase = PH_CACHE;
            return;
        }
        c->connector_mask = mask & 0x3fffU;
        if (!c->connector_mask) {
            advance_expansion_connector(c);
            return;
        }
    }
    unsigned kind = 0, side = 0, turn_speed = 0;
    uint32_t connector = INF;
    /* At most42 cheap bit/table probes. Infeasible edges consume no geometry
     * work and never enter the Dijkstra graph. The original edge order stays. */
    while (c->edge_index < 42U) {
        unsigned index = c->edge_index++;
        kind = index / 6U;
        side = index % 2U;
        turn_speed = (index / 2U) % 3U;
        if (!(c->connector_mask & (1U << (kind * 2U + side))))
            continue;
        connector = nf_mcu_connector_us[!f413_rp_is_cardinal(c->heading)][c->speed][turn_speed]
                                       [c->connector_steps];
        if (connector != INF)
            break;
    }
    if (connector == INF) {
        advance_expansion_connector(c);
        return;
    }
    const NfMcuTurnTemplate *shape = shape_for(c->connector, c->heading, kind, side);
    f413_rp_anchor_t destination = {c->connector.half_x + shape->delta_hx,
                                    c->connector.half_y + shape->delta_hy};
    f413_rp_heading_t heading = (f413_rp_heading_t)shape->end_heading;
    uint32_t edge;
    if (!add_us(connector, nf_mcu_turn_us[turn_speed][kind], &edge)) {
        fail(c, NF_MCU_SLALOM_OVERFLOW);
        return;
    }
    const NfMcuGoalCross *cross = first_turn_goal(maze, c->connector, shape);
    if (cross) {
        c->goal_cross_reachable = true;
        Goal g = {0};
        uint32_t cross_edge, edge_end;
        g.valid = true;
        g.source = c->current;
        g.connector_steps = c->connector_steps;
        g.kind = (uint8_t)kind;
        g.side = (uint8_t)side;
        g.speed = (uint8_t)turn_speed;
        g.goal_x = (uint8_t)(c->connector.half_x / 2 + cross->dx);
        g.goal_y = (uint8_t)(c->connector.half_y / 2 + cross->dy);
        if (!add_us(connector, cross->time_us[turn_speed], &cross_edge) ||
            !add_us(c->dist[c->current], cross_edge, &g.entry_us) ||
            !add_us(c->dist[c->current], edge, &edge_end)) {
            fail(c, NF_MCU_SLALOM_OVERFLOW);
            return;
        }
        if (c->goal.valid && g.entry_us > c->goal.entry_us)
            return;
        int x, y;
        if (!f413_rp_anchor_region(maze, destination, heading, &x, &y))
            return;
        c->pending_goal = g;
        c->pending_edge_end_us = edge_end;
        c->brake_cursor = destination;
        c->brake_heading = heading;
        c->brake_step = 0;
        c->brake_best_us = INF;
        c->phase = PH_BRAKE;
    } else {
        uint16_t state =
            f413_rp_state_index(maze, destination, heading, (f413_rp_speed_t)turn_speed);
        if (!relax(c, state, edge))
            fail(c, NF_MCU_SLALOM_OVERFLOW);
    }
}
/* Requirement reconstruction uses one connector half-step per dispatch.
 * The required pointer is installed only during that dispatch, never while
 * control is yielded to the caller. */
static bool start_mark_action(NfMcuSlalom *c, uint16_t source, unsigned steps, unsigned kind,
                              unsigned side, bool direct, unsigned stop_steps) {
    f413_rp_speed_t speed;
    int x, y;
    if (!f413_rp_state_decode(&c->maze, source, &c->mark_cursor, &c->mark_heading, &speed))
        return false;
    c->maze.required = c->required;
    bool ok = f413_rp_anchor_region(&c->maze, c->mark_cursor, c->mark_heading, &x, &y);
    c->maze.required = NULL;
    if (!ok)
        return false;
    c->mark_source = source;
    c->mark_step = 0;
    c->mark_steps = (uint16_t)steps;
    c->mark_stop_steps = (uint16_t)stop_steps;
    c->mark_kind = (uint8_t)kind;
    c->mark_side = (uint8_t)side;
    c->mark_direct = direct;
    c->mark_stage = 0;
    c->phase = PH_MARK;
    return true;
}
static void mark_action_step(NfMcuSlalom *c) {
    f413_rp_anchor_t next;
    f413_rp_heading_t heading;
    const NfMcuTurnTemplate *shape;
    int x, y;
    bool ok = true;
    c->maze.required = c->required;
    if (c->mark_step < c->mark_steps) {
        ok = f413_rp_advance_connector(&c->maze, c->mark_cursor, c->mark_heading, &next) &&
             f413_rp_anchor_region(&c->maze, next, c->mark_heading, &x, &y);
        if (ok) {
            c->mark_cursor = next;
            c->mark_step++;
        }
    } else if (c->mark_stage == 0 && !c->mark_direct) {
        ok = turn_open(&c->maze, c->mark_cursor, c->mark_heading, c->mark_kind, c->mark_side, &next,
                       &heading, &shape);
        if (ok) {
            c->mark_cursor = next;
            c->mark_heading = heading;
            c->mark_stage = 1;
            c->mark_step = 0;
            c->mark_steps = c->mark_stop_steps;
        }
    } else {
        c->actions++;
        c->reconstruct_state = c->mark_source;
        c->reconstruct_template = 0;
        c->phase = PH_RECON_TEMPLATE;
    }
    c->maze.required = NULL;
    if (!ok)
        fail(c, NF_MCU_SLALOM_INVALID);
}
static void reconstruct_init(NfMcuSlalom *c) {
    if (!start_mark_action(c, c->goal.source, c->goal.connector_steps, c->goal.kind, c->goal.side,
                           c->goal.direct, c->goal.direct ? 0 : c->goal.stop_steps))
        fail(c, NF_MCU_SLALOM_INVALID);
}
/* Recover parents from d(u)+cost(u,v)==d(v), inspecting the exact inverse
 * turn topology and connector ray. Positive edge costs make d strictly fall,
 * so no parent array or loop detector is needed. */
static void reconstruct_template(NfMcuSlalom *c) {
    const f413_rp_maze_t *maze = &c->maze;
    if (c->reconstruct_state == c->start_state) {
        c->status = NF_MCU_SLALOM_EXACT;
        c->phase = PH_DONE;
        return;
    }
    if (c->reconstruct_template >= 3U * 8U * 7U * 2U) {
        fail(c, NF_MCU_SLALOM_INVALID);
        return;
    }
    if (!f413_rp_state_decode(maze, c->reconstruct_state, &c->reconstruct_end, &c->heading,
                              &c->reconstruct_exit_speed)) {
        fail(c, NF_MCU_SLALOM_INVALID);
        return;
    }
    unsigned index = c->reconstruct_template++, side = index % 2, kind = (index / 2) % 7;
    unsigned heading = (index / 14) % 8, aclass = index / (14 * 8);
    const NfMcuTurnTemplate *shape = &nf_mcu_turn_templates[aclass][heading][kind][side];
    if (shape->edge_count == NF_MCU_SLALOM_INVALID_TEMPLATE || shape->end_heading != c->heading)
        return;
    f413_rp_anchor_t turn = {c->reconstruct_end.half_x - shape->delta_hx,
                             c->reconstruct_end.half_y - shape->delta_hy};
    if (anchor_class(turn) != aclass)
        return;
    f413_rp_anchor_t end;
    f413_rp_heading_t end_heading;
    const NfMcuTurnTemplate *checked;
    if (!turn_open(maze, turn, (f413_rp_heading_t)heading, kind, side, &end, &end_heading,
                   &checked) ||
        first_turn_goal(maze, turn, checked))
        return;
    c->reconstruct_turn = turn;
    c->reconstruct_source = turn;
    c->reconstruct_heading = (f413_rp_heading_t)heading;
    c->reconstruct_kind = (uint8_t)kind;
    c->reconstruct_side = (uint8_t)side;
    c->reconstruct_shape = shape;
    c->reconstruct_step = 0;
    c->reconstruct_speed = 0;
    c->phase = PH_RECON_EDGE;
}
static void reconstruct_edge(NfMcuSlalom *c) {
    const f413_rp_maze_t *maze = &c->maze;
    if (c->reconstruct_speed >= 3) {
        f413_rp_anchor_t previous;
        int x, y;
        if (c->reconstruct_step >= NF_MCU_SLALOM_MAX_STEPS ||
            !f413_rp_advance_connector(maze, c->reconstruct_source,
                                       f413_rp_heading_add(c->reconstruct_heading, 4), &previous) ||
            !f413_rp_anchor_region(maze, previous, c->reconstruct_heading, &x, &y) ||
            f413_rp_goal_at(maze, x, y)) {
            c->phase = PH_RECON_TEMPLATE;
            return;
        }
        c->reconstruct_source = previous;
        c->reconstruct_step++;
        c->reconstruct_speed = 0;
        return;
    }
    unsigned speed = c->reconstruct_speed++;
    if (!f413_rp_state_pose_valid(maze, c->reconstruct_source, c->reconstruct_heading))
        return;
    uint16_t source = f413_rp_state_index(maze, c->reconstruct_source, c->reconstruct_heading,
                                          (f413_rp_speed_t)speed);
    uint32_t linear = nf_mcu_connector_us[!f413_rp_is_cardinal(c->reconstruct_heading)][speed]
                                         [c->reconstruct_exit_speed][c->reconstruct_step];
    uint32_t edge, total;
    if (source >= c->states || linear == INF || c->dist[source] == INF ||
        !add_us(linear, nf_mcu_turn_us[c->reconstruct_exit_speed][c->reconstruct_kind], &edge) ||
        !add_us(c->dist[source], edge, &total) || total != c->dist[c->reconstruct_state])
        return;
    int x, y;
    if (!f413_rp_anchor_region(maze, c->reconstruct_source, c->reconstruct_heading, &x, &y) ||
        f413_rp_goal_at(maze, x, y))
        return;
    c->verify_source = source;
    c->verify_cursor = c->reconstruct_source;
    c->verify_step = 0;
    c->phase = PH_VERIFY;
}
static void verify_connector_step(NfMcuSlalom *c) {
    if (c->verify_step >= c->reconstruct_step) {
        if (!start_mark_action(c, c->verify_source, c->reconstruct_step, c->reconstruct_kind,
                               c->reconstruct_side, false, 0))
            fail(c, NF_MCU_SLALOM_INVALID);
        return;
    }
    f413_rp_anchor_t next;
    int x, y;
    if (!f413_rp_advance_connector(&c->maze, c->verify_cursor, c->reconstruct_heading, &next) ||
        !f413_rp_anchor_region(&c->maze, next, c->reconstruct_heading, &x, &y) ||
        f413_rp_goal_at(&c->maze, x, y)) {
        c->phase = PH_RECON_EDGE;
        return;
    }
    c->verify_cursor = next;
    c->verify_step++;
}

static void pop_state(NfMcuSlalom *c) {
    if (c->heap_count == 0) {
        if (c->goal.valid)
            c->phase = PH_RECON_INIT;
        else
            fail(c, c->goal_cross_reachable ? NF_MCU_SLALOM_NO_FEASIBLE_TERMINAL
                                            : NF_MCU_SLALOM_NO_PATH);
        return;
    }
    uint16_t state = c->best[c->heap[0]];
    if (state >= c->states || settled(c, state) || c->dist[state] < c->last_key) {
        fail(c, NF_MCU_SLALOM_INVALID);
        return;
    }
    if (c->goal.valid && c->dist[state] > c->goal.entry_us) {
        c->phase = PH_RECON_INIT;
        return;
    }
    c->last_key = c->dist[state];
    c->settled[state >> 3] |= (uint8_t)(1U << (state & 7U));
    c->expanded++;
    c->current = state;
    queue_remove_min(c);
    int x, y;
    if (!f413_rp_state_decode(&c->maze, state, &c->anchor, &c->heading, &c->speed) ||
        !f413_rp_anchor_region(&c->maze, c->anchor, c->heading, &x, &y)) {
        fail(c, NF_MCU_SLALOM_INVALID);
        return;
    }
    memset(&c->ray, 0, sizeof(c->ray));
    if (f413_rp_goal_at(&c->maze, x, y)) {
        c->ray.has_goal = true;
        c->ray.goal_x = (uint8_t)x;
        c->ray.goal_y = (uint8_t)y;
    }
    c->scan_cursor = c->anchor;
    c->scan_step = 0;
    c->connector = c->anchor;
    c->connector_steps = 0;
    c->edge_index = 0;
    c->direct_step = 1;
    c->phase = PH_SCAN;
}
static void initialize_step(NfMcuSlalom *c) {
    unsigned first = c->init_cursor, end = first + 32U;
    if (c->phase == PH_INIT_DIST) {
        if (end > c->states)
            end = c->states;
        for (unsigned i = first; i < end; i++)
            c->dist[i] = INF;
        if (end == c->states) {
            c->init_cursor = 0;
            c->phase = PH_INIT_GROUP;
        } else
            c->init_cursor = (uint16_t)end;
    } else if (c->phase == PH_INIT_GROUP) {
        if (end > c->groups)
            end = c->groups;
        for (unsigned i = first; i < end; i++) {
            c->position[i] = NONE;
            c->best[i] = NONE;
        }
        if (end == c->groups) {
            c->init_cursor = 0;
            c->phase = PH_INIT_CACHE;
        } else
            c->init_cursor = (uint16_t)end;
    } else if (c->phase == PH_INIT_CACHE) {
        if (end > c->poses)
            end = c->poses;
        for (unsigned i = first; i < end; i++)
            c->turn_mask[i] = 0;
        if (end == c->poses) {
            c->init_cursor = 0;
            c->phase = PH_INIT_SETTLED;
        } else
            c->init_cursor = (uint16_t)end;
    } else {
        unsigned bytes = (c->states + 7U) / 8U;
        if (end > bytes)
            end = bytes;
        for (unsigned i = first; i < end; i++)
            c->settled[i] = 0;
        if (end == bytes) {
            c->dist[c->start_state] = nf_mcu_start_us;
            queue_insert(c, c->start_state);
            c->phase = PH_POP;
        } else
            c->init_cursor = (uint16_t)end;
    }
}

NfMcuSlalomStatus nf_mcu_slalom_step(NfMcuSlalom *c, uint32_t work_budget) {
    if (c == NULL)
        return NF_MCU_SLALOM_INVALID;
    while (work_budget-- && c->status == NF_MCU_SLALOM_PENDING) {
        c->work++;
        switch (c->phase) {
        case PH_INIT_DIST:
        case PH_INIT_GROUP:
        case PH_INIT_CACHE:
        case PH_INIT_SETTLED:
            initialize_step(c);
            break;
        case PH_POP:
            pop_state(c);
            break;
        case PH_SCAN:
            scan_ray_step(c);
            break;
        case PH_BRAKE:
            brake_tail_step(c);
            break;
        case PH_VERIFY:
            verify_connector_step(c);
            break;
        case PH_MARK:
            mark_action_step(c);
            break;
        case PH_DIRECT:
            consider_direct(c);
            break;
        case PH_CACHE:
            cache_turn_step(c);
            break;
        case PH_EDGE:
            expand_edge(c);
            break;
        case PH_RECON_INIT:
            reconstruct_init(c);
            break;
        case PH_RECON_TEMPLATE:
            reconstruct_template(c);
            break;
        case PH_RECON_EDGE:
            reconstruct_edge(c);
            break;
        default:
            fail(c, NF_MCU_SLALOM_INVALID);
            break;
        }
    }
    return c->status;
}
size_t nf_mcu_slalom_workspace_bytes_for(uint8_t width, uint8_t height) {
    if (width < 2 || height < 2 || width > 32 || height > 32)
        return 0;
    size_t states = ((size_t)width * height + (width - 1U) * height + (height - 1U) * width) * 12U;
    size_t groups = (states + GROUP_SIZE - 1U) / GROUP_SIZE;
    return (sizeof(NfMcuSlalom) + 7U + states * 4U + groups * 6U + (states / 3U) * 2U +
            (states + 7U) / 8U + 7U) &
           ~(size_t)7U;
}
size_t nf_mcu_slalom_workspace_bytes(void) {
    return nf_mcu_slalom_workspace_bytes_for(32, 32);
}
const char *nf_mcu_slalom_status_name(NfMcuSlalomStatus status) {
    static const char *const names[] = {"pending", "exact",    "no-path", "no-feasible-terminal",
                                        "invalid", "capacity", "overflow"};
    return (unsigned)status < sizeof(names) / sizeof(names[0]) ? names[status] : "invalid";
}
NfMcuSlalomStatus nf_mcu_slalom_begin(void *workspace, size_t bytes, uint8_t width, uint8_t height,
                                      const uint8_t *walls, const uint8_t *goals, uint8_t sx,
                                      uint8_t sy, uint8_t sh, NfMcuSlalom **out) {
    if (out)
        *out = NULL;
    size_t needed = nf_mcu_slalom_workspace_bytes_for(width, height);
    if (!workspace || !walls || !goals || !out || !needed || sx >= width || sy >= height ||
        sh > 3 || ((uintptr_t)workspace & (NF_MCU_SLALOM_WORKSPACE_ALIGNMENT - 1U)))
        return NF_MCU_SLALOM_INVALID;
    if (bytes < needed)
        return NF_MCU_SLALOM_CAPACITY;
    NfMcuSlalom *c = (NfMcuSlalom *)workspace;
    memset(c, 0, sizeof(*c));
    c->maze.width = width;
    c->maze.height = height;
    const f413_rp_maze_t *maze = &c->maze;
    c->states = (uint16_t)F413_RP_STATE_COUNT;
    c->poses = c->states / 3U;
    c->groups = (uint16_t)((c->states + GROUP_SIZE - 1U) / GROUP_SIZE);
    c->workspace_used = needed;
    uintptr_t cursor = ((uintptr_t)(c + 1) + 3U) & ~(uintptr_t)3U;
    c->dist = (uint32_t *)cursor;
    cursor += (size_t)c->states * 4U;
    c->heap = (uint16_t *)cursor;
    cursor += (size_t)c->groups * 2U;
    c->position = (uint16_t *)cursor;
    cursor += (size_t)c->groups * 2U;
    c->best = (uint16_t *)cursor;
    cursor += (size_t)c->groups * 2U;
    c->turn_mask = (uint16_t *)cursor;
    cursor += (size_t)c->poses * 2U;
    c->settled = (uint8_t *)cursor;
    bool has_goal = false;
    for (unsigned y = 0; y < height; y++)
        for (unsigned x = 0; x < width; x++) {
            unsigned i = y * width + x;
            uint8_t v = walls[i];
            if (v > 15 || (x == 0 && !(v & 8)) || (y == 0 && !(v & 4)) ||
                (x + 1 == width && !(v & 2)) || (y + 1 == height && !(v & 1)) ||
                (x + 1 < width && !!(v & 2) != !!(walls[i + 1] & 8)) ||
                (y + 1 < height && !!(v & 1) != !!(walls[i + width] & 4)))
                return NF_MCU_SLALOM_INVALID;
            c->maze.walls[i] = v;
            if (goals[i]) {
                f413_rp_goal_set(&c->maze, (uint8_t)x, (uint8_t)y);
                has_goal = true;
            }
        }
    if (!has_goal)
        return NF_MCU_SLALOM_INVALID;
    f413_rp_anchor_t start = {(int16_t)(2U * sx + 1U), (int16_t)(2U * sy + 1U)};
    c->start_state =
        f413_rp_state_index(maze, start, (f413_rp_heading_t)(2U * sh), F413_RP_SPEED_CRAWL);
    c->phase = PH_INIT_DIST;
    c->status = NF_MCU_SLALOM_PENDING;
    *out = c;
    if (f413_rp_goal_at(maze, sx, sy)) {
        c->status = NF_MCU_SLALOM_EXACT;
        c->phase = PH_DONE;
        c->goal.valid = true;
        c->goal.goal_x = sx;
        c->goal.goal_y = sy;
        return c->status;
    }
    return c->status;
}
NfMcuSlalomStatus nf_mcu_slalom_result(const NfMcuSlalom *c, NfMcuSlalomResult *result,
                                       uint8_t *required, size_t required_capacity) {
    if (!result)
        return NF_MCU_SLALOM_INVALID;
    memset(result, 0, sizeof(*result));
    result->status = NF_MCU_SLALOM_INVALID;
    if (!c)
        return NF_MCU_SLALOM_INVALID;
    size_t cells = (size_t)c->maze.width * c->maze.height;
    if (required && required_capacity < cells) {
        memset(required, 0, required_capacity);
        result->status = NF_MCU_SLALOM_CAPACITY;
        return NF_MCU_SLALOM_CAPACITY;
    }
    result->status = c->status;
    result->expanded_states = c->expanded;
    result->relaxed_edges = c->relaxed;
    result->work_units = c->work;
    result->heap_peak = c->heap_peak;
    result->action_count = c->actions;
    result->workspace_used = c->workspace_used;
    if (c->status == NF_MCU_SLALOM_EXACT) {
        result->goal_entry_us = c->goal.entry_us;
        result->stop_us = c->goal.stop_us;
        result->goal_x = c->goal.goal_x;
        result->goal_y = c->goal.goal_y;
        result->requirements_complete = true;
        if (required)
            memcpy(required, c->required, cells);
    } else if (required)
        memset(required, 0, cells);
    return c->status;
}
