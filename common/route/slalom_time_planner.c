#include "slalom_time_planner.h"

#include <float.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define NF_SLALOM_PI 3.14159265358979323846264338327950288
#define NF_SLALOM_SQRT2 1.41421356237309504880168872420969808
#define NF_SLALOM_EPS 1.0e-8
#define NF_SLALOM_INF UINT64_MAX
#define NF_SLALOM_TURN_COUNT ((size_t)NF_SLALOM_ACTION_START_OFFSET)

typedef struct {
    uint64_t distance_us;
    uint16_t turn_count;
    bool settled;
} NfSlalomNode;

typedef struct {
    bool valid;
    size_t previous_state;
    uint16_t connector_steps;
    NfSlalomActionKind kind;
    NfRouteSide side;
} NfSlalomParent;

typedef struct {
    size_t *states;
    ptrdiff_t *positions;
    size_t size;
    size_t capacity;
    const NfSlalomNode *nodes;
} NfSlalomHeap;

typedef struct {
    bool present;
    uint16_t step;
    uint8_t x;
    uint8_t y;
} NfConnectorGoal;

typedef struct {
    bool feasible;
    bool has_goal;
    uint8_t goal_x;
    uint8_t goal_y;
    double goal_time_s;
    double goal_velocity_mm_s;
    NfSlalomHeading8 goal_heading;
    bool clearance_checked;
} NfTurnTrace;

typedef struct {
    bool feasible;
    size_t destination_state;
    NfSlalomAction action;
    double goal_entry_velocity_mm_s;
    bool has_stop_action;
    NfSlalomAction stop_action;
} NfBuiltEdge;

typedef struct {
    uint16_t stop_steps;
    NfSlalomAnchor stop_anchor;
    NfConnectorGoal first_goal;
} NfRayScan;

typedef struct {
    bool valid;
    size_t source_state;
    NfSlalomAction terminal_action;
    bool has_stop_action;
    NfSlalomAction stop_action;
    uint64_t goal_entry_us;
    uint64_t stop_us;
    double goal_entry_velocity_mm_s;
    uint16_t turn_count;
} NfGoalRecord;

typedef struct {
    NfTurnPose *poses;
    double *elapsed_s;
    uint32_t intervals;
} NfTurnTrajectoryCache;

typedef struct {
    const NfRouteMaze *maze;
    const NfSlalomPlannerConfig *config;
    /* Calibrated/provisional source of edge duration and boundary velocity. */
    NfTurnPlan turn_plans[NF_SLALOM_TURN_COUNT];
    /* Exact-closing PC centre-line used for every spatial operation. */
    NfTurnPlan geometry_turn_plans[NF_SLALOM_TURN_COUNT];
    NfTurnTrajectoryCache turn_trajectories[NF_SLALOM_TURN_COUNT];
    NfLinearPlan start_plan;
    uint64_t start_time_us;
    double start_boundary_velocity_mm_s;
    NfSlalomNode *nodes;
    NfSlalomParent *parents;
    size_t anchor_count;
    size_t state_count;
    NfSlalomHeap heap;
    NfGoalRecord goal;
    uint32_t expanded_states;
    uint32_t relaxed_edges;
} NfSlalomContext;

typedef struct {
    double fraction;
    bool vertical;
    int line;
} NfBoundaryEvent;

static const int8_t k_heading_dx[8] = {0, 1, 1, 1, 0, -1, -1, -1};
static const int8_t k_heading_dy[8] = {1, 1, 0, -1, -1, -1, 0, 1};
static const uint8_t k_wall_masks[4] = {
    NF_ROUTE_WALL_NORTH,
    NF_ROUTE_WALL_EAST,
    NF_ROUTE_WALL_SOUTH,
    NF_ROUTE_WALL_WEST,
};

static bool nf_slalom_is_cardinal(NfSlalomHeading8 heading)
{
    return ((unsigned int)heading & 1U) == 0U;
}

static bool nf_slalom_is_diagonal(NfSlalomHeading8 heading)
{
    return !nf_slalom_is_cardinal(heading);
}

static NfSlalomHeading8 nf_slalom_heading_add(NfSlalomHeading8 heading,
                                               int delta)
{
    int result = ((int)heading + delta) % 8;
    if (result < 0) {
        result += 8;
    }
    return (NfSlalomHeading8)result;
}

static int nf_slalom_side_sign(NfRouteSide side)
{
    return (side == NF_ROUTE_SIDE_RIGHT) ? 1 : -1;
}

static double nf_slalom_heading_math_deg(NfSlalomHeading8 heading)
{
    double degrees = 90.0 - (45.0 * (double)heading);
    while (degrees <= -180.0) {
        degrees += 360.0;
    }
    return degrees;
}

static NfSlalomHeading8 nf_slalom_heading_from_math_deg(double degrees)
{
    long index;
    while (degrees < -180.0) {
        degrees += 360.0;
    }
    while (degrees >= 180.0) {
        degrees -= 360.0;
    }
    index = lround((90.0 - degrees) / 45.0);
    index %= 8L;
    if (index < 0L) {
        index += 8L;
    }
    return (NfSlalomHeading8)index;
}

static bool nf_slalom_u64_add(uint64_t left, uint64_t right, uint64_t *out)
{
    if (out == NULL || UINT64_MAX - left < right) {
        return false;
    }
    *out = left + right;
    return true;
}

static NfSlalomPlanStatus nf_slalom_seconds_to_us(double seconds,
                                                  uint64_t *out_us)
{
    const NfMotionStatus status = nf_motion_seconds_to_us(seconds, out_us);
    if (status == NF_MOTION_OK) {
        return NF_SLALOM_PLAN_OK;
    }
    return (status == NF_MOTION_OVERFLOW) ?
        NF_SLALOM_PLAN_OVERFLOW : NF_SLALOM_PLAN_INVALID_CONFIG;
}

static bool nf_slalom_finite_positive(double value)
{
    return isfinite(value) && value > 0.0;
}

static void nf_slalom_free_turn_trajectories(NfSlalomContext *context)
{
    for (size_t i = 0U; i < NF_SLALOM_TURN_COUNT; i++) {
        free(context->turn_trajectories[i].poses);
        free(context->turn_trajectories[i].elapsed_s);
        context->turn_trajectories[i].poses = NULL;
        context->turn_trajectories[i].elapsed_s = NULL;
        context->turn_trajectories[i].intervals = 0U;
    }
}

const char *nf_slalom_plan_status_name(NfSlalomPlanStatus status)
{
    switch (status) {
    case NF_SLALOM_PLAN_OK: return "ok";
    case NF_SLALOM_PLAN_INVALID_ARGUMENT: return "invalid-argument";
    case NF_SLALOM_PLAN_INVALID_MAZE: return "invalid-maze";
    case NF_SLALOM_PLAN_INVALID_CONFIG: return "invalid-config";
    case NF_SLALOM_PLAN_UNSUPPORTED_SAFETY: return "unsupported-safety";
    case NF_SLALOM_PLAN_NO_PATH: return "no-path";
    case NF_SLALOM_PLAN_CAPACITY: return "capacity";
    case NF_SLALOM_PLAN_OVERFLOW: return "overflow";
    default: return "unknown";
    }
}

const char *nf_slalom_heading_name(NfSlalomHeading8 heading)
{
    static const char *const names[8] = {
        "north", "north-east", "east", "south-east",
        "south", "south-west", "west", "north-west",
    };
    return ((unsigned int)heading < 8U) ? names[(unsigned int)heading] :
        "invalid";
}

const char *nf_slalom_action_name(NfSlalomActionKind kind,
                                  NfRouteSide side)
{
    static const char *const base[NF_SLALOM_ACTION_KIND_COUNT] = {
        "small-90", "large-90", "large-180", "45-in", "45-out",
        "v90", "135-in", "135-out", "start-offset", "goal-stop",
    };
    static char name[32];
    const char *suffix;
    size_t base_length;

    if ((unsigned int)kind >= NF_SLALOM_ACTION_KIND_COUNT) {
        return "invalid";
    }
    if (kind == NF_SLALOM_ACTION_GOAL_STOP ||
        kind == NF_SLALOM_ACTION_START_OFFSET) {
        return (side == NF_ROUTE_SIDE_NONE) ?
            base[(unsigned int)kind] : "invalid";
    }
    if (side != NF_ROUTE_SIDE_RIGHT && side != NF_ROUTE_SIDE_LEFT) {
        return "invalid";
    }
    suffix = (side == NF_ROUTE_SIDE_RIGHT) ? "-right" : "-left";
    base_length = strlen(base[(unsigned int)kind]);
    if (base_length + strlen(suffix) + 1U > sizeof(name)) {
        return "invalid";
    }
    memcpy(name, base[(unsigned int)kind], base_length);
    (void)strcpy(&name[base_length], suffix);
    return name;
}

static bool nf_slalom_valid_maze(const NfRouteMaze *maze)
{
    bool has_goal = false;

    if (maze == NULL || maze->width == 0U || maze->height == 0U ||
        maze->width > NF_ROUTE_MAZE_MAX_SIZE ||
        maze->height > NF_ROUTE_MAZE_MAX_SIZE) {
        return false;
    }
    for (uint8_t y = 0U; y < maze->height; y++) {
        for (uint8_t x = 0U; x < maze->width; x++) {
            const uint8_t walls = maze->walls[y][x];
            if ((walls & (uint8_t)~(NF_ROUTE_WALL_NORTH |
                                     NF_ROUTE_WALL_EAST |
                                     NF_ROUTE_WALL_SOUTH |
                                     NF_ROUTE_WALL_WEST)) != 0U ||
                (x == 0U && (walls & NF_ROUTE_WALL_WEST) == 0U) ||
                (x + 1U == maze->width &&
                 (walls & NF_ROUTE_WALL_EAST) == 0U) ||
                (y == 0U && (walls & NF_ROUTE_WALL_SOUTH) == 0U) ||
                (y + 1U == maze->height &&
                 (walls & NF_ROUTE_WALL_NORTH) == 0U)) {
                return false;
            }
            if (x + 1U < maze->width) {
                const bool east = (walls & NF_ROUTE_WALL_EAST) != 0U;
                const bool west =
                    (maze->walls[y][x + 1U] & NF_ROUTE_WALL_WEST) != 0U;
                if (east != west) {
                    return false;
                }
            }
            if (y + 1U < maze->height) {
                const bool north = (walls & NF_ROUTE_WALL_NORTH) != 0U;
                const bool south =
                    (maze->walls[y + 1U][x] & NF_ROUTE_WALL_SOUTH) != 0U;
                if (north != south) {
                    return false;
                }
            }
            has_goal = has_goal || maze->goals[y][x];
        }
    }
    return has_goal;
}

static bool nf_slalom_cell_in_bounds(const NfRouteMaze *maze, int x, int y)
{
    return x >= 0 && y >= 0 && x < (int)maze->width &&
           y < (int)maze->height;
}

static bool nf_slalom_cells_open(const NfRouteMaze *maze,
                                 int from_x,
                                 int from_y,
                                 int to_x,
                                 int to_y)
{
    NfRouteDirection direction;
    NfRouteDirection opposite;

    if (!nf_slalom_cell_in_bounds(maze, from_x, from_y) ||
        !nf_slalom_cell_in_bounds(maze, to_x, to_y)) {
        return false;
    }
    if (to_x == from_x && to_y == from_y + 1) {
        direction = NF_ROUTE_DIR_NORTH;
    } else if (to_x == from_x + 1 && to_y == from_y) {
        direction = NF_ROUTE_DIR_EAST;
    } else if (to_x == from_x && to_y == from_y - 1) {
        direction = NF_ROUTE_DIR_SOUTH;
    } else if (to_x == from_x - 1 && to_y == from_y) {
        direction = NF_ROUTE_DIR_WEST;
    } else {
        return false;
    }
    opposite = (NfRouteDirection)(((unsigned int)direction + 2U) & 3U);
    return (maze->walls[from_y][from_x] &
            k_wall_masks[(unsigned int)direction]) == 0U &&
           (maze->walls[to_y][to_x] &
            k_wall_masks[(unsigned int)opposite]) == 0U;
}

static size_t nf_slalom_anchor_count(const NfRouteMaze *maze)
{
    const size_t centers = (size_t)maze->width * maze->height;
    const size_t vertical = (maze->width > 1U) ?
        (size_t)(maze->width - 1U) * maze->height : 0U;
    const size_t horizontal = (maze->height > 1U) ?
        (size_t)maze->width * (maze->height - 1U) : 0U;
    return centers + vertical + horizontal;
}

static bool nf_slalom_anchor_from_id(const NfRouteMaze *maze,
                                     size_t id,
                                     NfSlalomAnchor *out)
{
    const size_t centers = (size_t)maze->width * maze->height;
    const size_t vertical = (maze->width > 1U) ?
        (size_t)(maze->width - 1U) * maze->height : 0U;

    if (out == NULL || id >= nf_slalom_anchor_count(maze)) {
        return false;
    }
    if (id < centers) {
        const size_t x = id % maze->width;
        const size_t y = id / maze->width;
        out->half_x = (int16_t)((2U * x) + 1U);
        out->half_y = (int16_t)((2U * y) + 1U);
        return true;
    }
    id -= centers;
    if (id < vertical) {
        const size_t x_line = (id % (maze->width - 1U)) + 1U;
        const size_t y = id / (maze->width - 1U);
        out->half_x = (int16_t)(2U * x_line);
        out->half_y = (int16_t)((2U * y) + 1U);
        return true;
    }
    id -= vertical;
    {
        const size_t x = id % maze->width;
        const size_t y_line = (id / maze->width) + 1U;
        out->half_x = (int16_t)((2U * x) + 1U);
        out->half_y = (int16_t)(2U * y_line);
    }
    return true;
}

static bool nf_slalom_anchor_id(const NfRouteMaze *maze,
                                NfSlalomAnchor anchor,
                                size_t *out_id)
{
    const int hx = anchor.half_x;
    const int hy = anchor.half_y;
    const size_t centers = (size_t)maze->width * maze->height;
    const size_t vertical = (maze->width > 1U) ?
        (size_t)(maze->width - 1U) * maze->height : 0U;

    if (out_id == NULL || hx <= 0 || hy <= 0 ||
        hx >= (int)(2U * maze->width) ||
        hy >= (int)(2U * maze->height) ||
        ((hx & 1) == 0 && (hy & 1) == 0)) {
        return false;
    }
    if ((hx & 1) != 0 && (hy & 1) != 0) {
        const size_t x = (size_t)(hx - 1) / 2U;
        const size_t y = (size_t)(hy - 1) / 2U;
        *out_id = (y * maze->width) + x;
        return true;
    }
    if ((hx & 1) == 0) {
        const size_t x_line = (size_t)hx / 2U;
        const size_t y = (size_t)(hy - 1) / 2U;
        if (x_line == 0U || x_line >= maze->width) {
            return false;
        }
        *out_id = centers + (y * (maze->width - 1U)) + (x_line - 1U);
        return true;
    }
    {
        const size_t x = (size_t)(hx - 1) / 2U;
        const size_t y_line = (size_t)hy / 2U;
        if (y_line == 0U || y_line >= maze->height) {
            return false;
        }
        *out_id = centers + vertical +
                  ((y_line - 1U) * maze->width) + x;
    }
    return true;
}

static bool nf_slalom_anchor_is_center(NfSlalomAnchor anchor)
{
    return (anchor.half_x & 1) != 0 && (anchor.half_y & 1) != 0;
}

static bool nf_slalom_anchor_open(const NfRouteMaze *maze,
                                  NfSlalomAnchor anchor)
{
    if (nf_slalom_anchor_is_center(anchor)) {
        return true;
    }
    if ((anchor.half_x & 1) == 0) {
        const int right_x = anchor.half_x / 2;
        const int y = (anchor.half_y - 1) / 2;
        return nf_slalom_cells_open(maze, right_x - 1, y, right_x, y);
    }
    {
        const int x = (anchor.half_x - 1) / 2;
        const int upper_y = anchor.half_y / 2;
        return nf_slalom_cells_open(maze, x, upper_y - 1, x, upper_y);
    }
}

static bool nf_slalom_pose_valid(const NfRouteMaze *maze,
                                 NfSlalomAnchor anchor,
                                 NfSlalomHeading8 heading)
{
    size_t ignored_id;
    if ((unsigned int)heading >= 8U ||
        !nf_slalom_anchor_id(maze, anchor, &ignored_id)) {
        return false;
    }
    if (nf_slalom_anchor_is_center(anchor)) {
        return nf_slalom_is_cardinal(heading);
    }
    if (!nf_slalom_anchor_open(maze, anchor)) {
        return false;
    }
    if (nf_slalom_is_diagonal(heading)) {
        return true;
    }
    if ((anchor.half_x & 1) == 0) {
        return heading == NF_SLALOM_HEADING_EAST ||
               heading == NF_SLALOM_HEADING_WEST;
    }
    return heading == NF_SLALOM_HEADING_NORTH ||
           heading == NF_SLALOM_HEADING_SOUTH;
}

static bool nf_slalom_anchor_region(const NfRouteMaze *maze,
                                    NfSlalomAnchor anchor,
                                    NfSlalomHeading8 heading,
                                    int *out_x,
                                    int *out_y)
{
    int x;
    int y;
    if (!nf_slalom_pose_valid(maze, anchor, heading) ||
        out_x == NULL || out_y == NULL) {
        return false;
    }
    if (nf_slalom_anchor_is_center(anchor)) {
        x = (anchor.half_x - 1) / 2;
        y = (anchor.half_y - 1) / 2;
    } else if ((anchor.half_x & 1) == 0) {
        const int line = anchor.half_x / 2;
        x = (k_heading_dx[(unsigned int)heading] > 0) ? line : line - 1;
        y = (anchor.half_y - 1) / 2;
    } else {
        const int line = anchor.half_y / 2;
        x = (anchor.half_x - 1) / 2;
        y = (k_heading_dy[(unsigned int)heading] > 0) ? line : line - 1;
    }
    if (!nf_slalom_cell_in_bounds(maze, x, y)) {
        return false;
    }
    *out_x = x;
    *out_y = y;
    return true;
}

static bool nf_slalom_advance_connector(const NfRouteMaze *maze,
                                        NfSlalomAnchor current,
                                        NfSlalomHeading8 heading,
                                        NfSlalomAnchor *out)
{
    NfSlalomAnchor next;
    size_t ignored_id;

    if (out == NULL || !nf_slalom_pose_valid(maze, current, heading)) {
        return false;
    }
    next.half_x = (int16_t)(current.half_x +
                             k_heading_dx[(unsigned int)heading]);
    next.half_y = (int16_t)(current.half_y +
                             k_heading_dy[(unsigned int)heading]);
    if (!nf_slalom_anchor_id(maze, next, &ignored_id) ||
        !nf_slalom_pose_valid(maze, next, heading)) {
        return false;
    }
    if (!nf_slalom_anchor_is_center(next) &&
        !nf_slalom_anchor_open(maze, next)) {
        return false;
    }
    *out = next;
    return true;
}

static size_t nf_slalom_state_index(const NfSlalomContext *context,
                                    NfSlalomAnchor anchor,
                                    NfSlalomHeading8 heading,
                                    NfSlalomSpeedClass speed_class)
{
    size_t anchor_id = 0U;
    (void)nf_slalom_anchor_id(context->maze, anchor, &anchor_id);
    return (((anchor_id * 8U) + (size_t)heading) *
            NF_SLALOM_SPEED_CLASS_COUNT) + (size_t)speed_class;
}

static bool nf_slalom_state_decode(const NfSlalomContext *context,
                                   size_t state,
                                   NfSlalomAnchor *out_anchor,
                                   NfSlalomHeading8 *out_heading,
                                   NfSlalomSpeedClass *out_class)
{
    size_t pose;
    size_t anchor_id;
    if (state >= context->state_count || out_anchor == NULL ||
        out_heading == NULL || out_class == NULL) {
        return false;
    }
    *out_class = (NfSlalomSpeedClass)(state % NF_SLALOM_SPEED_CLASS_COUNT);
    pose = state / NF_SLALOM_SPEED_CLASS_COUNT;
    *out_heading = (NfSlalomHeading8)(pose % 8U);
    anchor_id = pose / 8U;
    return nf_slalom_anchor_from_id(context->maze, anchor_id, out_anchor);
}

static const NfTurnSpec *nf_slalom_turn_spec(
    const NfSlalomPlannerConfig *config,
    NfSlalomActionKind kind)
{
    switch (kind) {
    case NF_SLALOM_ACTION_SMALL_90: return &config->small_90;
    case NF_SLALOM_ACTION_LARGE_90: return &config->large_90;
    case NF_SLALOM_ACTION_LARGE_180: return &config->large_180;
    case NF_SLALOM_ACTION_45_IN: return &config->turn_45_in;
    case NF_SLALOM_ACTION_45_OUT: return &config->turn_45_out;
    case NF_SLALOM_ACTION_V90: return &config->v_90;
    case NF_SLALOM_ACTION_135_IN: return &config->turn_135_in;
    case NF_SLALOM_ACTION_135_OUT: return &config->turn_135_out;
    default: return NULL;
    }
}

static const NfTurnSpec *nf_slalom_geometry_turn_spec(
    const NfSlalomPlannerConfig *config,
    NfSlalomActionKind kind)
{
    if (config == NULL ||
        (unsigned int)kind >= NF_SLALOM_ACTION_START_OFFSET) {
        return NULL;
    }
    return &config->geometry_turns[(size_t)kind];
}

static NfSlalomSpeedClass nf_slalom_speed_class(NfSlalomActionKind kind,
                                                NfRouteSide side)
{
    const unsigned int side_offset =
        (side == NF_ROUTE_SIDE_LEFT) ? 1U : 0U;
    return (NfSlalomSpeedClass)(1U + (2U * (unsigned int)kind) + side_offset);
}

static double nf_slalom_class_velocity(const NfSlalomContext *context,
                                       NfSlalomSpeedClass speed_class)
{
    unsigned int value;
    NfSlalomActionKind kind;
    if (speed_class == NF_SLALOM_SPEED_START) {
        return context->start_boundary_velocity_mm_s;
    }
    value = (unsigned int)speed_class - 1U;
    kind = (NfSlalomActionKind)(value / 2U);
    if ((unsigned int)kind >= NF_SLALOM_ACTION_START_OFFSET) {
        return -1.0;
    }
    return nf_slalom_turn_spec(context->config, kind)->velocity_mm_s;
}

static void nf_slalom_expected_closure(const NfSlalomPlannerConfig *config,
                                       NfSlalomActionKind kind,
                                       double *out_forward,
                                       double *out_lateral,
                                       double *out_angle)
{
    const double half = config->half_cell_mm;
    switch (kind) {
    case NF_SLALOM_ACTION_SMALL_90:
        *out_forward = half; *out_lateral = half; *out_angle = 90.0; break;
    case NF_SLALOM_ACTION_LARGE_90:
        *out_forward = 2.0 * half; *out_lateral = 2.0 * half;
        *out_angle = 90.0; break;
    case NF_SLALOM_ACTION_LARGE_180:
        *out_forward = 0.0; *out_lateral = 2.0 * half;
        *out_angle = 180.0; break;
    case NF_SLALOM_ACTION_45_IN:
        *out_forward = 2.0 * half; *out_lateral = half;
        *out_angle = 45.0; break;
    case NF_SLALOM_ACTION_45_OUT:
        *out_forward = 3.0 * half / NF_SLALOM_SQRT2;
        *out_lateral = half / NF_SLALOM_SQRT2;
        *out_angle = 45.0; break;
    case NF_SLALOM_ACTION_V90:
        *out_forward = half * NF_SLALOM_SQRT2;
        *out_lateral = half * NF_SLALOM_SQRT2;
        *out_angle = 90.0; break;
    case NF_SLALOM_ACTION_135_IN:
        *out_forward = half; *out_lateral = 2.0 * half;
        *out_angle = 135.0; break;
    case NF_SLALOM_ACTION_135_OUT:
        *out_forward = half / NF_SLALOM_SQRT2;
        *out_lateral = 3.0 * half / NF_SLALOM_SQRT2;
        *out_angle = 135.0; break;
    default:
        *out_forward = 0.0; *out_lateral = 0.0; *out_angle = 0.0; break;
    }
}

/*
 * Parameterize the exact-closing centre line over the calibrated turn
 * duration.  The smooth sin^2 speed correction preserves the configured
 * boundary velocity at both ends and integrates to the surrogate path
 * length.  This makes geometry, elapsed time, and reported goal velocity one
 * coherent PC model even when empirical timing parameters do not close in
 * the simplified raised-cosine kinematics.
 */
static bool nf_slalom_turn_time_map(const NfSlalomContext *context,
                                    NfSlalomActionKind kind,
                                    double geometry_fraction,
                                    double *out_elapsed_s)
{
    const NfTurnSpec *timing_spec =
        nf_slalom_turn_spec(context->config, kind);
    const NfTurnPlan *timing_plan =
        &context->turn_plans[(size_t)kind];
    const NfTurnPlan *geometry_plan =
        &context->geometry_turn_plans[(size_t)kind];
    const double duration_s = timing_plan->total_time_s;
    const double boundary_velocity = timing_spec->velocity_mm_s;
    const double average_velocity =
        geometry_plan->travel_distance_mm / duration_s;
    const double amplitude =
        2.0 * (average_velocity - boundary_velocity);
    double lower = 0.0;
    double upper = 1.0;

    if (out_elapsed_s == NULL || !isfinite(geometry_fraction) ||
        geometry_fraction < 0.0 || geometry_fraction > 1.0 ||
        !nf_slalom_finite_positive(duration_s) ||
        !nf_slalom_finite_positive(boundary_velocity) ||
        !nf_slalom_finite_positive(average_velocity) ||
        (amplitude < 0.0 && boundary_velocity + amplitude <= 0.0)) {
        return false;
    }
    if (geometry_fraction <= 0.0) {
        *out_elapsed_s = 0.0;
        return true;
    }
    if (geometry_fraction >= 1.0) {
        *out_elapsed_s = duration_s;
        return true;
    }
    for (unsigned int i = 0U; i < 64U; i++) {
        const double middle = 0.5 * (lower + upper);
        const double distance_mm = duration_s *
            ((boundary_velocity * middle) +
             (amplitude *
              ((0.5 * middle) -
               (sin(2.0 * NF_SLALOM_PI * middle) /
                (4.0 * NF_SLALOM_PI)))));
        if (distance_mm < geometry_fraction *
                                  geometry_plan->travel_distance_mm) {
            lower = middle;
        } else {
            upper = middle;
        }
    }
    *out_elapsed_s = duration_s * (0.5 * (lower + upper));
    return isfinite(*out_elapsed_s);
}

static double nf_slalom_turn_velocity_at_time(
    const NfSlalomContext *context,
    NfSlalomActionKind kind,
    double elapsed_s)
{
    const NfTurnSpec *timing_spec =
        nf_slalom_turn_spec(context->config, kind);
    const NfTurnPlan *timing_plan =
        &context->turn_plans[(size_t)kind];
    const NfTurnPlan *geometry_plan =
        &context->geometry_turn_plans[(size_t)kind];
    const double fraction = fmin(1.0, fmax(0.0,
        elapsed_s / timing_plan->total_time_s));
    const double average_velocity = geometry_plan->travel_distance_mm /
                                    timing_plan->total_time_s;
    const double amplitude =
        2.0 * (average_velocity - timing_spec->velocity_mm_s);

    {
        const double wave = sin(NF_SLALOM_PI * fraction);
        return timing_spec->velocity_mm_s +
               (amplitude * wave * wave);
    }
}

static NfSlalomPlanStatus nf_slalom_prepare_config(NfSlalomContext *context)
{
    NfLinearPlan linear_check;

    if (context == NULL || context->config == NULL ||
        !nf_slalom_finite_positive(context->config->half_cell_mm) ||
        !nf_slalom_finite_positive(
            context->config->diagonal_half_command_mm) ||
        !nf_slalom_finite_positive(context->config->start_offset_mm) ||
        context->config->start_offset_mm >
            context->config->half_cell_mm + NF_SLALOM_EPS ||
        !isfinite(context->config->start_velocity_mm_s) ||
        context->config->start_velocity_mm_s < 0.0 ||
        !isfinite(context->config->orthogonal_anchor_closure_tolerance_mm) ||
        context->config->orthogonal_anchor_closure_tolerance_mm < 0.0 ||
        context->config->orthogonal_anchor_closure_tolerance_mm >
            NF_SLALOM_MAX_ENDPOINT_CANONICALIZATION_MM ||
        !isfinite(context->config->diagonal_anchor_closure_tolerance_mm) ||
        context->config->diagonal_anchor_closure_tolerance_mm < 0.0 ||
        context->config->diagonal_anchor_closure_tolerance_mm >
            NF_SLALOM_MAX_ENDPOINT_CANONICALIZATION_MM ||
        !isfinite(context->config->heading_closure_tolerance_deg) ||
        context->config->heading_closure_tolerance_deg < 0.0 ||
        context->config->heading_closure_tolerance_deg >
            NF_SLALOM_MAX_HEADING_CANONICALIZATION_DEG ||
        !nf_slalom_finite_positive(
            context->config->max_turn_sample_step_mm) ||
        context->config->max_turn_sample_step_mm >
            0.25 * context->config->half_cell_mm ||
        (context->config->enabled_actions & ~NF_SLALOM_ENABLE_ALL) != 0U) {
        return NF_SLALOM_PLAN_INVALID_CONFIG;
    }
    if (context->config->require_swept_clearance) {
        /* Straight swept-volume checking is intentionally not implemented. */
        return NF_SLALOM_PLAN_UNSUPPORTED_SAFETY;
    }
    if (nf_motion_linear_plan(&context->config->orthogonal, 0.0,
                              context->config->start_velocity_mm_s,
                              context->config->start_velocity_mm_s,
                              &linear_check) != NF_MOTION_OK ||
        nf_motion_linear_plan(&context->config->diagonal, 0.0, 0.0, 0.0,
                              &linear_check) != NF_MOTION_OK ||
        context->config->start_velocity_mm_s >
            context->config->orthogonal.vmax_mm_s + NF_SLALOM_EPS) {
        return NF_SLALOM_PLAN_INVALID_CONFIG;
    }
    if (nf_motion_accelerating_exit_velocity(
            &context->config->orthogonal,
            context->config->start_offset_mm,
            context->config->start_velocity_mm_s,
            &context->start_boundary_velocity_mm_s) != NF_MOTION_OK ||
        nf_motion_linear_plan(&context->config->orthogonal,
                              context->config->start_offset_mm,
                              context->config->start_velocity_mm_s,
                              context->start_boundary_velocity_mm_s,
                              &context->start_plan) != NF_MOTION_OK) {
        return NF_SLALOM_PLAN_INVALID_CONFIG;
    }
    {
        const NfSlalomPlanStatus status = nf_slalom_seconds_to_us(
            context->start_plan.total_time_s, &context->start_time_us);
        if (status != NF_SLALOM_PLAN_OK) {
            return status;
        }
    }

    for (size_t i = 0U; i < NF_SLALOM_TURN_COUNT; i++) {
        const NfSlalomActionKind kind = (NfSlalomActionKind)i;
        const NfTurnSpec *timing_spec =
            nf_slalom_turn_spec(context->config, kind);
        const NfTurnSpec *geometry_spec =
            nf_slalom_geometry_turn_spec(context->config, kind);
        double expected_forward;
        double expected_lateral;
        double expected_angle;
        double residual;

        memset(&context->turn_plans[i], 0, sizeof(context->turn_plans[i]));
        memset(&context->geometry_turn_plans[i], 0,
               sizeof(context->geometry_turn_plans[i]));
        if ((context->config->enabled_actions & (1U << i)) == 0U) {
            continue;
        }
        if (timing_spec == NULL || geometry_spec == NULL ||
            !timing_spec->enabled || !geometry_spec->enabled ||
            !isfinite(timing_spec->velocity_mm_s) ||
            !isfinite(geometry_spec->velocity_mm_s) ||
            fabs(timing_spec->velocity_mm_s - geometry_spec->velocity_mm_s) >
                NF_SLALOM_EPS) {
            return NF_SLALOM_PLAN_INVALID_CONFIG;
        }
        if (nf_motion_turn_plan(timing_spec,
                                &context->config->turn_environment,
                                &context->turn_plans[i]) != NF_MOTION_OK) {
            return NF_SLALOM_PLAN_INVALID_CONFIG;
        }
        if (nf_motion_turn_plan(geometry_spec,
                                &context->config->turn_environment,
                                &context->geometry_turn_plans[i]) !=
            NF_MOTION_OK) {
            return NF_SLALOM_PLAN_INVALID_CONFIG;
        }
        nf_slalom_expected_closure(context->config, kind,
                                   &expected_forward, &expected_lateral,
                                   &expected_angle);
        residual = hypot(
            context->geometry_turn_plans[i].displacement_forward_mm -
                expected_forward,
            context->geometry_turn_plans[i].displacement_lateral_mm -
                expected_lateral);
        if (!isfinite(residual) ||
            residual >
                (((kind == NF_SLALOM_ACTION_SMALL_90 ||
                   kind == NF_SLALOM_ACTION_LARGE_90 ||
                   kind == NF_SLALOM_ACTION_LARGE_180) ?
                      context->config->orthogonal_anchor_closure_tolerance_mm :
                      context->config->diagonal_anchor_closure_tolerance_mm) +
                 NF_SLALOM_EPS) ||
            fabs(timing_spec->angle_deg - expected_angle) >
                context->config->heading_closure_tolerance_deg +
                    NF_SLALOM_EPS ||
            fabs(geometry_spec->angle_deg - expected_angle) >
                context->config->heading_closure_tolerance_deg +
                    NF_SLALOM_EPS) {
            return NF_SLALOM_PLAN_INVALID_CONFIG;
        }
    }
    for (size_t i = 0U; i < NF_SLALOM_TURN_COUNT; i++) {
        const NfSlalomActionKind kind = (NfSlalomActionKind)i;
        const NfTurnSpec *geometry_spec =
            nf_slalom_geometry_turn_spec(context->config, kind);
        double expected_forward;
        double expected_lateral;
        double expected_angle;
        double required_intervals;
        uint32_t intervals;
        NfTurnPose *poses;
        double *elapsed_s;

        if ((context->config->enabled_actions & (1U << i)) == 0U) {
            continue;
        }
        nf_slalom_expected_closure(context->config, kind,
                                   &expected_forward, &expected_lateral,
                                   &expected_angle);
        (void)expected_forward;
        (void)expected_lateral;
        required_intervals = fmax(
            context->geometry_turn_plans[i].travel_distance_mm /
                context->config->max_turn_sample_step_mm,
            expected_angle / 2.0);
        if (!isfinite(required_intervals) || required_intervals > 16384.0) {
            nf_slalom_free_turn_trajectories(context);
            return NF_SLALOM_PLAN_OVERFLOW;
        }
        intervals = (uint32_t)ceil(required_intervals);
        if (intervals < 2U) {
            intervals = 2U;
        }
        if ((size_t)intervals + 1U > SIZE_MAX / sizeof(*poses) ||
            (size_t)intervals + 1U > SIZE_MAX / sizeof(*elapsed_s)) {
            nf_slalom_free_turn_trajectories(context);
            return NF_SLALOM_PLAN_OVERFLOW;
        }
        poses = (NfTurnPose *)malloc(
            ((size_t)intervals + 1U) * sizeof(*poses));
        elapsed_s = (double *)malloc(
            ((size_t)intervals + 1U) * sizeof(*elapsed_s));
        if (poses == NULL || elapsed_s == NULL) {
            free(poses);
            free(elapsed_s);
            nf_slalom_free_turn_trajectories(context);
            return NF_SLALOM_PLAN_CAPACITY;
        }
        if (nf_motion_turn_pose_uniform(
                                        geometry_spec,
                                        &context->geometry_turn_plans[i],
                                        intervals, poses,
                                        (size_t)intervals + 1U) !=
            NF_MOTION_OK) {
            free(poses);
            free(elapsed_s);
            nf_slalom_free_turn_trajectories(context);
            return NF_SLALOM_PLAN_INVALID_CONFIG;
        }
        for (uint32_t sample = 0U; sample <= intervals; sample++) {
            if (!nf_slalom_turn_time_map(
                    context, kind, (double)sample / (double)intervals,
                    &elapsed_s[sample])) {
                free(poses);
                free(elapsed_s);
                nf_slalom_free_turn_trajectories(context);
                return NF_SLALOM_PLAN_INVALID_CONFIG;
            }
        }
        /* Canonicalize only the already-gated sub-0.001 mm endpoint. */
        poses[intervals].forward_mm = expected_forward;
        poses[intervals].lateral_mm = expected_lateral;
        poses[intervals].heading_deg = expected_angle;
        context->turn_trajectories[i].poses = poses;
        context->turn_trajectories[i].elapsed_s = elapsed_s;
        context->turn_trajectories[i].intervals = intervals;
    }
    return NF_SLALOM_PLAN_OK;
}

static bool nf_slalom_heap_less(const NfSlalomHeap *heap,
                                size_t left_state,
                                size_t right_state)
{
    const NfSlalomNode *left = &heap->nodes[left_state];
    const NfSlalomNode *right = &heap->nodes[right_state];
    if (left->distance_us != right->distance_us) {
        return left->distance_us < right->distance_us;
    }
    if (left->turn_count != right->turn_count) {
        return left->turn_count < right->turn_count;
    }
    return left_state < right_state;
}

static void nf_slalom_heap_swap(NfSlalomHeap *heap, size_t left, size_t right)
{
    const size_t state = heap->states[left];
    heap->states[left] = heap->states[right];
    heap->states[right] = state;
    heap->positions[heap->states[left]] = (ptrdiff_t)left;
    heap->positions[heap->states[right]] = (ptrdiff_t)right;
}

static bool nf_slalom_heap_push_or_decrease(NfSlalomHeap *heap, size_t state)
{
    size_t position;
    if (heap->positions[state] < 0) {
        if (heap->size >= heap->capacity) {
            return false;
        }
        position = heap->size++;
        heap->states[position] = state;
        heap->positions[state] = (ptrdiff_t)position;
    } else {
        position = (size_t)heap->positions[state];
    }
    while (position > 0U) {
        const size_t parent = (position - 1U) / 2U;
        if (!nf_slalom_heap_less(heap, heap->states[position],
                                 heap->states[parent])) {
            break;
        }
        nf_slalom_heap_swap(heap, position, parent);
        position = parent;
    }
    return true;
}

static bool nf_slalom_heap_pop(NfSlalomHeap *heap, size_t *out_state)
{
    size_t position = 0U;
    if (heap->size == 0U || out_state == NULL) {
        return false;
    }
    *out_state = heap->states[0];
    heap->positions[*out_state] = -1;
    heap->size--;
    if (heap->size == 0U) {
        return true;
    }
    heap->states[0] = heap->states[heap->size];
    heap->positions[heap->states[0]] = 0;
    while (1) {
        const size_t left = (2U * position) + 1U;
        const size_t right = left + 1U;
        size_t smallest = position;
        if (left < heap->size &&
            nf_slalom_heap_less(heap, heap->states[left],
                                heap->states[smallest])) {
            smallest = left;
        }
        if (right < heap->size &&
            nf_slalom_heap_less(heap, heap->states[right],
                                heap->states[smallest])) {
            smallest = right;
        }
        if (smallest == position) {
            break;
        }
        nf_slalom_heap_swap(heap, position, smallest);
        position = smallest;
    }
    return true;
}

static bool nf_slalom_turn_source_valid(const NfRouteMaze *maze,
                                        NfSlalomAnchor anchor,
                                        NfSlalomHeading8 heading,
                                        NfSlalomActionKind kind)
{
    if (!nf_slalom_pose_valid(maze, anchor, heading)) {
        return false;
    }
    switch (kind) {
    case NF_SLALOM_ACTION_SMALL_90:
        return !nf_slalom_anchor_is_center(anchor) &&
               nf_slalom_is_cardinal(heading);
    case NF_SLALOM_ACTION_LARGE_90:
    case NF_SLALOM_ACTION_LARGE_180:
    case NF_SLALOM_ACTION_45_IN:
    case NF_SLALOM_ACTION_135_IN:
        return nf_slalom_anchor_is_center(anchor) &&
               nf_slalom_is_cardinal(heading);
    case NF_SLALOM_ACTION_45_OUT:
    case NF_SLALOM_ACTION_V90:
    case NF_SLALOM_ACTION_135_OUT:
        return !nf_slalom_anchor_is_center(anchor) &&
               nf_slalom_is_diagonal(heading);
    default:
        return false;
    }
}

static bool nf_slalom_turn_destination(const NfRouteMaze *maze,
                                       NfSlalomAnchor source,
                                       NfSlalomHeading8 start_heading,
                                       NfSlalomActionKind kind,
                                       NfRouteSide side,
                                       NfSlalomAnchor *out_anchor,
                                       NfSlalomHeading8 *out_heading)
{
    const int sign = nf_slalom_side_sign(side);
    NfSlalomHeading8 end_heading;
    NfSlalomHeading8 intermediate;
    int dx = 0;
    int dy = 0;

    if (out_anchor == NULL || out_heading == NULL ||
        (side != NF_ROUTE_SIDE_RIGHT && side != NF_ROUTE_SIDE_LEFT) ||
        !nf_slalom_turn_source_valid(maze, source, start_heading, kind)) {
        return false;
    }
    switch (kind) {
    case NF_SLALOM_ACTION_SMALL_90:
        end_heading = nf_slalom_heading_add(start_heading, 2 * sign);
        dx = k_heading_dx[(unsigned int)start_heading] +
             k_heading_dx[(unsigned int)end_heading];
        dy = k_heading_dy[(unsigned int)start_heading] +
             k_heading_dy[(unsigned int)end_heading];
        break;
    case NF_SLALOM_ACTION_LARGE_90:
        end_heading = nf_slalom_heading_add(start_heading, 2 * sign);
        dx = 2 * (k_heading_dx[(unsigned int)start_heading] +
                  k_heading_dx[(unsigned int)end_heading]);
        dy = 2 * (k_heading_dy[(unsigned int)start_heading] +
                  k_heading_dy[(unsigned int)end_heading]);
        break;
    case NF_SLALOM_ACTION_LARGE_180:
        intermediate = nf_slalom_heading_add(start_heading, 2 * sign);
        end_heading = nf_slalom_heading_add(start_heading, 4 * sign);
        dx = 2 * k_heading_dx[(unsigned int)intermediate];
        dy = 2 * k_heading_dy[(unsigned int)intermediate];
        break;
    case NF_SLALOM_ACTION_45_IN:
    case NF_SLALOM_ACTION_45_OUT:
        end_heading = nf_slalom_heading_add(start_heading, sign);
        dx = k_heading_dx[(unsigned int)start_heading] +
             k_heading_dx[(unsigned int)end_heading];
        dy = k_heading_dy[(unsigned int)start_heading] +
             k_heading_dy[(unsigned int)end_heading];
        break;
    case NF_SLALOM_ACTION_V90:
        end_heading = nf_slalom_heading_add(start_heading, 2 * sign);
        dx = k_heading_dx[(unsigned int)start_heading] +
             k_heading_dx[(unsigned int)end_heading];
        dy = k_heading_dy[(unsigned int)start_heading] +
             k_heading_dy[(unsigned int)end_heading];
        break;
    case NF_SLALOM_ACTION_135_IN:
        intermediate = nf_slalom_heading_add(start_heading, sign);
        end_heading = nf_slalom_heading_add(start_heading, 3 * sign);
        dx = k_heading_dx[(unsigned int)start_heading] +
             k_heading_dx[(unsigned int)intermediate] +
             k_heading_dx[(unsigned int)end_heading];
        dy = k_heading_dy[(unsigned int)start_heading] +
             k_heading_dy[(unsigned int)intermediate] +
             k_heading_dy[(unsigned int)end_heading];
        break;
    case NF_SLALOM_ACTION_135_OUT:
        intermediate = nf_slalom_heading_add(start_heading, sign);
        end_heading = nf_slalom_heading_add(start_heading, 3 * sign);
        dx = (2 * k_heading_dx[(unsigned int)intermediate]) +
             k_heading_dx[(unsigned int)end_heading];
        dy = (2 * k_heading_dy[(unsigned int)intermediate]) +
             k_heading_dy[(unsigned int)end_heading];
        break;
    default:
        return false;
    }
    out_anchor->half_x = (int16_t)(source.half_x + dx);
    out_anchor->half_y = (int16_t)(source.half_y + dy);
    *out_heading = end_heading;
    return nf_slalom_pose_valid(maze, *out_anchor, *out_heading);
}

static void nf_slalom_sort_events(NfBoundaryEvent *events, size_t count)
{
    for (size_t i = 1U; i < count; i++) {
        NfBoundaryEvent value = events[i];
        size_t j = i;
        while (j > 0U && events[j - 1U].fraction > value.fraction) {
            events[j] = events[j - 1U];
            j--;
        }
        events[j] = value;
    }
}

static bool nf_slalom_trace_segment(const NfSlalomContext *context,
                                    double x0,
                                    double y0,
                                    double x1,
                                    double y1,
                                    double time0_s,
                                    double time1_s,
                                    double heading0_deg,
                                    double heading1_deg,
                                    int *cell_x,
                                    int *cell_y,
                                    NfTurnTrace *trace)
{
    NfBoundaryEvent events[(2U * NF_ROUTE_MAZE_MAX_SIZE) + 4U];
    size_t event_count = 0U;
    const double pitch = 2.0 * context->config->half_cell_mm;
    const double dx = x1 - x0;
    const double dy = y1 - y0;

    if (fabs(dx) > NF_SLALOM_EPS) {
        for (int line = 0; line <= (int)context->maze->width; line++) {
            const double fraction = (((double)line * pitch) - x0) / dx;
            if (fraction > NF_SLALOM_EPS &&
                fraction <= 1.0 + NF_SLALOM_EPS) {
                if (event_count >= sizeof(events) / sizeof(events[0])) {
                    return false;
                }
                events[event_count++] = (NfBoundaryEvent){
                    fmin(fraction, 1.0), true, line,
                };
            }
        }
    }
    if (fabs(dy) > NF_SLALOM_EPS) {
        for (int line = 0; line <= (int)context->maze->height; line++) {
            const double fraction = (((double)line * pitch) - y0) / dy;
            if (fraction > NF_SLALOM_EPS &&
                fraction <= 1.0 + NF_SLALOM_EPS) {
                if (event_count >= sizeof(events) / sizeof(events[0])) {
                    return false;
                }
                events[event_count++] = (NfBoundaryEvent){
                    fmin(fraction, 1.0), false, line,
                };
            }
        }
    }
    nf_slalom_sort_events(events, event_count);
    for (size_t i = 0U; i < event_count; i++) {
        int next_x = *cell_x;
        int next_y = *cell_y;
        const bool from_goal =
            context->maze->goals[*cell_y][*cell_x];
        const double fraction = events[i].fraction;

        if (i + 1U < event_count &&
            fabs(events[i + 1U].fraction - fraction) <= 1.0e-7 &&
            events[i + 1U].vertical != events[i].vertical) {
            /* A centre line through a maze post is never a valid primitive. */
            return false;
        }
        if (events[i].vertical) {
            if (dx > 0.0) {
                if (*cell_x == events[i].line) {
                    continue;
                }
                if (*cell_x != events[i].line - 1) {
                    return false;
                }
                next_x++;
            } else {
                if (*cell_x == events[i].line - 1) {
                    continue;
                }
                if (*cell_x != events[i].line) {
                    return false;
                }
                next_x--;
            }
        } else if (dy > 0.0) {
            if (*cell_y == events[i].line) {
                continue;
            }
            if (*cell_y != events[i].line - 1) {
                return false;
            }
            next_y++;
        } else {
            if (*cell_y == events[i].line - 1) {
                continue;
            }
            if (*cell_y != events[i].line) {
                return false;
            }
            next_y--;
        }
        if (!nf_slalom_cells_open(context->maze, *cell_x, *cell_y,
                                   next_x, next_y)) {
            return false;
        }
        if (!trace->has_goal && !from_goal &&
            context->maze->goals[next_y][next_x]) {
            trace->has_goal = true;
            trace->goal_x = (uint8_t)next_x;
            trace->goal_y = (uint8_t)next_y;
            trace->goal_time_s = time0_s +
                (fraction * (time1_s - time0_s));
            trace->goal_heading = nf_slalom_heading_from_math_deg(
                heading0_deg +
                (fraction * (heading1_deg - heading0_deg)));
        }
        *cell_x = next_x;
        *cell_y = next_y;
    }
    return true;
}

static NfSlalomPlanStatus nf_slalom_trace_turn(
    const NfSlalomContext *context,
    NfSlalomAnchor source,
    NfSlalomHeading8 start_heading,
    NfSlalomActionKind kind,
    NfRouteSide side,
    NfSlalomAnchor destination,
    NfSlalomHeading8 end_heading,
    NfTurnTrace *out)
{
    const NfTurnSpec *geometry_spec =
        nf_slalom_geometry_turn_spec(context->config, kind);
    const NfTurnTrajectoryCache *trajectory =
        &context->turn_trajectories[(size_t)kind];
    const double start_x = source.half_x * context->config->half_cell_mm;
    const double start_y = source.half_y * context->config->half_cell_mm;
    const double start_heading_deg =
        nf_slalom_heading_math_deg(start_heading);
    uint32_t intervals;
    int current_x;
    int current_y;
    double previous_x = start_x;
    double previous_y = start_y;
    double previous_heading_deg = start_heading_deg;
    double previous_time_s = 0.0;

    if (out == NULL || geometry_spec == NULL ||
        !nf_slalom_anchor_region(context->maze, source, start_heading,
                                 &current_x, &current_y)) {
        return NF_SLALOM_PLAN_INVALID_ARGUMENT;
    }
    memset(out, 0, sizeof(*out));
    out->feasible = true;
    if (context->maze->goals[current_y][current_x]) {
        out->has_goal = true;
        out->goal_x = (uint8_t)current_x;
        out->goal_y = (uint8_t)current_y;
        out->goal_heading = start_heading;
        out->goal_velocity_mm_s =
            nf_slalom_turn_velocity_at_time(context, kind, 0.0);
    }

    intervals = trajectory->intervals;
    if (intervals < 2U || trajectory->poses == NULL ||
        trajectory->elapsed_s == NULL) {
        return NF_SLALOM_PLAN_INVALID_CONFIG;
    }

    for (uint32_t i = 1U; i <= intervals; i++) {
        const double elapsed_s = trajectory->elapsed_s[i];
        const bool had_goal = out->has_goal;
        NfTurnPose pose = trajectory->poses[i];
        double signed_lateral;
        double signed_heading;
        double heading_rad;
        double x;
        double y;
        double global_heading;

        /*
         * The exact-closing centre line is sampled uniformly by arc length.
         * Cached elapsed times apply the duration-constrained sin^2 speed
         * profile, so spatial and temporal consumers use one trajectory.
         */
        signed_lateral = (side == NF_ROUTE_SIDE_LEFT) ?
            pose.lateral_mm : -pose.lateral_mm;
        signed_heading = (side == NF_ROUTE_SIDE_LEFT) ?
            pose.heading_deg : -pose.heading_deg;
        heading_rad = start_heading_deg * (NF_SLALOM_PI / 180.0);
        x = start_x + (pose.forward_mm * cos(heading_rad)) -
            (signed_lateral * sin(heading_rad));
        y = start_y + (pose.forward_mm * sin(heading_rad)) +
            (signed_lateral * cos(heading_rad));
        global_heading = start_heading_deg + signed_heading;
        if (!nf_slalom_trace_segment(context, previous_x, previous_y, x, y,
                                      previous_time_s, elapsed_s,
                                      previous_heading_deg, global_heading,
                                      &current_x, &current_y, out)) {
            out->feasible = false;
            return NF_SLALOM_PLAN_OK;
        }
        if (!had_goal && out->has_goal) {
            out->goal_velocity_mm_s = nf_slalom_turn_velocity_at_time(
                context, kind, out->goal_time_s);
        }
        previous_x = x;
        previous_y = y;
        previous_heading_deg = global_heading;
        previous_time_s = elapsed_s;
    }
    {
        int expected_x;
        int expected_y;
        if (!nf_slalom_anchor_region(context->maze, destination, end_heading,
                                     &expected_x, &expected_y) ||
            current_x != expected_x || current_y != expected_y) {
            out->feasible = false;
        }
    }
    return NF_SLALOM_PLAN_OK;
}

static double nf_slalom_connector_command_unit(
    const NfSlalomPlannerConfig *config,
    NfSlalomHeading8 heading)
{
    return nf_slalom_is_diagonal(heading) ?
        config->diagonal_half_command_mm : config->half_cell_mm;
}

static double nf_slalom_connector_geometry_unit(
    const NfSlalomPlannerConfig *config,
    NfSlalomHeading8 heading)
{
    return nf_slalom_is_diagonal(heading) ?
        config->half_cell_mm * NF_SLALOM_SQRT2 : config->half_cell_mm;
}

static const NfLinearLimits *nf_slalom_connector_limits(
    const NfSlalomPlannerConfig *config,
    NfSlalomHeading8 heading)
{
    return nf_slalom_is_diagonal(heading) ?
        &config->diagonal : &config->orthogonal;
}

static bool nf_slalom_scan_ray(const NfSlalomContext *context,
                               NfSlalomAnchor source,
                               NfSlalomHeading8 heading,
                               NfRayScan *out)
{
    NfSlalomAnchor current = source;
    int current_x;
    int current_y;
    uint16_t step = 0U;
    const uint16_t step_limit = (uint16_t)(
        (2U * ((uint16_t)context->maze->width +
               (uint16_t)context->maze->height)) + 4U);

    if (out == NULL ||
        !nf_slalom_anchor_region(context->maze, source, heading,
                                 &current_x, &current_y)) {
        return false;
    }
    memset(out, 0, sizeof(*out));
    out->stop_anchor = source;
    if (context->maze->goals[current_y][current_x]) {
        out->first_goal.present = true;
        out->first_goal.x = (uint8_t)current_x;
        out->first_goal.y = (uint8_t)current_y;
    }

    while (step < step_limit) {
        NfSlalomAnchor next;
        int next_x;
        int next_y;
        bool safe_stop;

        if (!nf_slalom_advance_connector(context->maze, current, heading,
                                          &next) ||
            !nf_slalom_anchor_region(context->maze, next, heading,
                                     &next_x, &next_y)) {
            break;
        }
        step++;
        if (!out->first_goal.present &&
            !context->maze->goals[current_y][current_x] &&
            context->maze->goals[next_y][next_x]) {
            out->first_goal.present = true;
            out->first_goal.step = step;
            out->first_goal.x = (uint8_t)next_x;
            out->first_goal.y = (uint8_t)next_y;
        }
        safe_stop = nf_slalom_is_diagonal(heading) ||
                    nf_slalom_anchor_is_center(next);
        if (safe_stop) {
            out->stop_steps = step;
            out->stop_anchor = next;
        }
        current = next;
        current_x = next_x;
        current_y = next_y;
    }
    return true;
}

static void nf_slalom_fill_straight_action(
    const NfSlalomContext *context,
    NfSlalomAction *action,
    NfSlalomAnchor source,
    NfSlalomAnchor destination,
    NfSlalomHeading8 heading,
    NfSlalomSpeedClass source_class,
    uint16_t steps,
    double entry_velocity,
    double exit_velocity,
    uint64_t duration_us)
{
    memset(action, 0, sizeof(*action));
    action->kind = NF_SLALOM_ACTION_GOAL_STOP;
    action->side = NF_ROUTE_SIDE_NONE;
    action->start_anchor = source;
    action->connector_end_anchor = destination;
    action->end_anchor = destination;
    action->start_heading = heading;
    action->end_heading = heading;
    action->start_speed_class = source_class;
    action->end_speed_class = NF_SLALOM_SPEED_START;
    action->connector_is_diagonal = nf_slalom_is_diagonal(heading);
    action->connector_steps = steps;
    action->connector_geometry_distance_mm =
        steps * nf_slalom_connector_geometry_unit(context->config, heading);
    action->connector_command_distance_mm =
        steps * nf_slalom_connector_command_unit(context->config, heading);
    action->entry_velocity_mm_s = entry_velocity;
    action->exit_velocity_mm_s = exit_velocity;
    action->connector_time_us = duration_us;
    action->duration_us = duration_us;
    action->required_open_checked = true;
}

static bool nf_slalom_goal_better(const NfGoalRecord *candidate,
                                  const NfGoalRecord *current)
{
    if (!current->valid) {
        return true;
    }
    if (candidate->goal_entry_us != current->goal_entry_us) {
        return candidate->goal_entry_us < current->goal_entry_us;
    }
    if (candidate->stop_us != current->stop_us) {
        return candidate->stop_us < current->stop_us;
    }
    if (candidate->turn_count != current->turn_count) {
        return candidate->turn_count < current->turn_count;
    }
    if (candidate->terminal_action.goal_y !=
        current->terminal_action.goal_y) {
        return candidate->terminal_action.goal_y <
               current->terminal_action.goal_y;
    }
    if (candidate->terminal_action.goal_x !=
        current->terminal_action.goal_x) {
        return candidate->terminal_action.goal_x <
               current->terminal_action.goal_x;
    }
    if (candidate->terminal_action.kind != current->terminal_action.kind) {
        return candidate->terminal_action.kind < current->terminal_action.kind;
    }
    if (candidate->terminal_action.side != current->terminal_action.side) {
        return candidate->terminal_action.side < current->terminal_action.side;
    }
    return candidate->source_state < current->source_state;
}

static NfSlalomPlanStatus nf_slalom_make_brake_action(
    const NfSlalomContext *context,
    NfSlalomAnchor source,
    NfSlalomHeading8 heading,
    NfSlalomSpeedClass source_class,
    double entry_velocity,
    bool *out_feasible,
    NfSlalomAction *out_action)
{
    NfRayScan ray;
    NfSlalomAnchor current;
    uint64_t best_duration_us = UINT64_MAX;
    const double unit =
        nf_slalom_connector_command_unit(context->config, heading);

    if (out_feasible == NULL || out_action == NULL ||
        !nf_slalom_scan_ray(context, source, heading, &ray)) {
        return NF_SLALOM_PLAN_INVALID_ARGUMENT;
    }
    *out_feasible = false;
    if (ray.stop_steps < context->config->minimum_post_goal_connector_steps ||
        ray.stop_steps == 0U) {
        return NF_SLALOM_PLAN_OK;
    }
    current = source;
    for (uint16_t step = 1U; step <= ray.stop_steps; step++) {
        NfSlalomAnchor next;
        NfLinearPlan linear;
        uint64_t duration_us;
        const bool minimum_tail_reached =
            step >= context->config->minimum_post_goal_connector_steps;

        if (!nf_slalom_advance_connector(context->maze, current, heading,
                                          &next)) {
            return NF_SLALOM_PLAN_INVALID_CONFIG;
        }
        current = next;
        if (!minimum_tail_reached ||
            (nf_slalom_is_cardinal(heading) &&
             !nf_slalom_anchor_is_center(current)) ||
            nf_motion_linear_plan(
                nf_slalom_connector_limits(context->config, heading),
                step * unit, entry_velocity, 0.0, &linear) != NF_MOTION_OK) {
            continue;
        }
        {
            const NfSlalomPlanStatus status =
                nf_slalom_seconds_to_us(linear.total_time_s, &duration_us);
            if (status != NF_SLALOM_PLAN_OK) {
                return status;
            }
        }
        if (duration_us < best_duration_us) {
            nf_slalom_fill_straight_action(
                context, out_action, source, current, heading, source_class,
                step, entry_velocity, 0.0, duration_us);
            best_duration_us = duration_us;
        }
    }
    if (best_duration_us != UINT64_MAX) {
        *out_feasible = true;
    }
    return NF_SLALOM_PLAN_OK;
}

static NfSlalomPlanStatus nf_slalom_consider_direct_goal(
    NfSlalomContext *context,
    size_t state,
    NfSlalomAnchor source,
    NfSlalomHeading8 heading,
    NfSlalomSpeedClass speed_class)
{
    NfRayScan ray;
    NfSlalomAnchor current;
    const double unit =
        nf_slalom_connector_command_unit(context->config, heading);
    const double entry_velocity =
        nf_slalom_class_velocity(context, speed_class);

    if (!nf_slalom_scan_ray(context, source, heading, &ray)) {
        return NF_SLALOM_PLAN_INVALID_ARGUMENT;
    }
    if (!ray.first_goal.present || ray.stop_steps == 0U ||
        ray.stop_steps < ray.first_goal.step ||
        (uint16_t)(ray.stop_steps - ray.first_goal.step) <
            context->config->minimum_post_goal_connector_steps) {
        return NF_SLALOM_PLAN_OK;
    }
    current = source;
    for (uint16_t step = 1U; step <= ray.stop_steps; step++) {
        NfSlalomAnchor next;
        NfLinearPlan linear;
        NfGoalRecord candidate;
        double cross_time_s;
        double cross_velocity;
        uint64_t cross_us;
        uint64_t duration_us;

        if (!nf_slalom_advance_connector(context->maze, current, heading,
                                          &next)) {
            return NF_SLALOM_PLAN_INVALID_CONFIG;
        }
        current = next;
        if (step < ray.first_goal.step ||
            (uint16_t)(step - ray.first_goal.step) <
                context->config->minimum_post_goal_connector_steps ||
            (nf_slalom_is_cardinal(heading) &&
             !nf_slalom_anchor_is_center(current)) ||
            nf_motion_linear_plan(
                nf_slalom_connector_limits(context->config, heading),
                step * unit, entry_velocity, 0.0, &linear) != NF_MOTION_OK ||
            nf_motion_linear_time_at_distance(
                &linear, ray.first_goal.step * unit,
                &cross_time_s, &cross_velocity) != NF_MOTION_OK) {
            continue;
        }
        {
            NfSlalomPlanStatus status =
                nf_slalom_seconds_to_us(cross_time_s, &cross_us);
            if (status != NF_SLALOM_PLAN_OK) {
                return status;
            }
            status = nf_slalom_seconds_to_us(linear.total_time_s,
                                              &duration_us);
            if (status != NF_SLALOM_PLAN_OK) {
                return status;
            }
        }
        memset(&candidate, 0, sizeof(candidate));
        candidate.valid = true;
        candidate.source_state = state;
        candidate.turn_count = context->nodes[state].turn_count;
        candidate.goal_entry_velocity_mm_s = cross_velocity;
        nf_slalom_fill_straight_action(
            context, &candidate.terminal_action, source, current, heading,
            speed_class, step, entry_velocity, 0.0, duration_us);
        candidate.terminal_action.has_goal_cross = true;
        candidate.terminal_action.goal_phase = NF_SLALOM_GOAL_CONNECTOR;
        candidate.terminal_action.goal_x = ray.first_goal.x;
        candidate.terminal_action.goal_y = ray.first_goal.y;
        candidate.terminal_action.goal_cross_heading = heading;
        candidate.terminal_action.goal_cross_time_us = cross_us;
        if (!nf_slalom_u64_add(context->nodes[state].distance_us, cross_us,
                               &candidate.goal_entry_us) ||
            !nf_slalom_u64_add(context->nodes[state].distance_us, duration_us,
                               &candidate.stop_us)) {
            return NF_SLALOM_PLAN_OVERFLOW;
        }
        if (nf_slalom_goal_better(&candidate, &context->goal)) {
            context->goal = candidate;
        }
    }
    return NF_SLALOM_PLAN_OK;
}

static NfSlalomPlanStatus nf_slalom_build_turn_edge(
    NfSlalomContext *context,
    size_t source_state,
    uint16_t connector_steps,
    NfSlalomActionKind kind,
    NfRouteSide side,
    NfBuiltEdge *out)
{
    NfSlalomAnchor source;
    NfSlalomAnchor connector_end;
    NfSlalomAnchor destination;
    NfSlalomHeading8 start_heading;
    NfSlalomHeading8 end_heading;
    NfSlalomSpeedClass source_class;
    NfSlalomSpeedClass destination_class;
    NfConnectorGoal connector_goal;
    NfTurnTrace turn_trace;
    NfLinearPlan connector_plan;
    const NfTurnSpec *turn;
    const NfLinearLimits *limits;
    double connector_distance;
    double entry_velocity;
    uint64_t connector_us;
    uint64_t turn_us;
    uint64_t duration_us;
    int current_x;
    int current_y;
    NfSlalomPlanStatus status;

    if (out == NULL || (unsigned int)kind >= NF_SLALOM_ACTION_START_OFFSET ||
        (side != NF_ROUTE_SIDE_RIGHT && side != NF_ROUTE_SIDE_LEFT) ||
        !nf_slalom_state_decode(context, source_state, &source,
                                &start_heading, &source_class)) {
        return NF_SLALOM_PLAN_INVALID_ARGUMENT;
    }
    memset(out, 0, sizeof(*out));
    memset(&connector_goal, 0, sizeof(connector_goal));
    connector_end = source;
    if (!nf_slalom_anchor_region(context->maze, source, start_heading,
                                 &current_x, &current_y)) {
        return NF_SLALOM_PLAN_INVALID_ARGUMENT;
    }
    if (context->maze->goals[current_y][current_x]) {
        connector_goal.present = true;
        connector_goal.x = (uint8_t)current_x;
        connector_goal.y = (uint8_t)current_y;
    }
    for (uint16_t step = 1U; step <= connector_steps; step++) {
        NfSlalomAnchor next;
        int next_x;
        int next_y;
        if (!nf_slalom_advance_connector(context->maze, connector_end,
                                          start_heading, &next) ||
            !nf_slalom_anchor_region(context->maze, next, start_heading,
                                     &next_x, &next_y)) {
            return NF_SLALOM_PLAN_OK;
        }
        if (!connector_goal.present &&
            !context->maze->goals[current_y][current_x] &&
            context->maze->goals[next_y][next_x]) {
            connector_goal.present = true;
            connector_goal.step = step;
            connector_goal.x = (uint8_t)next_x;
            connector_goal.y = (uint8_t)next_y;
        }
        connector_end = next;
        current_x = next_x;
        current_y = next_y;
    }
    if (connector_goal.present) {
        /*
         * Once a straight connector first enters a goal cell, only a
         * straight stopping extension is admissible.  Starting a new turn
         * after that crossing would optimize an unrequested post-goal
         * maneuver; direct-goal consideration handles the straight tail.
         */
        return NF_SLALOM_PLAN_OK;
    }
    if (!nf_slalom_turn_source_valid(context->maze, connector_end,
                                      start_heading, kind) ||
        !nf_slalom_turn_destination(context->maze, connector_end,
                                    start_heading, kind, side,
                                    &destination, &end_heading)) {
        return NF_SLALOM_PLAN_OK;
    }
    turn = nf_slalom_turn_spec(context->config, kind);
    limits = nf_slalom_connector_limits(context->config, start_heading);
    entry_velocity = nf_slalom_class_velocity(context, source_class);
    connector_distance = connector_steps *
        nf_slalom_connector_command_unit(context->config, start_heading);
    if (turn == NULL || turn->velocity_mm_s > limits->vmax_mm_s +
                                           NF_SLALOM_EPS ||
        nf_motion_linear_plan(limits, connector_distance, entry_velocity,
                              turn->velocity_mm_s,
                              &connector_plan) != NF_MOTION_OK) {
        return NF_SLALOM_PLAN_OK;
    }
    status = nf_slalom_seconds_to_us(connector_plan.total_time_s,
                                     &connector_us);
    if (status != NF_SLALOM_PLAN_OK) {
        return status;
    }
    status = nf_slalom_seconds_to_us(
        context->turn_plans[(size_t)kind].total_time_s, &turn_us);
    if (status != NF_SLALOM_PLAN_OK ||
        !nf_slalom_u64_add(connector_us, turn_us, &duration_us)) {
        return (status != NF_SLALOM_PLAN_OK) ? status :
            NF_SLALOM_PLAN_OVERFLOW;
    }
    status = nf_slalom_trace_turn(context, connector_end, start_heading,
                                  kind, side, destination, end_heading,
                                  &turn_trace);
    if (status != NF_SLALOM_PLAN_OK) {
        return status;
    }
    if (!turn_trace.feasible) {
        return NF_SLALOM_PLAN_OK;
    }

    destination_class = nf_slalom_speed_class(kind, side);
    memset(&out->action, 0, sizeof(out->action));
    out->action.kind = kind;
    out->action.side = side;
    out->action.start_anchor = source;
    out->action.connector_end_anchor = connector_end;
    out->action.end_anchor = destination;
    out->action.start_heading = start_heading;
    out->action.end_heading = end_heading;
    out->action.start_speed_class = source_class;
    out->action.end_speed_class = destination_class;
    out->action.connector_is_diagonal =
        nf_slalom_is_diagonal(start_heading);
    out->action.connector_steps = connector_steps;
    out->action.connector_geometry_distance_mm = connector_steps *
        nf_slalom_connector_geometry_unit(context->config, start_heading);
    out->action.connector_command_distance_mm = connector_distance;
    out->action.entry_velocity_mm_s = entry_velocity;
    out->action.turn_velocity_mm_s = turn->velocity_mm_s;
    out->action.exit_velocity_mm_s = turn->velocity_mm_s;
    out->action.connector_time_us = connector_us;
    out->action.turn_time_us = turn_us;
    out->action.duration_us = duration_us;
    out->action.turn_start_x_mm =
        connector_end.half_x * context->config->half_cell_mm;
    out->action.turn_start_y_mm =
        connector_end.half_y * context->config->half_cell_mm;
    out->action.turn_start_heading_deg =
        nf_slalom_heading_math_deg(start_heading);
    out->action.required_open_checked = true;
    out->action.swept_clearance_checked = false;
    out->feasible = true;

    if (turn_trace.has_goal) {
        uint64_t turn_cross_us;
        status = nf_slalom_seconds_to_us(turn_trace.goal_time_s,
                                         &turn_cross_us);
        if (status != NF_SLALOM_PLAN_OK ||
            !nf_slalom_u64_add(connector_us, turn_cross_us,
                               &out->action.goal_cross_time_us)) {
            return (status != NF_SLALOM_PLAN_OK) ? status :
                NF_SLALOM_PLAN_OVERFLOW;
        }
        out->action.has_goal_cross = true;
        out->action.goal_phase = NF_SLALOM_GOAL_TURN;
        out->action.goal_x = turn_trace.goal_x;
        out->action.goal_y = turn_trace.goal_y;
        out->action.goal_cross_heading = turn_trace.goal_heading;
        out->goal_entry_velocity_mm_s = turn_trace.goal_velocity_mm_s;
    }

    if (out->action.has_goal_cross) {
        bool brake_feasible;
        status = nf_slalom_make_brake_action(
            context, destination, end_heading, destination_class,
            turn->velocity_mm_s, &brake_feasible, &out->stop_action);
        if (status != NF_SLALOM_PLAN_OK) {
            return status;
        }
        if (!brake_feasible) {
            memset(out, 0, sizeof(*out));
            return NF_SLALOM_PLAN_OK;
        }
        out->has_stop_action = true;
        return NF_SLALOM_PLAN_OK;
    }

    out->destination_state = nf_slalom_state_index(
        context, destination, end_heading, destination_class);
    return NF_SLALOM_PLAN_OK;
}

static NfSlalomPlanStatus nf_slalom_consider_goal_edge(
    NfSlalomContext *context,
    size_t source_state,
    const NfBuiltEdge *edge)
{
    NfGoalRecord candidate;
    uint64_t edge_end_us;

    if (!edge->feasible || !edge->action.has_goal_cross ||
        !edge->has_stop_action) {
        return NF_SLALOM_PLAN_INVALID_ARGUMENT;
    }
    memset(&candidate, 0, sizeof(candidate));
    candidate.valid = true;
    candidate.source_state = source_state;
    candidate.terminal_action = edge->action;
    candidate.has_stop_action = true;
    candidate.stop_action = edge->stop_action;
    candidate.goal_entry_velocity_mm_s = edge->goal_entry_velocity_mm_s;
    candidate.turn_count = (uint16_t)(
        context->nodes[source_state].turn_count + 1U);
    if (!nf_slalom_u64_add(context->nodes[source_state].distance_us,
                           edge->action.goal_cross_time_us,
                           &candidate.goal_entry_us) ||
        !nf_slalom_u64_add(context->nodes[source_state].distance_us,
                           edge->action.duration_us, &edge_end_us) ||
        !nf_slalom_u64_add(edge_end_us, edge->stop_action.duration_us,
                           &candidate.stop_us)) {
        return NF_SLALOM_PLAN_OVERFLOW;
    }
    if (nf_slalom_goal_better(&candidate, &context->goal)) {
        context->goal = candidate;
    }
    return NF_SLALOM_PLAN_OK;
}

static NfSlalomPlanStatus nf_slalom_relax_edge(
    NfSlalomContext *context,
    size_t source_state,
    const NfBuiltEdge *edge)
{
    NfSlalomNode *destination;
    uint64_t distance_us;
    uint16_t turn_count;

    if (!edge->feasible || edge->action.has_goal_cross ||
        edge->destination_state >= context->state_count) {
        return NF_SLALOM_PLAN_INVALID_ARGUMENT;
    }
    if (!nf_slalom_u64_add(context->nodes[source_state].distance_us,
                           edge->action.duration_us, &distance_us) ||
        context->nodes[source_state].turn_count == UINT16_MAX) {
        return NF_SLALOM_PLAN_OVERFLOW;
    }
    turn_count = (uint16_t)(context->nodes[source_state].turn_count + 1U);
    destination = &context->nodes[edge->destination_state];
    if (distance_us > destination->distance_us ||
        (distance_us == destination->distance_us &&
         turn_count >= destination->turn_count)) {
        return NF_SLALOM_PLAN_OK;
    }
    destination->distance_us = distance_us;
    destination->turn_count = turn_count;
    context->parents[edge->destination_state] = (NfSlalomParent){
        true,
        source_state,
        edge->action.connector_steps,
        edge->action.kind,
        edge->action.side,
    };
    context->relaxed_edges++;
    if (!nf_slalom_heap_push_or_decrease(&context->heap,
                                          edge->destination_state)) {
        return NF_SLALOM_PLAN_CAPACITY;
    }
    return NF_SLALOM_PLAN_OK;
}

static bool nf_slalom_append_action(NfSlalomRoutePlan *plan,
                                    const NfSlalomAction *action)
{
    if (plan->action_count >= NF_SLALOM_MAX_ACTIONS) {
        return false;
    }
    plan->actions[plan->action_count++] = *action;
    return true;
}

static NfSlalomPlanStatus nf_slalom_reconstruct(
    NfSlalomContext *context,
    size_t start_state,
    const NfSlalomPlannerRequest *request,
    NfSlalomRoutePlan *out)
{
    size_t *chain;
    size_t chain_count = 0U;
    size_t current;
    NfSlalomAction start_action;

    if (!context->goal.valid) {
        return NF_SLALOM_PLAN_NO_PATH;
    }
    chain = (size_t *)malloc(context->state_count * sizeof(*chain));
    if (chain == NULL) {
        return NF_SLALOM_PLAN_CAPACITY;
    }
    current = context->goal.source_state;
    while (current != start_state) {
        if (chain_count >= context->state_count ||
            !context->parents[current].valid) {
            free(chain);
            return NF_SLALOM_PLAN_INVALID_ARGUMENT;
        }
        chain[chain_count++] = current;
        current = context->parents[current].previous_state;
    }

    memset(&start_action, 0, sizeof(start_action));
    start_action.kind = NF_SLALOM_ACTION_START_OFFSET;
    start_action.side = NF_ROUTE_SIDE_NONE;
    start_action.start_anchor = (NfSlalomAnchor){
        (int16_t)((2U * request->start_x) + 1U),
        (int16_t)((2U * request->start_y) + 1U),
    };
    start_action.connector_end_anchor = start_action.start_anchor;
    start_action.end_anchor = start_action.start_anchor;
    start_action.start_heading = request->start_heading;
    start_action.end_heading = request->start_heading;
    start_action.start_speed_class = NF_SLALOM_SPEED_START;
    start_action.end_speed_class = NF_SLALOM_SPEED_START;
    start_action.connector_geometry_distance_mm =
        context->config->start_offset_mm;
    start_action.connector_command_distance_mm =
        context->config->start_offset_mm;
    start_action.entry_velocity_mm_s =
        context->config->start_velocity_mm_s;
    start_action.exit_velocity_mm_s =
        context->start_boundary_velocity_mm_s;
    start_action.connector_time_us = context->start_time_us;
    start_action.duration_us = context->start_time_us;
    start_action.turn_start_heading_deg =
        nf_slalom_heading_math_deg(request->start_heading);
    {
        const double radians = start_action.turn_start_heading_deg *
                               (NF_SLALOM_PI / 180.0);
        const double center_x = start_action.start_anchor.half_x *
                                context->config->half_cell_mm;
        const double center_y = start_action.start_anchor.half_y *
                                context->config->half_cell_mm;
        start_action.turn_start_x_mm = center_x -
            (context->config->start_offset_mm * cos(radians));
        start_action.turn_start_y_mm = center_y -
            (context->config->start_offset_mm * sin(radians));
    }
    start_action.required_open_checked = true;
    if (!nf_slalom_append_action(out, &start_action)) {
        free(chain);
        return NF_SLALOM_PLAN_CAPACITY;
    }

    for (size_t reverse = chain_count; reverse > 0U; reverse--) {
        const size_t child = chain[reverse - 1U];
        const NfSlalomParent *parent = &context->parents[child];
        NfBuiltEdge edge;
        const NfSlalomPlanStatus status = nf_slalom_build_turn_edge(
            context, parent->previous_state, parent->connector_steps,
            parent->kind, parent->side, &edge);
        if (status != NF_SLALOM_PLAN_OK || !edge.feasible ||
            edge.action.has_goal_cross || edge.destination_state != child) {
            free(chain);
            return (status != NF_SLALOM_PLAN_OK) ? status :
                NF_SLALOM_PLAN_INVALID_ARGUMENT;
        }
        if (!nf_slalom_append_action(out, &edge.action)) {
            free(chain);
            return NF_SLALOM_PLAN_CAPACITY;
        }
    }
    free(chain);

    if (!nf_slalom_append_action(out, &context->goal.terminal_action) ||
        (context->goal.has_stop_action &&
         !nf_slalom_append_action(out, &context->goal.stop_action))) {
        return NF_SLALOM_PLAN_CAPACITY;
    }
    out->goal_x = context->goal.terminal_action.goal_x;
    out->goal_y = context->goal.terminal_action.goal_y;
    out->goal_heading =
        context->goal.terminal_action.goal_cross_heading;
    out->goal_entry_us = context->goal.goal_entry_us;
    out->stop_us = context->goal.stop_us;
    out->goal_entry_velocity_mm_s =
        context->goal.goal_entry_velocity_mm_s;
    out->anchor_closure_validated = true;
    out->required_open_validated = true;
    out->swept_clearance_validated = false;
    out->expanded_states = context->expanded_states;
    out->relaxed_edges = context->relaxed_edges;
    return NF_SLALOM_PLAN_OK;
}

static NfSlalomPlanStatus nf_slalom_postcheck_clearance(
    const NfSlalomContext *context,
    NfSlalomRoutePlan *plan)
{
    if (!context->config->check_swept_clearance) {
        return NF_SLALOM_PLAN_OK;
    }
    for (size_t i = 0U; i < plan->action_count; i++) {
        NfSlalomAction *action = &plan->actions[i];
        const NfTurnSpec *turn;
        const NfTurnPlan *turn_plan;
        NfClearanceResult result;
        NfClearanceStatus status;

        if ((unsigned int)action->kind >= NF_SLALOM_ACTION_START_OFFSET) {
            continue;
        }
        turn = nf_slalom_geometry_turn_spec(context->config, action->kind);
        turn_plan = &context->geometry_turn_plans[(size_t)action->kind];
        status = nf_route_turn_clearance(
            context->maze, &context->config->clearance, turn, turn_plan,
            action->turn_start_x_mm, action->turn_start_y_mm,
            action->turn_start_heading_deg,
            action->side == NF_ROUTE_SIDE_LEFT, &result);
        if (status == NF_CLEARANCE_COLLISION) {
            plan->swept_clearance_validated = false;
            /*
             * Dijkstra was required-open optimal.  Returning this explicit
             * status avoids pretending that an alternative clearance-safe
             * route was searched after the selected route collided.
             */
            return NF_SLALOM_PLAN_UNSUPPORTED_SAFETY;
        }
        if (status == NF_CLEARANCE_OVERFLOW) {
            return NF_SLALOM_PLAN_OVERFLOW;
        }
        if (status != NF_CLEARANCE_OK || !result.clear) {
            return NF_SLALOM_PLAN_INVALID_CONFIG;
        }
        action->swept_clearance_checked = true;
    }
    plan->swept_clearance_validated = true;
    return NF_SLALOM_PLAN_OK;
}

NfSlalomPlanStatus nf_slalom_time_plan(
    const NfRouteMaze *maze,
    const NfSlalomPlannerConfig *config,
    const NfSlalomPlannerRequest *request,
    NfSlalomRoutePlan *out)
{
    NfSlalomContext context;
    NfSlalomAnchor start_anchor;
    size_t start_state = 0U;
    NfSlalomPlanStatus result;

    if (maze == NULL || config == NULL || request == NULL || out == NULL ||
        request->start_x >= maze->width || request->start_y >= maze->height ||
        (unsigned int)request->start_heading >= 8U ||
        !nf_slalom_is_cardinal(request->start_heading)) {
        return NF_SLALOM_PLAN_INVALID_ARGUMENT;
    }
    memset(out, 0, sizeof(*out));
    if (!nf_slalom_valid_maze(maze)) {
        return NF_SLALOM_PLAN_INVALID_MAZE;
    }
    memset(&context, 0, sizeof(context));
    context.maze = maze;
    context.config = config;
    result = nf_slalom_prepare_config(&context);
    if (result != NF_SLALOM_PLAN_OK) {
        nf_slalom_free_turn_trajectories(&context);
        return result;
    }
    if (maze->goals[request->start_y][request->start_x]) {
        out->goal_x = request->start_x;
        out->goal_y = request->start_y;
        out->goal_heading = request->start_heading;
        out->anchor_closure_validated = true;
        out->required_open_validated = true;
        out->swept_clearance_validated = config->check_swept_clearance;
        nf_slalom_free_turn_trajectories(&context);
        return NF_SLALOM_PLAN_OK;
    }

    context.anchor_count = nf_slalom_anchor_count(maze);
    if (context.anchor_count > SIZE_MAX / 8U ||
        context.anchor_count * 8U >
            SIZE_MAX / NF_SLALOM_SPEED_CLASS_COUNT) {
        nf_slalom_free_turn_trajectories(&context);
        return NF_SLALOM_PLAN_OVERFLOW;
    }
    context.state_count = context.anchor_count * 8U *
                          NF_SLALOM_SPEED_CLASS_COUNT;
    if (context.state_count == 0U ||
        context.state_count > SIZE_MAX / sizeof(*context.nodes) ||
        context.state_count > SIZE_MAX / sizeof(*context.parents) ||
        context.state_count > SIZE_MAX / sizeof(*context.heap.states) ||
        context.state_count > SIZE_MAX / sizeof(*context.heap.positions)) {
        nf_slalom_free_turn_trajectories(&context);
        return NF_SLALOM_PLAN_OVERFLOW;
    }
    context.nodes = (NfSlalomNode *)malloc(
        context.state_count * sizeof(*context.nodes));
    context.parents = (NfSlalomParent *)calloc(
        context.state_count, sizeof(*context.parents));
    context.heap.states = (size_t *)malloc(
        context.state_count * sizeof(*context.heap.states));
    context.heap.positions = (ptrdiff_t *)malloc(
        context.state_count * sizeof(*context.heap.positions));
    if (context.nodes == NULL || context.parents == NULL ||
        context.heap.states == NULL || context.heap.positions == NULL) {
        result = NF_SLALOM_PLAN_CAPACITY;
        goto cleanup;
    }
    context.heap.capacity = context.state_count;
    context.heap.nodes = context.nodes;
    for (size_t i = 0U; i < context.state_count; i++) {
        context.nodes[i].distance_us = NF_SLALOM_INF;
        context.nodes[i].turn_count = UINT16_MAX;
        context.nodes[i].settled = false;
        context.heap.positions[i] = -1;
    }
    start_anchor = (NfSlalomAnchor){
        (int16_t)((2U * request->start_x) + 1U),
        (int16_t)((2U * request->start_y) + 1U),
    };
    start_state = nf_slalom_state_index(&context, start_anchor,
                                        request->start_heading,
                                        NF_SLALOM_SPEED_START);
    context.nodes[start_state].distance_us = context.start_time_us;
    context.nodes[start_state].turn_count = 0U;
    if (!nf_slalom_heap_push_or_decrease(&context.heap, start_state)) {
        result = NF_SLALOM_PLAN_CAPACITY;
        goto cleanup;
    }

    while (context.heap.size > 0U) {
        size_t state;
        NfSlalomAnchor anchor;
        NfSlalomHeading8 heading;
        NfSlalomSpeedClass speed_class;
        NfSlalomAnchor connector_anchor;
        uint16_t connector_steps = 0U;
        const uint16_t step_limit = (uint16_t)(
            (2U * ((uint16_t)maze->width + (uint16_t)maze->height)) + 4U);

        (void)nf_slalom_heap_pop(&context.heap, &state);
        if (context.nodes[state].settled) {
            continue;
        }
        if (context.goal.valid &&
            context.nodes[state].distance_us > context.goal.goal_entry_us) {
            break;
        }
        context.nodes[state].settled = true;
        context.expanded_states++;
        if (!nf_slalom_state_decode(&context, state, &anchor, &heading,
                                    &speed_class) ||
            !nf_slalom_pose_valid(maze, anchor, heading)) {
            result = NF_SLALOM_PLAN_INVALID_ARGUMENT;
            goto cleanup;
        }
        result = nf_slalom_consider_direct_goal(
            &context, state, anchor, heading, speed_class);
        if (result != NF_SLALOM_PLAN_OK) {
            goto cleanup;
        }

        connector_anchor = anchor;
        while (connector_steps <= step_limit) {
            for (size_t kind_index = 0U;
                 kind_index < NF_SLALOM_TURN_COUNT; kind_index++) {
                const NfSlalomActionKind kind =
                    (NfSlalomActionKind)kind_index;
                static const NfRouteSide sides[2] = {
                    NF_ROUTE_SIDE_RIGHT,
                    NF_ROUTE_SIDE_LEFT,
                };

                if ((config->enabled_actions & (1U << kind_index)) == 0U ||
                    !nf_slalom_turn_source_valid(maze, connector_anchor,
                                                  heading, kind)) {
                    continue;
                }
                for (size_t side_index = 0U; side_index < 2U; side_index++) {
                    NfBuiltEdge edge;
                    result = nf_slalom_build_turn_edge(
                        &context, state, connector_steps, kind,
                        sides[side_index], &edge);
                    if (result != NF_SLALOM_PLAN_OK) {
                        goto cleanup;
                    }
                    if (!edge.feasible) {
                        continue;
                    }
                    if (edge.action.has_goal_cross) {
                        result = nf_slalom_consider_goal_edge(
                            &context, state, &edge);
                    } else {
                        result = nf_slalom_relax_edge(
                            &context, state, &edge);
                    }
                    if (result != NF_SLALOM_PLAN_OK) {
                        goto cleanup;
                    }
                }
            }
            if (connector_steps == step_limit ||
                !nf_slalom_advance_connector(maze, connector_anchor, heading,
                                              &connector_anchor)) {
                break;
            }
            connector_steps++;
        }
    }

    if (!context.goal.valid) {
        result = NF_SLALOM_PLAN_NO_PATH;
        goto cleanup;
    }
    result = nf_slalom_reconstruct(&context, start_state, request, out);
    if (result == NF_SLALOM_PLAN_OK) {
        result = nf_slalom_postcheck_clearance(&context, out);
    }

cleanup:
    nf_slalom_free_turn_trajectories(&context);
    free(context.nodes);
    free(context.parents);
    free(context.heap.states);
    free(context.heap.positions);
    return result;
}

NfSlalomPlanStatus nf_slalom_primitive_check(
    const NfRouteMaze *maze,
    const NfSlalomPlannerConfig *config,
    NfSlalomAnchor source,
    NfSlalomHeading8 start_heading,
    NfSlalomActionKind kind,
    NfRouteSide side,
    NfSlalomPrimitiveCheck *out)
{
    NfSlalomContext context;
    NfSlalomAnchor destination;
    NfSlalomHeading8 end_heading;
    NfTurnTrace trace;
    NfSlalomPlanStatus status;

    if (maze == NULL || config == NULL || out == NULL ||
        (unsigned int)kind >= NF_SLALOM_ACTION_START_OFFSET ||
        (side != NF_ROUTE_SIDE_RIGHT && side != NF_ROUTE_SIDE_LEFT) ||
        (config->enabled_actions & (1U << (unsigned int)kind)) == 0U) {
        return NF_SLALOM_PLAN_INVALID_ARGUMENT;
    }
    memset(out, 0, sizeof(*out));
    if (!nf_slalom_valid_maze(maze)) {
        return NF_SLALOM_PLAN_INVALID_MAZE;
    }
    memset(&context, 0, sizeof(context));
    context.maze = maze;
    context.config = config;
    status = nf_slalom_prepare_config(&context);
    if (status != NF_SLALOM_PLAN_OK) {
        nf_slalom_free_turn_trajectories(&context);
        return status;
    }
    if (!nf_slalom_turn_destination(maze, source, start_heading, kind, side,
                                    &destination, &end_heading)) {
        nf_slalom_free_turn_trajectories(&context);
        return NF_SLALOM_PLAN_OK;
    }
    status = nf_slalom_trace_turn(&context, source, start_heading, kind, side,
                                  destination, end_heading, &trace);
    if (status == NF_SLALOM_PLAN_OK && trace.feasible) {
        uint64_t turn_us;
        status = nf_slalom_seconds_to_us(
            context.turn_plans[(size_t)kind].total_time_s, &turn_us);
        if (status == NF_SLALOM_PLAN_OK) {
            out->feasible = true;
            out->destination_anchor = destination;
            out->destination_heading = end_heading;
            out->turn_time_us = turn_us;
            out->required_open_checked = true;
        }
    }
    nf_slalom_free_turn_trajectories(&context);
    return status;
}

static bool nf_slalom_validation_fail(NfSlalomValidation *validation,
                                      size_t action_index,
                                      const char *message)
{
    if (validation != NULL) {
        validation->valid = false;
        validation->action_index = action_index;
        (void)snprintf(validation->message, sizeof(validation->message),
                       "%s", message);
    }
    return false;
}

static bool nf_slalom_close_double(double left, double right)
{
    return isfinite(left) && isfinite(right) &&
           fabs(left - right) <= 1.0e-6 *
               fmax(1.0, fmax(fabs(left), fabs(right)));
}

static bool nf_slalom_same_anchor(NfSlalomAnchor left,
                                  NfSlalomAnchor right)
{
    return left.half_x == right.half_x && left.half_y == right.half_y;
}

bool nf_slalom_route_validate(
    const NfRouteMaze *maze,
    const NfSlalomPlannerConfig *config,
    const NfSlalomPlannerRequest *request,
    const NfSlalomRoutePlan *plan,
    NfSlalomValidation *validation)
{
    NfSlalomContext context;
    NfSlalomAnchor current_anchor;
    NfSlalomHeading8 current_heading;
    NfSlalomSpeedClass current_class = NF_SLALOM_SPEED_START;
    double current_velocity;
    uint64_t elapsed_us;
    uint64_t goal_entry_us = 0U;
    double goal_velocity = 0.0;
    uint8_t goal_x = 0U;
    uint8_t goal_y = 0U;
    NfSlalomHeading8 goal_heading = NF_SLALOM_HEADING_NORTH;
    bool goal_seen = false;
    bool valid = false;

    if (validation != NULL) {
        memset(validation, 0, sizeof(*validation));
    }
    if (maze == NULL || config == NULL || request == NULL || plan == NULL ||
        !nf_slalom_valid_maze(maze) ||
        request->start_x >= maze->width || request->start_y >= maze->height ||
        (unsigned int)request->start_heading >= 8U ||
        !nf_slalom_is_cardinal(request->start_heading)) {
        return nf_slalom_validation_fail(validation, 0U,
                                          "invalid replay arguments");
    }
    if (plan->action_count > NF_SLALOM_MAX_ACTIONS) {
        return nf_slalom_validation_fail(validation, 0U,
                                          "action count exceeds capacity");
    }

    memset(&context, 0, sizeof(context));
    context.maze = maze;
    context.config = config;
    if (nf_slalom_prepare_config(&context) != NF_SLALOM_PLAN_OK) {
        nf_slalom_free_turn_trajectories(&context);
        return nf_slalom_validation_fail(validation, 0U,
                                          "config replay failed");
    }
    if (plan->action_count == 0U) {
        if (maze->goals[request->start_y][request->start_x] &&
            plan->goal_entry_us == 0U && plan->stop_us == 0U &&
            plan->goal_x == request->start_x &&
            plan->goal_y == request->start_y &&
            plan->goal_heading == request->start_heading &&
            nf_slalom_close_double(plan->goal_entry_velocity_mm_s, 0.0) &&
            plan->anchor_closure_validated &&
            plan->required_open_validated &&
            plan->swept_clearance_validated ==
                config->check_swept_clearance) {
            nf_slalom_free_turn_trajectories(&context);
            if (validation != NULL) {
                validation->valid = true;
                (void)snprintf(validation->message,
                               sizeof(validation->message), "ok");
            }
            return true;
        }
        nf_slalom_free_turn_trajectories(&context);
        return nf_slalom_validation_fail(validation, 0U,
                                          "empty non-goal plan");
    }
    current_anchor = (NfSlalomAnchor){
        (int16_t)((2U * request->start_x) + 1U),
        (int16_t)((2U * request->start_y) + 1U),
    };
    current_heading = request->start_heading;
    current_velocity = context.start_boundary_velocity_mm_s;
    elapsed_us = context.start_time_us;

    {
        const NfSlalomAction *start = &plan->actions[0];
        const double start_heading_deg =
            nf_slalom_heading_math_deg(current_heading);
        const double start_heading_rad =
            start_heading_deg * (NF_SLALOM_PI / 180.0);
        const double center_x_mm =
            current_anchor.half_x * config->half_cell_mm;
        const double center_y_mm =
            current_anchor.half_y * config->half_cell_mm;
        const double physical_start_x_mm = center_x_mm -
            (config->start_offset_mm * cos(start_heading_rad));
        const double physical_start_y_mm = center_y_mm -
            (config->start_offset_mm * sin(start_heading_rad));

        if (start->kind != NF_SLALOM_ACTION_START_OFFSET ||
            start->side != NF_ROUTE_SIDE_NONE ||
            !nf_slalom_same_anchor(start->start_anchor, current_anchor) ||
            !nf_slalom_same_anchor(start->connector_end_anchor,
                                    current_anchor) ||
            !nf_slalom_same_anchor(start->end_anchor, current_anchor) ||
            start->start_heading != current_heading ||
            start->end_heading != current_heading ||
            start->start_speed_class != NF_SLALOM_SPEED_START ||
            start->end_speed_class != NF_SLALOM_SPEED_START ||
            start->connector_is_diagonal || start->connector_steps != 0U ||
            !nf_slalom_close_double(start->connector_geometry_distance_mm,
                                     config->start_offset_mm) ||
            !nf_slalom_close_double(start->connector_command_distance_mm,
                                     config->start_offset_mm) ||
            start->connector_time_us != context.start_time_us ||
            start->turn_time_us != 0U ||
            start->duration_us != context.start_time_us ||
            !nf_slalom_close_double(start->entry_velocity_mm_s,
                                     config->start_velocity_mm_s) ||
            !nf_slalom_close_double(start->turn_velocity_mm_s, 0.0) ||
            !nf_slalom_close_double(start->exit_velocity_mm_s,
                                     current_velocity) ||
            !nf_slalom_close_double(start->turn_start_x_mm,
                                     physical_start_x_mm) ||
            !nf_slalom_close_double(start->turn_start_y_mm,
                                     physical_start_y_mm) ||
            !nf_slalom_close_double(start->turn_start_heading_deg,
                                     start_heading_deg) ||
            !start->required_open_checked || start->swept_clearance_checked ||
            start->has_goal_cross ||
            start->goal_phase != NF_SLALOM_GOAL_NONE) {
            nf_slalom_free_turn_trajectories(&context);
            return nf_slalom_validation_fail(validation, 0U,
                                              "invalid start offset action");
        }
    }

    for (size_t index = 1U; index < plan->action_count; index++) {
        const NfSlalomAction *action = &plan->actions[index];
        NfSlalomAnchor connector_end = current_anchor;
        int current_x;
        int current_y;
        NfConnectorGoal connector_goal;

        memset(&connector_goal, 0, sizeof(connector_goal));
        if ((goal_seen && action->kind != NF_SLALOM_ACTION_GOAL_STOP) ||
            !nf_slalom_same_anchor(action->start_anchor, current_anchor) ||
            action->start_heading != current_heading ||
            action->start_speed_class != current_class ||
            !nf_slalom_close_double(action->entry_velocity_mm_s,
                                     current_velocity) ||
            !nf_slalom_anchor_region(maze, current_anchor, current_heading,
                                     &current_x, &current_y)) {
            nf_slalom_free_turn_trajectories(&context);
            return nf_slalom_validation_fail(validation, index,
                                              "action does not join replay state");
        }
        for (uint16_t step = 1U; step <= action->connector_steps; step++) {
            NfSlalomAnchor next;
            int next_x;
            int next_y;
            if (!nf_slalom_advance_connector(maze, connector_end,
                                              current_heading, &next) ||
                !nf_slalom_anchor_region(maze, next, current_heading,
                                         &next_x, &next_y)) {
                nf_slalom_free_turn_trajectories(&context);
                return nf_slalom_validation_fail(validation, index,
                                                  "connector crosses a wall");
            }
            if (!goal_seen && !connector_goal.present &&
                !maze->goals[current_y][current_x] &&
                maze->goals[next_y][next_x]) {
                connector_goal.present = true;
                connector_goal.step = step;
                connector_goal.x = (uint8_t)next_x;
                connector_goal.y = (uint8_t)next_y;
            }
            connector_end = next;
            current_x = next_x;
            current_y = next_y;
        }
        if (!nf_slalom_same_anchor(action->connector_end_anchor,
                                    connector_end) ||
            action->connector_is_diagonal !=
                nf_slalom_is_diagonal(current_heading) ||
            !nf_slalom_close_double(
                action->connector_command_distance_mm,
                action->connector_steps * nf_slalom_connector_command_unit(
                    config, current_heading)) ||
            !nf_slalom_close_double(
                action->connector_geometry_distance_mm,
                action->connector_steps * nf_slalom_connector_geometry_unit(
                    config, current_heading))) {
            nf_slalom_free_turn_trajectories(&context);
            return nf_slalom_validation_fail(validation, index,
                                              "connector metadata mismatch");
        }

        if ((unsigned int)action->kind < NF_SLALOM_ACTION_START_OFFSET) {
            const NfTurnSpec *turn = nf_slalom_turn_spec(config, action->kind);
            const NfLinearLimits *limits =
                nf_slalom_connector_limits(config, current_heading);
            NfLinearPlan connector_plan;
            NfSlalomAnchor destination;
            NfSlalomHeading8 end_heading;
            NfTurnTrace trace;
            uint64_t connector_us;
            uint64_t turn_us;
            uint64_t duration_us;
            bool expected_goal = false;
            NfSlalomGoalPhase expected_phase = NF_SLALOM_GOAL_NONE;
            uint64_t cross_us = 0U;
            double cross_velocity = 0.0;
            uint8_t cross_x = 0U;
            uint8_t cross_y = 0U;
            NfSlalomHeading8 cross_heading = current_heading;

            if (connector_goal.present ||
                (config->enabled_actions &
                 (1U << (unsigned int)action->kind)) == 0U ||
                !nf_slalom_turn_destination(maze, connector_end,
                                            current_heading, action->kind,
                                            action->side, &destination,
                                            &end_heading) ||
                nf_motion_linear_plan(
                    limits, action->connector_command_distance_mm,
                    current_velocity, turn->velocity_mm_s,
                    &connector_plan) != NF_MOTION_OK ||
                nf_slalom_seconds_to_us(connector_plan.total_time_s,
                                         &connector_us) != NF_SLALOM_PLAN_OK ||
                nf_slalom_seconds_to_us(
                    context.turn_plans[(size_t)action->kind].total_time_s,
                    &turn_us) != NF_SLALOM_PLAN_OK ||
                !nf_slalom_u64_add(connector_us, turn_us, &duration_us) ||
                nf_slalom_trace_turn(&context, connector_end, current_heading,
                                     action->kind, action->side, destination,
                                     end_heading, &trace) !=
                    NF_SLALOM_PLAN_OK ||
                !trace.feasible) {
                nf_slalom_free_turn_trajectories(&context);
                return nf_slalom_validation_fail(validation, index,
                                                  "turn replay failed");
            }
            if (!goal_seen && trace.has_goal) {
                uint64_t turn_cross_us;
                expected_goal = true;
                expected_phase = NF_SLALOM_GOAL_TURN;
                cross_x = trace.goal_x;
                cross_y = trace.goal_y;
                cross_heading = trace.goal_heading;
                cross_velocity = trace.goal_velocity_mm_s;
                if (nf_slalom_seconds_to_us(trace.goal_time_s,
                                             &turn_cross_us) !=
                        NF_SLALOM_PLAN_OK ||
                    !nf_slalom_u64_add(connector_us, turn_cross_us,
                                       &cross_us)) {
                    nf_slalom_free_turn_trajectories(&context);
                    return nf_slalom_validation_fail(
                        validation, index, "turn goal replay failed");
                }
            }
            if (!nf_slalom_same_anchor(action->end_anchor, destination) ||
                action->end_heading != end_heading ||
                action->end_speed_class !=
                    nf_slalom_speed_class(action->kind, action->side) ||
                action->connector_time_us != connector_us ||
                action->turn_time_us != turn_us ||
                action->duration_us != duration_us ||
                !nf_slalom_close_double(action->turn_velocity_mm_s,
                                         turn->velocity_mm_s) ||
                !nf_slalom_close_double(action->exit_velocity_mm_s,
                                         turn->velocity_mm_s) ||
                !nf_slalom_close_double(
                    action->turn_start_x_mm,
                    connector_end.half_x * config->half_cell_mm) ||
                !nf_slalom_close_double(
                    action->turn_start_y_mm,
                    connector_end.half_y * config->half_cell_mm) ||
                !nf_slalom_close_double(
                    action->turn_start_heading_deg,
                    nf_slalom_heading_math_deg(current_heading)) ||
                !action->required_open_checked ||
                action->swept_clearance_checked !=
                    config->check_swept_clearance ||
                action->has_goal_cross != expected_goal ||
                (!expected_goal &&
                 action->goal_phase != NF_SLALOM_GOAL_NONE) ||
                (expected_goal &&
                 (action->goal_phase != expected_phase ||
                  action->goal_x != cross_x || action->goal_y != cross_y ||
                  action->goal_cross_heading != cross_heading ||
                  action->goal_cross_time_us != cross_us))) {
                nf_slalom_free_turn_trajectories(&context);
                return nf_slalom_validation_fail(validation, index,
                                                  "turn metadata mismatch");
            }
            if (expected_goal) {
                if (!nf_slalom_u64_add(elapsed_us, cross_us,
                                       &goal_entry_us)) {
                    nf_slalom_free_turn_trajectories(&context);
                    return nf_slalom_validation_fail(validation, index,
                                                      "goal time overflow");
                }
                goal_seen = true;
                goal_x = cross_x;
                goal_y = cross_y;
                goal_heading = cross_heading;
                goal_velocity = cross_velocity;
            }
            if (!nf_slalom_u64_add(elapsed_us, duration_us, &elapsed_us)) {
                nf_slalom_free_turn_trajectories(&context);
                return nf_slalom_validation_fail(validation, index,
                                                  "turn time overflow");
            }
            current_anchor = destination;
            current_heading = end_heading;
            current_class = nf_slalom_speed_class(action->kind, action->side);
            current_velocity = turn->velocity_mm_s;
        } else if (action->kind == NF_SLALOM_ACTION_GOAL_STOP) {
            const NfLinearLimits *limits =
                nf_slalom_connector_limits(config, current_heading);
            NfLinearPlan terminal;
            uint64_t duration_us;

            if (index + 1U != plan->action_count ||
                action->side != NF_ROUTE_SIDE_NONE ||
                action->connector_steps == 0U ||
                !nf_slalom_same_anchor(action->end_anchor, connector_end) ||
                action->end_heading != current_heading ||
                (nf_slalom_is_cardinal(current_heading) &&
                 !nf_slalom_anchor_is_center(connector_end)) ||
                !nf_slalom_close_double(action->turn_velocity_mm_s, 0.0) ||
                !nf_slalom_close_double(action->exit_velocity_mm_s, 0.0) ||
                nf_motion_linear_plan(
                    limits, action->connector_command_distance_mm,
                    current_velocity, 0.0, &terminal) != NF_MOTION_OK ||
                nf_slalom_seconds_to_us(terminal.total_time_s, &duration_us) !=
                    NF_SLALOM_PLAN_OK ||
                action->duration_us != duration_us ||
                action->connector_time_us != duration_us ||
                action->turn_time_us != 0U ||
                action->end_speed_class != NF_SLALOM_SPEED_START ||
                !action->required_open_checked ||
                action->swept_clearance_checked) {
                nf_slalom_free_turn_trajectories(&context);
                return nf_slalom_validation_fail(validation, index,
                                                  "stop replay failed");
            }
            if (!goal_seen) {
                double cross_time_s;
                double cross_velocity;
                uint64_t cross_us;
                if (!connector_goal.present || !action->has_goal_cross ||
                    action->goal_phase != NF_SLALOM_GOAL_CONNECTOR ||
                    (uint16_t)(action->connector_steps -
                               connector_goal.step) <
                        config->minimum_post_goal_connector_steps ||
                    nf_motion_linear_time_at_distance(
                        &terminal,
                        connector_goal.step *
                            nf_slalom_connector_command_unit(
                                config, current_heading),
                        &cross_time_s, &cross_velocity) != NF_MOTION_OK ||
                    nf_slalom_seconds_to_us(cross_time_s, &cross_us) !=
                        NF_SLALOM_PLAN_OK ||
                    action->goal_cross_time_us != cross_us ||
                    action->goal_x != connector_goal.x ||
                    action->goal_y != connector_goal.y ||
                    action->goal_cross_heading != current_heading ||
                    !nf_slalom_u64_add(elapsed_us, cross_us,
                                       &goal_entry_us)) {
                    nf_slalom_free_turn_trajectories(&context);
                    return nf_slalom_validation_fail(validation, index,
                                                      "stop goal mismatch");
                }
                goal_seen = true;
                goal_x = connector_goal.x;
                goal_y = connector_goal.y;
                goal_heading = current_heading;
                goal_velocity = cross_velocity;
            } else if (action->has_goal_cross ||
                       action->goal_phase != NF_SLALOM_GOAL_NONE ||
                       action->connector_steps <
                           config->minimum_post_goal_connector_steps) {
                nf_slalom_free_turn_trajectories(&context);
                return nf_slalom_validation_fail(validation, index,
                                                  "duplicate goal crossing");
            }
            if (!nf_slalom_u64_add(elapsed_us, duration_us, &elapsed_us)) {
                nf_slalom_free_turn_trajectories(&context);
                return nf_slalom_validation_fail(validation, index,
                                                  "stop time overflow");
            }
            current_anchor = connector_end;
            current_velocity = 0.0;
        } else {
            nf_slalom_free_turn_trajectories(&context);
            return nf_slalom_validation_fail(validation, index,
                                              "unexpected action kind");
        }
    }

    if (!goal_seen || current_velocity != 0.0 ||
        goal_entry_us != plan->goal_entry_us || elapsed_us != plan->stop_us ||
        goal_x != plan->goal_x || goal_y != plan->goal_y ||
        goal_heading != plan->goal_heading ||
        !nf_slalom_close_double(goal_velocity,
                                 plan->goal_entry_velocity_mm_s) ||
        !plan->anchor_closure_validated ||
        !plan->required_open_validated ||
        (!config->check_swept_clearance &&
         plan->swept_clearance_validated)) {
        nf_slalom_free_turn_trajectories(&context);
        return nf_slalom_validation_fail(validation, plan->action_count,
                                          "plan summary mismatch");
    }
    if (config->check_swept_clearance) {
        if (!plan->swept_clearance_validated) {
            nf_slalom_free_turn_trajectories(&context);
            return nf_slalom_validation_fail(validation, plan->action_count,
                                              "clearance gate not validated");
        }
        for (size_t i = 1U; i < plan->action_count; i++) {
            const NfSlalomAction *action = &plan->actions[i];
            if ((unsigned int)action->kind < NF_SLALOM_ACTION_START_OFFSET &&
                !action->swept_clearance_checked) {
                nf_slalom_free_turn_trajectories(&context);
                return nf_slalom_validation_fail(validation, i,
                                                  "turn clearance missing");
            }
            if ((unsigned int)action->kind < NF_SLALOM_ACTION_START_OFFSET) {
                NfClearanceResult clearance;
                const NfClearanceStatus status = nf_route_turn_clearance(
                    maze, &config->clearance,
                    nf_slalom_geometry_turn_spec(config, action->kind),
                    &context.geometry_turn_plans[(size_t)action->kind],
                    action->turn_start_x_mm, action->turn_start_y_mm,
                    action->turn_start_heading_deg,
                    action->side == NF_ROUTE_SIDE_LEFT, &clearance);
                if (status != NF_CLEARANCE_OK || !clearance.clear) {
                    nf_slalom_free_turn_trajectories(&context);
                    return nf_slalom_validation_fail(
                        validation, i, "turn clearance replay failed");
                }
            }
        }
    }
    valid = true;
    nf_slalom_free_turn_trajectories(&context);
    if (validation != NULL) {
        validation->valid = true;
        validation->action_index = plan->action_count;
        validation->recomputed_goal_entry_us = goal_entry_us;
        validation->recomputed_stop_us = elapsed_us;
        (void)snprintf(validation->message, sizeof(validation->message), "ok");
    }
    return valid;
}
