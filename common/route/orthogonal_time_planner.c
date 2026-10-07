#include "orthogonal_time_planner.h"

#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define NF_ROUTE_STATE_CLASS_COUNT 4U
#define NF_ROUTE_INF UINT64_MAX
#define NF_ROUTE_SPEED_EPS 1.0e-3

/*
 * A state is a logical cell/heading plus the primitive that established its
 * physical connector boundary.  The boundary class is required because a
 * large turn owns one outgoing half-cell and has a different boundary speed
 * from a small turn.  Keeping it in the state makes every outgoing edge cost
 * depend only on that state, which is the condition Dijkstra needs.
 */
typedef enum {
    NF_BOUNDARY_START = 0,
    NF_BOUNDARY_SMALL_90,
    NF_BOUNDARY_LARGE_90,
    NF_BOUNDARY_LARGE_180,
} NfBoundaryClass;

typedef struct {
    uint64_t distance_us;
    uint16_t turn_count;
    bool settled;
} NfPlannerNode;

typedef struct {
    bool valid;
    size_t previous_state;
    uint16_t straight_cells;
    NfRouteMotionKind turn_kind;
    NfRouteSide side;
    double connector_distance_mm;
    uint64_t connector_time_us;
    uint64_t turn_time_us;
} NfPlannerParent;

typedef enum {
    NF_GOAL_NONE = 0,
    NF_GOAL_STRAIGHT,
    NF_GOAL_TURN,
} NfGoalKind;

typedef struct {
    NfGoalKind kind;
    size_t source_state;
    uint64_t score_us;
    uint64_t stop_us;
    uint16_t turn_count;
    uint8_t goal_x;
    uint8_t goal_y;
    NfRouteDirection goal_heading;
    uint16_t extension_cells;
    double goal_entry_velocity_mm_s;

    uint16_t straight_cells_to_goal;
    double terminal_distance_mm;
    uint64_t terminal_time_us;
    uint64_t terminal_cross_time_us;

    uint16_t connector_cells;
    double connector_distance_mm;
    uint64_t connector_time_us;
    NfRouteMotionKind turn_kind;
    NfRouteSide turn_side;
    uint64_t turn_time_us;
    uint64_t turn_cross_time_us;
    double brake_distance_mm;
    uint64_t brake_time_us;
} NfGoalRecord;

typedef struct {
    size_t *states;
    ptrdiff_t *positions;
    size_t size;
    size_t capacity;
    const NfPlannerNode *nodes;
} NfMinHeap;

typedef struct {
    const NfRouteMaze *maze;
    const NfOrthogonalPlannerConfig *config;
    NfPlannerNode *nodes;
    NfPlannerParent *parents;
    size_t state_count;
    NfMinHeap heap;
    NfTurnPlan small_90_plan;
    NfTurnPlan large_90_plan;
    NfTurnPlan large_180_plan;
    NfLinearPlan start_plan;
    uint64_t start_time_us;
    double start_speed_mm_s;
    NfGoalRecord goal;
    uint32_t expanded_states;
    uint32_t relaxed_edges;
} NfPlannerContext;

static const int8_t k_dir_dx[4] = {0, 1, 0, -1};
static const int8_t k_dir_dy[4] = {1, 0, -1, 0};
static const uint8_t k_wall_for_dir[4] = {
    NF_ROUTE_WALL_NORTH,
    NF_ROUTE_WALL_EAST,
    NF_ROUTE_WALL_SOUTH,
    NF_ROUTE_WALL_WEST,
};

static bool nf_route_in_bounds(const NfRouteMaze *maze, int x, int y)
{
    return maze != NULL && maze->width > 0U && maze->height > 0U &&
           maze->width <= NF_ROUTE_MAZE_MAX_SIZE &&
           maze->height <= NF_ROUTE_MAZE_MAX_SIZE && x >= 0 && y >= 0 &&
           x < (int)maze->width && y < (int)maze->height;
}

static NfRouteDirection nf_route_opposite(NfRouteDirection direction)
{
    return (NfRouteDirection)(((unsigned int)direction + 2U) & 3U);
}

static NfRouteDirection nf_route_turn_heading(NfRouteDirection heading,
                                              NfRouteSide side)
{
    const unsigned int delta = (side == NF_ROUTE_SIDE_RIGHT) ? 1U : 3U;
    return (NfRouteDirection)(((unsigned int)heading + delta) & 3U);
}

static bool nf_route_move(const NfRouteMaze *maze,
                          uint8_t x,
                          uint8_t y,
                          NfRouteDirection direction,
                          uint8_t *out_x,
                          uint8_t *out_y)
{
    const int nx = (int)x + k_dir_dx[(unsigned int)direction];
    const int ny = (int)y + k_dir_dy[(unsigned int)direction];
    if (!nf_route_maze_can_move(maze, x, y, direction) ||
        !nf_route_in_bounds(maze, nx, ny)) {
        return false;
    }
    *out_x = (uint8_t)nx;
    *out_y = (uint8_t)ny;
    return true;
}

static size_t nf_state_index(const NfRouteMaze *maze,
                             uint8_t x,
                             uint8_t y,
                             NfRouteDirection heading,
                             NfBoundaryClass boundary_class)
{
    return (((((size_t)y * maze->width) + x) * 4U +
             (size_t)heading) * NF_ROUTE_STATE_CLASS_COUNT) +
           (size_t)boundary_class;
}

static void nf_state_decode(const NfRouteMaze *maze,
                            size_t index,
                            uint8_t *out_x,
                            uint8_t *out_y,
                            NfRouteDirection *out_heading,
                            NfBoundaryClass *out_class)
{
    size_t cell;
    *out_class = (NfBoundaryClass)(index % NF_ROUTE_STATE_CLASS_COUNT);
    index /= NF_ROUTE_STATE_CLASS_COUNT;
    *out_heading = (NfRouteDirection)(index % 4U);
    cell = index / 4U;
    *out_x = (uint8_t)(cell % maze->width);
    *out_y = (uint8_t)(cell / maze->width);
}

static bool nf_u64_add(uint64_t a, uint64_t b, uint64_t *out)
{
    if (UINT64_MAX - a < b) {
        return false;
    }
    *out = a + b;
    return true;
}

static NfRoutePlanStatus nf_route_seconds_to_us(double seconds,
                                                uint64_t *out_us)
{
    const NfMotionStatus status = nf_motion_seconds_to_us(seconds, out_us);
    if (status == NF_MOTION_OK) {
        return NF_ROUTE_PLAN_OK;
    }
    return (status == NF_MOTION_OVERFLOW) ?
        NF_ROUTE_PLAN_OVERFLOW : NF_ROUTE_PLAN_INVALID_CONFIG;
}

static NfRoutePlanStatus nf_route_config_motion_status(NfMotionStatus status)
{
    if (status == NF_MOTION_OK) {
        return NF_ROUTE_PLAN_OK;
    }
    return (status == NF_MOTION_OVERFLOW) ?
        NF_ROUTE_PLAN_OVERFLOW : NF_ROUTE_PLAN_INVALID_CONFIG;
}

static bool nf_heap_less(const NfMinHeap *heap, size_t left_state, size_t right_state)
{
    const NfPlannerNode *left = &heap->nodes[left_state];
    const NfPlannerNode *right = &heap->nodes[right_state];
    if (left->distance_us != right->distance_us) {
        return left->distance_us < right->distance_us;
    }
    if (left->turn_count != right->turn_count) {
        return left->turn_count < right->turn_count;
    }
    return left_state < right_state;
}

static void nf_heap_swap(NfMinHeap *heap, size_t left, size_t right)
{
    const size_t state = heap->states[left];
    heap->states[left] = heap->states[right];
    heap->states[right] = state;
    heap->positions[heap->states[left]] = (ptrdiff_t)left;
    heap->positions[heap->states[right]] = (ptrdiff_t)right;
}

static bool nf_heap_push_or_decrease(NfMinHeap *heap, size_t state)
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
        if (!nf_heap_less(heap, heap->states[position], heap->states[parent])) {
            break;
        }
        nf_heap_swap(heap, position, parent);
        position = parent;
    }
    return true;
}

static bool nf_heap_pop(NfMinHeap *heap, size_t *out_state)
{
    size_t position = 0U;
    if (heap->size == 0U) {
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
            nf_heap_less(heap, heap->states[left], heap->states[smallest])) {
            smallest = left;
        }
        if (right < heap->size &&
            nf_heap_less(heap, heap->states[right], heap->states[smallest])) {
            smallest = right;
        }
        if (smallest == position) {
            break;
        }
        nf_heap_swap(heap, position, smallest);
        position = smallest;
    }
    return true;
}

static const NfTurnSpec *nf_turn_spec(const NfOrthogonalPlannerConfig *config,
                                      NfRouteMotionKind kind)
{
    switch (kind) {
    case NF_ROUTE_MOTION_SMALL_90: return &config->small_90;
    case NF_ROUTE_MOTION_LARGE_90: return &config->large_90;
    case NF_ROUTE_MOTION_LARGE_180: return &config->large_180;
    default: return NULL;
    }
}

static const NfTurnPlan *nf_turn_plan(const NfPlannerContext *context,
                                      NfRouteMotionKind kind)
{
    switch (kind) {
    case NF_ROUTE_MOTION_SMALL_90: return &context->small_90_plan;
    case NF_ROUTE_MOTION_LARGE_90: return &context->large_90_plan;
    case NF_ROUTE_MOTION_LARGE_180: return &context->large_180_plan;
    default: return NULL;
    }
}

static NfBoundaryClass nf_boundary_for_turn(NfRouteMotionKind kind)
{
    switch (kind) {
    case NF_ROUTE_MOTION_SMALL_90: return NF_BOUNDARY_SMALL_90;
    case NF_ROUTE_MOTION_LARGE_90: return NF_BOUNDARY_LARGE_90;
    case NF_ROUTE_MOTION_LARGE_180: return NF_BOUNDARY_LARGE_180;
    default: return NF_BOUNDARY_START;
    }
}

static unsigned int nf_boundary_out_half_sections(NfBoundaryClass boundary_class)
{
    return (boundary_class == NF_BOUNDARY_START ||
            boundary_class == NF_BOUNDARY_LARGE_90 ||
            boundary_class == NF_BOUNDARY_LARGE_180) ? 1U : 0U;
}

static unsigned int nf_turn_in_half_sections(NfRouteMotionKind kind)
{
    return (kind == NF_ROUTE_MOTION_LARGE_90 ||
            kind == NF_ROUTE_MOTION_LARGE_180) ? 1U : 0U;
}

static double nf_boundary_speed(const NfPlannerContext *context,
                                NfBoundaryClass boundary_class)
{
    switch (boundary_class) {
    case NF_BOUNDARY_START: return context->start_speed_mm_s;
    case NF_BOUNDARY_SMALL_90: return context->config->small_90.velocity_mm_s;
    case NF_BOUNDARY_LARGE_90: return context->config->large_90.velocity_mm_s;
    case NF_BOUNDARY_LARGE_180: return context->config->large_180.velocity_mm_s;
    default: return 0.0;
    }
}

static uint16_t nf_max_forward_cells(const NfRouteMaze *maze,
                                     uint8_t x,
                                     uint8_t y,
                                     NfRouteDirection heading)
{
    uint16_t count = 0U;
    uint8_t current_x = x;
    uint8_t current_y = y;
    uint8_t next_x;
    uint8_t next_y;
    while (nf_route_move(maze, current_x, current_y, heading, &next_x, &next_y)) {
        count++;
        current_x = next_x;
        current_y = next_y;
    }
    return count;
}

static bool nf_goal_is_better(const NfGoalRecord *candidate,
                              const NfGoalRecord *current)
{
    if (current->kind == NF_GOAL_NONE) {
        return true;
    }
    if (candidate->score_us != current->score_us) {
        return candidate->score_us < current->score_us;
    }
    if (candidate->turn_count != current->turn_count) {
        return candidate->turn_count < current->turn_count;
    }
    if (candidate->goal_y != current->goal_y) {
        return candidate->goal_y < current->goal_y;
    }
    if (candidate->goal_x != current->goal_x) {
        return candidate->goal_x < current->goal_x;
    }
    if (candidate->goal_heading != current->goal_heading) {
        return candidate->goal_heading < current->goal_heading;
    }
    return candidate->kind < current->kind;
}

static NfRoutePlanStatus nf_consider_straight_goal(
    NfPlannerContext *context,
    size_t state,
    uint8_t goal_x,
    uint8_t goal_y,
    NfRouteDirection heading,
    NfBoundaryClass boundary_class,
    uint16_t straight_cells)
{
    const unsigned int out_half = nf_boundary_out_half_sections(boundary_class);
    /*
     * The connector ends at the first non-goal -> goal boundary.  The final
     * half-cell (plus any whole-cell extension) belongs to the stopping tail,
     * so it affects the attainable crossing speed but not the score prefix.
     */
    const int pre_half_sections = (int)(2U * straight_cells) - (int)out_half;
    const uint16_t extension_cells =
        nf_max_forward_cells(context->maze, goal_x, goal_y, heading);
    NfGoalTerminalPlan terminal;
    NfGoalRecord candidate;
    uint64_t cross_us;
    uint64_t terminal_us;
    uint64_t score_us;
    uint64_t stop_us;
    NfMotionStatus motion_status;
    NfRoutePlanStatus route_status;

    if (pre_half_sections < 0) {
        return NF_ROUTE_PLAN_OK;
    }
    motion_status = nf_motion_goal_terminal_plan(
        &context->config->straight,
        (double)pre_half_sections * context->config->half_cell_mm,
        context->config->half_cell_mm +
            ((double)extension_cells * 2.0 * context->config->half_cell_mm),
        nf_boundary_speed(context, boundary_class),
        &terminal);
    if (motion_status != NF_MOTION_OK) {
        return NF_ROUTE_PLAN_OK;
    }
    route_status = nf_route_seconds_to_us(terminal.goal_cross_time_s, &cross_us);
    if (route_status != NF_ROUTE_PLAN_OK) {
        return route_status;
    }
    route_status = nf_route_seconds_to_us(terminal.full_plan.total_time_s,
                                          &terminal_us);
    if (route_status != NF_ROUTE_PLAN_OK) {
        return route_status;
    }
    if (!nf_u64_add(context->nodes[state].distance_us, cross_us, &score_us) ||
        !nf_u64_add(context->nodes[state].distance_us, terminal_us, &stop_us)) {
        return NF_ROUTE_PLAN_OVERFLOW;
    }

    memset(&candidate, 0, sizeof(candidate));
    candidate.kind = NF_GOAL_STRAIGHT;
    candidate.source_state = state;
    candidate.score_us = score_us;
    candidate.stop_us = stop_us;
    candidate.turn_count = context->nodes[state].turn_count;
    candidate.goal_x = goal_x;
    candidate.goal_y = goal_y;
    candidate.goal_heading = heading;
    candidate.extension_cells = extension_cells;
    candidate.goal_entry_velocity_mm_s = terminal.goal_cross_velocity_mm_s;
    candidate.straight_cells_to_goal = straight_cells;
    candidate.terminal_distance_mm = terminal.full_plan.distance_mm;
    candidate.terminal_time_us = terminal_us;
    candidate.terminal_cross_time_us = cross_us;
    if (nf_goal_is_better(&candidate, &context->goal)) {
        context->goal = candidate;
    }
    return NF_ROUTE_PLAN_OK;
}

static bool nf_turn_destination(const NfRouteMaze *maze,
                                uint8_t pivot_x,
                                uint8_t pivot_y,
                                NfRouteDirection heading,
                                NfRouteMotionKind kind,
                                NfRouteSide side,
                                uint8_t *out_x,
                                uint8_t *out_y,
                                NfRouteDirection *out_heading,
                                bool *out_intermediate_goal)
{
    NfRouteDirection side_heading = nf_route_turn_heading(heading, side);
    uint8_t first_x;
    uint8_t first_y;
    if (!nf_route_move(maze, pivot_x, pivot_y, side_heading, &first_x, &first_y)) {
        return false;
    }
    *out_intermediate_goal = false;
    if (kind != NF_ROUTE_MOTION_LARGE_180) {
        *out_x = first_x;
        *out_y = first_y;
        *out_heading = side_heading;
        return true;
    }

    *out_intermediate_goal = maze->goals[first_y][first_x];
    *out_heading = nf_route_opposite(heading);
    return nf_route_move(maze, first_x, first_y, *out_heading, out_x, out_y);
}

static NfRoutePlanStatus nf_consider_turn_goal(
    NfPlannerContext *context,
    size_t source_state,
    uint16_t connector_cells,
    double connector_distance_mm,
    uint64_t connector_time_us,
    NfRouteMotionKind turn_kind,
    NfRouteSide side,
    uint8_t goal_x,
    uint8_t goal_y,
    NfRouteDirection goal_heading)
{
    const NfTurnSpec *turn = nf_turn_spec(context->config, turn_kind);
    const NfTurnPlan *turn_plan = nf_turn_plan(context, turn_kind);
    const bool is_large = turn_kind == NF_ROUTE_MOTION_LARGE_90 ||
                          turn_kind == NF_ROUTE_MOTION_LARGE_180;
    const uint16_t extension_cells =
        nf_max_forward_cells(context->maze, goal_x, goal_y, goal_heading);
    /*
     * A large turn ends at its logical cell-center anchor.  Its boundary
     * crossing is inside the turn and is obtained from the 2D nominal
     * trajectory, not by treating the final half-cell as arc length.  Small
     * turns retain the path[] contract that their end is the goal boundary.
     */
    const double brake_distance_mm = is_large ?
        ((double)extension_cells * 2.0 * context->config->half_cell_mm) :
        (context->config->half_cell_mm +
         ((double)extension_cells * 2.0 * context->config->half_cell_mm));
    NfLinearPlan brake_plan;
    NfGoalRecord candidate;
    double turn_cross_s;
    double turn_cross_lateral_mm = 0.0;
    uint64_t turn_cross_us;
    uint64_t turn_us;
    uint64_t brake_us;
    uint64_t edge_prefix_us;
    uint64_t edge_full_us;
    uint64_t score_us;
    uint64_t stop_us;
    NfRoutePlanStatus route_status;

    if (is_large && extension_cells == 0U) {
        return NF_ROUTE_PLAN_OK;
    }
    if (is_large) {
        if (nf_motion_turn_exit_boundary_cross(
                turn, turn_plan, context->config->half_cell_mm,
                &turn_cross_s, &turn_cross_lateral_mm) != NF_MOTION_OK ||
            fabs(turn_cross_lateral_mm) >
                context->config->half_cell_mm + NF_ROUTE_SPEED_EPS) {
            return NF_ROUTE_PLAN_OK;
        }
    } else {
        turn_cross_s = turn_plan->total_time_s;
    }
    if (nf_motion_linear_plan(&context->config->straight, brake_distance_mm,
                              turn->velocity_mm_s, 0.0, &brake_plan) !=
        NF_MOTION_OK) {
        return NF_ROUTE_PLAN_OK;
    }
    route_status = nf_route_seconds_to_us(turn_cross_s, &turn_cross_us);
    if (route_status != NF_ROUTE_PLAN_OK) {
        return route_status;
    }
    route_status = nf_route_seconds_to_us(turn_plan->total_time_s, &turn_us);
    if (route_status != NF_ROUTE_PLAN_OK) {
        return route_status;
    }
    route_status = nf_route_seconds_to_us(brake_plan.total_time_s, &brake_us);
    if (route_status != NF_ROUTE_PLAN_OK) {
        return route_status;
    }
    if (!nf_u64_add(connector_time_us, turn_cross_us, &edge_prefix_us) ||
        !nf_u64_add(connector_time_us, turn_us, &edge_full_us) ||
        !nf_u64_add(context->nodes[source_state].distance_us, edge_prefix_us, &score_us) ||
        !nf_u64_add(context->nodes[source_state].distance_us, edge_full_us, &stop_us) ||
        !nf_u64_add(stop_us, brake_us, &stop_us)) {
        return NF_ROUTE_PLAN_OVERFLOW;
    }

    memset(&candidate, 0, sizeof(candidate));
    candidate.kind = NF_GOAL_TURN;
    candidate.source_state = source_state;
    candidate.score_us = score_us;
    candidate.stop_us = stop_us;
    candidate.turn_count = (uint16_t)(context->nodes[source_state].turn_count + 1U);
    candidate.goal_x = goal_x;
    candidate.goal_y = goal_y;
    candidate.goal_heading = goal_heading;
    candidate.extension_cells = extension_cells;
    candidate.goal_entry_velocity_mm_s = turn->velocity_mm_s;
    candidate.connector_cells = connector_cells;
    candidate.connector_distance_mm = connector_distance_mm;
    candidate.connector_time_us = connector_time_us;
    candidate.turn_kind = turn_kind;
    candidate.turn_side = side;
    candidate.turn_time_us = turn_us;
    candidate.turn_cross_time_us = turn_cross_us;
    candidate.brake_distance_mm = brake_distance_mm;
    candidate.brake_time_us = brake_us;
    if (nf_goal_is_better(&candidate, &context->goal)) {
        context->goal = candidate;
    }
    return NF_ROUTE_PLAN_OK;
}

static NfRoutePlanStatus nf_try_turn(NfPlannerContext *context,
                                     size_t source_state,
                                     uint8_t pivot_x,
                                     uint8_t pivot_y,
                                     NfRouteDirection heading,
                                     NfBoundaryClass source_class,
                                     uint16_t straight_cells,
                                     NfRouteMotionKind turn_kind,
                                     NfRouteSide side)
{
    const NfTurnSpec *turn = nf_turn_spec(context->config, turn_kind);
    const NfTurnPlan *turn_plan = nf_turn_plan(context, turn_kind);
    const NfBoundaryClass destination_class = nf_boundary_for_turn(turn_kind);
    const int half_sections = (int)(2U * straight_cells) -
        (int)nf_boundary_out_half_sections(source_class) -
        (int)nf_turn_in_half_sections(turn_kind);
    uint8_t destination_x;
    uint8_t destination_y;
    NfRouteDirection destination_heading;
    bool intermediate_goal;
    NfLinearPlan connector_plan;
    uint64_t connector_us;
    uint64_t turn_us;
    uint64_t edge_us;
    uint64_t candidate_distance;
    NfRoutePlanStatus route_status;
    size_t destination_state;
    NfPlannerNode *destination_node;
    bool better;

    if (turn == NULL || turn_plan == NULL || !turn->enabled || half_sections < 0) {
        return NF_ROUTE_PLAN_OK;
    }
    if (source_class == NF_BOUNDARY_START &&
        nf_turn_in_half_sections(turn_kind) != 0U && half_sections == 0) {
        return NF_ROUTE_PLAN_OK;
    }
    if (!nf_turn_destination(context->maze, pivot_x, pivot_y, heading,
                             turn_kind, side,
                             &destination_x, &destination_y,
                             &destination_heading, &intermediate_goal)) {
        return NF_ROUTE_PLAN_OK;
    }
    if (intermediate_goal) {
        /* A large 180 may not run through the first goal cell. */
        return NF_ROUTE_PLAN_OK;
    }

    if (nf_motion_linear_plan(&context->config->straight,
                              (double)half_sections * context->config->half_cell_mm,
                              nf_boundary_speed(context, source_class),
                              turn->velocity_mm_s,
                              &connector_plan) != NF_MOTION_OK) {
        return NF_ROUTE_PLAN_OK;
    }
    route_status = nf_route_seconds_to_us(connector_plan.total_time_s,
                                          &connector_us);
    if (route_status != NF_ROUTE_PLAN_OK) {
        return route_status;
    }
    route_status = nf_route_seconds_to_us(turn_plan->total_time_s, &turn_us);
    if (route_status != NF_ROUTE_PLAN_OK) {
        return route_status;
    }
    if (!nf_u64_add(connector_us, turn_us, &edge_us)) {
        return NF_ROUTE_PLAN_OVERFLOW;
    }

    if (context->maze->goals[destination_y][destination_x]) {
        return nf_consider_turn_goal(context, source_state, straight_cells,
                                     connector_plan.distance_mm, connector_us,
                                     turn_kind, side, destination_x,
                                     destination_y, destination_heading);
    }

    if (!nf_u64_add(context->nodes[source_state].distance_us, edge_us,
                    &candidate_distance)) {
        return NF_ROUTE_PLAN_OVERFLOW;
    }
    destination_state = nf_state_index(context->maze,
                                       destination_x, destination_y,
                                       destination_heading, destination_class);
    destination_node = &context->nodes[destination_state];
    better = candidate_distance < destination_node->distance_us ||
        (candidate_distance == destination_node->distance_us &&
         context->nodes[source_state].turn_count + 1U < destination_node->turn_count);
    if (!better || destination_node->settled) {
        return NF_ROUTE_PLAN_OK;
    }

    destination_node->distance_us = candidate_distance;
    destination_node->turn_count =
        (uint16_t)(context->nodes[source_state].turn_count + 1U);
    context->parents[destination_state].valid = true;
    context->parents[destination_state].previous_state = source_state;
    context->parents[destination_state].straight_cells = straight_cells;
    context->parents[destination_state].turn_kind = turn_kind;
    context->parents[destination_state].side = side;
    context->parents[destination_state].connector_distance_mm =
        connector_plan.distance_mm;
    context->parents[destination_state].connector_time_us = connector_us;
    context->parents[destination_state].turn_time_us = turn_us;
    context->relaxed_edges++;
    if (!nf_heap_push_or_decrease(&context->heap, destination_state)) {
        return NF_ROUTE_PLAN_CAPACITY;
    }
    return NF_ROUTE_PLAN_OK;
}

static bool nf_validate_maze(const NfRouteMaze *maze)
{
    if (maze == NULL || maze->width == 0U || maze->height == 0U ||
        maze->width > NF_ROUTE_MAZE_MAX_SIZE ||
        maze->height > NF_ROUTE_MAZE_MAX_SIZE) {
        return false;
    }
    for (uint8_t y = 0U; y < maze->height; y++) {
        for (uint8_t x = 0U; x < maze->width; x++) {
            const uint8_t walls = maze->walls[y][x];
            if ((x == 0U && (walls & NF_ROUTE_WALL_WEST) == 0U) ||
                (x + 1U == maze->width && (walls & NF_ROUTE_WALL_EAST) == 0U) ||
                (y == 0U && (walls & NF_ROUTE_WALL_SOUTH) == 0U) ||
                (y + 1U == maze->height && (walls & NF_ROUTE_WALL_NORTH) == 0U)) {
                return false;
            }
            if (x + 1U < maze->width) {
                const bool east = (walls & NF_ROUTE_WALL_EAST) != 0U;
                const bool west = (maze->walls[y][x + 1U] & NF_ROUTE_WALL_WEST) != 0U;
                if (east != west) {
                    return false;
                }
            }
            if (y + 1U < maze->height) {
                const bool north = (walls & NF_ROUTE_WALL_NORTH) != 0U;
                const bool south = (maze->walls[y + 1U][x] & NF_ROUTE_WALL_SOUTH) != 0U;
                if (north != south) {
                    return false;
                }
            }
        }
    }
    return true;
}

static NfRoutePlanStatus nf_validate_config(NfPlannerContext *context)
{
    const NfOrthogonalPlannerConfig *config = context->config;
    NfLinearPlan probe;
    uint64_t ignored_us;
    NfRoutePlanStatus route_status;
    NfMotionStatus motion_status;
    double target_speed;

    if (!isfinite(config->half_cell_mm) || config->half_cell_mm <= 0.0 ||
        !isfinite(config->start_offset_mm) || config->start_offset_mm <= 0.0 ||
        config->start_offset_mm > config->half_cell_mm ||
        fabs(config->small_90.angle_deg - 90.0) > 1.0e-9) {
        return NF_ROUTE_PLAN_INVALID_CONFIG;
    }
    motion_status = nf_motion_turn_plan(&config->small_90,
                                        &config->turn_environment,
                                        &context->small_90_plan);
    route_status = nf_route_config_motion_status(motion_status);
    if (route_status != NF_ROUTE_PLAN_OK) {
        return route_status;
    }
    if (config->small_90.velocity_mm_s > config->straight.vmax_mm_s) {
        return NF_ROUTE_PLAN_INVALID_CONFIG;
    }
    route_status = nf_route_seconds_to_us(context->small_90_plan.total_time_s,
                                          &ignored_us);
    if (route_status != NF_ROUTE_PLAN_OK) {
        return route_status;
    }
    if (config->allow_large_turns) {
        if (fabs(config->large_90.angle_deg - 90.0) > 1.0e-9 ||
            fabs(config->large_180.angle_deg - 180.0) > 1.0e-9 ||
            config->large_90.velocity_mm_s > config->straight.vmax_mm_s ||
            config->large_180.velocity_mm_s > config->straight.vmax_mm_s) {
            return NF_ROUTE_PLAN_INVALID_CONFIG;
        }
        motion_status = nf_motion_turn_plan(&config->large_90,
                                            &config->turn_environment,
                                            &context->large_90_plan);
        route_status = nf_route_config_motion_status(motion_status);
        if (route_status != NF_ROUTE_PLAN_OK) {
            return route_status;
        }
        motion_status = nf_motion_turn_plan(&config->large_180,
                                            &config->turn_environment,
                                            &context->large_180_plan);
        route_status = nf_route_config_motion_status(motion_status);
        if (route_status != NF_ROUTE_PLAN_OK) {
            return route_status;
        }
        route_status = nf_route_seconds_to_us(context->large_90_plan.total_time_s,
                                              &ignored_us);
        if (route_status != NF_ROUTE_PLAN_OK) {
            return route_status;
        }
        route_status = nf_route_seconds_to_us(context->large_180_plan.total_time_s,
                                              &ignored_us);
        if (route_status != NF_ROUTE_PLAN_OK) {
            return route_status;
        }
    }

    motion_status = nf_motion_accelerating_exit_velocity(
        &config->straight, config->start_offset_mm, 0.0, &target_speed);
    route_status = nf_route_config_motion_status(motion_status);
    if (route_status != NF_ROUTE_PLAN_OK) {
        return route_status;
    }
    motion_status = nf_motion_linear_plan(&config->straight,
                                          config->start_offset_mm,
                                          0.0, target_speed, &probe);
    route_status = nf_route_config_motion_status(motion_status);
    if (route_status != NF_ROUTE_PLAN_OK) {
        return route_status;
    }
    route_status = nf_route_seconds_to_us(probe.total_time_s,
                                          &context->start_time_us);
    if (route_status != NF_ROUTE_PLAN_OK) {
        return route_status;
    }
    context->start_plan = probe;
    context->start_speed_mm_s = target_speed;
    return NF_ROUTE_PLAN_OK;
}

static bool nf_add_action(NfOrthogonalRoutePlan *plan, const NfRouteMotion *action)
{
    if (plan->action_count >= NF_ROUTE_MAX_ACTIONS) {
        return false;
    }
    plan->actions[plan->action_count++] = *action;
    return true;
}

static void nf_advance_logical(uint8_t *x, uint8_t *y,
                               NfRouteDirection heading, uint16_t cells)
{
    *x = (uint8_t)((int)*x + ((int)cells * k_dir_dx[(unsigned int)heading]));
    *y = (uint8_t)((int)*y + ((int)cells * k_dir_dy[(unsigned int)heading]));
}

static bool nf_append_connector_action(const NfPlannerContext *context,
                                       NfOrthogonalRoutePlan *plan,
                                       size_t source_state,
                                       uint16_t straight_cells,
                                       double distance_mm,
                                       uint64_t duration_us,
                                       double exit_velocity_mm_s,
                                       uint8_t *out_x,
                                       uint8_t *out_y,
                                       NfRouteDirection *out_heading)
{
    uint8_t x;
    uint8_t y;
    NfRouteDirection heading;
    NfBoundaryClass boundary_class;
    NfRouteMotion action;

    nf_state_decode(context->maze, source_state, &x, &y, &heading, &boundary_class);
    *out_x = x;
    *out_y = y;
    *out_heading = heading;
    if (straight_cells == 0U && distance_mm <= 0.0) {
        return true;
    }

    memset(&action, 0, sizeof(action));
    action.kind = NF_ROUTE_MOTION_STRAIGHT;
    action.side = NF_ROUTE_SIDE_NONE;
    action.start_x = x;
    action.start_y = y;
    action.start_heading = heading;
    action.end_heading = heading;
    action.logical_cells = straight_cells;
    action.distance_mm = distance_mm;
    action.entry_velocity_mm_s = nf_boundary_speed(context, boundary_class);
    action.exit_velocity_mm_s = exit_velocity_mm_s;
    action.duration_us = duration_us;
    nf_advance_logical(&x, &y, heading, straight_cells);
    action.end_x = x;
    action.end_y = y;
    *out_x = x;
    *out_y = y;
    return nf_add_action(plan, &action);
}

static bool nf_append_turn_action(const NfPlannerContext *context,
                                  NfOrthogonalRoutePlan *plan,
                                  uint8_t pivot_x,
                                  uint8_t pivot_y,
                                  NfRouteDirection heading,
                                  NfRouteMotionKind turn_kind,
                                  NfRouteSide side,
                                  uint64_t duration_us,
                                  bool has_goal_cross,
                                  uint8_t goal_x,
                                  uint8_t goal_y,
                                  uint64_t goal_cross_time_us,
                                  uint8_t *out_x,
                                  uint8_t *out_y,
                                  NfRouteDirection *out_heading)
{
    const NfTurnSpec *turn = nf_turn_spec(context->config, turn_kind);
    bool intermediate_goal;
    NfRouteMotion action;
    if (!nf_turn_destination(context->maze, pivot_x, pivot_y, heading,
                             turn_kind, side, out_x, out_y, out_heading,
                             &intermediate_goal)) {
        return false;
    }
    memset(&action, 0, sizeof(action));
    action.kind = turn_kind;
    action.side = side;
    action.start_x = pivot_x;
    action.start_y = pivot_y;
    action.end_x = *out_x;
    action.end_y = *out_y;
    action.start_heading = heading;
    action.end_heading = *out_heading;
    action.logical_cells = (turn_kind == NF_ROUTE_MOTION_LARGE_180) ? 2U : 1U;
    action.distance_mm = nf_turn_plan(context, turn_kind)->travel_distance_mm;
    action.entry_velocity_mm_s = turn->velocity_mm_s;
    action.exit_velocity_mm_s = turn->velocity_mm_s;
    action.duration_us = duration_us;
    action.has_goal_cross = has_goal_cross;
    action.goal_cross_after_cells = action.logical_cells;
    action.goal_x = goal_x;
    action.goal_y = goal_y;
    action.goal_cross_time_us = goal_cross_time_us;
    return nf_add_action(plan, &action);
}

static NfRoutePlanStatus nf_reconstruct_plan(NfPlannerContext *context,
                                             size_t start_state,
                                             NfOrthogonalRoutePlan *out)
{
    size_t *chain;
    size_t chain_count = 0U;
    size_t current = context->goal.source_state;
    NfRouteMotion action;

    chain = (size_t *)malloc(context->state_count * sizeof(*chain));
    if (chain == NULL) {
        return NF_ROUTE_PLAN_CAPACITY;
    }
    while (current != start_state) {
        if (chain_count >= context->state_count ||
            !context->parents[current].valid) {
            free(chain);
            return NF_ROUTE_PLAN_INVALID_ARGUMENT;
        }
        chain[chain_count++] = current;
        current = context->parents[current].previous_state;
    }

    memset(&action, 0, sizeof(action));
    action.kind = NF_ROUTE_MOTION_START_OFFSET;
    action.side = NF_ROUTE_SIDE_NONE;
    nf_state_decode(context->maze, start_state,
                    &action.start_x, &action.start_y,
                    &action.start_heading, &(NfBoundaryClass){0});
    action.end_x = action.start_x;
    action.end_y = action.start_y;
    action.end_heading = action.start_heading;
    action.distance_mm = context->config->start_offset_mm;
    action.entry_velocity_mm_s = 0.0;
    action.exit_velocity_mm_s = context->start_speed_mm_s;
    action.duration_us = context->start_time_us;
    if (!nf_add_action(out, &action)) {
        free(chain);
        return NF_ROUTE_PLAN_CAPACITY;
    }

    for (size_t reverse_index = chain_count; reverse_index > 0U; reverse_index--) {
        const size_t child_state = chain[reverse_index - 1U];
        const NfPlannerParent *parent = &context->parents[child_state];
        const NfTurnSpec *turn = nf_turn_spec(context->config, parent->turn_kind);
        uint8_t pivot_x;
        uint8_t pivot_y;
        uint8_t destination_x;
        uint8_t destination_y;
        NfRouteDirection heading;
        NfRouteDirection destination_heading;
        if (!nf_append_connector_action(context, out, parent->previous_state,
                                        parent->straight_cells,
                                        parent->connector_distance_mm,
                                        parent->connector_time_us,
                                        turn->velocity_mm_s,
                                        &pivot_x, &pivot_y, &heading) ||
            !nf_append_turn_action(context, out, pivot_x, pivot_y, heading,
                                   parent->turn_kind, parent->side,
                                   parent->turn_time_us,
                                   false, 0U, 0U, 0U,
                                   &destination_x, &destination_y,
                                   &destination_heading)) {
            free(chain);
            return NF_ROUTE_PLAN_CAPACITY;
        }
    }
    free(chain);

    if (context->goal.kind == NF_GOAL_STRAIGHT) {
        uint8_t x;
        uint8_t y;
        NfRouteDirection heading;
        NfBoundaryClass boundary_class;
        nf_state_decode(context->maze, context->goal.source_state,
                        &x, &y, &heading, &boundary_class);
        memset(&action, 0, sizeof(action));
        action.kind = NF_ROUTE_MOTION_STRAIGHT;
        action.side = NF_ROUTE_SIDE_NONE;
        action.start_x = x;
        action.start_y = y;
        action.start_heading = heading;
        action.end_heading = heading;
        action.logical_cells = (uint16_t)(context->goal.straight_cells_to_goal +
                                          context->goal.extension_cells);
        action.distance_mm = context->goal.terminal_distance_mm;
        action.entry_velocity_mm_s = nf_boundary_speed(context, boundary_class);
        action.exit_velocity_mm_s = 0.0;
        action.duration_us = context->goal.terminal_time_us;
        action.has_goal_cross = true;
        action.goal_cross_after_cells = context->goal.straight_cells_to_goal;
        action.goal_x = context->goal.goal_x;
        action.goal_y = context->goal.goal_y;
        action.goal_cross_time_us = context->goal.terminal_cross_time_us;
        nf_advance_logical(&x, &y, heading, action.logical_cells);
        action.end_x = x;
        action.end_y = y;
        if (!nf_add_action(out, &action)) {
            return NF_ROUTE_PLAN_CAPACITY;
        }
    } else {
        const NfTurnSpec *turn =
            nf_turn_spec(context->config, context->goal.turn_kind);
        uint8_t pivot_x;
        uint8_t pivot_y;
        uint8_t destination_x;
        uint8_t destination_y;
        NfRouteDirection heading;
        NfRouteDirection destination_heading;
        if (!nf_append_connector_action(context, out, context->goal.source_state,
                                        context->goal.connector_cells,
                                        context->goal.connector_distance_mm,
                                        context->goal.connector_time_us,
                                        turn->velocity_mm_s,
                                        &pivot_x, &pivot_y, &heading) ||
            !nf_append_turn_action(context, out, pivot_x, pivot_y, heading,
                                   context->goal.turn_kind,
                                   context->goal.turn_side,
                                   context->goal.turn_time_us,
                                   true,
                                   context->goal.goal_x,
                                   context->goal.goal_y,
                                   context->goal.turn_cross_time_us,
                                   &destination_x, &destination_y,
                                   &destination_heading)) {
            return NF_ROUTE_PLAN_CAPACITY;
        }

        memset(&action, 0, sizeof(action));
        action.kind = NF_ROUTE_MOTION_STRAIGHT;
        action.side = NF_ROUTE_SIDE_NONE;
        action.start_x = destination_x;
        action.start_y = destination_y;
        action.start_heading = destination_heading;
        action.end_heading = destination_heading;
        action.logical_cells = context->goal.extension_cells;
        action.distance_mm = context->goal.brake_distance_mm;
        action.entry_velocity_mm_s = turn->velocity_mm_s;
        action.exit_velocity_mm_s = 0.0;
        action.duration_us = context->goal.brake_time_us;
        nf_advance_logical(&destination_x, &destination_y,
                           destination_heading, action.logical_cells);
        action.end_x = destination_x;
        action.end_y = destination_y;
        if (!nf_add_action(out, &action)) {
            return NF_ROUTE_PLAN_CAPACITY;
        }
    }

    out->goal_x = context->goal.goal_x;
    out->goal_y = context->goal.goal_y;
    out->goal_heading = context->goal.goal_heading;
    out->post_goal_extension_cells = context->goal.extension_cells;
    out->goal_entry_us = context->goal.score_us;
    out->stop_us = context->goal.stop_us;
    out->goal_entry_velocity_mm_s = context->goal.goal_entry_velocity_mm_s;
    out->expanded_states = context->expanded_states;
    out->relaxed_edges = context->relaxed_edges;
    return NF_ROUTE_PLAN_OK;
}

const char *nf_route_plan_status_name(NfRoutePlanStatus status)
{
    switch (status) {
    case NF_ROUTE_PLAN_OK: return "ok";
    case NF_ROUTE_PLAN_INVALID_ARGUMENT: return "invalid-argument";
    case NF_ROUTE_PLAN_INVALID_MAZE: return "invalid-maze";
    case NF_ROUTE_PLAN_INVALID_CONFIG: return "invalid-config";
    case NF_ROUTE_PLAN_NO_PATH: return "no-path";
    case NF_ROUTE_PLAN_CAPACITY: return "capacity";
    case NF_ROUTE_PLAN_OVERFLOW: return "overflow";
    default: return "unknown";
    }
}

const char *nf_route_direction_name(NfRouteDirection direction)
{
    static const char *const names[4] = {"N", "E", "S", "W"};
    return ((unsigned int)direction < 4U) ? names[(unsigned int)direction] : "?";
}

const char *nf_route_motion_name(NfRouteMotionKind kind, NfRouteSide side)
{
    switch (kind) {
    case NF_ROUTE_MOTION_START_OFFSET: return "START";
    case NF_ROUTE_MOTION_STRAIGHT: return "STRAIGHT";
    case NF_ROUTE_MOTION_SMALL_90:
        return (side == NF_ROUTE_SIDE_RIGHT) ? "SMALL-R90" : "SMALL-L90";
    case NF_ROUTE_MOTION_LARGE_90:
        return (side == NF_ROUTE_SIDE_RIGHT) ? "LARGE-R90" : "LARGE-L90";
    case NF_ROUTE_MOTION_LARGE_180:
        return (side == NF_ROUTE_SIDE_RIGHT) ? "LARGE-R180" : "LARGE-L180";
    default: return "UNKNOWN";
    }
}

bool nf_route_maze_init(NfRouteMaze *maze, uint8_t width, uint8_t height)
{
    if (maze == NULL || width == 0U || height == 0U ||
        width > NF_ROUTE_MAZE_MAX_SIZE || height > NF_ROUTE_MAZE_MAX_SIZE) {
        return false;
    }
    memset(maze, 0, sizeof(*maze));
    maze->width = width;
    maze->height = height;
    return true;
}

bool nf_route_maze_set_wall(NfRouteMaze *maze, uint8_t x, uint8_t y,
                            NfRouteDirection direction)
{
    int nx;
    int ny;
    if (maze == NULL || (unsigned int)direction >= 4U ||
        !nf_route_in_bounds(maze, x, y)) {
        return false;
    }
    nx = (int)x + k_dir_dx[(unsigned int)direction];
    ny = (int)y + k_dir_dy[(unsigned int)direction];
    maze->walls[y][x] |= k_wall_for_dir[(unsigned int)direction];
    if (nf_route_in_bounds(maze, nx, ny)) {
        maze->walls[ny][nx] |=
            k_wall_for_dir[(unsigned int)nf_route_opposite(direction)];
    }
    return true;
}

bool nf_route_maze_add_boundaries(NfRouteMaze *maze)
{
    if (maze == NULL || maze->width == 0U || maze->height == 0U ||
        maze->width > NF_ROUTE_MAZE_MAX_SIZE ||
        maze->height > NF_ROUTE_MAZE_MAX_SIZE) {
        return false;
    }
    for (uint8_t x = 0U; x < maze->width; x++) {
        (void)nf_route_maze_set_wall(maze, x, 0U, NF_ROUTE_DIR_SOUTH);
        (void)nf_route_maze_set_wall(maze, x, (uint8_t)(maze->height - 1U),
                                     NF_ROUTE_DIR_NORTH);
    }
    for (uint8_t y = 0U; y < maze->height; y++) {
        (void)nf_route_maze_set_wall(maze, 0U, y, NF_ROUTE_DIR_WEST);
        (void)nf_route_maze_set_wall(maze, (uint8_t)(maze->width - 1U), y,
                                     NF_ROUTE_DIR_EAST);
    }
    return true;
}

bool nf_route_maze_can_move(const NfRouteMaze *maze, uint8_t x, uint8_t y,
                            NfRouteDirection direction)
{
    int nx;
    int ny;
    NfRouteDirection opposite;
    if (maze == NULL || (unsigned int)direction >= 4U ||
        !nf_route_in_bounds(maze, x, y)) {
        return false;
    }
    nx = (int)x + k_dir_dx[(unsigned int)direction];
    ny = (int)y + k_dir_dy[(unsigned int)direction];
    if (!nf_route_in_bounds(maze, nx, ny)) {
        return false;
    }
    opposite = nf_route_opposite(direction);
    return (maze->walls[y][x] & k_wall_for_dir[(unsigned int)direction]) == 0U &&
           (maze->walls[ny][nx] & k_wall_for_dir[(unsigned int)opposite]) == 0U;
}

NfRoutePlanStatus nf_orthogonal_time_plan(const NfRouteMaze *maze,
                                          const NfOrthogonalPlannerConfig *config,
                                          const NfOrthogonalPlannerRequest *request,
                                          NfOrthogonalRoutePlan *out)
{
    NfPlannerContext context;
    size_t start_state;
    NfRoutePlanStatus result = NF_ROUTE_PLAN_OK;
    bool has_goal = false;

    if (maze == NULL || config == NULL || request == NULL || out == NULL ||
        (unsigned int)request->start_heading >= 4U ||
        !nf_route_in_bounds(maze, request->start_x, request->start_y)) {
        return NF_ROUTE_PLAN_INVALID_ARGUMENT;
    }
    memset(out, 0, sizeof(*out));
    if (!nf_validate_maze(maze)) {
        return NF_ROUTE_PLAN_INVALID_MAZE;
    }
    for (uint8_t y = 0U; y < maze->height; y++) {
        for (uint8_t x = 0U; x < maze->width; x++) {
            has_goal = has_goal || maze->goals[y][x];
        }
    }
    if (!has_goal) {
        return NF_ROUTE_PLAN_INVALID_MAZE;
    }

    memset(&context, 0, sizeof(context));
    context.maze = maze;
    context.config = config;
    result = nf_validate_config(&context);
    if (result != NF_ROUTE_PLAN_OK) {
        return result;
    }
    if (maze->goals[request->start_y][request->start_x]) {
        out->goal_x = request->start_x;
        out->goal_y = request->start_y;
        out->goal_heading = request->start_heading;
        return NF_ROUTE_PLAN_OK;
    }

    context.state_count = (size_t)maze->width * maze->height * 4U *
                          NF_ROUTE_STATE_CLASS_COUNT;
    context.nodes = (NfPlannerNode *)malloc(context.state_count * sizeof(*context.nodes));
    context.parents = (NfPlannerParent *)calloc(context.state_count,
                                                sizeof(*context.parents));
    context.heap.states = (size_t *)malloc(context.state_count * sizeof(*context.heap.states));
    context.heap.positions = (ptrdiff_t *)malloc(context.state_count *
                                                 sizeof(*context.heap.positions));
    if (context.nodes == NULL || context.parents == NULL ||
        context.heap.states == NULL || context.heap.positions == NULL) {
        result = NF_ROUTE_PLAN_CAPACITY;
        goto cleanup;
    }
    context.heap.capacity = context.state_count;
    context.heap.nodes = context.nodes;
    for (size_t i = 0U; i < context.state_count; i++) {
        context.nodes[i].distance_us = NF_ROUTE_INF;
        context.nodes[i].turn_count = UINT16_MAX;
        context.nodes[i].settled = false;
        context.heap.positions[i] = -1;
    }
    start_state = nf_state_index(maze, request->start_x, request->start_y,
                                 request->start_heading, NF_BOUNDARY_START);
    context.nodes[start_state].distance_us = context.start_time_us;
    context.nodes[start_state].turn_count = 0U;
    if (!nf_heap_push_or_decrease(&context.heap, start_state)) {
        result = NF_ROUTE_PLAN_CAPACITY;
        goto cleanup;
    }

    while (context.heap.size > 0U) {
        size_t state;
        uint8_t x;
        uint8_t y;
        NfRouteDirection heading;
        NfBoundaryClass boundary_class;
        uint8_t scan_x;
        uint8_t scan_y;
        const uint16_t max_scan = (maze->width > maze->height) ?
            maze->width : maze->height;

        (void)nf_heap_pop(&context.heap, &state);
        if (context.nodes[state].settled) {
            continue;
        }
        if (context.goal.kind != NF_GOAL_NONE &&
            context.nodes[state].distance_us >= context.goal.score_us) {
            break;
        }
        context.nodes[state].settled = true;
        context.expanded_states++;
        nf_state_decode(maze, state, &x, &y, &heading, &boundary_class);
        scan_x = x;
        scan_y = y;

        for (uint16_t straight_cells = 0U; straight_cells <= max_scan;
             straight_cells++) {
            result = nf_try_turn(&context, state, scan_x, scan_y,
                                 heading, boundary_class, straight_cells,
                                 NF_ROUTE_MOTION_SMALL_90, NF_ROUTE_SIDE_RIGHT);
            if (result != NF_ROUTE_PLAN_OK) goto cleanup;
            result = nf_try_turn(&context, state, scan_x, scan_y,
                                 heading, boundary_class, straight_cells,
                                 NF_ROUTE_MOTION_SMALL_90, NF_ROUTE_SIDE_LEFT);
            if (result != NF_ROUTE_PLAN_OK) goto cleanup;
            if (config->allow_large_turns) {
                result = nf_try_turn(&context, state, scan_x, scan_y,
                                     heading, boundary_class, straight_cells,
                                     NF_ROUTE_MOTION_LARGE_90, NF_ROUTE_SIDE_RIGHT);
                if (result != NF_ROUTE_PLAN_OK) goto cleanup;
                result = nf_try_turn(&context, state, scan_x, scan_y,
                                     heading, boundary_class, straight_cells,
                                     NF_ROUTE_MOTION_LARGE_90, NF_ROUTE_SIDE_LEFT);
                if (result != NF_ROUTE_PLAN_OK) goto cleanup;
                result = nf_try_turn(&context, state, scan_x, scan_y,
                                     heading, boundary_class, straight_cells,
                                     NF_ROUTE_MOTION_LARGE_180, NF_ROUTE_SIDE_RIGHT);
                if (result != NF_ROUTE_PLAN_OK) goto cleanup;
                result = nf_try_turn(&context, state, scan_x, scan_y,
                                     heading, boundary_class, straight_cells,
                                     NF_ROUTE_MOTION_LARGE_180, NF_ROUTE_SIDE_LEFT);
                if (result != NF_ROUTE_PLAN_OK) goto cleanup;
            }

            if (straight_cells == max_scan) {
                break;
            }
            {
                uint8_t next_x;
                uint8_t next_y;
                if (!nf_route_move(maze, scan_x, scan_y, heading, &next_x, &next_y)) {
                    break;
                }
                scan_x = next_x;
                scan_y = next_y;
            }
            if (maze->goals[scan_y][scan_x]) {
                result = nf_consider_straight_goal(
                    &context, state, scan_x, scan_y, heading,
                    boundary_class, (uint16_t)(straight_cells + 1U));
                if (result != NF_ROUTE_PLAN_OK) goto cleanup;
                break;
            }
        }
    }

    if (context.goal.kind == NF_GOAL_NONE) {
        result = NF_ROUTE_PLAN_NO_PATH;
        goto cleanup;
    }
    result = nf_reconstruct_plan(&context, start_state, out);

cleanup:
    free(context.heap.positions);
    free(context.heap.states);
    free(context.parents);
    free(context.nodes);
    return result;
}

static bool nf_validation_fail(NfRouteValidation *validation,
                               size_t action_index,
                               const char *message)
{
    if (validation != NULL) {
        validation->valid = false;
        validation->action_index = action_index;
        snprintf(validation->message, sizeof(validation->message), "%s", message);
    }
    return false;
}

static bool nf_validation_close(double actual, double expected)
{
    const double scale = fmax(1.0, fmax(fabs(actual), fabs(expected)));
    return isfinite(actual) && isfinite(expected) &&
           fabs(actual - expected) <= (1.0e-8 * scale);
}

static bool nf_validation_motion_matches(const NfRouteMotion *action,
                                         double expected_distance_mm,
                                         double expected_entry_velocity_mm_s,
                                         double expected_exit_velocity_mm_s,
                                         uint64_t expected_duration_us)
{
    return nf_validation_close(action->distance_mm, expected_distance_mm) &&
           nf_validation_close(action->entry_velocity_mm_s,
                               expected_entry_velocity_mm_s) &&
           nf_validation_close(action->exit_velocity_mm_s,
                               expected_exit_velocity_mm_s) &&
           action->duration_us == expected_duration_us;
}

bool nf_orthogonal_route_validate(const NfRouteMaze *maze,
                                  const NfOrthogonalPlannerConfig *config,
                                  const NfOrthogonalPlannerRequest *request,
                                  const NfOrthogonalRoutePlan *plan,
                                  NfRouteValidation *validation)
{
    NfPlannerContext timing_context;
    uint8_t x;
    uint8_t y;
    NfRouteDirection heading;
    NfRouteDirection observed_goal_heading = NF_ROUTE_DIR_NORTH;
    NfBoundaryClass boundary_class = NF_BOUNDARY_START;
    double velocity = 0.0;
    uint64_t elapsed_us = 0U;
    uint64_t observed_goal_us = 0U;
    uint16_t observed_extension_cells = 0U;
    bool goal_seen = false;
    uint8_t observed_goal_x = 0U;
    uint8_t observed_goal_y = 0U;

    if (validation != NULL) {
        memset(validation, 0, sizeof(*validation));
    }
    if (maze == NULL || config == NULL || request == NULL || plan == NULL) {
        return nf_validation_fail(validation, 0U, "null argument");
    }
    if ((unsigned int)request->start_heading >= 4U ||
        !nf_route_in_bounds(maze, request->start_x, request->start_y) ||
        !nf_validate_maze(maze)) {
        return nf_validation_fail(validation, 0U, "invalid maze or start pose");
    }
    memset(&timing_context, 0, sizeof(timing_context));
    timing_context.maze = maze;
    timing_context.config = config;
    if (nf_validate_config(&timing_context) != NF_ROUTE_PLAN_OK) {
        return nf_validation_fail(validation, 0U, "invalid timing config");
    }
    if (plan->action_count > NF_ROUTE_MAX_ACTIONS) {
        return nf_validation_fail(validation, 0U, "action count exceeds capacity");
    }
    if (maze->goals[request->start_y][request->start_x]) {
        if (plan->goal_entry_us != 0U || plan->stop_us != 0U ||
            plan->action_count != 0U || plan->goal_x != request->start_x ||
            plan->goal_y != request->start_y ||
            plan->goal_heading != request->start_heading ||
            plan->post_goal_extension_cells != 0U ||
            !nf_validation_close(plan->goal_entry_velocity_mm_s, 0.0)) {
            return nf_validation_fail(validation, 0U, "invalid start-goal plan");
        }
        if (validation != NULL) {
            validation->valid = true;
            snprintf(validation->message, sizeof(validation->message), "ok");
        }
        return true;
    }

    x = request->start_x;
    y = request->start_y;
    heading = request->start_heading;
    for (size_t i = 0U; i < plan->action_count; i++) {
        const NfRouteMotion *action = &plan->actions[i];
        bool marker_matched = false;
        uint16_t move_count = 0U;

        if (action->start_x != x || action->start_y != y ||
            action->start_heading != heading) {
            return nf_validation_fail(validation, i, "action start pose mismatch");
        }
        if (fabs(action->entry_velocity_mm_s - velocity) > NF_ROUTE_SPEED_EPS) {
            return nf_validation_fail(validation, i, "action speed discontinuity");
        }
        if (action->has_goal_cross && action->goal_cross_time_us > action->duration_us) {
            return nf_validation_fail(validation, i, "goal marker exceeds action duration");
        }
        if (goal_seen && action->kind != NF_ROUTE_MOTION_STRAIGHT) {
            return nf_validation_fail(validation, i, "non-straight action follows goal entry");
        }

        if (action->kind == NF_ROUTE_MOTION_START_OFFSET) {
            if (i != 0U || action->logical_cells != 0U ||
                action->side != NF_ROUTE_SIDE_NONE || action->has_goal_cross ||
                !nf_validation_motion_matches(
                    action, config->start_offset_mm, 0.0,
                    timing_context.start_speed_mm_s,
                    timing_context.start_time_us)) {
                return nf_validation_fail(validation, i,
                                          "invalid start-offset timing");
            }
        } else if (action->kind == NF_ROUTE_MOTION_STRAIGHT) {
            NfLinearPlan expected_linear;
            uint64_t expected_duration_us;
            double expected_distance_mm;
            double expected_entry_velocity_mm_s =
                nf_boundary_speed(&timing_context, boundary_class);
            double expected_exit_velocity_mm_s;

            if (action->side != NF_ROUTE_SIDE_NONE || action->end_heading != heading) {
                return nf_validation_fail(validation, i, "invalid straight action");
            }
            if (action->has_goal_cross) {
                const unsigned int out_half =
                    nf_boundary_out_half_sections(boundary_class);
                int pre_half_sections;
                uint16_t extension_cells;
                NfGoalTerminalPlan terminal;
                uint64_t expected_cross_us;

                if (goal_seen || action->goal_cross_after_cells == 0U ||
                    action->logical_cells < action->goal_cross_after_cells) {
                    return nf_validation_fail(validation, i,
                                              "invalid straight goal marker");
                }
                extension_cells = (uint16_t)(action->logical_cells -
                                             action->goal_cross_after_cells);
                pre_half_sections =
                    (int)(2U * action->goal_cross_after_cells) - (int)out_half;
                if (pre_half_sections < 0 ||
                    nf_motion_goal_terminal_plan(
                        &config->straight,
                        (double)pre_half_sections * config->half_cell_mm,
                        config->half_cell_mm +
                            ((double)extension_cells * 2.0 * config->half_cell_mm),
                        expected_entry_velocity_mm_s, &terminal) != NF_MOTION_OK ||
                    nf_motion_seconds_to_us(terminal.full_plan.total_time_s,
                                            &expected_duration_us) != NF_MOTION_OK ||
                    nf_motion_seconds_to_us(terminal.goal_cross_time_s,
                                            &expected_cross_us) != NF_MOTION_OK) {
                    return nf_validation_fail(validation, i,
                                              "straight goal timing is infeasible");
                }
                expected_distance_mm = terminal.full_plan.distance_mm;
                expected_exit_velocity_mm_s = 0.0;
                if (action->goal_cross_time_us != expected_cross_us ||
                    extension_cells != plan->post_goal_extension_cells ||
                    !nf_validation_close(plan->goal_entry_velocity_mm_s,
                                         terminal.goal_cross_velocity_mm_s)) {
                    return nf_validation_fail(validation, i,
                                              "straight goal timing mismatch");
                }
            } else if (goal_seen) {
                const bool follows_large =
                    boundary_class == NF_BOUNDARY_LARGE_90 ||
                    boundary_class == NF_BOUNDARY_LARGE_180;
                const double post_cross_mm = config->half_cell_mm +
                    ((double)plan->post_goal_extension_cells * 2.0 *
                     config->half_cell_mm);

                if (i + 1U != plan->action_count ||
                    action->logical_cells != plan->post_goal_extension_cells) {
                    return nf_validation_fail(validation, i,
                                              "invalid post-goal stopping tail");
                }
                expected_distance_mm = post_cross_mm -
                    (follows_large ? config->half_cell_mm : 0.0);
                expected_exit_velocity_mm_s = 0.0;
                if (expected_distance_mm < 0.0 ||
                    nf_motion_linear_plan(&config->straight,
                                          expected_distance_mm,
                                          expected_entry_velocity_mm_s, 0.0,
                                          &expected_linear) != NF_MOTION_OK ||
                    nf_motion_seconds_to_us(expected_linear.total_time_s,
                                            &expected_duration_us) != NF_MOTION_OK) {
                    return nf_validation_fail(validation, i,
                                              "post-goal stopping tail is infeasible");
                }
            } else {
                const NfRouteMotion *next_action;
                const NfTurnSpec *next_turn;
                int connector_half_sections;

                if (i + 1U >= plan->action_count) {
                    return nf_validation_fail(validation, i,
                                              "unterminated connector straight");
                }
                next_action = &plan->actions[i + 1U];
                next_turn = nf_turn_spec(config, next_action->kind);
                if (next_turn == NULL || !next_turn->enabled ||
                    ((next_action->kind == NF_ROUTE_MOTION_LARGE_90 ||
                      next_action->kind == NF_ROUTE_MOTION_LARGE_180) &&
                     !config->allow_large_turns)) {
                    return nf_validation_fail(validation, i,
                                              "connector does not lead to enabled turn");
                }
                connector_half_sections = (int)(2U * action->logical_cells) -
                    (int)nf_boundary_out_half_sections(boundary_class) -
                    (int)nf_turn_in_half_sections(next_action->kind);
                if (connector_half_sections < 0) {
                    return nf_validation_fail(validation, i,
                                              "negative connector distance");
                }
                expected_distance_mm =
                    (double)connector_half_sections * config->half_cell_mm;
                expected_exit_velocity_mm_s = next_turn->velocity_mm_s;
                if (nf_motion_linear_plan(&config->straight,
                                          expected_distance_mm,
                                          expected_entry_velocity_mm_s,
                                          expected_exit_velocity_mm_s,
                                          &expected_linear) != NF_MOTION_OK ||
                    nf_motion_seconds_to_us(expected_linear.total_time_s,
                                            &expected_duration_us) != NF_MOTION_OK) {
                    return nf_validation_fail(validation, i,
                                              "connector timing is infeasible");
                }
            }
            if (!nf_validation_motion_matches(action, expected_distance_mm,
                                              expected_entry_velocity_mm_s,
                                              expected_exit_velocity_mm_s,
                                              expected_duration_us)) {
                return nf_validation_fail(validation, i,
                                          "straight timing mismatch");
            }

            for (uint16_t cell = 1U; cell <= action->logical_cells; cell++) {
                uint8_t next_x;
                uint8_t next_y;
                const bool goal_was_seen = goal_seen;
                if (!nf_route_move(maze, x, y, heading, &next_x, &next_y)) {
                    return nf_validation_fail(validation, i, "straight crosses a wall");
                }
                x = next_x;
                y = next_y;
                if (!goal_seen && maze->goals[y][x]) {
                    if (!action->has_goal_cross ||
                        action->goal_cross_after_cells != cell ||
                        action->goal_x != x || action->goal_y != y) {
                        return nf_validation_fail(validation, i,
                                                  "first goal entry is not marked");
                    }
                    marker_matched = true;
                    goal_seen = true;
                    observed_goal_x = x;
                    observed_goal_y = y;
                    observed_goal_heading = heading;
                    if (!nf_u64_add(elapsed_us, action->goal_cross_time_us,
                                    &observed_goal_us)) {
                        return nf_validation_fail(validation, i, "goal time overflow");
                    }
                } else if (goal_was_seen) {
                    observed_extension_cells++;
                }
            }
        } else {
            const NfTurnSpec *turn = nf_turn_spec(config, action->kind);
            const NfTurnPlan *turn_plan = nf_turn_plan(&timing_context,
                                                       action->kind);
            NfRouteDirection first_heading;
            uint8_t next_x;
            uint8_t next_y;
            uint64_t expected_turn_us;

            if (turn == NULL || turn_plan == NULL || !turn->enabled ||
                ((action->kind == NF_ROUTE_MOTION_LARGE_90 ||
                  action->kind == NF_ROUTE_MOTION_LARGE_180) &&
                 !config->allow_large_turns) ||
                (action->side != NF_ROUTE_SIDE_RIGHT &&
                 action->side != NF_ROUTE_SIDE_LEFT) ||
                nf_motion_seconds_to_us(turn_plan->total_time_s,
                                        &expected_turn_us) != NF_MOTION_OK ||
                !nf_validation_motion_matches(action,
                                              turn_plan->travel_distance_mm,
                                              turn->velocity_mm_s,
                                              turn->velocity_mm_s,
                                              expected_turn_us)) {
                return nf_validation_fail(validation, i, "invalid turn timing");
            }
            if (i > 0U && plan->actions[i - 1U].kind != NF_ROUTE_MOTION_STRAIGHT &&
                (nf_boundary_out_half_sections(boundary_class) != 0U ||
                 nf_turn_in_half_sections(action->kind) != 0U)) {
                return nf_validation_fail(validation, i,
                                          "turn is missing a connector action");
            }
            if (action->has_goal_cross) {
                const bool is_large =
                    action->kind == NF_ROUTE_MOTION_LARGE_90 ||
                    action->kind == NF_ROUTE_MOTION_LARGE_180;
                double expected_cross_s;
                double expected_cross_lateral_mm = 0.0;
                uint64_t expected_cross_us;
                NfMotionStatus cross_status;
                if (is_large) {
                    cross_status = nf_motion_turn_exit_boundary_cross(
                        turn, turn_plan, config->half_cell_mm,
                        &expected_cross_s, &expected_cross_lateral_mm);
                } else {
                    expected_cross_s = turn_plan->total_time_s;
                    cross_status = NF_MOTION_OK;
                }
                if (cross_status != NF_MOTION_OK ||
                    fabs(expected_cross_lateral_mm) >
                        config->half_cell_mm + NF_ROUTE_SPEED_EPS ||
                    nf_motion_seconds_to_us(expected_cross_s,
                                            &expected_cross_us) != NF_MOTION_OK ||
                    action->goal_cross_time_us != expected_cross_us ||
                    !nf_validation_close(plan->goal_entry_velocity_mm_s,
                                         turn->velocity_mm_s)) {
                    return nf_validation_fail(validation, i,
                                              "turn goal timing mismatch");
                }
            }

            first_heading = nf_route_turn_heading(heading, action->side);
            if (!nf_route_move(maze, x, y, first_heading, &next_x, &next_y)) {
                return nf_validation_fail(validation, i, "turn crosses its first wall");
            }
            x = next_x;
            y = next_y;
            move_count = 1U;
            if (action->kind == NF_ROUTE_MOTION_LARGE_180) {
                if (!goal_seen && maze->goals[y][x]) {
                    return nf_validation_fail(validation, i,
                                              "large-180 passes through first goal");
                }
                heading = nf_route_opposite(heading);
                if (!nf_route_move(maze, x, y, heading, &next_x, &next_y)) {
                    return nf_validation_fail(validation, i,
                                              "large-180 crosses its second wall");
                }
                x = next_x;
                y = next_y;
                move_count = 2U;
            } else {
                heading = first_heading;
            }
            if (!goal_seen && maze->goals[y][x]) {
                if (!action->has_goal_cross ||
                    action->goal_cross_after_cells != move_count ||
                    action->goal_x != x || action->goal_y != y) {
                    return nf_validation_fail(validation, i,
                                              "turn goal entry is not marked");
                }
                marker_matched = true;
                goal_seen = true;
                observed_goal_x = x;
                observed_goal_y = y;
                observed_goal_heading = heading;
                if (!nf_u64_add(elapsed_us, action->goal_cross_time_us,
                                &observed_goal_us)) {
                    return nf_validation_fail(validation, i, "goal time overflow");
                }
            }
            if (action->logical_cells != move_count) {
                return nf_validation_fail(validation, i, "turn logical length mismatch");
            }
            boundary_class = nf_boundary_for_turn(action->kind);
        }

        if (action->has_goal_cross && !marker_matched) {
            return nf_validation_fail(validation, i, "goal marker does not match maze goal");
        }
        if (action->end_x != x || action->end_y != y ||
            action->end_heading != heading) {
            return nf_validation_fail(validation, i, "action end pose mismatch");
        }
        if (!nf_u64_add(elapsed_us, action->duration_us, &elapsed_us)) {
            return nf_validation_fail(validation, i, "route time overflow");
        }
        velocity = action->exit_velocity_mm_s;
    }

    if (!goal_seen) {
        return nf_validation_fail(validation, plan->action_count,
                                  "route never enters a goal");
    }
    if (observed_goal_x != plan->goal_x || observed_goal_y != plan->goal_y ||
        observed_goal_heading != plan->goal_heading ||
        observed_goal_us != plan->goal_entry_us ||
        observed_extension_cells != plan->post_goal_extension_cells) {
        return nf_validation_fail(validation, plan->action_count,
                                  "goal result does not match replay");
    }
    if (elapsed_us != plan->stop_us || fabs(velocity) > NF_ROUTE_SPEED_EPS) {
        return nf_validation_fail(validation, plan->action_count,
                                  "stop result does not match replay");
    }
    if (validation != NULL) {
        validation->valid = true;
        validation->action_index = plan->action_count;
        snprintf(validation->message, sizeof(validation->message), "ok");
    }
    return true;
}
