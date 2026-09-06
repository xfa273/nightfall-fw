/* Host-only generator: use the reference planner's private geometry verbatim.
 * Build with generate_mcu_slalom_tables.sh; no firmware code executes here. */
#include "../../common/route/slalom_time_planner.c"
#include "../../common/route/mcu_slalom_tables.h"
#include "slalom_time_plan_host.h"
#include <assert.h>

#define POOL_CAPACITY 8192U
#define LOCAL_MIN 4
#define LOCAL_MAX 12
#define LOCAL_SIDE 9
#define ORIGIN_CELL 8
static NfMcuTurnTemplate templates[3][8][7][2];
static NfMcuCrossedEdge edge_pool[POOL_CAPACITY];
static NfMcuGoalCross cross_pool[POOL_CAPACITY];
static size_t edge_count, cross_count;
static uint32_t connectors[2][3][4][65];
static uint32_t stop_cross[2][3][65][65];
static uint32_t turn_times[3][7];
static uint32_t stop_values[2U * 3U * NF_MCU_STOP_CROSS_TRIANGLE_COUNT];
static uint16_t stop_indices[2][3][NF_MCU_STOP_CROSS_TRIANGLE_COUNT];
static size_t stop_value_count;
static const double speeds[4] = {500.0, 300.0, 100.0, 0.0};

static void require(bool condition, const char *message) {
    if (!condition) {
        fprintf(stderr, "generator assertion: %s\n", message);
        exit(1);
    }
}
static uint32_t microseconds(double seconds) {
    uint64_t value;
    require(nf_slalom_seconds_to_us(seconds, &value) == NF_SLALOM_PLAN_OK && value < UINT32_MAX,
            "microsecond conversion overflow");
    return (uint32_t)value;
}
static void block(NfRouteMaze *maze, int x, int y, unsigned direction, bool wall) {
    const int nx = x + (direction == 1), ny = y + (direction == 0);
    const uint8_t here = k_wall_masks[direction], there = k_wall_masks[direction + 2];
    if (wall) {
        maze->walls[y][x] |= here;
        maze->walls[ny][nx] |= there;
    } else {
        maze->walls[y][x] &= (uint8_t)~here;
        maze->walls[ny][nx] &= (uint8_t)~there;
    }
}
static bool trace(const NfSlalomContext *context, NfSlalomAnchor source, unsigned heading,
                  unsigned kind, unsigned variant, NfRouteSide side, NfSlalomAnchor destination,
                  NfSlalomHeading8 end_heading, NfTurnTrace *out) {
    return nf_slalom_trace_turn(context, source, (NfSlalomHeading8)heading,
                                (NfSlalomActionKind)kind, (NfSlalomTurnVariant)variant, side,
                                destination, end_heading, out) == NF_SLALOM_PLAN_OK &&
           out->feasible;
}
/* The combined predicate is monotone in open edges. Single-edge blocking
 * establishes each dependency's necessity; closing every other internal edge
 * establishes the conjunction's sufficiency. Both are checked on the entire
 * 32x32 reference maze, not merely a chosen local bounding box. */
static bool guarded_trace(const NfSlalomContext *context, NfSlalomAnchor source, unsigned heading,
                          unsigned kind, unsigned variant, NfRouteSide side,
                          NfSlalomAnchor destination, NfSlalomHeading8 end_heading,
                          NfTurnTrace *out) {
    NfSlalomAnchor actual;
    NfSlalomHeading8 actual_heading;
    return nf_slalom_turn_destination(context->maze, source, (NfSlalomHeading8)heading,
                                      (NfSlalomActionKind)kind, side, &actual, &actual_heading) &&
           actual.half_x == destination.half_x && actual.half_y == destination.half_y &&
           actual_heading == end_heading &&
           trace(context, source, heading, kind, variant, side, destination, end_heading, out);
}
static void linear_tables(NfSlalomContext *context) {
    require(fabs(context->start_boundary_velocity_mm_s - speeds[2]) < 1e-6,
            "START speed is no longer crawl100; update MCU state schema");
    for (unsigned variant = 0; variant < 3; variant++)
        for (unsigned kind = 1; kind <= 7; kind++) {
            require(fabs(context->turn_specs[variant][kind].velocity_mm_s - speeds[variant]) < 1e-6,
                    "turn boundary speed changed; update MCU state schema");
            turn_times[variant][kind - 1] =
                microseconds(context->turn_plans[variant][kind].total_time_s);
        }
    for (unsigned diagonal = 0; diagonal < 2; diagonal++) {
        const NfSlalomHeading8 heading =
            diagonal ? NF_SLALOM_HEADING_NORTH_EAST : NF_SLALOM_HEADING_NORTH;
        const double unit = nf_slalom_connector_command_unit(context->config, heading);
        const NfLinearLimits *limits = nf_slalom_connector_limits(context->config, heading);
        for (unsigned entry = 0; entry < 3; entry++)
            for (unsigned exit_index = 0; exit_index < 4; exit_index++) {
                for (unsigned steps = 0; steps <= 64; steps++) {
                    NfLinearPlan plan;
                    connectors[diagonal][entry][exit_index][steps] =
                        nf_motion_linear_plan(limits, steps * unit, speeds[entry],
                                              speeds[exit_index], &plan) == NF_MOTION_OK
                            ? microseconds(plan.total_time_s)
                            : UINT32_MAX;
                }
            }
        for (unsigned entry = 0; entry < 3; entry++)
            for (unsigned stop_steps = 0; stop_steps <= 64; stop_steps++) {
                NfLinearPlan plan;
                const bool feasible =
                    nf_motion_linear_plan(limits, stop_steps * unit, speeds[entry], 0.0, &plan) ==
                    NF_MOTION_OK;
                for (unsigned cross_steps = 0; cross_steps <= 64; cross_steps++) {
                    double elapsed, velocity;
                    stop_cross[diagonal][entry][stop_steps][cross_steps] =
                        feasible && cross_steps <= stop_steps &&
                                nf_motion_linear_time_at_distance(
                                    &plan, cross_steps * unit, &elapsed, &velocity) == NF_MOTION_OK
                            ? microseconds(elapsed)
                            : UINT32_MAX;
                }
            }
    }
}
static int compare_u32(const void *left, const void *right) {
    const uint32_t a = *(const uint32_t *)left, b = *(const uint32_t *)right;
    return (a > b) - (a < b);
}
static void compress_stop_tables(void) {
    size_t count = 0;
    for (unsigned d = 0; d < 2; d++)
        for (unsigned e = 0; e < 3; e++)
            for (unsigned n = 0; n <= 64; n++)
                for (unsigned c = 0; c <= n; c++)
                    stop_values[count++] = stop_cross[d][e][n][c];
    qsort(stop_values, count, sizeof(uint32_t), compare_u32);
    for (size_t i = 0; i < count; i++)
        if (i == 0 || stop_values[i] != stop_values[i - 1])
            stop_values[stop_value_count++] = stop_values[i];
    require(stop_value_count < UINT16_MAX, "stop dictionary exceeds uint16 index");
    for (unsigned d = 0; d < 2; d++)
        for (unsigned e = 0; e < 3; e++)
            for (unsigned n = 0; n <= 64; n++)
                for (unsigned c = 0; c <= n; c++) {
                    const uint32_t value = stop_cross[d][e][n][c];
                    const uint32_t *found = bsearch(&value, stop_values, stop_value_count,
                                                    sizeof(uint32_t), compare_u32);
                    require(found != NULL, "stop dictionary lookup");
                    stop_indices[d][e][n * (n + 1U) / 2U + c] = (uint16_t)(found - stop_values);
                    require(stop_values[stop_indices[d][e][n * (n + 1U) / 2U + c]] == value,
                            "lossy dictionary compression");
                }
    fprintf(stderr, "stop_dictionary=%zu compressed_bytes=%zu\n", stop_value_count,
            sizeof(stop_indices) + stop_value_count * sizeof(uint32_t));
}
static int cross_order(const void *left, const void *right) {
    const NfMcuGoalCross *a = left, *b = right;
    if (a->time_us[0] != b->time_us[0])
        return a->time_us[0] < b->time_us[0] ? -1 : 1;
    if (a->dy != b->dy)
        return a->dy < b->dy ? -1 : 1;
    return (a->dx > b->dx) - (a->dx < b->dx);
}
static void geometry_tables(NfSlalomContext *context, NfRouteMaze *maze) {
    const NfSlalomAnchor sources[3] = {{17, 17}, {16, 17}, {17, 16}};
    unsigned valid_count = 0;
    for (unsigned anchor_class = 0; anchor_class < 3; anchor_class++)
        for (unsigned heading = 0; heading < 8; heading++)
            for (unsigned kind_index = 0; kind_index < 7; kind_index++)
                for (unsigned side_index = 0; side_index < 2; side_index++) {
                    const unsigned kind = kind_index + 1;
                    const NfRouteSide side = side_index ? NF_ROUTE_SIDE_LEFT : NF_ROUTE_SIDE_RIGHT;
                    const NfSlalomAnchor source = sources[anchor_class];
                    NfSlalomAnchor destination;
                    NfSlalomHeading8 end_heading;
                    NfTurnTrace traced;
                    NfMcuTurnTemplate *item =
                        &templates[anchor_class][heading][kind_index][side_index];
                    item->edge_count = NF_MCU_SLALOM_INVALID_TEMPLATE;
                    /* Establish the template destination on all-open geometry; perturbations
                     * below rerun both exact KERI topology and sampled centre-line trace. */
                    if (!nf_slalom_turn_destination(maze, source, (NfSlalomHeading8)heading,
                                                    (NfSlalomActionKind)kind, side, &destination,
                                                    &end_heading))
                        continue;
                    const bool nominal_valid = trace(context, source, heading, kind, 0, side,
                                                     destination, end_heading, &traced);
                    for (unsigned variant = 1; variant < 3; variant++) {
                        require(trace(context, source, heading, kind, variant, side, destination,
                                      end_heading, &traced) == nominal_valid,
                                "variant geometry feasibility differs");
                    }
                    if (!nominal_valid)
                        continue;
                    bool required[3][32][32][2] = {{{{false}}}};
                    for (unsigned variant = 0; variant < 3; variant++)
                        for (int y = 0; y < 32; y++)
                            for (int x = 0; x < 32; x++)
                                for (unsigned direction = 0; direction < 2; direction++) {
                                    if ((direction == 0 && y == 31) || (direction == 1 && x == 31))
                                        continue;
                                    block(maze, x, y, direction, true);
                                    required[variant][y][x][direction] =
                                        !guarded_trace(context, source, heading, kind, variant,
                                                       side, destination, end_heading, &traced);
                                    block(maze, x, y, direction, false);
                                }
                    require(
                        memcmp(required[0], required[1], sizeof(required[0])) == 0 &&
                            memcmp(required[0], required[2], sizeof(required[0])) == 0,
                        "speed variants have different combined topology/geometry requirements");
                    item->edge_count = 0;
                    item->edge_offset = (uint16_t)edge_count;
                    item->cross_offset = (uint16_t)cross_count;
                    item->delta_hx = (int8_t)(destination.half_x - source.half_x);
                    item->delta_hy = (int8_t)(destination.half_y - source.half_y);
                    item->end_heading = (uint8_t)end_heading;
                    for (int y = 0; y < 32; y++)
                        for (int x = 0; x < 32; x++)
                            for (unsigned direction = 0; direction < 2; direction++) {
                                if ((direction == 0 && y == 31) || (direction == 1 && x == 31))
                                    continue;
                                if (required[0][y][x][direction]) {
                                    require(edge_count < POOL_CAPACITY && item->edge_count < 254,
                                            "edge pool capacity");
                                    edge_pool[edge_count++] = (NfMcuCrossedEdge){
                                        (int8_t)(x - ORIGIN_CELL), (int8_t)(y - ORIGIN_CELL),
                                        (uint8_t)direction};
                                    item->edge_count++;
                                } else
                                    block(maze, x, y, direction, true);
                            }
                    /* Independent sufficiency: every other internal edge in 32x32 blocked. */
                    for (unsigned variant = 0; variant < 3; variant++)
                        require(guarded_trace(context, source, heading, kind, variant, side,
                                              destination, end_heading, &traced),
                                "single-edge dependency conjunction is not sufficient for full "
                                "KERI+geometry predicate");
                    for (int y = 0; y < 32; y++)
                        for (int x = 0; x < 32; x++)
                            for (unsigned direction = 0; direction < 2; direction++)
                                if (!((direction == 0 && y == 31) || (direction == 1 && x == 31)))
                                    block(maze, x, y, direction, false);
                    for (int y = LOCAL_MIN; y <= LOCAL_MAX; y++)
                        for (int x = LOCAL_MIN; x <= LOCAL_MAX; x++) {
                            NfMcuGoalCross crossing = {
                                (int8_t)(x - ORIGIN_CELL), (int8_t)(y - ORIGIN_CELL), {0, 0, 0}};
                            bool present[3];
                            maze->goals[y][x] = true;
                            for (unsigned variant = 0; variant < 3; variant++) {
                                require(trace(context, source, heading, kind, variant, side,
                                              destination, end_heading, &traced),
                                        "goal changed geometry");
                                present[variant] = traced.has_goal;
                                if (traced.has_goal)
                                    crossing.time_us[variant] = microseconds(traced.goal_time_s);
                            }
                            maze->goals[y][x] = false;
                            require(present[0] == present[1] && present[0] == present[2],
                                    "variant goal-cell sets differ");
                            if (present[0]) {
                                require(cross_count < POOL_CAPACITY && item->cross_count < 254,
                                        "goal pool capacity");
                                cross_pool[cross_count++] = crossing;
                                item->cross_count++;
                            }
                        }
                    qsort(cross_pool + item->cross_offset, item->cross_count,
                          sizeof(NfMcuGoalCross), cross_order);
                    for (unsigned i = 1; i < item->cross_count; i++)
                        for (unsigned variant = 0; variant < 3; variant++) {
                            require(cross_pool[item->cross_offset + i - 1].time_us[variant] <=
                                        cross_pool[item->cross_offset + i].time_us[variant],
                                    "speed variants change chronological cell order");
                        }
                    require(item->cross_count > 0 && cross_pool[item->cross_offset].time_us[0] == 0,
                            "source-cell t0 crossing missing");
                    valid_count++;
                }
    fprintf(stderr, "templates=%u required_edges=%zu goal_crossings=%zu\n", valid_count, edge_count,
            cross_count);
}
static void emit_u32(FILE *out, uint32_t value) {
    if (value == UINT32_MAX)
        fputs("UINT32_MAX", out);
    else
        fprintf(out, "%uU", value);
}
static void write_tables(FILE *out, uint32_t start_us, const char *inputs_sha256) {
    fputs(
        "/* Generated by tools/solver_host/generate_mcu_slalom_tables.sh.\n * f413-preorder-mode2 "
        "case8; reference planner source 4fb45ed.\n * Timing is integer microseconds; required "
        "edges include exact KERI guards and geometry. */\n#include \"mcu_slalom_tables.h\"\n\n",
        out);
    fprintf(out, "const char nf_mcu_slalom_inputs_sha256[65] = \"%s\";\n", inputs_sha256);
    fprintf(out, "const uint32_t nf_mcu_start_us = %uU;\n", start_us);
    fputs("const uint32_t nf_mcu_turn_us[3][7] = {\n", out);
    for (unsigned v = 0; v < 3; v++) {
        fputs(" {", out);
        for (unsigned k = 0; k < 7; k++) {
            if (k)
                fputc(',', out);
            emit_u32(out, turn_times[v][k]);
        }
        fputs("},\n", out);
    }
    fputs("};\n", out);
    fputs("const uint32_t nf_mcu_connector_us[2][3][4][65] = {\n", out);
    for (unsigned d = 0; d < 2; d++) {
        fputs(" {\n", out);
        for (unsigned e = 0; e < 3; e++) {
            fputs("  {\n", out);
            for (unsigned x = 0; x < 4; x++) {
                fputs("   {", out);
                for (unsigned n = 0; n <= 64; n++) {
                    if (n)
                        fputc(',', out);
                    emit_u32(out, connectors[d][e][x][n]);
                }
                fputs("},\n", out);
            }
            fputs("  },\n", out);
        }
        fputs(" },\n", out);
    }
    fputs("};\n", out);
    fprintf(out, "const uint16_t nf_mcu_stop_cross_value_count = %zuU;\n", stop_value_count);
    fputs("const uint32_t nf_mcu_stop_cross_values[] = {\n", out);
    for (size_t i = 0; i < stop_value_count; i++) {
        if (i)
            fputc(',', out);
        if (i % 16 == 0)
            fputs("\n ", out);
        emit_u32(out, stop_values[i]);
    }
    fputs("\n};\n", out);
    fputs("const uint16_t nf_mcu_stop_cross_index[2][3][NF_MCU_STOP_CROSS_TRIANGLE_COUNT] = {\n",
          out);
    for (unsigned d = 0; d < 2; d++) {
        fputs(" {\n", out);
        for (unsigned e = 0; e < 3; e++) {
            fputs("  {", out);
            for (unsigned i = 0; i < NF_MCU_STOP_CROSS_TRIANGLE_COUNT; i++) {
                if (i)
                    fputc(',', out);
                if (i % 32 == 0)
                    fputs("\n   ", out);
                fprintf(out, "%u", stop_indices[d][e][i]);
            }
            fputs("\n  },\n", out);
        }
        fputs(" },\n", out);
    }
    fputs("};\n", out);
    fputs("const NfMcuTurnTemplate nf_mcu_turn_templates[3][8][7][2] = {\n", out);
    for (unsigned a = 0; a < 3; a++) {
        fputs(" {\n", out);
        for (unsigned h = 0; h < 8; h++) {
            fputs("  {\n", out);
            for (unsigned k = 0; k < 7; k++) {
                fputs("   {", out);
                for (unsigned s = 0; s < 2; s++) {
                    const NfMcuTurnTemplate *t = &templates[a][h][k][s];
                    if (s)
                        fputc(',', out);
                    fprintf(out, "{%d,%d,%u,%u,%u,%u,%u}", t->delta_hx, t->delta_hy, t->end_heading,
                            t->edge_count, t->cross_count, t->edge_offset, t->cross_offset);
                }
                fputs("},\n", out);
            }
            fputs("  },\n", out);
        }
        fputs(" },\n", out);
    }
    fputs("};\n", out);
    fputs("const NfMcuCrossedEdge nf_mcu_required_turn_edges[] = {\n", out);
    for (size_t i = 0; i < edge_count; i++)
        fprintf(out, " {%d,%d,%u},\n", edge_pool[i].dx, edge_pool[i].dy, edge_pool[i].dir);
    fputs("};\n", out);
    fputs("const NfMcuGoalCross nf_mcu_goal_crosses[] = {\n", out);
    for (size_t i = 0; i < cross_count; i++)
        fprintf(out, " {%d,%d,{%uU,%uU,%uU}},\n", cross_pool[i].dx, cross_pool[i].dy,
                cross_pool[i].time_us[0], cross_pool[i].time_us[1], cross_pool[i].time_us[2]);
    fputs("};\n", out);
    fprintf(out,
            "const uint16_t nf_mcu_required_turn_edge_count = %zuU;\nconst uint16_t "
            "nf_mcu_goal_cross_count = %zuU;\n",
            edge_count, cross_count);
}
int main(int argc, char **argv) {
    NfRouteMaze maze;
    NfSlalomPlannerConfig config;
    const NfAuditProfile *profile;
    char error[256];
    require(argc == 3,
            "usage: generate_mcu_slalom_tables OUTPUT.c INPUTS_SHA256 (use the .sh wrapper)");
    require(strlen(argv[2]) == 64 && strspn(argv[2], "0123456789abcdef") == 64,
            "INPUTS_SHA256 must contain 64 lowercase hexadecimal characters");
    require(nf_route_maze_init(&maze, 32, 32) && nf_route_maze_add_boundaries(&maze), "maze init");
    require(nf_host_slalom_make_config("f413-preorder-mode2", 8, NF_SLALOM_ENABLE_SHORTEST_1_TO_5,
                                       &config, &profile, error, sizeof(error)),
            error);
    NfSlalomContext context = {0};
    context.maze = &maze;
    context.config = &config;
    require(nf_slalom_prepare_config(&context) == NF_SLALOM_PLAN_OK, "prepare fixed configuration");
    linear_tables(&context);
    compress_stop_tables();
    geometry_tables(&context, &maze);
    FILE *out = fopen(argv[1], "wb");
    require(out != NULL, "open output");
    write_tables(out, (uint32_t)context.start_time_us, argv[2]);
    require(fclose(out) == 0, "write output");
    nf_slalom_free_turn_trajectories(&context);
    return 0;
}
