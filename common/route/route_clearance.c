#include "route_clearance.h"

#include <float.h>
#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define NF_CLEARANCE_PI 3.14159265358979323846264338327950288
#define NF_CLEARANCE_EPS 1.0e-9

typedef struct {
    double center_x;
    double center_y;
    double half_x;
    double half_y;
    uint8_t cell_x;
    uint8_t cell_y;
    NfRouteDirection direction;
} NfWallRectangle;

static bool nf_clearance_finite_positive(double value)
{
    return isfinite(value) && value > 0.0;
}

static bool nf_clearance_valid_maze(const NfRouteMaze *maze)
{
    if (maze == NULL || maze->width == 0U || maze->height == 0U ||
        maze->width > NF_ROUTE_MAZE_MAX_SIZE ||
        maze->height > NF_ROUTE_MAZE_MAX_SIZE) {
        return false;
    }
    for (uint8_t y = 0U; y < maze->height; y++) {
        for (uint8_t x = 0U; x < maze->width; x++) {
            const uint8_t walls = maze->walls[y][x];
            if ((walls & (uint8_t)~0x0FU) != 0U ||
                (x == 0U && (walls & NF_ROUTE_WALL_WEST) == 0U) ||
                (x + 1U == maze->width &&
                 (walls & NF_ROUTE_WALL_EAST) == 0U) ||
                (y == 0U && (walls & NF_ROUTE_WALL_SOUTH) == 0U) ||
                (y + 1U == maze->height &&
                 (walls & NF_ROUTE_WALL_NORTH) == 0U)) {
                return false;
            }
            if (x + 1U < maze->width &&
                ((walls & NF_ROUTE_WALL_EAST) != 0U) !=
                ((maze->walls[y][x + 1U] & NF_ROUTE_WALL_WEST) != 0U)) {
                return false;
            }
            if (y + 1U < maze->height &&
                ((walls & NF_ROUTE_WALL_NORTH) != 0U) !=
                ((maze->walls[y + 1U][x] & NF_ROUTE_WALL_SOUTH) != 0U)) {
                return false;
            }
        }
    }
    return true;
}

static bool nf_clearance_valid_config(const NfClearanceConfig *config)
{
    return config != NULL &&
           nf_clearance_finite_positive(config->cell_pitch_mm) &&
           nf_clearance_finite_positive(config->wall_thickness_mm) &&
           config->wall_thickness_mm < config->cell_pitch_mm &&
           nf_clearance_finite_positive(config->robot_half_length_mm) &&
           nf_clearance_finite_positive(config->robot_half_width_mm) &&
           nf_clearance_finite_positive(config->max_translation_step_mm) &&
           nf_clearance_finite_positive(config->max_heading_step_deg);
}

static bool nf_clearance_rectangles_intersect(
    double robot_x,
    double robot_y,
    double robot_heading_rad,
    double robot_half_length,
    double robot_half_width,
    const NfWallRectangle *wall)
{
    const double ux = cos(robot_heading_rad);
    const double uy = sin(robot_heading_rad);
    const double vx = -uy;
    const double vy = ux;
    const double dx = wall->center_x - robot_x;
    const double dy = wall->center_y - robot_y;
    double robot_projection;
    double wall_projection;

    robot_projection = robot_half_length * fabs(ux) +
                       robot_half_width * fabs(vx);
    if (fabs(dx) > wall->half_x + robot_projection) {
        return false;
    }
    robot_projection = robot_half_length * fabs(uy) +
                       robot_half_width * fabs(vy);
    if (fabs(dy) > wall->half_y + robot_projection) {
        return false;
    }

    wall_projection = wall->half_x * fabs(ux) + wall->half_y * fabs(uy);
    if (fabs((dx * ux) + (dy * uy)) >
        robot_half_length + wall_projection) {
        return false;
    }
    wall_projection = wall->half_x * fabs(vx) + wall->half_y * fabs(vy);
    if (fabs((dx * vx) + (dy * vy)) >
        robot_half_width + wall_projection) {
        return false;
    }
    return true;
}

static bool nf_clearance_wall_at(const NfRouteMaze *maze,
                                 uint8_t x,
                                 uint8_t y,
                                 NfRouteDirection direction)
{
    static const uint8_t masks[4] = {
        NF_ROUTE_WALL_NORTH,
        NF_ROUTE_WALL_EAST,
        NF_ROUTE_WALL_SOUTH,
        NF_ROUTE_WALL_WEST,
    };
    return (maze->walls[y][x] & masks[(unsigned int)direction]) != 0U;
}

static NfWallRectangle nf_clearance_wall_rectangle(
    const NfClearanceConfig *config,
    uint8_t x,
    uint8_t y,
    NfRouteDirection direction)
{
    const double pitch = config->cell_pitch_mm;
    const double half_wall = 0.5 * config->wall_thickness_mm;
    NfWallRectangle wall;

    memset(&wall, 0, sizeof(wall));
    wall.cell_x = x;
    wall.cell_y = y;
    wall.direction = direction;
    if (direction == NF_ROUTE_DIR_NORTH ||
        direction == NF_ROUTE_DIR_SOUTH) {
        wall.center_x = ((double)x + 0.5) * pitch;
        wall.center_y = ((double)y +
                         ((direction == NF_ROUTE_DIR_NORTH) ? 1.0 : 0.0)) * pitch;
        wall.half_x = (0.5 * pitch) + half_wall;
        wall.half_y = half_wall;
    } else {
        wall.center_x = ((double)x +
                         ((direction == NF_ROUTE_DIR_EAST) ? 1.0 : 0.0)) * pitch;
        wall.center_y = ((double)y + 0.5) * pitch;
        wall.half_x = half_wall;
        wall.half_y = (0.5 * pitch) + half_wall;
    }
    return wall;
}

static bool nf_clearance_check_all_walls(
    const NfRouteMaze *maze,
    const NfClearanceConfig *config,
    double robot_x,
    double robot_y,
    double robot_heading_rad,
    double robot_half_length,
    double robot_half_width,
    NfWallRectangle *out_wall)
{
    for (uint8_t y = 0U; y < maze->height; y++) {
        for (uint8_t x = 0U; x < maze->width; x++) {
            static const NfRouteDirection unique_directions[2] = {
                NF_ROUTE_DIR_NORTH,
                NF_ROUTE_DIR_EAST,
            };
            for (size_t i = 0U; i < 2U; i++) {
                const NfRouteDirection direction = unique_directions[i];
                NfWallRectangle wall;
                if (!nf_clearance_wall_at(maze, x, y, direction)) {
                    continue;
                }
                wall = nf_clearance_wall_rectangle(config, x, y, direction);
                if (nf_clearance_rectangles_intersect(
                        robot_x, robot_y, robot_heading_rad,
                        robot_half_length, robot_half_width, &wall)) {
                    *out_wall = wall;
                    return true;
                }
            }
            if (y == 0U &&
                nf_clearance_wall_at(maze, x, y, NF_ROUTE_DIR_SOUTH)) {
                const NfWallRectangle wall = nf_clearance_wall_rectangle(
                    config, x, y, NF_ROUTE_DIR_SOUTH);
                if (nf_clearance_rectangles_intersect(
                        robot_x, robot_y, robot_heading_rad,
                        robot_half_length, robot_half_width, &wall)) {
                    *out_wall = wall;
                    return true;
                }
            }
            if (x == 0U &&
                nf_clearance_wall_at(maze, x, y, NF_ROUTE_DIR_WEST)) {
                const NfWallRectangle wall = nf_clearance_wall_rectangle(
                    config, x, y, NF_ROUTE_DIR_WEST);
                if (nf_clearance_rectangles_intersect(
                        robot_x, robot_y, robot_heading_rad,
                        robot_half_length, robot_half_width, &wall)) {
                    *out_wall = wall;
                    return true;
                }
            }
        }
    }
    return false;
}

const char *nf_clearance_status_name(NfClearanceStatus status)
{
    switch (status) {
    case NF_CLEARANCE_OK: return "ok";
    case NF_CLEARANCE_INVALID_ARGUMENT: return "invalid-argument";
    case NF_CLEARANCE_COLLISION: return "collision";
    case NF_CLEARANCE_OVERFLOW: return "overflow";
    default: return "unknown";
    }
}

NfClearanceStatus nf_route_turn_clearance(
    const NfRouteMaze *maze,
    const NfClearanceConfig *config,
    const NfTurnSpec *turn,
    const NfTurnPlan *plan,
    double start_x_mm,
    double start_y_mm,
    double start_heading_deg,
    bool turn_left,
    NfClearanceResult *out)
{
    double translation_samples;
    double heading_samples;
    double required_samples;
    uint32_t intervals;
    double interval_time_s;
    double between_sample_margin_mm;
    NfTurnPose *poses;
    const double robot_corner_radius = (config != NULL) ?
        hypot(config->robot_half_length_mm, config->robot_half_width_mm) : 0.0;
    const double start_heading_rad =
        start_heading_deg * (NF_CLEARANCE_PI / 180.0);

    if (out == NULL || !nf_clearance_valid_maze(maze) ||
        !nf_clearance_valid_config(config) || turn == NULL || plan == NULL ||
        !isfinite(start_x_mm) || !isfinite(start_y_mm) ||
        !isfinite(start_heading_deg) || !isfinite(plan->total_time_s) ||
        plan->total_time_s <= 0.0 || !isfinite(plan->travel_distance_mm) ||
        plan->travel_distance_mm <= 0.0 ||
        !isfinite(plan->omega_peak_deg_s) || plan->omega_peak_deg_s <= 0.0) {
        return NF_CLEARANCE_INVALID_ARGUMENT;
    }
    memset(out, 0, sizeof(*out));

    translation_samples = plan->travel_distance_mm /
                          config->max_translation_step_mm;
    heading_samples = (plan->omega_peak_deg_s * plan->total_time_s) /
                      config->max_heading_step_deg;
    required_samples = fmax(translation_samples, heading_samples);
    if (!isfinite(required_samples) || required_samples > (double)UINT32_MAX) {
        return NF_CLEARANCE_OVERFLOW;
    }
    intervals = (uint32_t)ceil(required_samples);
    if (intervals == 0U) {
        intervals = 1U;
    }
    if (intervals == UINT32_MAX) {
        return NF_CLEARANCE_OVERFLOW;
    }
    out->sample_count = intervals + 1U;
    if ((size_t)out->sample_count > SIZE_MAX / sizeof(*poses)) {
        return NF_CLEARANCE_OVERFLOW;
    }
    poses = (NfTurnPose *)malloc((size_t)out->sample_count * sizeof(*poses));
    if (poses == NULL) {
        return NF_CLEARANCE_OVERFLOW;
    }
    if (nf_motion_turn_pose_uniform(
            turn, plan, intervals, poses, out->sample_count) != NF_MOTION_OK) {
        free(poses);
        return NF_CLEARANCE_INVALID_ARGUMENT;
    }
    interval_time_s = plan->total_time_s / (double)intervals;
    between_sample_margin_mm =
        0.5 * ((turn->velocity_mm_s * interval_time_s) +
               (robot_corner_radius * plan->omega_peak_deg_s *
                interval_time_s * (NF_CLEARANCE_PI / 180.0)));

    for (uint32_t i = 0U; i <= intervals; i++) {
        const double elapsed_s = plan->total_time_s * (double)i /
                                 (double)intervals;
        const NfTurnPose local_pose = poses[i];
        NfWallRectangle wall;
        double lateral_mm;
        double heading_deg;
        double robot_x;
        double robot_y;
        double robot_heading_rad;
        lateral_mm = turn_left ? local_pose.lateral_mm : -local_pose.lateral_mm;
        heading_deg = turn_left ? local_pose.heading_deg : -local_pose.heading_deg;
        robot_x = start_x_mm +
            (local_pose.forward_mm * cos(start_heading_rad)) -
            (lateral_mm * sin(start_heading_rad));
        robot_y = start_y_mm +
            (local_pose.forward_mm * sin(start_heading_rad)) +
            (lateral_mm * cos(start_heading_rad));
        robot_heading_rad = (start_heading_deg + heading_deg) *
                            (NF_CLEARANCE_PI / 180.0);

        if (nf_clearance_check_all_walls(
                maze, config, robot_x, robot_y, robot_heading_rad,
                config->robot_half_length_mm + between_sample_margin_mm,
                config->robot_half_width_mm + between_sample_margin_mm,
                &wall)) {
            out->clear = false;
            out->first_collision_time_s = elapsed_s;
            out->robot_x_mm = robot_x;
            out->robot_y_mm = robot_y;
            out->robot_heading_deg = start_heading_deg + heading_deg;
            out->wall_x = wall.cell_x;
            out->wall_y = wall.cell_y;
            out->wall_direction = wall.direction;
            free(poses);
            return NF_CLEARANCE_COLLISION;
        }
    }

    out->clear = true;
    free(poses);
    return NF_CLEARANCE_OK;
}
