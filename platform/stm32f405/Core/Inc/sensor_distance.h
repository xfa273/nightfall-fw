/*
 * sensor_distance.h
 *
 *  Front wall sensor distance conversion (AD -> distance [mm]) using LUTs.
 *  - Individual LUTs for FL and FR
 *  - Supports non-uniform distance grid (e.g., 0..20mm at 1mm step, then coarse)
 *  - Monotonic assumption: distance increases as AD decreases
 */
#ifndef INC_SENSOR_DISTANCE_H_
#define INC_SENSOR_DISTANCE_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Maximum number of LUT points per sensor
#ifndef SENSOR_DIST_LUT_MAX_POINTS
#define SENSOR_DIST_LUT_MAX_POINTS 128
#endif

// Initialize module with default coarse LUT (0..90mm, 10mm step)
// Safe to call multiple times.
void sensor_distance_init(void);

// Set/replace FL/FR/L/R LUTs
//  - mm[i]: distance in mm (monotonically increasing)
//  - ad[i]: AD counts (monotonically decreasing wrt mm)
//  - n: number of points (>=2, <= SENSOR_DIST_LUT_MAX_POINTS)
// Returns 0 on success, -1 on invalid args.
int sensor_distance_set_lut_fl(const uint16_t *mm, const uint16_t *ad, size_t n);
int sensor_distance_set_lut_fr(const uint16_t *mm, const uint16_t *ad, size_t n);
int sensor_distance_set_lut_l (const uint16_t *mm, const uint16_t *ad, size_t n);
int sensor_distance_set_lut_r (const uint16_t *mm, const uint16_t *ad, size_t n);

// Query current LUT sizes
size_t sensor_distance_lut_size_fl(void);
size_t sensor_distance_lut_size_fr(void);
size_t sensor_distance_lut_size_l (void);
size_t sensor_distance_lut_size_r (void);

// Convert AD -> distance [mm] using LUTs with linear interpolation.
// If input is outside the LUT AD range, perform linear extrapolation at the nearest edge (no clipping).
// Returns distance in mm. If LUT is not initialized, default LUT is used where available.
float sensor_distance_from_fl(uint16_t ad_value);
float sensor_distance_from_fr(uint16_t ad_value);
// Unwarped accessors (LUT only)
float sensor_distance_from_fl_unwarped(uint16_t ad_value);
float sensor_distance_from_fr_unwarped(uint16_t ad_value);
float sensor_distance_from_l (uint16_t ad_value);
float sensor_distance_from_r (uint16_t ad_value);

// Combined front distance using both sensors (FL+FR)
// Default LUT is generated from the provided FL/FR tables (sum per distance).
// - sensor_distance_from_fsum: use pre-summed AD (FR+FL)
// - sensor_distance_from_front_sum: take both ADs and sum inside
int   sensor_distance_set_lut_front_sum(const uint16_t *mm, const uint16_t *ad_sum, size_t n);
size_t sensor_distance_lut_size_front_sum(void);
float sensor_distance_from_fsum(uint16_t ad_sum);
float sensor_distance_from_front_sum(uint16_t ad_fl, uint16_t ad_fr);

// Optional distance-domain warp (mm_est -> mm_true) for FRONT SUM only.
// Use a 3-point monotone cubic (PCHIP) mapping. If not set, identity is used.
// - x_mm_est[3]: distances estimated by current LUT at anchor ADs
// - y_mm_true[3]: ground-truth distances (e.g., {0,24,114})
void sensor_distance_set_warp_front_sum_3pt(const float x_mm_est[3], const float y_mm_true[3]);
void sensor_distance_clear_warp_front_sum(void);

// Raw distance from FRONT SUM LUT without warp (uses SENSOR_DIST_GAIN).
float sensor_distance_from_fsum_unwarped(uint16_t ad_sum);

// Optional distance-domain warps for individual FL/FR channels
void sensor_distance_set_warp_fl_3pt(const float x_mm_est[3], const float y_mm_true[3]);
void sensor_distance_clear_warp_fl(void);
void sensor_distance_set_warp_fr_3pt(const float x_mm_est[3], const float y_mm_true[3]);
void sensor_distance_clear_warp_fr(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_SENSOR_DISTANCE_H_ */
