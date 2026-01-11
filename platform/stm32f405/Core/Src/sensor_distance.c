/*
 * sensor_distance.c
 */
#include "sensor_distance.h"
#include <params.h>
#include <string.h>
#include <math.h>

// Internal storage for LUTs (FL/FR)
static uint16_t s_mm_fl[SENSOR_DIST_LUT_MAX_POINTS];
static uint16_t s_ad_fl[SENSOR_DIST_LUT_MAX_POINTS];
static size_t   s_n_fl = 0;

static uint16_t s_mm_fr[SENSOR_DIST_LUT_MAX_POINTS];
static uint16_t s_ad_fr[SENSOR_DIST_LUT_MAX_POINTS];
static size_t   s_n_fr = 0;

// Side wall sensors (L/R)
static uint16_t s_mm_l[SENSOR_DIST_LUT_MAX_POINTS];
static uint16_t s_ad_l[SENSOR_DIST_LUT_MAX_POINTS];
static size_t   s_n_l = 0;

static uint16_t s_mm_r[SENSOR_DIST_LUT_MAX_POINTS];
static uint16_t s_ad_r[SENSOR_DIST_LUT_MAX_POINTS];
static size_t   s_n_r = 0;

// Combined front (FL+FR) LUT storage
static uint16_t s_mm_fsum[SENSOR_DIST_LUT_MAX_POINTS];
static uint16_t s_ad_fsum[SENSOR_DIST_LUT_MAX_POINTS];
static size_t   s_n_fsum = 0;

// Optional FRONT-SUM distance-domain warp: mm_est -> mm_true (3-point PCHIP)
static int   s_fsum_warp_valid = 0;
static float s_warp_x[3]; // mm_est anchors (strictly increasing)
static float s_warp_y[3]; // mm_true anchors (strictly increasing)
static float s_warp_m[3]; // slopes at anchors

// Optional FL/FR distance-domain warps
static int   s_fl_warp_valid = 0;
static float s_warp_x_fl[3];
static float s_warp_y_fl[3];
static float s_warp_m_fl[3];

static int   s_fr_warp_valid = 0;
static float s_warp_x_fr[3];
static float s_warp_y_fr[3];
static float s_warp_m_fr[3];

// Default fine LUT: 0..20mm at 1mm steps, then 30..90mm at 10mm steps
// Values are based on provided measurements, with re-measured 4mm values applied:
//   FL@4mm=2092, FR@4mm=1957 (monotonicity preserved).
static const uint16_t s_mm_fine[] = {
    0,  1,  2,  3,  4,  5,  6,  7,  8,  9,
   10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
   20, 30, 40, 50, 60, 70, 80, 90
};
static const uint16_t s_fl_fine[] = {
    2644, 2500, 2363, 2202, 2092, 1919, 1818, 1705, 1595, 1522,
    1423, 1367, 1302, 1240, 1183, 1123, 1062, 1010,  964,  917,
     851,  558,  387,  284,  217,  168,  132,  107
};
static const uint16_t s_fr_fine[] = {
    2409, 2311, 2139, 2029, 1957, 1804, 1680, 1633, 1532, 1444,
    1337, 1308, 1226, 1163, 1126, 1057,  992,  958,  902,  845,
     794,  546,  368,  272,  203,  151,  129,  105
};
static const uint16_t s_l_side_fine[] = {
    1979, 1856, 1732, 1599, 1495, 1381, 1273, 1199, 1116, 1024,
     948,  894,  843,  794,  746,  706,  660,  634,  592,  562,
     532,  336,  225,  159,  118,   89,   68,   53
};
static const uint16_t s_r_side_fine[] = {
    2230, 2074, 1939, 1778, 1649, 1565, 1418, 1302, 1216, 1131,
    1075,  993,  915,  890,  807,  776,  745,  691,  651,  619,
     576,  378,  233,  157,  105,   79,   62,   45
};
static const size_t s_n_fine = sizeof(s_mm_fine)/sizeof(s_mm_fine[0]);

static int validate_lut(const uint16_t *mm, const uint16_t *ad, size_t n)
{
    if (!mm || !ad) return -1;
    if (n < 2 || n > SENSOR_DIST_LUT_MAX_POINTS) return -1;
    // distance: strictly increasing, ad: strictly decreasing (monotonic)
    for (size_t i = 1; i < n; ++i) {
        if (!(mm[i] > mm[i-1])) return -1;
        if (!(ad[i] < ad[i-1])) return -1;
    }
    return 0;
}

void sensor_distance_init(void)
{
    // Load default fine-grained tables
    (void)sensor_distance_set_lut_fl(s_mm_fine, s_fl_fine, s_n_fine);
    (void)sensor_distance_set_lut_fr(s_mm_fine, s_fr_fine, s_n_fine);
    // Load side (L/R) default LUTs using the same mm grid
    (void)sensor_distance_set_lut_l (s_mm_fine, s_l_side_fine, s_n_fine);
    (void)sensor_distance_set_lut_r (s_mm_fine, s_r_side_fine, s_n_fine);

    // Build default front-sum LUT from fine FL/FR at the same distances
    static uint16_t s_fsum_mm_init[sizeof(s_mm_fine)/sizeof(s_mm_fine[0])];
    static uint16_t s_fsum_ad_init[sizeof(s_mm_fine)/sizeof(s_mm_fine[0])];
    for (size_t i = 0; i < s_n_fine; ++i) {
        s_fsum_mm_init[i] = s_mm_fine[i];
        // Sum stays within 16-bit range; provided data sums up to ~5k
        s_fsum_ad_init[i] = (uint16_t)(s_fl_fine[i] + s_fr_fine[i]);
    }
    (void)sensor_distance_set_lut_front_sum(s_fsum_mm_init, s_fsum_ad_init, s_n_fine);
}

int sensor_distance_set_lut_fl(const uint16_t *mm, const uint16_t *ad, size_t n)
{
    if (validate_lut(mm, ad, n) != 0) return -1;
    memcpy(s_mm_fl, mm, n * sizeof(uint16_t));
    memcpy(s_ad_fl, ad, n * sizeof(uint16_t));
    s_n_fl = n;
    return 0;
}

int sensor_distance_set_lut_fr(const uint16_t *mm, const uint16_t *ad, size_t n)
{
    if (validate_lut(mm, ad, n) != 0) return -1;
    memcpy(s_mm_fr, mm, n * sizeof(uint16_t));
    memcpy(s_ad_fr, ad, n * sizeof(uint16_t));
    s_n_fr = n;
    return 0;
}

int sensor_distance_set_lut_l(const uint16_t *mm, const uint16_t *ad, size_t n)
{
    if (validate_lut(mm, ad, n) != 0) return -1;
    memcpy(s_mm_l, mm, n * sizeof(uint16_t));
    memcpy(s_ad_l, ad, n * sizeof(uint16_t));
    s_n_l = n;
    return 0;
}

int sensor_distance_set_lut_r(const uint16_t *mm, const uint16_t *ad, size_t n)
{
    if (validate_lut(mm, ad, n) != 0) return -1;
    memcpy(s_mm_r, mm, n * sizeof(uint16_t));
    memcpy(s_ad_r, ad, n * sizeof(uint16_t));
    s_n_r = n;
    return 0;
}

size_t sensor_distance_lut_size_fl(void) { return s_n_fl; }
size_t sensor_distance_lut_size_fr(void) { return s_n_fr; }
size_t sensor_distance_lut_size_l(void)  { return s_n_l; }
size_t sensor_distance_lut_size_r(void)  { return s_n_r; }

int sensor_distance_set_lut_front_sum(const uint16_t *mm, const uint16_t *ad_sum, size_t n)
{
    if (validate_lut(mm, ad_sum, n) != 0) return -1;
    memcpy(s_mm_fsum, mm, n * sizeof(uint16_t));
    memcpy(s_ad_fsum, ad_sum, n * sizeof(uint16_t));
    s_n_fsum = n;
    return 0;
}

size_t sensor_distance_lut_size_front_sum(void) { return s_n_fsum; }

// Binary search in a strictly decreasing ad[] array to find segment [i, i+1]
// such that ad[i] >= ad_in >= ad[i+1]. Assumes n>=2 and that ad[0] > ad[n-1].
// Returns index i in [0, n-2]. If outside range, returns 0 for upper extrap,
// or n-2 for lower extrap. Sets *extrap to 1 if outside, otherwise 0.
static size_t find_segment_desc(const uint16_t *ad, size_t n, uint16_t ad_in, int *extrap)
{
    if (ad_in >= ad[0]) {
        if (extrap) *extrap = 1; // upper-side extrapolation (near 0mm)
        return 0;
    }
    if (ad_in <= ad[n-1]) {
        if (extrap) *extrap = 1; // lower-side extrapolation (farther than last mm)
        return n - 2;
    }
    if (extrap) *extrap = 0;

    size_t lo = 0, hi = n - 1; // invariant: ad[lo] > ad[hi]
    while (lo + 1 < hi) {
        size_t mid = (lo + hi) / 2;
        if (ad_in > ad[mid]) {
            // Go toward larger ad (smaller mm)
            hi = mid;
        } else {
            // Go toward smaller ad (larger mm)
            lo = mid;
        }
    }
    return lo; // segment [lo, lo+1]
}

static float interpolate_mm_from_ad(const uint16_t *mm, const uint16_t *ad, size_t n, uint16_t ad_in)
{
    if (n < 2) return 0.0f;

    int extrap = 0;
    size_t i = find_segment_desc(ad, n, ad_in, &extrap);

    const float ad1 = (float)ad[i];
    const float ad2 = (float)ad[i+1];
    const float mm1 = (float)mm[i];
    const float mm2 = (float)mm[i+1];

    // Linear interpolation in (ad,mm) space. ad decreases with mm.
    const float dad = ad2 - ad1; // negative in normal case
    if (dad == 0.0f) {
        return mm1; // degenerate, return nearer endpoint
    }
    const float t = ((float)ad_in - ad1) / dad; // typically in [0,1], may be <0 or >1 when extrapolating
    return mm1 + t * (mm2 - mm1);
}

float sensor_distance_from_fl_unwarped(uint16_t ad_value)
{
    if (s_n_fl < 2) {
        sensor_distance_init();
    }
    return SENSOR_DIST_GAIN * interpolate_mm_from_ad(s_mm_fl, s_ad_fl, s_n_fl, ad_value);
}

float sensor_distance_from_fl(uint16_t ad_value)
{
    float mm_est = sensor_distance_from_fl_unwarped(ad_value);
    if (!s_fl_warp_valid) return mm_est;

    const float x0 = s_warp_x_fl[0], x1 = s_warp_x_fl[1], x2 = s_warp_x_fl[2];
    const float y0 = s_warp_y_fl[0], y1 = s_warp_y_fl[1], y2 = s_warp_y_fl[2];
    const float m0 = s_warp_m_fl[0], m1 = s_warp_m_fl[1], m2 = s_warp_m_fl[2];
    if (mm_est <= x0) return y0 + m0 * (mm_est - x0);
    if (mm_est >= x2) return y2 + m2 * (mm_est - x2);
    if (mm_est <= x1) {
        float h = x1 - x0; float t = (mm_est - x0) / h; float t2 = t*t; float t3 = t2*t;
        float h00 = (2.0f*t3 - 3.0f*t2 + 1.0f);
        float h10 = (t3 - 2.0f*t2 + t);
        float h01 = (-2.0f*t3 + 3.0f*t2);
        float h11 = (t3 - t2);
        return h00*y0 + h10*h*m0 + h01*y1 + h11*h*m1;
    } else {
        float h = x2 - x1; float t = (mm_est - x1) / h; float t2 = t*t; float t3 = t2*t;
        float h00 = (2.0f*t3 - 3.0f*t2 + 1.0f);
        float h10 = (t3 - 2.0f*t2 + t);
        float h01 = (-2.0f*t3 + 3.0f*t2);
        float h11 = (t3 - t2);
        return h00*y1 + h10*h*m1 + h01*y2 + h11*h*m2;
    }
}

float sensor_distance_from_fr_unwarped(uint16_t ad_value)
{
    if (s_n_fr < 2) {
        sensor_distance_init();
    }
    return SENSOR_DIST_GAIN * interpolate_mm_from_ad(s_mm_fr, s_ad_fr, s_n_fr, ad_value);
}

float sensor_distance_from_fr(uint16_t ad_value)
{
    float mm_est = sensor_distance_from_fr_unwarped(ad_value);
    if (!s_fr_warp_valid) return mm_est;

    const float x0 = s_warp_x_fr[0], x1 = s_warp_x_fr[1], x2 = s_warp_x_fr[2];
    const float y0 = s_warp_y_fr[0], y1 = s_warp_y_fr[1], y2 = s_warp_y_fr[2];
    const float m0 = s_warp_m_fr[0], m1 = s_warp_m_fr[1], m2 = s_warp_m_fr[2];
    if (mm_est <= x0) return y0 + m0 * (mm_est - x0);
    if (mm_est >= x2) return y2 + m2 * (mm_est - x2);
    if (mm_est <= x1) {
        float h = x1 - x0; float t = (mm_est - x0) / h; float t2 = t*t; float t3 = t2*t;
        float h00 = (2.0f*t3 - 3.0f*t2 + 1.0f);
        float h10 = (t3 - 2.0f*t2 + t);
        float h01 = (-2.0f*t3 + 3.0f*t2);
        float h11 = (t3 - t2);
        // Use endpoints (x0,y0,m0) and (x1,y1,m1) for the first segment
        return h00*y0 + h10*h*m0 + h01*y1 + h11*h*m1;
    } else {
        float h = x2 - x1; float t = (mm_est - x1) / h; float t2 = t*t; float t3 = t2*t;
        float h00 = (2.0f*t3 - 3.0f*t2 + 1.0f);
        float h10 = (t3 - 2.0f*t2 + t);
        float h01 = (-2.0f*t3 + 3.0f*t2);
        float h11 = (t3 - t2);
        return h00*y1 + h10*h*m1 + h01*y2 + h11*h*m2;
    }
}

float sensor_distance_from_l(uint16_t ad_value)
{
    // No default for side sensors; require explicit LUT set by user.
    // If not set, return 0.0f as a safe fallback.
    if (s_n_l < 2) {
        // Optional: lazy-init front tables (already done), but sides remain unset.
        sensor_distance_init();
        if (s_n_l < 2) return 0.0f;
    }
    return SENSOR_DIST_GAIN * interpolate_mm_from_ad(s_mm_l, s_ad_l, s_n_l, ad_value);
}

float sensor_distance_from_r(uint16_t ad_value)
{
    if (s_n_r < 2) {
        sensor_distance_init();
        if (s_n_r < 2) return 0.0f;
    }
    return SENSOR_DIST_GAIN * interpolate_mm_from_ad(s_mm_r, s_ad_r, s_n_r, ad_value);
}

float sensor_distance_from_fsum(uint16_t ad_sum)
{
    // First, compute unwarped distance from LUT
    float mm_est = sensor_distance_from_fsum_unwarped(ad_sum);

    // If no warp configured, return mm_est
    if (!s_fsum_warp_valid) {
        return mm_est;
    }

    // Evaluate monotone cubic Hermite (PCHIP) between 3 anchors
    const float x0 = s_warp_x[0], x1 = s_warp_x[1], x2 = s_warp_x[2];
    const float y0 = s_warp_y[0], y1 = s_warp_y[1], y2 = s_warp_y[2];
    const float m0 = s_warp_m[0], m1 = s_warp_m[1], m2 = s_warp_m[2];

    if (mm_est <= x0) {
        return y0 + m0 * (mm_est - x0); // linear extrap at lower end
    } else if (mm_est >= x2) {
        return y2 + m2 * (mm_est - x2); // linear extrap at upper end
    } else if (mm_est <= x1) {
        float h = x1 - x0;
        float t = (mm_est - x0) / h;
        float t2 = t * t;
        float t3 = t2 * t;
        float h00 = (2.0f*t3 - 3.0f*t2 + 1.0f);
        float h10 = (t3 - 2.0f*t2 + t);
        float h01 = (-2.0f*t3 + 3.0f*t2);
        float h11 = (t3 - t2);
        return h00*y0 + h10*h*m0 + h01*y1 + h11*h*m1;
    } else {
        float h = x2 - x1;
        float t = (mm_est - x1) / h;
        float t2 = t * t;
        float t3 = t2 * t;
        float h00 = (2.0f*t3 - 3.0f*t2 + 1.0f);
        float h10 = (t3 - 2.0f*t2 + t);
        float h01 = (-2.0f*t3 + 3.0f*t2);
        float h11 = (t3 - t2);
        return h00*y1 + h10*h*m1 + h01*y2 + h11*h*m2;
    }
}

float sensor_distance_from_fsum_unwarped(uint16_t ad_sum)
{
    if (s_n_fsum < 2) {
        sensor_distance_init();
    }
    return SENSOR_DIST_GAIN * interpolate_mm_from_ad(s_mm_fsum, s_ad_fsum, s_n_fsum, ad_sum);
}

// Helper: compute monotone PCHIP slopes for 3 points
static void pchip3_slopes(const float x[3], const float y[3], float m_out[3])
{
    const float h0 = x[1] - x[0];
    const float h1 = x[2] - x[1];
    const float d0 = (y[1] - y[0]) / h0;
    const float d1 = (y[2] - y[1]) / h1;

    // Interior slope (harmonic mean) if same sign; else 0
    float m1;
    if (d0 * d1 <= 0.0f) {
        m1 = 0.0f;
    } else {
        m1 = (h0 + h1) / (h0 / d0 + h1 / d1);
    }

    // Endpoint slopes with one-sided shape-preserving formulas
    float m0 = ((2.0f*h0 + h1) * d0 - h0 * d1) / (h0 + h1);
    float m2 = ((2.0f*h1 + h0) * d1 - h1 * d0) / (h0 + h1);

    // Enforce sign and magnitude limits relative to adjacent secant slopes
    if (m0 * d0 <= 0.0f) m0 = 0.0f;
    else if (fabsf(m0) > 3.0f * fabsf(d0)) m0 = 3.0f * d0;

    if (m2 * d1 <= 0.0f) m2 = 0.0f;
    else if (fabsf(m2) > 3.0f * fabsf(d1)) m2 = 3.0f * d1;

    m_out[0] = m0;
    m_out[1] = m1;
    m_out[2] = m2;
}

void sensor_distance_set_warp_front_sum_3pt(const float x_mm_est[3], const float y_mm_true[3])
{
    // Assume caller ensures monotonic increasing x and y
    s_warp_x[0] = x_mm_est[0];
    s_warp_x[1] = x_mm_est[1];
    s_warp_x[2] = x_mm_est[2];
    s_warp_y[0] = y_mm_true[0];
    s_warp_y[1] = y_mm_true[1];
    s_warp_y[2] = y_mm_true[2];

    // Minimal monotonicity enforcement if needed
    if (!(s_warp_x[0] < s_warp_x[1])) s_warp_x[1] = s_warp_x[0] + 1e-3f;
    if (!(s_warp_x[1] < s_warp_x[2])) s_warp_x[2] = s_warp_x[1] + 1e-3f;
    if (!(s_warp_y[0] < s_warp_y[1])) s_warp_y[1] = s_warp_y[0] + 1e-3f;
    if (!(s_warp_y[1] < s_warp_y[2])) s_warp_y[2] = s_warp_y[1] + 1e-3f;

    pchip3_slopes(s_warp_x, s_warp_y, s_warp_m);
    s_fsum_warp_valid = 1;
}

void sensor_distance_clear_warp_front_sum(void)
{
    s_fsum_warp_valid = 0;
}

void sensor_distance_set_warp_fl_3pt(const float x_mm_est[3], const float y_mm_true[3])
{
    s_warp_x_fl[0] = x_mm_est[0];
    s_warp_x_fl[1] = x_mm_est[1];
    s_warp_x_fl[2] = x_mm_est[2];
    s_warp_y_fl[0] = y_mm_true[0];
    s_warp_y_fl[1] = y_mm_true[1];
    s_warp_y_fl[2] = y_mm_true[2];
    if (!(s_warp_x_fl[0] < s_warp_x_fl[1])) s_warp_x_fl[1] = s_warp_x_fl[0] + 1e-3f;
    if (!(s_warp_x_fl[1] < s_warp_x_fl[2])) s_warp_x_fl[2] = s_warp_x_fl[1] + 1e-3f;
    if (!(s_warp_y_fl[0] < s_warp_y_fl[1])) s_warp_y_fl[1] = s_warp_y_fl[0] + 1e-3f;
    if (!(s_warp_y_fl[1] < s_warp_y_fl[2])) s_warp_y_fl[2] = s_warp_y_fl[1] + 1e-3f;
    pchip3_slopes(s_warp_x_fl, s_warp_y_fl, s_warp_m_fl);
    s_fl_warp_valid = 1;
}

void sensor_distance_clear_warp_fl(void)
{
    s_fl_warp_valid = 0;
}

void sensor_distance_set_warp_fr_3pt(const float x_mm_est[3], const float y_mm_true[3])
{
    s_warp_x_fr[0] = x_mm_est[0];
    s_warp_x_fr[1] = x_mm_est[1];
    s_warp_x_fr[2] = x_mm_est[2];
    s_warp_y_fr[0] = y_mm_true[0];
    s_warp_y_fr[1] = y_mm_true[1];
    s_warp_y_fr[2] = y_mm_true[2];
    if (!(s_warp_x_fr[0] < s_warp_x_fr[1])) s_warp_x_fr[1] = s_warp_x_fr[0] + 1e-3f;
    if (!(s_warp_x_fr[1] < s_warp_x_fr[2])) s_warp_x_fr[2] = s_warp_x_fr[1] + 1e-3f;
    if (!(s_warp_y_fr[0] < s_warp_y_fr[1])) s_warp_y_fr[1] = s_warp_y_fr[0] + 1e-3f;
    if (!(s_warp_y_fr[1] < s_warp_y_fr[2])) s_warp_y_fr[2] = s_warp_y_fr[1] + 1e-3f;
    pchip3_slopes(s_warp_x_fr, s_warp_y_fr, s_warp_m_fr);
    s_fr_warp_valid = 1;
}

void sensor_distance_clear_warp_fr(void)
{
    s_fr_warp_valid = 0;
}

float sensor_distance_from_front_sum(uint16_t ad_fl, uint16_t ad_fr)
{
    uint32_t sum = (uint32_t)ad_fl + (uint32_t)ad_fr;
    if (sum > 0xFFFFu) sum = 0xFFFFu;
    // sensor_distance_from_fsum() で既に SENSOR_DIST_GAIN を適用済み
    return sensor_distance_from_fsum((uint16_t)sum);
}
