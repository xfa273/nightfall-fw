#ifndef F413_MEASUREMENTS_H
#define F413_MEASUREMENTS_H

#include "f413_machine.h"

/* Planar rigid-body kinematics, IMU on the forward/rear centreline:
   a_sensor_forward = a_centre_forward - omega^2 * r_forward.
   Yaw acceleration contributes only laterally for this mounting position.
   Input acceleration has already been axis-mapped and stationary-bias removed.
   This is not a gravity/tilt or cross-axis/slip correction. */
static inline float f413_imu_centre_forward_accel(float sensor_mm_s2,
                                                float omega_dps,
                                                float offset_mm)
{
  if (offset_mm == 0.0f) return sensor_mm_s2; /* Preserve the r2 path exactly. */
  const float omega_rad_s = omega_dps * 0.017453292519943295f;
  return sensor_mm_s2 + omega_rad_s * omega_rad_s * offset_mm;
}

/* Vref is an explicit assumption, not a calibrated measurement or cell count. */
static inline float f413_battery_voltage(uint16_t adc, float vref_v,
                                          float divider_ratio)
{
  return (float)adc * vref_v * divider_ratio / 4095.0f;
}

#endif
