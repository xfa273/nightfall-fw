#ifndef F413_MOTION_POLICY_H
#define F413_MOTION_POLICY_H

/* Explicit opt-ins after the mini_r2 57a9c3b (2026-08-29) motion baseline.
 * Never use a model-number comparison or an ALL mask in a machine profile:
 * adding an experiment must not opt another machine into new motion behavior.
 * These are compiled policy choices, not mutable NVM/run-mode settings. */
#define F413_MOTION_IMU_16G                 (1U << 0)
#define F413_MOTION_SETTLED_STOP            (1U << 1)
#define F413_MOTION_FRONT_RECOVERY          (1U << 2)
#define F413_MOTION_SEARCH_DISTANCE         (1U << 3)
#define F413_MOTION_PARAMETER_LIMITS        (1U << 4)
#define F413_MOTION_CASE0_TURN_SPEED        (1U << 5)
#define F413_MOTION_CASE0_LONG_R180         (1U << 6)
#define F413_MOTION_MODE2_CASE6_SAVED_MAZE   (1U << 7)
#define F413_MOTION_KNOWN_FEATURES          (0xFFU)

/* params.h supplies either a literal host profile or the boot-selected alias. */
#define F413_MOTION_ENABLED(feature) (((F413_MOTION_FEATURES) & (feature)) != 0U)

#endif
