#ifndef NIGHTFALL_COMMON_EXPLORATION_POLICY_H
#define NIGHTFALL_COMMON_EXPLORATION_POLICY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Override consistently in all translation units to reduce static storage. */
#ifndef NF_EXPLORATION_MAX_SIZE
#define NF_EXPLORATION_MAX_SIZE 32U
#endif
#define NF_EXPLORATION_MAX_CELLS (NF_EXPLORATION_MAX_SIZE * NF_EXPLORATION_MAX_SIZE)
#define NF_EXPLORATION_MAX_STATES (NF_EXPLORATION_MAX_CELLS * 4U)
#define NF_EXPLORATION_NO_DIRECTION 255U
#define NF_EXPLORATION_COST_TICK_US 5000U

/* Flat y*width+x arrays; x east, y north; direction N/E/S/W = 0/1/2/3;
 * walls and known use 1 << direction. Arrays must stay immutable during decide.
 * Every observed edge is reciprocal. Walls must be a subset of known.
 * generation increases whenever an observation changes. Within one epoch,
 * observations may only be added; known facts may NEVER change/disappear.
 * Increment epoch for sensor corrections, maze resets, goal/profile changes,
 * and before generation wraps. The adapter owns these version numbers.
 */
typedef struct {
    const uint8_t *known;
    const uint8_t *walls;
    const uint8_t *visited;
    const uint8_t *goals;
    uint32_t generation;
    uint32_t epoch;
    uint8_t width;
    uint8_t height;
    uint8_t x;
    uint8_t y;
    uint8_t heading;
    uint8_t start_x;
    uint8_t start_y;
} NfExplorationSnapshot;

typedef enum {
    NF_EXPLORATION_ORACLE_PENDING = 0,
    NF_EXPLORATION_ORACLE_EXACT,
    NF_EXPLORATION_ORACLE_NO_PATH,
    NF_EXPLORATION_ORACLE_BUDGET,
    NF_EXPLORATION_ORACLE_INVALID,
    NF_EXPLORATION_ORACLE_CAPACITY,
} NfExplorationOracleStatus;

typedef struct {
    NfExplorationOracleStatus status;
    uint32_t generation;
    uint32_t epoch;
    uint64_t goal_entry_us;
    uint64_t stop_us;
    /* True only after the exact planner has replayed the entire sufficient
     * required-open set, including every geometry guard and stopping tail. */
    bool dependencies_complete;
} NfExplorationOracleResult;

/* restart=true starts an optimistic-map solve from this snapshot. The callback
 * MUST copy anything needed after returning; subsequent calls may pass a newer
 * immutable snapshot and restart=false polls the existing solve. It may spend
 * no more than edge_budget units in its resumable solver. Hardware integration
 * must also impose its own cycle/time deadline. On EXACT, fill required_open
 * (width*height bytes, 1<<direction; canonical or reciprocal masks accepted).
 * The callback owns its solver workspace; policy workspace is independent.
 * A result from an older generation can only be reused within the same epoch,
 * after checking all of its dependencies against the current observed map.
 */
typedef void (*NfExplorationOracleFn)(void *context,
                                     const NfExplorationSnapshot *snapshot,
                                     bool restart,
                                     uint32_t edge_budget,
                                     uint8_t *required_open,
                                     NfExplorationOracleResult *result);

typedef struct {
    /* Approximate navigation action times, not route-oracle objective costs. */
    uint32_t straight_us;
    uint32_t known_straight_us;
    uint32_t turn90_us;
    uint32_t uturn_us;
    uint32_t navigation_edge_budget;
    uint32_t oracle_edge_budget;
    uint16_t unknown_cost_per_mille;
    uint16_t information_gain_per_mille;
    /* Probe a cheap cardinal route while the exact oracle is pending.
     * This supplies navigation only and can NEVER certify completion. */
    bool pending_surrogate;
} NfExplorationConfig;

typedef enum {
    NF_EXPLORATION_PHASE_GOAL = 0,
    NF_EXPLORATION_PHASE_RELEVANT,
    NF_EXPLORATION_PHASE_DONE,
} NfExplorationPhase;

typedef enum {
    NF_EXPLORATION_MOVE = 0,
    NF_EXPLORATION_FALLBACK,
    NF_EXPLORATION_COMPLETE,
    NF_EXPLORATION_INVALID,
    NF_EXPLORATION_PENDING,
    /* Predictive navigation only; never actuate without apply_result. */
    NF_EXPLORATION_PROPOSAL,
} NfExplorationStatus;

typedef enum {
    NF_EXPLORATION_REASON_NONE = 0,
    NF_EXPLORATION_REASON_BAD_SNAPSHOT,
    NF_EXPLORATION_REASON_BAD_CONFIG,
    NF_EXPLORATION_REASON_ORACLE_PENDING,
    NF_EXPLORATION_REASON_ORACLE_FAILURE,
    NF_EXPLORATION_REASON_STALE_ORACLE,
    NF_EXPLORATION_REASON_BAD_DEPENDENCIES,
    NF_EXPLORATION_REASON_NAVIGATION_BUDGET,
    NF_EXPLORATION_REASON_COST_RANGE,
    NF_EXPLORATION_REASON_NO_TARGET,
    NF_EXPLORATION_REASON_UNOBSERVED_STEP,
    NF_EXPLORATION_REASON_CERTIFIED,
    NF_EXPLORATION_REASON_BRAKE_STRAIGHT,
    NF_EXPLORATION_REASON_UNSAFE_ACCELERATION,
    NF_EXPLORATION_REASON_SURROGATE,
    NF_EXPLORATION_REASON_PROGRESS_FALLBACK,
} NfExplorationReason;

typedef struct {
    NfExplorationStatus status;
    NfExplorationReason reason;
    NfExplorationPhase phase;
    NfExplorationOracleStatus oracle_status;
    uint32_t generation;
    uint32_t epoch;
    uint32_t navigation_edges;
    uint64_t lower_us;
    uint8_t direction;
    uint8_t target_x;
    uint8_t target_y;
    bool has_target;
    bool known_straight;
    bool next_is_turn90;
    bool certified;
} NfExplorationDecision;

/* 29 KiB at 32x32, about 7.5 KiB at 16x16. No malloc and no recursion.
 * Navigation costs are rounded to 5 ms (range 327.67 s); overflow/budget
 * returns FALLBACK, never a shortened route or a certificate. The exact route
 * oracle keeps its independent microsecond cost and is not quantized.
 */
typedef struct {
    uint16_t distance[NF_EXPLORATION_MAX_STATES];
    uint16_t heap[NF_EXPLORATION_MAX_STATES];
    uint16_t heap_position[NF_EXPLORATION_MAX_STATES];
    uint8_t first_actions[NF_EXPLORATION_MAX_STATES];
    /* Canonical N/E requirements, two bits per cell. first_actions is reused
     * as the callback's temporary byte-per-cell required-mask output. */
    uint8_t required_open_bits[(NF_EXPLORATION_MAX_CELLS * 2U + 7U) / 8U];
    uint8_t previous_cells[NF_EXPLORATION_MAX_CELLS];
    uint8_t previous_goal_bits[(NF_EXPLORATION_MAX_CELLS + 7U) / 8U];
    NfExplorationConfig config;
    NfExplorationOracleResult cached_oracle;
    uint32_t epoch;
    uint32_t previous_generation;
    uint32_t reached_goal_epoch;
    uint32_t fallback_generation;
    uint32_t fallback_epoch;
    uint16_t heap_count;
    uint8_t width;
    uint8_t height;
    uint8_t start_x;
    uint8_t start_y;
    NfExplorationPhase phase;
    bool initialized;
    bool reached_goal;
    bool oracle_pending;
    bool oracle_cached;
    bool fallback_held;
    /* Private cooperative scheduler state. Pointed-to snapshot arrays are
     * caller-owned and immutable until the job finishes or is cancelled. */
    NfExplorationSnapshot job_snapshot;
    NfExplorationDecision job_decision;
    NfExplorationOracleResult job_oracle_result;
    NfExplorationOracleFn job_oracle;
    void *job_oracle_context;
    uint32_t job_best_denominator;
    uint32_t job_visited_hash;
    uint32_t job_check_hash;
    uint16_t job_cursor;
    uint16_t job_head;
    uint16_t job_tail;
    uint16_t job_active_state;
    uint16_t job_best_state;
    uint16_t job_best_cost;
    uint8_t job_stage;
    uint8_t job_direction;
    bool job_same_epoch;
    bool job_has_goal;
    bool job_all_known;
    bool job_any_requirement;
    bool job_surrogate;
    bool job_cost_overflow;
    bool job_predictive;
} NfExplorationWorkspace;

NfExplorationConfig nf_exploration_default_config(void);
void nf_exploration_reset(NfExplorationWorkspace *workspace,
                          const NfExplorationConfig *config);
NfExplorationStatus nf_exploration_decide(NfExplorationWorkspace *workspace,
                                         const NfExplorationSnapshot *snapshot,
                                         NfExplorationOracleFn oracle,
                                         void *oracle_context,
                                         NfExplorationDecision *decision);
size_t nf_exploration_workspace_bytes(void);

/* O(1) begin and cooperative step. One work unit is a cell scan, one outgoing
 * graph edge, or a bounded binary-heap operation (<=12 levels at 32x32).
 * The callback remains responsible for its own oracle edge/cycle budget.
 * A new begin cancels only the policy job; a pending oracle remains resumable.
 * predictive permits an unvisited predicted pose and returns PROPOSAL when its
 * first edge is still unknown. It NEVER marks a predicted goal as reached.
 */
NfExplorationStatus nf_exploration_begin(NfExplorationWorkspace *workspace,
    const NfExplorationSnapshot *snapshot, NfExplorationOracleFn oracle,
    void *oracle_context, bool predictive);
NfExplorationStatus nf_exploration_step(NfExplorationWorkspace *workspace,
                                       uint32_t work_budget);
NfExplorationStatus nf_exploration_result(const NfExplorationWorkspace *workspace,
                                         NfExplorationDecision *decision);
/* O(1), only at an actually visited goal; never pass a predicted pose. */
bool nf_exploration_note_goal_reached(NfExplorationWorkspace *workspace,
                                     const NfExplorationSnapshot *actual);
/* O(1) arrival check. Caller must guarantee monotonic facts within an epoch.
 * New walls may change a probe's desirability, but its first observed-open
 * move remains safe. A prior certificate only used already known-open edges,
 * so additive observations cannot invalidate it. Any correction needs epoch++.
 */
NfExplorationStatus nf_exploration_apply_result(
    const NfExplorationSnapshot *planned, const NfExplorationSnapshot *current,
    NfExplorationDecision *decision);

/* O(1) arrival-level liveness guard, applied AFTER apply_result and BEFORE
 * selecting legacy Adachi/guarding acceleration. A missed/rejected proposal
 * holds Adachi until actual observation generation/epoch changes. This avoids
 * ready-on-turn / missed-on-straight reversals on an unchanged map. Never pass
 * a predicted generation. A validated exact certificate always passes through.
 * No-result-before-the-first-job is not a rejected/missed proposal.
 */
NfExplorationStatus nf_exploration_guard_progress(NfExplorationWorkspace *workspace,
    const NfExplorationSnapshot *actual, bool rejected_or_missed,
    NfExplorationDecision *decision);

/* Apply after selecting either an improved decision or the legacy fallback.
 * accelerated means the previous motion ended above the nominal search speed
 * under a promise that this cell would also be straight. A turn/completion is
 * postponed by one observed-open straight cell with acceleration disabled.
 * INVALID means that promise cannot be fulfilled; the caller must abort via
 * its guarded motion stop, never retain the turn or unknown forward motion.
 */
NfExplorationStatus nf_exploration_guard_acceleration(
    const NfExplorationSnapshot *snapshot, bool accelerated,
    NfExplorationDecision *decision);

#ifdef __cplusplus
}
#endif
#endif
