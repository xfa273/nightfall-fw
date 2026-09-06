#include "exploration_policy.h"

#include <string.h>

_Static_assert(NF_EXPLORATION_MAX_SIZE >= 1U && NF_EXPLORATION_MAX_SIZE <= 32U,
               "Exploration supports at most 32 by 32 cells");
_Static_assert(sizeof(NfExplorationWorkspace) <= 30U * 1024U,
               "Exploration workspace exceeds 30 KiB");

#define COST_INF UINT16_MAX
#define HEAP_UNSEEN UINT16_MAX
#define HEAP_CLOSED (UINT16_MAX - 1U)
#define SECOND_ACTION_VALID 0x10U
#ifndef NF_EXPLORATION_DISABLE_NAV_PRUNING
#define NF_EXPLORATION_DISABLE_NAV_PRUNING 0
#endif

static const int8_t k_dx[4] = {0, 1, 0, -1};
static const int8_t k_dy[4] = {1, 0, -1, 0};
static const uint8_t k_relative[4] = {0, 1, 3, 2};

static uint16_t cell_count(const NfExplorationSnapshot *s)
{
    return (uint16_t)((uint16_t)s->width * s->height);
}

static bool neighbor(const NfExplorationSnapshot *s, uint16_t cell,
                     uint8_t direction, uint16_t *out)
{
    int x = (int)(cell % s->width) + k_dx[direction];
    int y = (int)(cell / s->width) + k_dy[direction];
    if (x < 0 || y < 0 || x >= s->width || y >= s->height) {
        return false;
    }
    *out = (uint16_t)((uint16_t)y * s->width + (uint16_t)x);
    return true;
}

static bool bit_get(const uint8_t *bits, uint16_t index)
{
    return (bits[index >> 3U] & (uint8_t)(1U << (index & 7U))) != 0U;
}

static void bit_set(uint8_t *bits, uint16_t index)
{
    bits[index >> 3U] |= (uint8_t)(1U << (index & 7U));
}

static NfExplorationStatus finish(NfExplorationDecision *d,
                                  NfExplorationStatus status,
                                  NfExplorationReason reason)
{
    d->status = status;
    d->reason = reason;
    return status;
}

NfExplorationConfig nf_exploration_default_config(void)
{
    NfExplorationConfig c;
    c.straight_us = 300000U;
    c.known_straight_us = 173205U;
    c.turn90_us = 280808U;
    c.uturn_us = 1200000U;
    c.navigation_edge_budget = NF_EXPLORATION_MAX_STATES * 4U;
    c.oracle_edge_budget = 256U;
    c.unknown_cost_per_mille = 1300U;
    c.information_gain_per_mille = 350U;
    c.pending_surrogate = false;
    return c;
}

void nf_exploration_reset(NfExplorationWorkspace *w,
                          const NfExplorationConfig *config)
{
    NfExplorationConfig copy;
    if (w == NULL) {
        return;
    }
    copy = (config != NULL) ? *config : nf_exploration_default_config();
    memset(w, 0, sizeof(*w));
    w->config = copy;
    w->phase = NF_EXPLORATION_PHASE_GOAL;
}

size_t nf_exploration_workspace_bytes(void)
{
    return sizeof(NfExplorationWorkspace);
}

static bool config_valid(const NfExplorationConfig *c)
{
    return c->straight_us > 0U && c->known_straight_us > 0U &&
           c->known_straight_us <= c->straight_us && c->turn90_us > 0U &&
           c->uturn_us > 0U && c->navigation_edge_budget > 0U &&
           c->oracle_edge_budget > 0U && c->unknown_cost_per_mille >= 1000U &&
           c->information_gain_per_mille <= 1000U;
}

/* Even if an adapter forgets to increment epoch, fail closed on a correction.
 * Keeping the previous packed observation also detects mutation during oracle
 * work without copying full route-planner storage into this small workspace. */
static bool known_open(const NfExplorationSnapshot *s, uint16_t cell, uint8_t dir)
{
    uint16_t next;
    uint8_t bit = (uint8_t)(1U << dir);
    return (s->known[cell] & bit) != 0U && (s->walls[cell] & bit) == 0U &&
           neighbor(s, cell, dir, &next);
}

NfExplorationStatus nf_exploration_guard_acceleration(
    const NfExplorationSnapshot *s, bool accelerated, NfExplorationDecision *d)
{
    if (d == NULL) return NF_EXPLORATION_INVALID;
    /* A new acceleration promise must itself be physically verified, even
     * when the old Adachi caller only supplied visited-cell eligibility. */
    if (d->known_straight) {
        uint16_t next;
        if (s == NULL || s->known == NULL || s->walls == NULL || s->visited == NULL ||
            s->width == 0U || s->height == 0U || s->width > NF_EXPLORATION_MAX_SIZE ||
            s->height > NF_EXPLORATION_MAX_SIZE || s->x >= s->width ||
            s->y >= s->height || s->heading >= 4U ||
            d->status != NF_EXPLORATION_MOVE || d->direction != s->heading ||
            !neighbor(s, (uint16_t)((uint16_t)s->y * s->width + s->x), s->heading, &next) ||
            s->visited[next] == 0U || !known_open(s, next, s->heading)) {
            d->known_straight = false;
        }
    }
    if (!accelerated) return d->status;
    if (s == NULL || s->known == NULL || s->walls == NULL ||
        s->width == 0U || s->height == 0U || s->width > NF_EXPLORATION_MAX_SIZE ||
        s->height > NF_EXPLORATION_MAX_SIZE || s->x >= s->width ||
        s->y >= s->height || s->heading >= 4U ||
        !known_open(s, (uint16_t)((uint16_t)s->y * s->width + s->x), s->heading)) {
        d->direction = NF_EXPLORATION_NO_DIRECTION;
        d->known_straight = false;
        d->next_is_turn90 = false;
        d->certified = false;
        return finish(d, NF_EXPLORATION_INVALID,
                      NF_EXPLORATION_REASON_UNSAFE_ACCELERATION);
    }
    if (d->status == NF_EXPLORATION_MOVE && d->direction == s->heading) {
        return d->status;
    }
    d->direction = s->heading;
    d->known_straight = false;
    d->next_is_turn90 = false;
    d->certified = false;
    d->has_target = false;
    if (d->phase == NF_EXPLORATION_PHASE_DONE) d->phase = NF_EXPLORATION_PHASE_RELEVANT;
    return finish(d, NF_EXPLORATION_MOVE, NF_EXPLORATION_REASON_BRAKE_STRAIGHT);
}

static uint8_t adachi_choose(const NfExplorationWorkspace *w,
                             const NfExplorationSnapshot *s,
                             uint16_t cell, uint8_t heading)
{
    uint8_t k;
    if (w->distance[cell] == COST_INF) {
        return NF_EXPLORATION_NO_DIRECTION;
    }
    for (k = 0U; k < 4U; ++k) {
        uint8_t dir = (uint8_t)((heading + k_relative[k]) & 3U);
        uint16_t next;
        if ((s->walls[cell] & (1U << dir)) == 0U && neighbor(s, cell, dir, &next) &&
            w->distance[next] < w->distance[cell]) {
            return dir;
        }
    }
    return NF_EXPLORATION_NO_DIRECTION;
}

static bool canonical_requirement(const NfExplorationSnapshot *s,
                                   uint16_t cell, uint8_t direction,
                                   uint16_t *index)
{
    uint16_t next;
    if (!neighbor(s, cell, direction, &next)) {
        return false;
    }
    if (direction >= 2U) {
        cell = next;
        direction = (uint8_t)(direction - 2U);
    }
    *index = (uint16_t)(cell * 2U + direction);
    return true;
}

static uint8_t unknown_gain(const NfExplorationWorkspace *w,
                            const NfExplorationSnapshot *s, uint16_t cell)
{
    uint8_t dir, count = 0U;
    for (dir = 0U; dir < 4U; ++dir) {
        uint16_t index;
        if ((s->known[cell] & (1U << dir)) == 0U &&
            canonical_requirement(s, cell, dir, &index) &&
            bit_get(w->required_open_bits, index)) {
            ++count;
        }
    }
    return count;
}

static uint16_t tie_key(const NfExplorationWorkspace *w, uint16_t state)
{
    uint16_t cell = state >> 2U;
    return (uint16_t)(((cell % w->width) << 7U) |
                      ((cell / w->width) << 2U) | (state & 3U));
}

static bool heap_less(const NfExplorationWorkspace *w, uint16_t a, uint16_t b)
{
    return w->distance[a] < w->distance[b] ||
           (w->distance[a] == w->distance[b] && tie_key(w, a) < tie_key(w, b));
}

static void heap_swap(NfExplorationWorkspace *w, uint16_t a, uint16_t b)
{
    uint16_t state = w->heap[a];
    w->heap[a] = w->heap[b];
    w->heap[b] = state;
    w->heap_position[w->heap[a]] = a;
    w->heap_position[w->heap[b]] = b;
}

static void heap_update(NfExplorationWorkspace *w, uint16_t state)
{
    uint16_t at = w->heap_position[state];
    if (at == HEAP_UNSEEN) {
        at = w->heap_count++;
        w->heap[at] = state;
        w->heap_position[state] = at;
    }
    while (at > 0U) {
        uint16_t parent = (uint16_t)((at - 1U) / 2U);
        if (!heap_less(w, w->heap[at], w->heap[parent])) {
            break;
        }
        heap_swap(w, at, parent);
        at = parent;
    }
}

static uint16_t heap_pop(NfExplorationWorkspace *w)
{
    uint16_t state = w->heap[0];
    --w->heap_count;
    if (w->heap_count > 0U) {
        uint16_t at = 0U;
        w->heap[0] = w->heap[w->heap_count];
        w->heap_position[w->heap[0]] = 0U;
        while ((uint32_t)at * 2U + 1U < w->heap_count) {
            uint16_t child = (uint16_t)(at * 2U + 1U);
            if ((uint16_t)(child + 1U) < w->heap_count &&
                heap_less(w, w->heap[child + 1U], w->heap[child])) {
                ++child;
            }
            if (!heap_less(w, w->heap[child], w->heap[at])) {
                break;
            }
            heap_swap(w, child, at);
            at = child;
        }
    }
    w->heap_position[state] = HEAP_CLOSED;
    return state;
}

static uint32_t action_ticks(const NfExplorationConfig *c,
                              const NfExplorationSnapshot *s,
                              uint16_t cell, uint16_t next,
                              uint8_t heading, uint8_t direction)
{
    uint64_t us;
    uint8_t relative = (uint8_t)((direction - heading) & 3U);
    if (relative == 2U) {
        us = c->uturn_us;
    } else if ((relative & 1U) != 0U) {
        us = c->turn90_us;
    } else {
        us = s->visited[cell] != 0U && s->visited[next] != 0U ?
             c->known_straight_us : c->straight_us;
    }
    if (s->visited[next] == 0U) {
        us = (us * c->unknown_cost_per_mille + 500U) / 1000U;
    }
    us = (us + NF_EXPLORATION_COST_TICK_US / 2U) / NF_EXPLORATION_COST_TICK_US;
    return us > UINT32_MAX ? UINT32_MAX : (uint32_t)(us > 0U ? us : 1U);
}

/* Each stage performs at most one cell scan, graph edge, or heap operation. */
enum {
    JOB_IDLE = 0, JOB_VALIDATE, JOB_STORE, JOB_GOAL_INIT, JOB_GOAL_BFS,
    JOB_GOAL_SELECT, JOB_CACHED_REQUIREMENTS, JOB_ORACLE_CLEAR, JOB_ORACLE,
    JOB_IMPORT_CLEAR, JOB_IMPORT, JOB_NEW_REQUIREMENTS, JOB_NAV_INIT,
    JOB_NAV_SEED, JOB_NAV_POP, JOB_NAV_EDGE, JOB_CANDIDATE, JOB_NAV_SELECT,
    JOB_FINAL_CHECK, JOB_DONE, JOB_SURROGATE_CLEAR, JOB_SURROGATE_INIT,
    JOB_SURROGATE_BFS, JOB_SURROGATE_PATH
};

static bool shape_valid(const NfExplorationSnapshot *s)
{
    return s != NULL && s->known != NULL && s->walls != NULL &&
        s->visited != NULL && s->goals != NULL && s->width > 0U &&
        s->height > 0U && s->width <= NF_EXPLORATION_MAX_SIZE &&
        s->height <= NF_EXPLORATION_MAX_SIZE && s->x < s->width &&
        s->y < s->height && s->start_x < s->width &&
        s->start_y < s->height && s->heading < 4U;
}

bool nf_exploration_note_goal_reached(NfExplorationWorkspace *w,
                                     const NfExplorationSnapshot *actual)
{
    uint16_t cell;
    if (w == NULL || !shape_valid(actual)) return false;
    cell = (uint16_t)((uint16_t)actual->y * actual->width + actual->x);
    if (actual->visited[cell] == 0U || actual->goals[cell] == 0U) return false;
    w->reached_goal = true;
    w->reached_goal_epoch = actual->epoch;
    return true;
}

static bool cell_valid(const NfExplorationSnapshot *s, uint16_t cell)
{
    uint8_t dir;
    if ((s->known[cell] & 0xF0U) != 0U ||
        (s->walls[cell] & (uint8_t)~s->known[cell]) != 0U) return false;
    for (dir = 0U; dir < 4U; ++dir) {
        uint16_t next;
        uint8_t bit = (uint8_t)(1U << dir);
        uint8_t opposite = (uint8_t)(1U << ((dir + 2U) & 3U));
        if (!neighbor(s, cell, dir, &next)) {
            if ((s->known[cell] & s->walls[cell] & bit) == 0U) return false;
        } else if (((s->known[cell] & bit) != 0U) !=
                   ((s->known[next] & opposite) != 0U) ||
                   ((s->walls[cell] & bit) != 0U) !=
                   ((s->walls[next] & opposite) != 0U)) return false;
    }
    return true;
}

static NfExplorationStatus job_invalid(NfExplorationWorkspace *w,
                                       NfExplorationReason reason)
{
    w->oracle_cached = false;
    w->oracle_pending = false;
    w->initialized = false;
    w->job_stage = JOB_DONE;
    w->job_decision.direction = NF_EXPLORATION_NO_DIRECTION;
    w->job_decision.certified = false;
    return finish(&w->job_decision, NF_EXPLORATION_INVALID, reason);
}

/* A result is withheld until a final cooperative pass checks the immutable
 * facts and visited flags used by this job. This also catches bad callbacks. */
static void job_finish(NfExplorationWorkspace *w, NfExplorationStatus status,
                       NfExplorationReason reason)
{
    finish(&w->job_decision, status, reason);
    w->job_stage = JOB_FINAL_CHECK;
    w->job_cursor = 0U;
    w->job_check_hash = 2166136261U;
}

static void stage(NfExplorationWorkspace *w, uint8_t next)
{
    w->job_stage = next;
    w->job_cursor = 0U;
}

static void start_oracle(NfExplorationWorkspace *w)
{
    if (w->job_oracle == NULL) {
        job_finish(w, NF_EXPLORATION_FALLBACK, NF_EXPLORATION_REASON_ORACLE_FAILURE);
    } else {
        stage(w, w->oracle_pending ? JOB_ORACLE : JOB_ORACLE_CLEAR);
    }
}

static void start_navigation_or_complete(NfExplorationWorkspace *w)
{
    NfExplorationDecision *d = &w->job_decision;
    d->oracle_status = w->cached_oracle.status;
    d->lower_us = w->cached_oracle.goal_entry_us;
    if (w->job_all_known) {
        w->phase = NF_EXPLORATION_PHASE_DONE;
        d->phase = w->phase;
        d->certified = true;
        job_finish(w, NF_EXPLORATION_COMPLETE, NF_EXPLORATION_REASON_CERTIFIED);
    } else {
        w->heap_count = 0U;
        w->job_cost_overflow = false;
        stage(w, JOB_NAV_INIT);
    }
}

NfExplorationStatus nf_exploration_begin(NfExplorationWorkspace *w,
    const NfExplorationSnapshot *s, NfExplorationOracleFn oracle,
    void *oracle_context, bool predictive)
{
    if (w == NULL) return NF_EXPLORATION_INVALID;
    memset(&w->job_decision, 0, sizeof(w->job_decision));
    w->job_decision.direction = NF_EXPLORATION_NO_DIRECTION;
    w->job_decision.status = NF_EXPLORATION_PENDING;
    w->job_decision.oracle_status = NF_EXPLORATION_ORACLE_INVALID;
    if (!config_valid(&w->config)) return job_invalid(w, NF_EXPLORATION_REASON_BAD_CONFIG);
    if (!shape_valid(s)) return job_invalid(w, NF_EXPLORATION_REASON_BAD_SNAPSHOT);
    w->job_snapshot = *s;
    w->job_oracle = oracle;
    w->job_oracle_context = oracle_context;
    w->job_predictive = predictive;
    w->job_surrogate = false;
    w->job_has_goal = false;
    w->job_visited_hash = 2166136261U;
    w->job_same_epoch = w->initialized && w->epoch == s->epoch;
    if (w->job_same_epoch && (w->width != s->width || w->height != s->height ||
        w->start_x != s->start_x || w->start_y != s->start_y ||
        s->generation < w->previous_generation)) {
        return job_invalid(w, NF_EXPLORATION_REASON_BAD_SNAPSHOT);
    }
    w->job_decision.generation = s->generation;
    w->job_decision.epoch = s->epoch;
    stage(w, JOB_VALIDATE);
    return NF_EXPLORATION_PENDING;
}

static void select_goal(NfExplorationWorkspace *w)
{
    const NfExplorationSnapshot *s = &w->job_snapshot;
    NfExplorationDecision *d = &w->job_decision;
    uint16_t current = (uint16_t)((uint16_t)s->y * s->width + s->x);
    d->direction = adachi_choose(w, s, current, s->heading);
    if (d->direction == NF_EXPLORATION_NO_DIRECTION) {
        job_finish(w, NF_EXPLORATION_FALLBACK, NF_EXPLORATION_REASON_NO_TARGET);
        return;
    }
    if (!known_open(s, current, d->direction)) {
        if (w->job_predictive) {
            job_finish(w, NF_EXPLORATION_PROPOSAL, NF_EXPLORATION_REASON_UNOBSERVED_STEP);
        } else {
            d->direction = NF_EXPLORATION_NO_DIRECTION;
            job_finish(w, NF_EXPLORATION_FALLBACK, NF_EXPLORATION_REASON_UNOBSERVED_STEP);
        }
        return;
    }
    if (d->direction == s->heading) {
        uint16_t next;
        if (neighbor(s, current, d->direction, &next) && s->visited[next] != 0U) {
            uint8_t following = adachi_choose(w, s, next, s->heading);
            d->known_straight = following == s->heading;
            d->next_is_turn90 = following != NF_EXPLORATION_NO_DIRECTION &&
                               ((following - s->heading) & 1U) != 0U;
        }
    }
    job_finish(w, NF_EXPLORATION_MOVE, NF_EXPLORATION_REASON_NONE);
}

static void select_probe(NfExplorationWorkspace *w)
{
    const NfExplorationSnapshot *s = &w->job_snapshot;
    NfExplorationDecision *d = &w->job_decision;
    uint16_t start = (uint16_t)((uint16_t)s->y * s->width + s->x);
    uint16_t best = w->job_best_state;
    if (best == HEAP_UNSEEN) {
        job_finish(w, NF_EXPLORATION_FALLBACK, NF_EXPLORATION_REASON_NO_TARGET);
        return;
    }
    d->direction = (uint8_t)(w->first_actions[best] & 3U);
    d->has_target = true;
    d->target_x = (uint8_t)((best >> 2U) % s->width);
    d->target_y = (uint8_t)((best >> 2U) / s->width);
    if (!known_open(s, start, d->direction)) {
        if (w->job_predictive) {
            job_finish(w, NF_EXPLORATION_PROPOSAL, w->job_surrogate ?
                NF_EXPLORATION_REASON_SURROGATE : NF_EXPLORATION_REASON_UNOBSERVED_STEP);
        } else {
            d->direction = NF_EXPLORATION_NO_DIRECTION;
            job_finish(w, NF_EXPLORATION_FALLBACK, NF_EXPLORATION_REASON_UNOBSERVED_STEP);
        }
        return;
    }
    if (d->direction == s->heading && (w->first_actions[best] & SECOND_ACTION_VALID) != 0U) {
        uint16_t next;
        uint8_t second = (uint8_t)((w->first_actions[best] >> 2U) & 3U);
        if (neighbor(s, start, d->direction, &next) && s->visited[next] != 0U) {
            d->known_straight = second == d->direction;
            d->next_is_turn90 = ((second - d->direction) & 1U) != 0U;
        }
    }
    job_finish(w, NF_EXPLORATION_MOVE, w->job_surrogate ?
        NF_EXPLORATION_REASON_SURROGATE : NF_EXPLORATION_REASON_NONE);
}

static void navigation_edge(NfExplorationWorkspace *w)
{
    const NfExplorationSnapshot *s = &w->job_snapshot;
    NfExplorationDecision *d = &w->job_decision;
    uint16_t state = w->job_active_state, cell = state >> 2U;
    uint16_t next, next_state;
    uint16_t start = (uint16_t)(((uint16_t)s->y * s->width + s->x) * 4U + s->heading);
    uint8_t heading = (uint8_t)(state & 3U);
    uint8_t direction = (uint8_t)((heading + k_relative[w->job_direction++]) & 3U);
    uint8_t first;
    uint32_t ticks, total;
    if (w->job_direction == 4U) w->job_stage = JOB_NAV_POP;
    if (d->navigation_edges >= w->config.navigation_edge_budget) {
        job_finish(w, NF_EXPLORATION_FALLBACK, NF_EXPLORATION_REASON_NAVIGATION_BUDGET);
        return;
    }
    ++d->navigation_edges;
    if ((s->walls[cell] & (1U << direction)) != 0U ||
        !neighbor(s, cell, direction, &next)) return;
    next_state = (uint16_t)(next * 4U + direction);
    if (w->heap_position[next_state] == HEAP_CLOSED) return;
    ticks = action_ticks(&w->config, s, cell, next, heading, direction);
    total = (uint32_t)w->distance[state] + ticks;
    if (ticks >= COST_INF || total >= COST_INF) {
        w->job_cost_overflow = true;
        return;
    }
    if (total >= w->distance[next_state]) return;
    first = w->first_actions[state];
    if (state == start) first = direction;
    else if ((first & SECOND_ACTION_VALID) == 0U)
        first |= (uint8_t)((direction << 2U) | SECOND_ACTION_VALID);
    w->distance[next_state] = (uint16_t)total;
    w->first_actions[next_state] = first;
    heap_update(w, next_state);
}

static void candidate_cell(NfExplorationWorkspace *w, uint16_t cell)
{
    const NfExplorationSnapshot *s = &w->job_snapshot;
    uint8_t heading, gain = unknown_gain(w, s, cell);
    uint16_t end = HEAP_UNSEEN;
    uint16_t start = (uint16_t)((uint16_t)s->y * s->width + s->x);
    uint32_t denominator;
    if (gain == 0U || cell == start) return;
    for (heading = 0U; heading < 4U; ++heading) {
        uint16_t state = (uint16_t)(cell * 4U + heading);
        if (w->distance[state] != COST_INF &&
            (end == HEAP_UNSEEN || w->distance[state] < w->distance[end])) end = state;
    }
    if (end == HEAP_UNSEEN) return;
    denominator = 1000U + (uint32_t)w->config.information_gain_per_mille * (gain - 1U);
    if (w->job_best_state == HEAP_UNSEEN ||
        (uint32_t)w->distance[end] * w->job_best_denominator <
            (uint32_t)w->job_best_cost * denominator ||
        ((uint32_t)w->distance[end] * w->job_best_denominator ==
            (uint32_t)w->job_best_cost * denominator &&
         (w->distance[end] < w->job_best_cost ||
          (w->distance[end] == w->job_best_cost &&
           tie_key(w, end) < tie_key(w, w->job_best_state))))) {
        w->job_best_state = end;
        w->job_best_cost = w->distance[end];
        w->job_best_denominator = denominator;
    }
}

static void job_unit(NfExplorationWorkspace *w)
{
    const NfExplorationSnapshot *s = &w->job_snapshot;
    NfExplorationDecision *d = &w->job_decision;
    uint16_t count = cell_count(s), cell = w->job_cursor;
    uint16_t current = (uint16_t)((uint16_t)s->y * s->width + s->x);
    switch (w->job_stage) {
    case JOB_VALIDATE: {
        uint8_t old = w->previous_cells[cell], old_known = old & 15U;
        uint8_t packed = (uint8_t)(s->known[cell] | (s->walls[cell] << 4U));
        if (!cell_valid(s, cell) || (w->job_same_epoch &&
            ((old_known & s->known[cell]) != old_known ||
             (((old >> 4U) ^ s->walls[cell]) & old_known) != 0U ||
             bit_get(w->previous_goal_bits, cell) != (s->goals[cell] != 0U) ||
             (s->generation == w->previous_generation && packed != old)))) {
            (void)job_invalid(w, NF_EXPLORATION_REASON_BAD_SNAPSHOT);
            break;
        }
        w->job_has_goal = w->job_has_goal || s->goals[cell] != 0U;
        w->job_visited_hash = (w->job_visited_hash ^ s->visited[cell]) * 16777619U;
        if (++w->job_cursor == count) {
            if (!w->job_has_goal || (!w->job_predictive && s->visited[current] == 0U)) {
                (void)job_invalid(w, NF_EXPLORATION_REASON_BAD_SNAPSHOT);
                break;
            }
            if (!w->job_same_epoch) {
                w->oracle_cached = false;
                w->oracle_pending = false;
                w->reached_goal = w->reached_goal && w->reached_goal_epoch == s->epoch;
            }
            stage(w, JOB_STORE);
        }
        break;
    }
    case JOB_STORE:
        w->previous_cells[cell] = (uint8_t)(s->known[cell] | (s->walls[cell] << 4U));
        if ((cell & 7U) == 0U) w->previous_goal_bits[cell >> 3U] = 0U;
        if (s->goals[cell] != 0U) bit_set(w->previous_goal_bits, cell);
        if (++w->job_cursor == count) {
            w->initialized = true;
            w->epoch = s->epoch;
            w->previous_generation = s->generation;
            w->width = s->width; w->height = s->height;
            w->start_x = s->start_x; w->start_y = s->start_y;
            if (!w->job_predictive && s->goals[current] != 0U) {
                w->reached_goal = true;
                w->reached_goal_epoch = s->epoch;
            }
            w->phase = w->reached_goal ? NF_EXPLORATION_PHASE_RELEVANT : NF_EXPLORATION_PHASE_GOAL;
            d->phase = w->phase;
            w->job_all_known = true;
            if (!w->reached_goal) {
                w->job_head = 0U; w->job_tail = 0U;
                stage(w, JOB_GOAL_INIT);
            } else if (w->oracle_cached) stage(w, JOB_CACHED_REQUIREMENTS);
            else start_oracle(w);
        }
        break;
    case JOB_GOAL_INIT:
        w->distance[cell] = COST_INF;
        if (s->goals[cell] != 0U) {
            w->distance[cell] = 0U;
            w->heap[w->job_tail++] = cell;
        }
        if (++w->job_cursor == count) stage(w, JOB_GOAL_BFS);
        break;
    case JOB_GOAL_BFS:
        if (w->job_head == w->job_tail || w->distance[current] != COST_INF) {
            w->job_stage = JOB_GOAL_SELECT;
        } else {
            uint8_t dir;
            cell = w->heap[w->job_head++];
            for (dir = 0U; dir < 4U; ++dir) {
                uint16_t next;
                if ((s->walls[cell] & (1U << dir)) == 0U && neighbor(s, cell, dir, &next) &&
                    w->distance[next] == COST_INF) {
                    w->distance[next] = (uint16_t)(w->distance[cell] + 1U);
                    w->heap[w->job_tail++] = next;
                }
            }
        }
        break;
    case JOB_GOAL_SELECT: select_goal(w); break;
    case JOB_CACHED_REQUIREMENTS:
    case JOB_NEW_REQUIREMENTS: {
        uint8_t dir;
        bool valid = true;
        for (dir = 0U; dir < 2U; ++dir) {
            uint8_t bit = (uint8_t)(1U << dir);
            if (bit_get(w->required_open_bits, (uint16_t)(cell * 2U + dir))) {
                if ((s->walls[cell] & bit) != 0U) valid = false;
                if ((s->known[cell] & bit) == 0U) w->job_all_known = false;
            }
        }
        if (!valid) {
            bool fresh = w->job_stage == JOB_NEW_REQUIREMENTS;
            w->oracle_cached = false;
            w->oracle_pending = false;
            if (fresh) job_finish(w, NF_EXPLORATION_FALLBACK, NF_EXPLORATION_REASON_STALE_ORACLE);
            else start_oracle(w);
        } else if (++w->job_cursor == count) {
            if (w->job_stage == JOB_NEW_REQUIREMENTS) {
                w->cached_oracle = w->job_oracle_result;
                w->oracle_cached = true;
            }
            start_navigation_or_complete(w);
        }
        break;
    }
    case JOB_ORACLE_CLEAR:
        w->first_actions[cell] = 0U;
        if (++w->job_cursor == count) w->job_stage = JOB_ORACLE;
        break;
    case JOB_ORACLE: {
        NfExplorationOracleResult *r = &w->job_oracle_result;
        memset(r, 0, sizeof(*r));
        r->status = NF_EXPLORATION_ORACLE_INVALID;
        w->job_oracle(w->job_oracle_context, s, !w->oracle_pending,
            w->config.oracle_edge_budget, w->first_actions, r);
        d->oracle_status = r->status;
        w->oracle_pending = r->status == NF_EXPLORATION_ORACLE_PENDING;
        if (w->oracle_pending) {
            if (w->config.pending_surrogate) {
                w->job_surrogate = true;
                w->job_all_known = true;
                w->job_head = 0U; w->job_tail = 0U;
                stage(w, JOB_SURROGATE_CLEAR);
            } else job_finish(w, NF_EXPLORATION_FALLBACK, NF_EXPLORATION_REASON_ORACLE_PENDING);
        }
        else if (r->status != NF_EXPLORATION_ORACLE_EXACT)
            job_finish(w, NF_EXPLORATION_FALLBACK, NF_EXPLORATION_REASON_ORACLE_FAILURE);
        else if (r->epoch != s->epoch || r->generation > s->generation)
            job_finish(w, NF_EXPLORATION_FALLBACK, NF_EXPLORATION_REASON_STALE_ORACLE);
        else if (!r->dependencies_complete || r->stop_us < r->goal_entry_us)
            job_finish(w, NF_EXPLORATION_FALLBACK, NF_EXPLORATION_REASON_BAD_DEPENDENCIES);
        else {
            w->job_any_requirement = false;
            w->job_all_known = true;
            stage(w, JOB_IMPORT_CLEAR);
        }
        break;
    }
    case JOB_SURROGATE_CLEAR:
        w->required_open_bits[cell] = 0U;
        if (++w->job_cursor == (uint16_t)((count * 2U + 7U) / 8U)) stage(w, JOB_SURROGATE_INIT);
        break;
    case JOB_SURROGATE_INIT:
        w->distance[cell] = COST_INF;
        if (s->goals[cell] != 0U) {
            w->distance[cell] = 0U;
            w->heap[w->job_tail++] = cell;
        }
        if (++w->job_cursor == count) stage(w, JOB_SURROGATE_BFS);
        break;
    case JOB_SURROGATE_BFS: {
        uint16_t start = (uint16_t)((uint16_t)s->start_y * s->width + s->start_x);
        if (w->distance[start] != COST_INF) {
            w->job_active_state = start;
            w->job_direction = 0U; /* Cardinal heuristic starts facing north. */
            w->job_stage = JOB_SURROGATE_PATH;
        } else if (w->job_head == w->job_tail) {
            job_finish(w, NF_EXPLORATION_FALLBACK, NF_EXPLORATION_REASON_ORACLE_PENDING);
        } else {
            uint8_t dir;
            cell = w->heap[w->job_head++];
            for (dir = 0U; dir < 4U; ++dir) {
                uint16_t next;
                if ((s->walls[cell] & (1U << dir)) == 0U && neighbor(s, cell, dir, &next) &&
                    w->distance[next] == COST_INF) {
                    w->distance[next] = (uint16_t)(w->distance[cell] + 1U);
                    w->heap[w->job_tail++] = next;
                }
            }
        }
        break;
    }
    case JOB_SURROGATE_PATH:
        cell = w->job_active_state;
        if (w->distance[cell] == 0U) {
            /* A completely observed cardinal route says nothing about the
             * still-pending exact slalom optimum: keep exploring by fallback. */
            if (w->job_all_known) {
                job_finish(w, NF_EXPLORATION_FALLBACK, NF_EXPLORATION_REASON_ORACLE_PENDING);
            } else {
                w->heap_count = 0U;
                w->job_cost_overflow = false;
                stage(w, JOB_NAV_INIT);
            }
        } else {
            uint16_t next, index;
            uint8_t dir = adachi_choose(w, s, cell, w->job_direction);
            if (dir == NF_EXPLORATION_NO_DIRECTION || !neighbor(s, cell, dir, &next) ||
                !canonical_requirement(s, cell, dir, &index)) {
                job_finish(w, NF_EXPLORATION_FALLBACK, NF_EXPLORATION_REASON_ORACLE_PENDING);
                break;
            }
            bit_set(w->required_open_bits, index);
            if ((s->known[cell] & (1U << dir)) == 0U) w->job_all_known = false;
            w->job_active_state = next;
            w->job_direction = dir;
        }
        break;
    case JOB_IMPORT_CLEAR:
        w->required_open_bits[cell] = 0U;
        if (++w->job_cursor == (uint16_t)((count * 2U + 7U) / 8U)) stage(w, JOB_IMPORT);
        break;
    case JOB_IMPORT: {
        uint8_t dir, mask = w->first_actions[cell];
        if ((mask & 0xF0U) != 0U) {
            job_finish(w, NF_EXPLORATION_FALLBACK, NF_EXPLORATION_REASON_BAD_DEPENDENCIES);
            break;
        }
        for (dir = 0U; dir < 4U; ++dir) {
            uint16_t index;
            if ((mask & (1U << dir)) == 0U) continue;
            if (!canonical_requirement(s, cell, dir, &index)) {
                job_finish(w, NF_EXPLORATION_FALLBACK, NF_EXPLORATION_REASON_BAD_DEPENDENCIES);
                return;
            }
            bit_set(w->required_open_bits, index);
            w->job_any_requirement = true;
        }
        if (++w->job_cursor == count) {
            if (!w->job_any_requirement && s->goals[(uint16_t)s->start_y * s->width + s->start_x] == 0U)
                job_finish(w, NF_EXPLORATION_FALLBACK, NF_EXPLORATION_REASON_BAD_DEPENDENCIES);
            else stage(w, JOB_NEW_REQUIREMENTS);
        }
        break;
    }
    case JOB_NAV_INIT:
        w->distance[cell] = COST_INF;
        w->heap_position[cell] = HEAP_UNSEEN;
        w->first_actions[cell] = NF_EXPLORATION_NO_DIRECTION;
        if (++w->job_cursor == count * 4U) w->job_stage = JOB_NAV_SEED;
        break;
    case JOB_NAV_SEED: {
        uint16_t start = (uint16_t)(current * 4U + s->heading);
        w->job_best_state = HEAP_UNSEEN;
        w->job_best_cost = COST_INF;
        w->job_best_denominator = 1U;
        w->distance[start] = 0U;
        heap_update(w, start);
        w->job_stage = JOB_NAV_POP;
        break;
    }
    case JOB_NAV_POP:
#if !NF_EXPLORATION_DISABLE_NAV_PRUNING
        /* Every unsettled candidate costs at least the next heap minimum,
         * and can reveal at most four required edges. Stop only at STRICTLY
         * worse score, preserving equal-score cost/coordinate tie breaking.
         * No exact-oracle proof depends on this navigation-only bound. */
        if (w->heap_count > 0U && w->job_best_state != HEAP_UNSEEN &&
            !w->job_cost_overflow &&
            (uint32_t)w->distance[w->heap[0]] * w->job_best_denominator >
                (uint32_t)w->job_best_cost *
                (1000U + 3U * w->config.information_gain_per_mille)) {
            w->heap_count = 0U;
        }
#endif
        if (w->heap_count == 0U) {
            if (w->job_cost_overflow) job_finish(w, NF_EXPLORATION_FALLBACK, NF_EXPLORATION_REASON_COST_RANGE);
            else {
                w->job_best_state = HEAP_UNSEEN;
                w->job_best_cost = COST_INF;
                w->job_best_denominator = 1U;
                stage(w, JOB_CANDIDATE);
            }
        } else {
            w->job_active_state = heap_pop(w);
#if !NF_EXPLORATION_DISABLE_NAV_PRUNING
            candidate_cell(w, w->job_active_state >> 2U);
#endif
            w->job_direction = 0U;
            w->job_stage = JOB_NAV_EDGE;
        }
        break;
    case JOB_NAV_EDGE: navigation_edge(w); break;
    case JOB_CANDIDATE:
        candidate_cell(w, cell);
        if (++w->job_cursor == count) w->job_stage = JOB_NAV_SELECT;
        break;
    case JOB_NAV_SELECT: select_probe(w); break;
    case JOB_FINAL_CHECK:
        if (w->previous_cells[cell] != (uint8_t)(s->known[cell] | (s->walls[cell] << 4U)) ||
            bit_get(w->previous_goal_bits, cell) != (s->goals[cell] != 0U)) {
            (void)job_invalid(w, NF_EXPLORATION_REASON_BAD_SNAPSHOT);
            break;
        }
        w->job_check_hash = (w->job_check_hash ^ s->visited[cell]) * 16777619U;
        if (++w->job_cursor == count) {
            if (w->job_check_hash != w->job_visited_hash)
                (void)job_invalid(w, NF_EXPLORATION_REASON_BAD_SNAPSHOT);
            else w->job_stage = JOB_DONE;
        }
        break;
    default: (void)job_invalid(w, NF_EXPLORATION_REASON_BAD_CONFIG); break;
    }
}

NfExplorationStatus nf_exploration_step(NfExplorationWorkspace *w, uint32_t work_budget)
{
    if (w == NULL || w->job_stage == JOB_IDLE) return NF_EXPLORATION_INVALID;
    while (work_budget > 0U && w->job_stage != JOB_DONE) {
        --work_budget;
        job_unit(w);
    }
    return w->job_stage == JOB_DONE ? w->job_decision.status : NF_EXPLORATION_PENDING;
}

NfExplorationStatus nf_exploration_result(const NfExplorationWorkspace *w,
                                         NfExplorationDecision *d)
{
    if (d == NULL) return NF_EXPLORATION_INVALID;
    memset(d, 0, sizeof(*d));
    d->direction = NF_EXPLORATION_NO_DIRECTION;
    if (w == NULL || w->job_stage == JOB_IDLE)
        return finish(d, NF_EXPLORATION_INVALID, NF_EXPLORATION_REASON_BAD_CONFIG);
    if (w->job_stage != JOB_DONE) return finish(d, NF_EXPLORATION_PENDING, NF_EXPLORATION_REASON_NONE);
    *d = w->job_decision;
    return d->status;
}

NfExplorationStatus nf_exploration_decide(NfExplorationWorkspace *w,
    const NfExplorationSnapshot *s, NfExplorationOracleFn oracle,
    void *oracle_context, NfExplorationDecision *d)
{
    NfExplorationStatus status;
    if (d == NULL) return NF_EXPLORATION_INVALID;
    status = nf_exploration_begin(w, s, oracle, oracle_context, false);
    while (status == NF_EXPLORATION_PENDING) status = nf_exploration_step(w, 1024U);
    return nf_exploration_result(w, d);
}

NfExplorationStatus nf_exploration_apply_result(
    const NfExplorationSnapshot *planned, const NfExplorationSnapshot *current,
    NfExplorationDecision *d)
{
    uint16_t cell;
    if (d == NULL) return NF_EXPLORATION_INVALID;
    if (!shape_valid(planned) || !shape_valid(current) ||
        planned->epoch != current->epoch || planned->generation > current->generation ||
        d->epoch != planned->epoch || d->generation != planned->generation ||
        planned->width != current->width || planned->height != current->height ||
        planned->start_x != current->start_x || planned->start_y != current->start_y ||
        planned->x != current->x || planned->y != current->y ||
        planned->heading != current->heading) {
        d->certified = false;
        d->direction = NF_EXPLORATION_NO_DIRECTION;
        return finish(d, NF_EXPLORATION_FALLBACK, NF_EXPLORATION_REASON_STALE_ORACLE);
    }
    cell = (uint16_t)((uint16_t)current->y * current->width + current->x);
    if (current->visited[cell] == 0U) {
        d->certified = false;
        d->direction = NF_EXPLORATION_NO_DIRECTION;
        return finish(d, NF_EXPLORATION_INVALID, NF_EXPLORATION_REASON_BAD_SNAPSHOT);
    }
    if (d->status == NF_EXPLORATION_MOVE || d->status == NF_EXPLORATION_PROPOSAL) {
        if (d->direction >= 4U || !known_open(current, cell, d->direction)) {
            d->direction = NF_EXPLORATION_NO_DIRECTION;
            return finish(d, NF_EXPLORATION_FALLBACK, NF_EXPLORATION_REASON_UNOBSERVED_STEP);
        }
        d->status = NF_EXPLORATION_MOVE;
        if (d->reason != NF_EXPLORATION_REASON_SURROGATE) d->reason = NF_EXPLORATION_REASON_NONE;
    }
    d->generation = current->generation;
    return d->status;
}

NfExplorationStatus nf_exploration_guard_progress(NfExplorationWorkspace *w,
    const NfExplorationSnapshot *actual, bool rejected_or_missed, NfExplorationDecision *d)
{
    if (d == NULL) return NF_EXPLORATION_INVALID;
    if (w == NULL || !shape_valid(actual)) {
        d->direction = NF_EXPLORATION_NO_DIRECTION;
        d->certified = false;
        return finish(d, NF_EXPLORATION_INVALID, NF_EXPLORATION_REASON_BAD_SNAPSHOT);
    }
    if (w->fallback_held && (w->fallback_epoch != actual->epoch ||
                             w->fallback_generation != actual->generation)) {
        w->fallback_held = false;
    }
    if (d->status == NF_EXPLORATION_COMPLETE && d->certified) return d->status;
    if (rejected_or_missed) {
        w->fallback_held = true;
        w->fallback_epoch = actual->epoch;
        w->fallback_generation = actual->generation;
    }
    if (w->fallback_held) {
        d->direction = NF_EXPLORATION_NO_DIRECTION;
        d->known_straight = false;
        d->next_is_turn90 = false;
        d->certified = false;
        return finish(d, NF_EXPLORATION_FALLBACK, NF_EXPLORATION_REASON_PROGRESS_FALLBACK);
    }
    return d->status;
}
