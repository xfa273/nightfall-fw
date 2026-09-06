/* Host-only replay bridge. No firmware/HAL/NVM/motion entry points. */
#include "../../common/exploration/exploration_policy.h"
#include "../../common/route/mcu_slalom_time_planner.h"
#include <stdlib.h>
#include <string.h>

typedef struct {
    uint64_t policy_jobs, policy_work_units, oracle_started, oracle_callbacks;
    uint64_t oracle_work_units, oracle_completed, oracle_aborted, navigation_edges;
    uint64_t policy_workspace_bytes, oracle_workspace_bytes;
} NfHostExplorationStats;

typedef struct {
    NfExplorationWorkspace policy;
    NfExplorationSnapshot planned;
    uint8_t known[1024], walls[1024], visited[1024], goals[1024];
    void *oracle_storage;
    size_t oracle_bytes;
    NfMcuSlalom *oracle;
    NfMcuSlalomStatus oracle_status;
    uint32_t oracle_generation, oracle_epoch, oracle_last_work;
    bool drain_oracle, counted_complete;
    NfHostExplorationStats stats;
} NfHostExploration;

static void oracle_status(NfHostExploration *h, uint8_t *required,
                          NfExplorationOracleResult *out)
{
    NfMcuSlalomResult result;
    memset(&result, 0, sizeof(result));
    if (h->oracle != NULL) {
        (void)nf_mcu_slalom_result(h->oracle, &result, required,
                                  (size_t)h->planned.width * h->planned.height);
        h->stats.oracle_work_units += result.work_units - h->oracle_last_work;
        h->oracle_last_work = result.work_units;
    }
    if (h->oracle_status != NF_MCU_SLALOM_PENDING && !h->counted_complete) {
        ++h->stats.oracle_completed;
        h->counted_complete = true;
    }
    if (out == NULL) return;
    memset(out, 0, sizeof(*out));
    out->generation = h->oracle_generation;
    out->epoch = h->oracle_epoch;
    out->goal_entry_us = result.goal_entry_us;
    out->stop_us = result.stop_us;
    out->dependencies_complete = result.requirements_complete;
    switch (h->oracle_status) {
    case NF_MCU_SLALOM_PENDING: out->status = NF_EXPLORATION_ORACLE_PENDING; break;
    case NF_MCU_SLALOM_EXACT: out->status = NF_EXPLORATION_ORACLE_EXACT; break;
    case NF_MCU_SLALOM_NO_PATH:
    case NF_MCU_SLALOM_NO_FEASIBLE_TERMINAL: out->status = NF_EXPLORATION_ORACLE_NO_PATH; break;
    case NF_MCU_SLALOM_CAPACITY: out->status = NF_EXPLORATION_ORACLE_CAPACITY; break;
    default: out->status = NF_EXPLORATION_ORACLE_INVALID; break;
    }
}

static void oracle_callback(void *context, const NfExplorationSnapshot *s,
                            bool restart, uint32_t budget, uint8_t *required,
                            NfExplorationOracleResult *out)
{
    NfHostExploration *h = context;
    ++h->stats.oracle_callbacks;
    if (restart) {
        if (h->oracle != NULL && h->oracle_status == NF_MCU_SLALOM_PENDING)
            ++h->stats.oracle_aborted;
        ++h->stats.oracle_started;
        h->oracle_generation = s->generation;
        h->oracle_epoch = s->epoch;
        h->oracle_last_work = 0U;
        h->counted_complete = false;
        h->oracle_status = nf_mcu_slalom_begin(h->oracle_storage, h->oracle_bytes,
            s->width, s->height, s->walls, s->goals, s->start_x, s->start_y, 0U,
            &h->oracle);
    }
    do {
        if (h->oracle_status == NF_MCU_SLALOM_PENDING)
            h->oracle_status = nf_mcu_slalom_step(h->oracle, budget);
    } while (h->drain_oracle && h->oracle_status == NF_MCU_SLALOM_PENDING);
    oracle_status(h, required, out);
}

void *nf_host_exploration_create(uint8_t width, uint8_t height,
                                 const NfExplorationConfig *config, bool drain)
{
    size_t bytes = nf_mcu_slalom_workspace_bytes_for(width, height);
    if (bytes == 0U || width > NF_EXPLORATION_MAX_SIZE || height > NF_EXPLORATION_MAX_SIZE)
        return NULL;
    NfHostExploration *h = calloc(1U, sizeof(*h));
    if (h == NULL) return NULL;
    h->oracle_storage = calloc(1U, bytes);
    if (h->oracle_storage == NULL) { free(h); return NULL; }
    h->oracle_bytes = bytes;
    h->drain_oracle = drain;
    h->planned.width = width; h->planned.height = height;
    h->stats.policy_workspace_bytes = sizeof(h->policy);
    h->stats.oracle_workspace_bytes = bytes;
    nf_exploration_reset(&h->policy, config);
    return h;
}

void nf_host_exploration_free(void *context)
{
    NfHostExploration *h = context;
    if (h != NULL) { free(h->oracle_storage); free(h); }
}

static NfExplorationSnapshot snapshot(NfHostExploration *h, const uint8_t *known,
    const uint8_t *walls, const uint8_t *visited, const uint8_t *goals,
    uint32_t generation, uint32_t epoch, uint8_t x, uint8_t y, uint8_t heading)
{
    NfExplorationSnapshot s = h->planned;
    s.known = known; s.walls = walls; s.visited = visited; s.goals = goals;
    s.generation = generation; s.epoch = epoch;
    s.x = x; s.y = y; s.heading = heading;
    return s;
}

int nf_host_exploration_begin(void *context, const uint8_t *known,
    const uint8_t *walls, const uint8_t *visited, const uint8_t *goals,
    uint32_t generation, uint32_t epoch, uint8_t x, uint8_t y, uint8_t heading,
    bool predictive)
{
    NfHostExploration *h = context;
    const size_t cells = (size_t)h->planned.width * h->planned.height;
    memcpy(h->known, known, cells); memcpy(h->walls, walls, cells);
    memcpy(h->visited, visited, cells); memcpy(h->goals, goals, cells);
    h->planned = snapshot(h, h->known, h->walls, h->visited, h->goals,
                          generation, epoch, x, y, heading);
    ++h->stats.policy_jobs;
    return nf_exploration_begin(&h->policy, &h->planned, oracle_callback, h, predictive);
}

int nf_host_exploration_step(void *context, uint32_t budget)
{
    NfHostExploration *h = context;
    NfExplorationDecision decision;
    NfExplorationStatus status = nf_exploration_result(&h->policy, &decision);
    while (budget-- > 0U && status == NF_EXPLORATION_PENDING) {
        uint32_t before = h->policy.job_decision.navigation_edges;
        ++h->stats.policy_work_units;
        status = nf_exploration_step(&h->policy, 1U);
        h->stats.navigation_edges += h->policy.job_decision.navigation_edges - before;
    }
    return status;
}

int nf_host_exploration_oracle_step(void *context, uint32_t budget)
{
    NfHostExploration *h = context;
    if (h->oracle != NULL && h->oracle_status == NF_MCU_SLALOM_PENDING)
        h->oracle_status = nf_mcu_slalom_step(h->oracle, budget);
    oracle_status(h, NULL, NULL);
    return h->oracle_status;
}

int nf_host_exploration_result(void *context, NfExplorationDecision *decision)
{
    NfHostExploration *h = context;
    return nf_exploration_result(&h->policy, decision);
}

int nf_host_exploration_apply(void *context, const uint8_t *known,
    const uint8_t *walls, const uint8_t *visited, const uint8_t *goals,
    uint32_t generation, uint32_t epoch, uint8_t x, uint8_t y, uint8_t heading,
    NfExplorationDecision *decision)
{
    NfHostExploration *h = context;
    NfExplorationSnapshot current = snapshot(h, known, walls, visited, goals,
                                             generation, epoch, x, y, heading);
    return nf_exploration_apply_result(&h->planned, &current, decision);
}

bool nf_host_exploration_note_goal(void *context, const uint8_t *known,
    const uint8_t *walls, const uint8_t *visited, const uint8_t *goals,
    uint32_t generation, uint32_t epoch, uint8_t x, uint8_t y, uint8_t heading)
{
    NfHostExploration *h = context;
    NfExplorationSnapshot current = snapshot(h, known, walls, visited, goals,
                                             generation, epoch, x, y, heading);
    return nf_exploration_note_goal_reached(&h->policy, &current);
}

int nf_host_exploration_guard(void *context, const uint8_t *known,
    const uint8_t *walls, const uint8_t *visited, const uint8_t *goals,
    uint32_t generation, uint32_t epoch, uint8_t x, uint8_t y, uint8_t heading,
    bool accelerated, NfExplorationDecision *decision)
{
    NfHostExploration *h = context;
    NfExplorationSnapshot current = snapshot(h, known, walls, visited, goals,
                                             generation, epoch, x, y, heading);
    return nf_exploration_guard_acceleration(&current, accelerated, decision);
}

int nf_host_exploration_progress(void *context, const uint8_t *known,
    const uint8_t *walls, const uint8_t *visited, const uint8_t *goals,
    uint32_t generation, uint32_t epoch, uint8_t x, uint8_t y, uint8_t heading,
    bool rejected_or_missed, NfExplorationDecision *decision)
{
    NfHostExploration *h = context;
    NfExplorationSnapshot current = snapshot(h, known, walls, visited, goals,
                                             generation, epoch, x, y, heading);
    return nf_exploration_guard_progress(&h->policy, &current, rejected_or_missed, decision);
}

void nf_host_exploration_stats(void *context, NfHostExplorationStats *stats)
{
    *stats = ((NfHostExploration *)context)->stats;
}
