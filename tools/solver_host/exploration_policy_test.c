#include "exploration_policy.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static NfExplorationWorkspace workspace;
static uint8_t known[16], walls[16], visited[16], goals[16], required[16];

#ifdef NF_EXPLORATION_TEST_REFERENCE
extern void nf_reference_reset(NfExplorationWorkspace *, const NfExplorationConfig *);
extern NfExplorationStatus nf_reference_decide(NfExplorationWorkspace *,
    const NfExplorationSnapshot *, NfExplorationOracleFn, void *, NfExplorationDecision *);
static NfExplorationWorkspace reference_workspace;
#endif

typedef struct {
    unsigned calls;
    unsigned restarts;
    unsigned pending_calls;
    bool incomplete;
    bool stale;
    bool mutate;
    uint32_t generation;
    uint32_t epoch;
} Stub;

static void edge(uint8_t x, uint8_t y, uint8_t d, bool wall)
{
    static const int dx[4] = {0, 1, 0, -1};
    static const int dy[4] = {1, 0, -1, 0};
    unsigned cell = y * 4U + x;
    int nx = x + dx[d], ny = y + dy[d];
    known[cell] |= (uint8_t)(1U << d);
    if (wall) walls[cell] |= (uint8_t)(1U << d);
    else walls[cell] &= (uint8_t)~(1U << d);
    if (nx >= 0 && nx < 4 && ny >= 0 && ny < 4) {
        unsigned next = (unsigned)ny * 4U + (unsigned)nx;
        uint8_t bit = (uint8_t)(1U << ((d + 2U) & 3U));
        known[next] |= bit;
        if (wall) walls[next] |= bit;
        else walls[next] &= (uint8_t)~bit;
    }
}

static NfExplorationSnapshot fixture(void)
{
    unsigned x, y;
    NfExplorationSnapshot s = {
        known, walls, visited, goals, 1U, 1U, 4U, 4U, 0U, 0U, 0U, 0U, 0U
    };
    memset(known, 0, sizeof(known));
    memset(walls, 0, sizeof(walls));
    memset(visited, 0, sizeof(visited));
    memset(goals, 0, sizeof(goals));
    memset(required, 0, sizeof(required));
    for (x = 0; x < 4; ++x) {
        edge((uint8_t)x, 0U, 2U, true);
        edge((uint8_t)x, 3U, 0U, true);
    }
    for (y = 0; y < 4; ++y) {
        edge(0U, (uint8_t)y, 3U, true);
        edge(3U, (uint8_t)y, 1U, true);
    }
    edge(0U, 0U, 0U, false);
    edge(0U, 0U, 1U, true);
    goals[4] = 1U;
    visited[0] = 1U;
    required[0] = 1U;
    nf_exploration_reset(&workspace, NULL);
    return s;
}

static void stub(void *context, const NfExplorationSnapshot *s, bool restart,
                 uint32_t budget, uint8_t *out, NfExplorationOracleResult *r)
{
    Stub *p = context;
    assert(budget > 0U);
    ++p->calls;
    if (restart) {
        ++p->restarts;
        p->generation = s->generation;
        p->epoch = s->epoch;
    }
    r->generation = p->generation;
    r->epoch = p->stale ? p->epoch - 1U : p->epoch;
    if (p->pending_calls > 0U) {
        --p->pending_calls;
        r->status = NF_EXPLORATION_ORACLE_PENDING;
        return;
    }
    r->status = NF_EXPLORATION_ORACLE_EXACT;
    r->goal_entry_us = 300000U;
    r->stop_us = 600000U;
    r->dependencies_complete = !p->incomplete;
    memcpy(out, required, 16);
    if (p->mutate) edge(2U, 2U, 0U, true);
}

static void reach_goal(NfExplorationSnapshot *s)
{
    s->y = 1U;
    visited[4] = 1U;
    edge(0U, 1U, 0U, false);
    edge(0U, 1U, 1U, false);
    ++s->generation;
}

static void test_sliced(void)
{
    NfExplorationSnapshot s = fixture();
    NfExplorationDecision expected, actual;
    Stub p;
    uint32_t budget;
    reach_goal(&s);
    required[8] = 1U;
    memset(&p, 0, sizeof(p));
    assert(nf_exploration_decide(&workspace, &s, stub, &p, &expected) == NF_EXPLORATION_MOVE);
    for (budget = 1U; budget <= 17U; budget += 4U) {
        uint32_t calls = 0U;
        NfExplorationStatus status;
        nf_exploration_reset(&workspace, NULL);
        memset(&p, 0, sizeof(p));
        assert(nf_exploration_begin(&workspace, &s, stub, &p, false) == NF_EXPLORATION_PENDING);
        assert(p.calls == 0U); /* begin is O(1), no oracle or map scan. */
        assert(nf_exploration_step(&workspace, 0U) == NF_EXPLORATION_PENDING);
        do {
            uint32_t before = workspace.job_decision.navigation_edges;
            status = nf_exploration_step(&workspace, budget);
            assert(workspace.job_decision.navigation_edges - before <= budget);
            assert(++calls < 10000U);
            if (status == NF_EXPLORATION_PENDING) {
                assert(nf_exploration_result(&workspace, &actual) == status);
                assert(!actual.certified && actual.direction == NF_EXPLORATION_NO_DIRECTION);
            }
        } while (status == NF_EXPLORATION_PENDING);
        assert(nf_exploration_result(&workspace, &actual) == NF_EXPLORATION_MOVE);
        assert(memcmp(&actual, &expected, sizeof(actual)) == 0);
        assert(p.calls == 1U && p.restarts == 1U);
    }
    /* Cancelled jobs do not publish partial work or destroy resumability. */
    nf_exploration_reset(&workspace, NULL);
    assert(nf_exploration_begin(&workspace, &s, stub, &p, false) == NF_EXPLORATION_PENDING);
    assert(nf_exploration_step(&workspace, 20U) == NF_EXPLORATION_PENDING);
    assert(nf_exploration_begin(&workspace, &s, stub, &p, false) == NF_EXPLORATION_PENDING);
    while (nf_exploration_step(&workspace, 1U) == NF_EXPLORATION_PENDING) {}
    assert(nf_exploration_result(&workspace, &actual) == NF_EXPLORATION_MOVE);
    assert(actual.direction == expected.direction);

    /* Prediction never invents visit/goal knowledge. After actual goal entry,
     * an unknown departure may become a safe proposal at the next cell. */
    s = fixture(); memset(&p, 0, sizeof(p)); s.y = 1U;
    assert(!nf_exploration_note_goal_reached(&workspace, &s));
    assert(nf_exploration_begin(&workspace, &s, stub, &p, true) == NF_EXPLORATION_PENDING);
    while (nf_exploration_step(&workspace, 7U) == NF_EXPLORATION_PENDING) {}
    assert(nf_exploration_result(&workspace, &actual) == NF_EXPLORATION_FALLBACK);
    assert(!actual.certified && !workspace.reached_goal && p.calls == 0U);

    s = fixture(); reach_goal(&s); memset(&p, 0, sizeof(p));
    required[8] = 1U; required[12] = 2U;
    assert(nf_exploration_note_goal_reached(&workspace, &s));
    s.y = 2U; /* This future cell has not been visited. */
    assert(nf_exploration_begin(&workspace, &s, stub, &p, true) == NF_EXPLORATION_PENDING);
    while (nf_exploration_step(&workspace, 7U) == NF_EXPLORATION_PENDING) {}
    assert(nf_exploration_result(&workspace, &actual) == NF_EXPLORATION_PROPOSAL);
    assert(actual.direction == 0U && !actual.certified);
    {
        NfExplorationSnapshot arrived = s;
        NfExplorationDecision blocked = actual, stale = actual;
        visited[8] = 1U;
        edge(0U, 2U, 0U, true); ++arrived.generation;
        assert(nf_exploration_apply_result(&s, &arrived, &blocked) == NF_EXPLORATION_FALLBACK);
        assert(blocked.direction == NF_EXPLORATION_NO_DIRECTION);
        edge(0U, 2U, 0U, false); /* Independent open-world arrival case. */
        assert(nf_exploration_apply_result(&s, &arrived, &actual) == NF_EXPLORATION_MOVE);
        ++arrived.epoch;
        assert(nf_exploration_apply_result(&s, &arrived, &stale) == NF_EXPLORATION_FALLBACK);
        assert(!stale.certified);
    }
}

static NfExplorationSnapshot surrogate_fixture(void)
{
    NfExplorationSnapshot s = fixture();
    goals[4] = 0U; goals[15] = 1U;
    s.x = 3U; s.y = 3U; s.heading = 3U;
    visited[15] = 1U;
    edge(3U, 3U, 2U, false); edge(3U, 3U, 3U, false);
    return s;
}

static void test_pending_surrogate(void)
{
    NfExplorationSnapshot s = surrogate_fixture();
    NfExplorationDecision d;
    NfExplorationConfig config = nf_exploration_default_config();
    Stub p = {0};
    p.pending_calls = 100U;
    assert(!config.pending_surrogate); /* Measured finite-budget default. */
    config.pending_surrogate = true;
    nf_exploration_reset(&workspace, &config);
    assert(nf_exploration_decide(&workspace, &s, stub, &p, &d) == NF_EXPLORATION_MOVE);
    assert(d.direction == 3U && d.reason == NF_EXPLORATION_REASON_SURROGATE);
    assert(!d.certified && d.lower_us == 0U && workspace.oracle_pending && !workspace.oracle_cached);
    assert(d.oracle_status == NF_EXPLORATION_ORACLE_PENDING);

    /* Exact takeover retains the same oracle rather than restarting it. */
    required[0] = 1U; required[4] = 1U; required[8] = 1U;
    required[12] = 2U; required[13] = 2U; required[14] = 2U;
    p.pending_calls = 0U;
    assert(nf_exploration_decide(&workspace, &s, stub, &p, &d) == NF_EXPLORATION_MOVE);
    assert(p.restarts == 1U && p.calls == 2U && workspace.oracle_cached);
    assert(d.reason != NF_EXPLORATION_REASON_SURROGATE && d.lower_us == 300000U);
    for (uint8_t y = 0U; y < 3U; ++y) edge(0U, y, 0U, false);
    for (uint8_t x = 0U; x < 3U; ++x) edge(x, 3U, 1U, false);
    ++s.generation;
    assert(nf_exploration_decide(&workspace, &s, stub, &p, &d) == NF_EXPLORATION_COMPLETE);
    assert(d.certified && p.calls == 2U);

    /* The same observed cardinal route cannot certify an unfinished exact
     * oracle, even though its entire surrogate path is now known-open. */
    nf_exploration_reset(&workspace, &config);
    memset(&p, 0, sizeof(p)); p.pending_calls = 100U;
    assert(nf_exploration_decide(&workspace, &s, stub, &p, &d) == NF_EXPLORATION_FALLBACK);
    assert(d.reason == NF_EXPLORATION_REASON_ORACLE_PENDING && !d.certified && d.lower_us == 0U);

    s = surrogate_fixture(); memset(&p, 0, sizeof(p)); p.pending_calls = 100U;
    config.pending_surrogate = false;
    nf_exploration_reset(&workspace, &config);
    assert(nf_exploration_decide(&workspace, &s, stub, &p, &d) == NF_EXPLORATION_FALLBACK);
    assert(d.reason == NF_EXPLORATION_REASON_ORACLE_PENDING && d.navigation_edges == 0U);

    s = surrogate_fixture(); memset(&p, 0, sizeof(p)); p.pending_calls = 100U;
    config.pending_surrogate = true;
    nf_exploration_reset(&workspace, &config);
    assert(nf_exploration_note_goal_reached(&workspace, &s));
    s.x = 2U; /* Predict an unvisited cell with an unknown forward departure. */
    assert(nf_exploration_begin(&workspace, &s, stub, &p, true) == NF_EXPLORATION_PENDING);
    while (nf_exploration_step(&workspace, 3U) == NF_EXPLORATION_PENDING) {}
    assert(nf_exploration_result(&workspace, &d) == NF_EXPLORATION_PROPOSAL);
    assert(d.reason == NF_EXPLORATION_REASON_SURROGATE && d.direction == 3U && !d.certified);
    {
        NfExplorationSnapshot arrived = s;
        visited[14] = 1U; edge(2U, 3U, 3U, false); ++arrived.generation;
        assert(nf_exploration_apply_result(&s, &arrived, &d) == NF_EXPLORATION_MOVE);
        assert(d.reason == NF_EXPLORATION_REASON_SURROGATE && !d.certified && d.lower_us == 0U);
    }
}

static void test_progress_guard(void)
{
    NfExplorationSnapshot s = fixture();
    NfExplorationDecision d = {0};
    visited[4] = 1U;
    d.status = NF_EXPLORATION_PENDING;
    assert(nf_exploration_guard_progress(&workspace, &s, true, &d) == NF_EXPLORATION_FALLBACK);
    assert(d.reason == NF_EXPLORATION_REASON_PROGRESS_FALLBACK && workspace.fallback_held);
    /* A cheap reversal can finish during a long turn while a straight-cell
     * proposal misses its deadline. Neither readiness nor pose changes alone
     * may switch away from the progress-making Adachi route. */
    for (unsigned i = 0U; i < 100U; ++i) {
        s.y = (uint8_t)(i & 1U);
        d.status = NF_EXPLORATION_MOVE;
        d.direction = s.y == 0U ? 0U : 2U;
        d.known_straight = true;
        assert(nf_exploration_guard_progress(&workspace, &s, false, &d) == NF_EXPLORATION_FALLBACK);
        assert(d.direction == NF_EXPLORATION_NO_DIRECTION && !d.certified && !d.known_straight);
    }
    assert(nf_exploration_begin(&workspace, &s, NULL, NULL, true) == NF_EXPLORATION_PENDING);
    d.status = NF_EXPLORATION_MOVE; d.direction = 0U;
    assert(nf_exploration_guard_progress(&workspace, &s, false, &d) == NF_EXPLORATION_FALLBACK);

    /* A valid exact certificate can finish while the fallback is held. */
    d.status = NF_EXPLORATION_COMPLETE; d.certified = true;
    d.oracle_status = NF_EXPLORATION_ORACLE_EXACT;
    assert(nf_exploration_guard_progress(&workspace, &s, false, &d) == NF_EXPLORATION_COMPLETE);
    assert(d.certified);

    edge(0U, 1U, 1U, false); ++s.generation;
    d.status = NF_EXPLORATION_MOVE; d.direction = 1U; d.certified = false;
    assert(nf_exploration_guard_progress(&workspace, &s, false, &d) == NF_EXPLORATION_MOVE);
    assert(!workspace.fallback_held && d.direction == 1U);
    assert(nf_exploration_guard_progress(&workspace, &s, true, &d) == NF_EXPLORATION_FALLBACK);
    ++s.epoch;
    d.status = NF_EXPLORATION_MOVE; d.direction = 1U;
    assert(nf_exploration_guard_progress(&workspace, &s, false, &d) == NF_EXPLORATION_MOVE);
    assert(!workspace.fallback_held);
}

#ifdef NF_EXPLORATION_TEST_REFERENCE
static uint32_t random_state = 31337U;
static uint32_t random_next(void)
{
    random_state = random_state * 1664525U + 1013904223U;
    return random_state;
}

static void test_pruning_reference(void)
{
    unsigned trial, pruned = 0U;
    for (trial = 0U; trial < 2000U; ++trial) {
        NfExplorationSnapshot s = fixture();
        NfExplorationDecision fast, full;
        NfExplorationConfig config = nf_exploration_default_config();
        Stub a = {0}, b = {0};
        unsigned x, y, dir;
        reach_goal(&s);
        for (y = 0U; y < 4U; ++y) for (x = 0U; x < 4U; ++x) {
            unsigned cell = y * 4U + x;
            if (cell != 0U && cell != 4U) visited[cell] = (uint8_t)(random_next() >> 31U);
            for (dir = 0U; dir < 2U; ++dir) {
                uint32_t r = random_next();
                if ((dir == 0U && y == 3U) || (dir == 1U && x == 3U) ||
                    (x == 0U && dir == 0U)) continue;
                if ((r & 3U) != 0U) edge((uint8_t)x, (uint8_t)y, (uint8_t)dir, (r & 12U) == 0U);
                if ((known[cell] & (1U << dir)) == 0U && (r & 64U) != 0U)
                    required[cell] |= (uint8_t)(1U << dir);
            }
        }
        required[8] |= 1U; /* There is always at least one unknown dependency. */
        config.information_gain_per_mille = (uint16_t)(random_next() % 1001U);
        config.unknown_cost_per_mille = (uint16_t)(1000U + random_next() % 501U);
        config.turn90_us = 100000U + random_next() % 600000U;
        config.uturn_us = 300000U + random_next() % 1200000U;
        s.heading = (uint8_t)(random_next() & 3U);
        if (trial % 3U == 0U) {
            config.pending_surrogate = true;
            goals[4] = 0U; goals[15] = 1U;
            s.x = 3U; s.y = 3U; visited[15] = 1U;
            edge(3U, 3U, 2U, false); edge(3U, 3U, 3U, false);
            a.pending_calls = 1U; b.pending_calls = 1U;
        }
        nf_exploration_reset(&workspace, &config);
        nf_reference_reset(&reference_workspace, &config);
        assert(nf_exploration_decide(&workspace, &s, stub, &a, &fast) ==
               nf_reference_decide(&reference_workspace, &s, stub, &b, &full));
        assert(fast.navigation_edges <= full.navigation_edges);
        pruned += fast.navigation_edges < full.navigation_edges;
        fast.navigation_edges = full.navigation_edges;
        assert(memcmp(&fast, &full, sizeof(fast)) == 0);
    }
    assert(pruned > 100U);
    printf("navigation pruning: 2000 full-graph comparisons PASS (%u pruned)\n", pruned);
}
#endif

int main(void)
{
    NfExplorationSnapshot s;
    NfExplorationDecision d;
    NfExplorationConfig config;
    Stub p;

    s = fixture();
    memset(&p, 0, sizeof(p));
    assert(nf_exploration_decide(&workspace, &s, stub, &p, &d) == NF_EXPLORATION_MOVE);
    assert(d.direction == 0U && !d.certified && p.calls == 0U);
    assert(!d.known_straight); /* Adjacent goal is not acceleration lookahead. */
    reach_goal(&s);
    assert(nf_exploration_decide(&workspace, &s, stub, &p, &d) == NF_EXPLORATION_COMPLETE);
    assert(d.certified && d.generation == s.generation && p.calls == 1U);

    s = fixture(); reach_goal(&s); memset(&p, 0, sizeof(p));
    required[8] = 1U; /* An unknown dependency beyond the observed next cell. */
    assert(nf_exploration_decide(&workspace, &s, stub, &p, &d) == NF_EXPLORATION_MOVE);
    assert(d.direction == 0U && !d.certified && d.target_x == 0U && d.target_y == 2U);
    assert(d.navigation_edges <= workspace.config.navigation_edge_budget);

    s = fixture(); reach_goal(&s); memset(&p, 0, sizeof(p)); p.incomplete = true;
    assert(nf_exploration_decide(&workspace, &s, stub, &p, &d) == NF_EXPLORATION_FALLBACK);
    assert(d.reason == NF_EXPLORATION_REASON_BAD_DEPENDENCIES && !d.certified);

    s = fixture(); reach_goal(&s); memset(&p, 0, sizeof(p)); p.stale = true;
    assert(nf_exploration_decide(&workspace, &s, stub, &p, &d) == NF_EXPLORATION_FALLBACK);
    assert(d.reason == NF_EXPLORATION_REASON_STALE_ORACLE && !d.certified);

    s = fixture(); reach_goal(&s); memset(&p, 0, sizeof(p)); p.pending_calls = 1U;
    required[8] = 1U;
    assert(nf_exploration_decide(&workspace, &s, stub, &p, &d) == NF_EXPLORATION_FALLBACK);
    assert(d.reason == NF_EXPLORATION_REASON_ORACLE_PENDING && !d.certified);
    edge(0U, 2U, 0U, false); ++s.generation;
    assert(nf_exploration_decide(&workspace, &s, stub, &p, &d) == NF_EXPLORATION_COMPLETE);
    assert(p.restarts == 1U && p.calls == 2U && d.generation == s.generation);

    s = fixture(); reach_goal(&s); memset(&p, 0, sizeof(p)); p.pending_calls = 1U;
    required[8] = 1U;
    assert(nf_exploration_decide(&workspace, &s, stub, &p, &d) == NF_EXPLORATION_FALLBACK);
    edge(0U, 2U, 0U, true); ++s.generation;
    assert(nf_exploration_decide(&workspace, &s, stub, &p, &d) == NF_EXPLORATION_FALLBACK);
    assert(d.reason == NF_EXPLORATION_REASON_STALE_ORACLE && !d.certified);

    s = fixture(); reach_goal(&s); memset(&p, 0, sizeof(p));
    required[8] = 1U;
    assert(nf_exploration_decide(&workspace, &s, stub, &p, &d) == NF_EXPLORATION_MOVE);
    edge(0U, 1U, 0U, true); ++s.generation; /* Correction without epoch increment. */
    assert(nf_exploration_decide(&workspace, &s, stub, &p, &d) == NF_EXPLORATION_INVALID);
    assert(!d.certified);
    ++s.epoch;
    assert(nf_exploration_decide(&workspace, &s, stub, &p, &d) == NF_EXPLORATION_MOVE);
    assert(d.direction == 1U); /* Epoch change restarts solver and accepts correction. */

    s = fixture(); reach_goal(&s); memset(&p, 0, sizeof(p));
    required[8] = 1U; config = nf_exploration_default_config();
    config.navigation_edge_budget = 1U;
    nf_exploration_reset(&workspace, &config);
    assert(nf_exploration_decide(&workspace, &s, stub, &p, &d) == NF_EXPLORATION_FALLBACK);
    assert(d.navigation_edges == 1U && d.reason == NF_EXPLORATION_REASON_NAVIGATION_BUDGET);
    assert(d.direction == NF_EXPLORATION_NO_DIRECTION && !d.certified);

    s = fixture(); reach_goal(&s); memset(&p, 0, sizeof(p));
    required[8] = 1U; config = nf_exploration_default_config(); config.turn90_us = 400000000U;
    nf_exploration_reset(&workspace, &config);
    assert(nf_exploration_decide(&workspace, &s, stub, &p, &d) == NF_EXPLORATION_FALLBACK);
    assert(d.reason == NF_EXPLORATION_REASON_COST_RANGE && !d.certified);

    s = fixture(); reach_goal(&s); memset(&p, 0, sizeof(p)); p.mutate = true;
    assert(nf_exploration_decide(&workspace, &s, stub, &p, &d) == NF_EXPLORATION_INVALID);
    assert(d.reason == NF_EXPLORATION_REASON_BAD_SNAPSHOT && !d.certified);

    s = fixture(); walls[5] = 1U; memset(&p, 0, sizeof(p));
    assert(nf_exploration_decide(&workspace, &s, stub, &p, &d) == NF_EXPLORATION_INVALID);
    assert(p.calls == 0U); /* Reject truth bits that were never observed. */

    /* A target change or a freshly ready certificate cannot break the prior
     * motion's acceleration promise, including one made by legacy Adachi. */
    s = fixture(); reach_goal(&s); memset(&d, 0, sizeof(d));
    d.status = NF_EXPLORATION_MOVE; d.direction = 1U; d.next_is_turn90 = true;
    assert(nf_exploration_guard_acceleration(&s, true, &d) == NF_EXPLORATION_MOVE);
    assert(d.direction == s.heading && !d.known_straight && !d.next_is_turn90);
    assert(d.reason == NF_EXPLORATION_REASON_BRAKE_STRAIGHT);
    d.status = NF_EXPLORATION_COMPLETE; d.certified = true;
    d.phase = NF_EXPLORATION_PHASE_DONE;
    assert(nf_exploration_guard_acceleration(&s, true, &d) == NF_EXPLORATION_MOVE);
    assert(!d.certified && d.phase == NF_EXPLORATION_PHASE_RELEVANT);
    d.known_straight = true;
    assert(nf_exploration_guard_acceleration(&s, true, &d) == NF_EXPLORATION_MOVE);
    assert(!d.known_straight); /* Unknown lookahead cannot create a new promise. */
    visited[8] = 1U; edge(0U, 2U, 0U, false); d.known_straight = true;
    assert(nf_exploration_guard_acceleration(&s, true, &d) == NF_EXPLORATION_MOVE);
    assert(d.known_straight); /* Continuing a verified straight may cruise. */
    edge(0U, 1U, 0U, true);
    assert(nf_exploration_guard_acceleration(&s, true, &d) == NF_EXPLORATION_INVALID);
    assert(d.reason == NF_EXPLORATION_REASON_UNSAFE_ACCELERATION);
    assert(d.direction == NF_EXPLORATION_NO_DIRECTION && !d.certified);
    d.status = NF_EXPLORATION_COMPLETE; d.certified = true;
    assert(nf_exploration_guard_acceleration(&s, false, &d) == NF_EXPLORATION_COMPLETE);
    assert(d.certified);

    test_sliced();
    test_pending_surrogate();
    test_progress_guard();
#ifdef NF_EXPLORATION_TEST_REFERENCE
    test_pruning_reference();
#endif

    assert(nf_exploration_workspace_bytes() <= 30U * 1024U);
    printf("exploration policy: PASS, workspace=%zu bytes, max_size=%u\n",
           nf_exploration_workspace_bytes(), (unsigned)NF_EXPLORATION_MAX_SIZE);
    return 0;
}
