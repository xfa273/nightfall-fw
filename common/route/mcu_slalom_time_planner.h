#ifndef NF_MCU_SLALOM_TIME_PLANNER_H
#define NF_MCU_SLALOM_TIME_PLANNER_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Exact F413 mode2/case8 motion graph, for rectangular mazes up to 32x32 cells.
 * The state is owned entirely by caller workspace; no allocation, I/O or HAL.
 * Unknown=open/closed policy belongs to the caller. Wall bits are 1<<NESW.
 * Input maps are copied by begin(), so subsequent observations cannot mutate
 * an in-flight solve. Retain that snapshot's generation/epoch in the caller.
 * Input/output buffers must not overlap workspace. A context has one owner;
 * do not call it concurrently. Abort by discarding it or calling begin again
 * after step returns; there are no allocations or external resources to free. */
#define NF_MCU_SLALOM_WIDTH 32U
#define NF_MCU_SLALOM_CELLS 1024U
#define NF_MCU_SLALOM_WORKSPACE_BYTES (200U * 1024U)
#define NF_MCU_SLALOM_WORKSPACE_ALIGNMENT 8U
#define NF_MCU_SLALOM_PROFILE_ID "f413-mode2-case8-keri1to5-us-v1"

typedef struct NfMcuSlalom NfMcuSlalom;
typedef enum {
    NF_MCU_SLALOM_PENDING = 0,
    NF_MCU_SLALOM_EXACT,
    NF_MCU_SLALOM_NO_PATH,
    NF_MCU_SLALOM_NO_FEASIBLE_TERMINAL,
    NF_MCU_SLALOM_INVALID,
    NF_MCU_SLALOM_CAPACITY,
    NF_MCU_SLALOM_OVERFLOW
} NfMcuSlalomStatus;

typedef struct {
    NfMcuSlalomStatus status;
    uint32_t goal_entry_us;
    uint32_t stop_us;
    uint32_t expanded_states;
    uint32_t relaxed_edges;
    uint32_t work_units;
    uint16_t heap_peak;
    uint16_t action_count;
    uint8_t goal_x;
    uint8_t goal_y;
    bool requirements_complete;
    size_t workspace_used;
} NfMcuSlalomResult;

size_t nf_mcu_slalom_workspace_bytes(void);
size_t nf_mcu_slalom_workspace_bytes_for(uint8_t width, uint8_t height);
const char *nf_mcu_slalom_status_name(NfMcuSlalomStatus status);
NfMcuSlalomStatus nf_mcu_slalom_begin(void *workspace, size_t workspace_bytes, uint8_t width,
                                      uint8_t height, const uint8_t *walls, const uint8_t *goals,
                                      uint8_t start_x, uint8_t start_y, uint8_t start_heading,
                                      NfMcuSlalom **out_context);
/* One work unit is one bounded edge/terminal/reconstruction operation.
 * Returns PENDING when budget is consumed; budget=0 is a read-only poll.
 * EXACT is returned only after a sufficient required-open set is complete. */
NfMcuSlalomStatus nf_mcu_slalom_step(NfMcuSlalom *context, uint32_t work_budget);
/* required_open may be NULL for status-only inspection. When non-NULL it
 * receives width*height symmetric cell masks, including the complete stopping tail.
 * Incomplete/failed results never claim requirements_complete. */
NfMcuSlalomStatus nf_mcu_slalom_result(const NfMcuSlalom *context, NfMcuSlalomResult *result,
                                       uint8_t *required_open, size_t required_capacity);
#endif
