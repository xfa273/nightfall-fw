#ifndef NIGHTFALL_MCU_SLALOM_TABLES_H
#define NIGHTFALL_MCU_SLALOM_TABLES_H
#include <stdint.h>
/* Generated timing/geometry: f413-preorder-mode2 case8, source planner 4fb45ed.
 * Speed indices: nominal500, low300, crawl/start100 mm/s. Kind indices map to
 * generic LARGE90..135_OUT (1..7). Side indices: right0,left1. */
#define NF_MCU_SLALOM_MAX_STEPS 64U
#define NF_MCU_SLALOM_TABLE_INF UINT32_MAX
#define NF_MCU_SLALOM_INVALID_TEMPLATE 255U
/* SHA256 of generator, profile and compiler-discovered local input files. */
extern const char nf_mcu_slalom_inputs_sha256[65];
/* Conjunctive requirements proved necessary and sufficient for both KERI
 * topology and sampled geometry. This is not only a centre-line crossing list. */
typedef struct {
    int8_t dx, dy;
    uint8_t dir;
} NfMcuCrossedEdge;
typedef struct {
    int8_t dx, dy;
    uint32_t time_us[3];
} NfMcuGoalCross;
typedef struct {
    int8_t delta_hx, delta_hy;
    uint8_t end_heading, edge_count, cross_count;
    uint16_t edge_offset, cross_offset;
} NfMcuTurnTemplate;
extern const uint32_t nf_mcu_connector_us[2][3][4][65];
#define NF_MCU_STOP_CROSS_TRIANGLE_COUNT 2145U
extern const uint16_t nf_mcu_stop_cross_index[2][3][NF_MCU_STOP_CROSS_TRIANGLE_COUNT];
extern const uint32_t nf_mcu_stop_cross_values[];
extern const uint16_t nf_mcu_stop_cross_value_count;
/* Exact dictionary lookup, triangular row stop*(stop+1)/2 + cross. */
static inline uint32_t nf_mcu_stop_cross_time_us(unsigned diagonal, unsigned entry, unsigned stop,
                                                 unsigned cross) {
    if (diagonal >= 2U || entry >= 3U || stop > NF_MCU_SLALOM_MAX_STEPS || cross > stop)
        return NF_MCU_SLALOM_TABLE_INF;
    return nf_mcu_stop_cross_values[nf_mcu_stop_cross_index[diagonal][entry]
                                                           [stop * (stop + 1U) / 2U + cross]];
}
extern const uint32_t nf_mcu_turn_us[3][7];
extern const uint32_t nf_mcu_start_us;
extern const NfMcuTurnTemplate nf_mcu_turn_templates[3][8][7][2];
extern const NfMcuCrossedEdge nf_mcu_required_turn_edges[];
extern const NfMcuGoalCross nf_mcu_goal_crosses[];
extern const uint16_t nf_mcu_required_turn_edge_count, nf_mcu_goal_cross_count;
#endif
