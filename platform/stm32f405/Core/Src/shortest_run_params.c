#include "../Inc/shortest_run_params.h"

static uint8_t g_shortest_turn_bank_id[8] = {0};

uint8_t shortest_get_turn_bank_id(uint8_t mode) {
    if (mode < (uint8_t)(sizeof(g_shortest_turn_bank_id) / sizeof(g_shortest_turn_bank_id[0]))) {
        return g_shortest_turn_bank_id[mode];
    }
    return 0;
}

void shortest_set_turn_bank_id(uint8_t mode, uint8_t bank_id) {
    if (bank_id >= SHORTEST_TURN_BANK_COUNT) {
        bank_id = 0;
    }
    if (mode < (uint8_t)(sizeof(g_shortest_turn_bank_id) / sizeof(g_shortest_turn_bank_id[0]))) {
        g_shortest_turn_bank_id[mode] = bank_id;
    }
}

static const ShortestRunModeParams_t* const* shortest_get_turn_bank_table(uint8_t mode) {
    switch (mode) {
        case 2: return &shortestTurnBankMode2[0];
        case 3: return &shortestTurnBankMode3[0];
        case 4: return &shortestTurnBankMode4[0];
        case 5: return &shortestTurnBankMode5[0];
        case 6: return &shortestTurnBankMode6[0];
        case 7: return &shortestTurnBankMode7[0];
        default: return &shortestTurnBankMode2[0];
    }
}

const ShortestRunModeParams_t* shortest_get_mode_params(uint8_t mode) {
    const ShortestRunModeParams_t* const* bank = shortest_get_turn_bank_table(mode);
    uint8_t bank_id = shortest_get_turn_bank_id(mode);
    if (bank_id >= SHORTEST_TURN_BANK_COUNT) {
        bank_id = 0;
    }
    const ShortestRunModeParams_t* p = bank[bank_id];
    if (p == 0) {
        p = bank[0];
    }
    return p;
}

const ShortestRunCaseParams_t* shortest_get_case_params(uint8_t mode, uint8_t case_index) {
    uint8_t idx = 0;
    if (case_index >= 1 && case_index <= 9) {
        idx = (uint8_t)(case_index - 1);
    }

    uint8_t max_idx = 8;
    if (mode >= 2 && mode <= 6) max_idx = 8;
    if (mode == 7) max_idx = 4;
    if (idx > max_idx) {
        idx = max_idx;
    }

    switch (mode) {
        case 2: return &shortestRunCaseParamsMode2[idx];
        case 3: return &shortestRunCaseParamsMode3[idx];
        case 4: return &shortestRunCaseParamsMode4[idx];
        case 5: return &shortestRunCaseParamsMode5[idx];
        case 6: return &shortestRunCaseParamsMode6[idx];
        case 7: return &shortestRunCaseParamsMode7[idx];
        default: return &shortestRunCaseParamsMode2[idx];
    }
}
