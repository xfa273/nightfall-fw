#ifndef F405_ORTHOGONAL_PREVIEW_H
#define F405_ORTHOGONAL_PREVIEW_H
#include "compact_orthogonal.h"

/* case is run_shortest's case argument, not necessarily the OP case label.
 * In v1.0.0 e.g. mode2 OP case3 calls run_shortest(2,4).
 * Input contains 16x16 F405 map bytes, bottom-left origin. All unknown or
 * inconsistent wall faces are conservatively closed. No NVM access or writes.
 * Foreground/non-reentrant. Output/result are unchanged on failure. */
NfRoutePlanStatus f405_orthogonal_preview(uint8_t mode, uint8_t case_index,
    const uint8_t *map_cells, size_t cell_count,
    uint16_t *output, size_t capacity, NfCompactResult *result);

/* Runtime entry: use the actual angle-accumulation flag, not the OP label. */
NfRoutePlanStatus f405_orthogonal_plan(uint8_t mode, uint8_t case_index, bool nominal_angles,
    const uint8_t *map_cells, size_t cell_count,
    uint16_t *output, size_t capacity, NfCompactResult *result);

/* Public for host inspection/differential tests; no run parameters modified. */
bool f405_orthogonal_config(uint8_t mode, uint8_t case_index,
                            NfOrthogonalPlannerConfig *config);
#endif
