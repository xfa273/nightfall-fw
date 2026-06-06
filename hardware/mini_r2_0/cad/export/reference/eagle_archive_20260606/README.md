        # Eagle reference archive 2026-06-06 for mini_r2_0

        This directory preserves Eagle-derived reference data while Autodesk EAGLE 9.6.2 is still available locally.

        Includes the main mini_r2_0 board and small adapter/encoder boards. The actually ordered main-board Gerber is archived separately under reference/eagle_ordered_2026-03-22.

        ## Boards

        - `HM_Nightfall-mini-2e_v1`: `eagle/HM_Nightfall-mini-2e_v1.brd` / `eagle/HM_Nightfall-mini-2e_v1.sch`
- `HM_Nightfall-mini-2e_Encoder-PCB-L_v1`: `eagle/HM_Nightfall-mini-2e_Encoder-PCB-L_v1.brd` / `eagle/HM_Nightfall-mini-2e_Encoder-PCB-L_v1.sch`
- `HM_Nightfall-mini-2e_Encoder-PCB-R_v1`: `eagle/HM_Nightfall-mini-2e_Encoder-PCB-R_v1.brd` / `eagle/HM_Nightfall-mini-2e_Encoder-PCB-R_v1.sch`
- `STLink-Adapter`: `eagle/STLink-Adapter.brd` / `eagle/STLink-Adapter.sch`
- `UART-Adapter`: `eagle/UART-Adapter.brd` / `eagle/UART-Adapter.sch`

        ## Files

        - `mini_r2_0_eagle_sources_20260606.zip`: Eagle source archive (`.sch`, `.brd`, `.cam`, `.dru`, `.scr` where available).
        - `mini_r2_0_eagle_exports_20260606.zip`: Eagle CLI exports for visual/manufacturing reference.

        ## Export contents

        For each board, the export zip contains:

        - Eagle CAM outputs generated with `hardware/mini_r2_0/cad/eagle/HM_Nightfall-mini_v1.cam`
        - Excellon drill output from the same CAM job
        - `*_board_all_layers.png` board image exported by Eagle
        - `*.net` and `*.partlist.txt` when a schematic exists

        These exports are for archival comparison and recovery. They are not automatically treated as current order candidates.
