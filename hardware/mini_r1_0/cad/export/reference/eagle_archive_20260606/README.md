        # Eagle reference archive 2026-06-06 for mini_r1_0

        This directory preserves Eagle-derived reference data while Autodesk EAGLE 9.6.2 is still available locally.

        Legacy mini_r1_0 reference archive. Fan mount has board data only.

        ## Boards

        - `HM_Nightfall-mini_v1`: `eagle/HM_Nightfall-mini_v1.brd` / `eagle/HM_Nightfall-mini_v1.sch`
- `HM_Nightfall-mini_Fan-Mount_v1`: `eagle/HM_Nightfall-mini_Fan-Mount_v1.brd` / board only

        ## Files

        - `mini_r1_0_eagle_sources_20260606.zip`: Eagle source archive (`.sch`, `.brd`, `.cam`, `.dru`, `.scr` where available).
        - `mini_r1_0_eagle_exports_20260606.zip`: Eagle CLI exports for visual/manufacturing reference.

        ## Export contents

        For each board, the export zip contains:

        - Eagle CAM outputs generated with `hardware/mini_r1_0/cad/eagle/HM_Nightfall-mini_v1.cam`
        - Excellon drill output from the same CAM job
        - `*_board_all_layers.png` board image exported by Eagle
        - `*.net` and `*.partlist.txt` when a schematic exists

        These exports are for archival comparison and recovery. They are not automatically treated as current order candidates.
