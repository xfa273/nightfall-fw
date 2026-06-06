    # Eagle reference archive 2026-06-06 for classic_r1_0

    This directory preserves Eagle-derived reference data while Autodesk EAGLE 9.6.2 is still available locally.

    Legacy classic_r1_0 reference archive. Uses the generic two-layer Eagle CAM job from mini_r1_0 for CAM exports because no classic-specific CAM job is present.

    ## Boards

    - `CM_Nightfall-Air_v1`: `eagle/CM_Nightfall-Air_v1.brd` / `eagle/CM_Nightfall-Air_v1.sch`
- `CM_Nightfall-Air_Fan-Mount_v1`: `eagle/CM_Nightfall-Air_Fan-Mount_v1.brd` / board only
- `Skirt-Base-PCB`: `eagle/Skirt-Base-PCB.brd` / board only

    ## Files

    - `classic_r1_0_eagle_sources_20260606.zip`: Eagle source archive (`.sch`, `.brd`, `.cam`, `.dru`, `.scr` where available).
    - `classic_r1_0_eagle_exports_20260606.zip`: Eagle CLI exports for visual/manufacturing reference.

    ## Export contents

    For each board, the export zip contains:

    - Eagle CAM outputs generated with `hardware/mini_r1_0/cad/eagle/HM_Nightfall-mini_v1.cam`
    - Excellon drill output from the same CAM job
    - `*_board_all_layers.png` board image exported by Eagle
    - `*.net` and `*.partlist.txt` when a schematic exists

    These exports are for archival comparison and recovery. They are not automatically treated as current order candidates.
