#include "global.h"
#include "f405_time_path.h"
#include "f405_orthogonal_preview.h"
#include "maze_grid.h"
#include <string.h>

_Static_assert(ROUTE_MAX_LEN == NF_COMPACT_PATH_CAPACITY, "path capacity mismatch");

/* Replay half-cell ownership for the existing ASCII maze overlay. */
static bool mark_step(int *x, int *y, unsigned heading)
{
    static const int dx[4]={0,1,0,-1}, dy[4]={1,0,-1,0};
    *x+=dx[heading]; *y+=dy[heading];
    if (*x<0 || *y<0 || *x>=MAZE_SIZE || *y>=MAZE_SIZE) return false;
    path_cell[*y][*x]=true;
    return true;
}

static bool mark_path(size_t length)
{
    int x=START_X,y=START_Y,pending=0;
    unsigned heading=0,previous_out=1;
    path_cell[y][x]=true;
    for (size_t i=0;i<=length;++i) {
        unsigned code=path[i];
        if (code>200 && code<300) { pending+=(int)code-200; continue; }
        if (code && code!=300 && code!=400 && code!=501 && code!=502 && code!=601 && code!=602)
            return false;
        unsigned incoming=code>=500;
        int halves=pending+(int)previous_out+(int)incoming;
        if (halves%2) return false;
        for (int j=0;j<halves/2;++j) if (!mark_step(&x,&y,heading)) return false;
        pending=0;
        if (!code) return i==length;
        unsigned side=(code==400 || code>=600)?3U:1U;
        heading=(heading+side)%4;
        if (!mark_step(&x,&y,heading)) return false;
        if (code==502 || code==602) {
            heading=(heading+side)%4;
            if (!mark_step(&x,&y,heading)) return false;
        }
        previous_out=incoming;
    }
    return false;
}

bool f405_time_build_path(uint8_t mode, uint8_t case_index)
{
    memset(path,0,sizeof(path));
    memset(path_cell,0,sizeof(path_cell));
    load_map_from_eeprom();
    /* Firmware map elements are uint16_t; the adapter consumes packed bytes.
     * Never cast map to uint8_t*: that would interleave padding/high bytes. */
    uint8_t cells[MAZE_SIZE][MAZE_SIZE];
    for (unsigned y=0;y<MAZE_SIZE;++y) for (unsigned x=0;x<MAZE_SIZE;++x)
        cells[y][x]=(uint8_t)map[y][x];
    NfCompactResult result;
    uint32_t started=HAL_GetTick();
    NfRoutePlanStatus status=f405_orthogonal_plan(mode,case_index,g_angle_accum_mode,
        &cells[0][0],sizeof(cells),path,ROUTE_MAX_LEN,&result);
    uint32_t elapsed=HAL_GetTick()-started;
    if (status!=NF_ROUTE_PLAN_OK || result.path_length==0 || !mark_path(result.path_length)) {
        memset(path,0,sizeof(path));
        memset(path_cell,0,sizeof(path_cell));
        printf("[TimePath] rejected status=%u (4=no-path), mode=%u case=%u, compute_ms=%lu\n",
            (unsigned)status,(unsigned)mode,(unsigned)case_index,(unsigned long)elapsed);
        return false;
    }
    /* Display uses a top-left origin, while map/path_cell use bottom-left. */
    for (unsigned y=0;y<MAZE_SIZE;++y) for (unsigned x=0;x<MAZE_SIZE;++x)
        cells[y][x]=(uint8_t)((cells[y][x]>>4)|(cells[y][x]&15U));
    reverseArrayYAxis(cells);
    setMazeWalls(cells);
    correctWallInconsistencies();
    printf("[TimePath] orthogonal nominal: goal_us=%lu stop_us=%lu compute_ms=%lu states=%lu goal=(%u,%u) extension=%u\n",
        (unsigned long)result.goal_entry_us,(unsigned long)result.stop_us,
        (unsigned long)elapsed,(unsigned long)result.expanded_states,
        (unsigned)result.goal_x,(unsigned)result.goal_y,(unsigned)result.post_goal_cells);
    printMaze();
    printf("[TimePath] path:");
    for (size_t i=0;i<result.path_length;++i) printf(" %u",(unsigned)path[i]);
    printf(" 0\n");
    return true;
}
