#include "compact_orthogonal.h"
#include "f405_orthogonal_preview.h"
#include "maze_ascii.h"
#include <assert.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static NfCompactWorkspace work;
static NfOrthogonalRoutePlan reference;
static unsigned checks, comparisons;
#define CHECK(x) do { ++checks; if (!(x)) { fprintf(stderr,"FAIL %s:%d: %s\n",__FILE__,__LINE__,#x); exit(1); } } while (0)
static const int dx[4]={0,1,0,-1},dy[4]={1,0,-1,0};

/* Independent integer replay of legacy half-cell ownership, first goal and
 * all crossed wall faces. No planner internals or parent encoding used. */
static bool step(const NfRouteMaze *m,int *x,int *y,unsigned d)
{
    if (!nf_route_maze_can_move(m,(uint8_t)*x,(uint8_t)*y,(NfRouteDirection)d)) return false;
    *x+=dx[d]; *y+=dy[d]; return true;
}
static bool replay(const NfRouteMaze *m,const NfOrthogonalPlannerRequest *q,
                   const uint16_t *path,const NfCompactResult *r)
{
    int x=q->start_x,y=q->start_y,pending=0;
    unsigned d=q->start_heading,previous_out=1;
    bool found=m->goals[y][x];
    int gx=found?x:-1,gy=found?y:-1;
    if (!r->path_length) return found && path[0]==0;
    for (size_t i=0;i<=r->path_length;++i) {
        unsigned code=path[i];
        if (code>200 && code<300) { pending+=(int)code-200; continue; }
        if (code!=0 && code!=300 && code!=400 && code!=501 && code!=502 && code!=601 && code!=602) return false;
        unsigned incoming=(code>=500);
        int cells_half=pending+(int)previous_out+(int)incoming;
        if (cells_half%2) return false;
        for (int j=0;j<cells_half/2;++j) {
            if (!step(m,&x,&y,d)) return false;
            if (m->goals[y][x]&&!found) { found=true;gx=x;gy=y; }
        }
        pending=0;
        if (!code) return i==r->path_length && found && gx==r->goal_x && gy==r->goal_y &&
            d==r->goal_heading && x==gx+dx[d]*r->post_goal_cells &&
            y==gy+dy[d]*r->post_goal_cells;
        if (found) return false; /* No further turns after the first goal. */
        bool left=code==400 || code>=600;
        d=(d+(left?3:1))%4;
        if (!step(m,&x,&y,d)) return false;
        if (m->goals[y][x]) { found=true;gx=x;gy=y; }
        if (code==502 || code==602) {
            if (found) return false;
            d=(d+(left?3:1))%4;
            if (!step(m,&x,&y,d)) return false;
            if (m->goals[y][x]) { found=true;gx=x;gy=y; }
        }
        previous_out=incoming;
    }
    return false;
}

static uint32_t seed=0x713090;
static uint32_t random32(void) { seed=seed*1664525U+1013904223U;return seed; }
static void fixture(NfRouteMaze *m,unsigned size,unsigned density)
{
    CHECK(nf_route_maze_init(m,(uint8_t)size,(uint8_t)size));
    CHECK(nf_route_maze_add_boundaries(m));
    for (unsigned y=0;y<size;++y) for (unsigned x=0;x<size;++x) {
        if (x+1<size && random32()%100<density) CHECK(nf_route_maze_set_wall(m,x,y,NF_ROUTE_DIR_EAST));
        if (y+1<size && random32()%100<density) CHECK(nf_route_maze_set_wall(m,x,y,NF_ROUTE_DIR_NORTH));
    }
    m->goals[size-1][size-1]=true;
}

static void compare(NfRouteMaze *m,NfOrthogonalPlannerConfig *c,NfOrthogonalPlannerRequest *q)
{
    uint16_t path[NF_COMPACT_PATH_CAPACITY],sentinel[NF_COMPACT_PATH_CAPACITY];
    memset(path,0xa5,sizeof(path));memcpy(sentinel,path,sizeof(path));
    NfCompactResult r;
    NfRoutePlanStatus compact=nf_compact_orthogonal_plan(m,c,q,&work,path,NF_COMPACT_PATH_CAPACITY,&r);
    NfRoutePlanStatus oracle=nf_orthogonal_time_plan(m,c,q,&reference);
    ++comparisons;
    if (compact!=oracle) fprintf(stderr,"status compact=%d reference=%d size=%u\n",compact,oracle,m->width);
    CHECK(compact==oracle);
    if (compact==NF_ROUTE_PLAN_OK) {
        if (r.goal_entry_us!=reference.goal_entry_us) fprintf(stderr,"time compact=%u ref=%llu\n",r.goal_entry_us,(unsigned long long)reference.goal_entry_us);
        CHECK(r.goal_entry_us==reference.goal_entry_us);
        CHECK(r.stop_us>=r.goal_entry_us);
        CHECK(r.expanded_states<=12U*m->width*m->height+1U);
        CHECK(replay(m,q,path,&r));
        NfRouteValidation validation;
        CHECK(nf_orthogonal_route_validate(m,c,q,&reference,&validation));
        uint16_t again[NF_COMPACT_PATH_CAPACITY]; NfCompactResult r2;
        CHECK(nf_compact_orthogonal_plan(m,c,q,&work,again,NF_COMPACT_PATH_CAPACITY,&r2)==compact);
        CHECK(r.goal_entry_us==r2.goal_entry_us && r.path_length==r2.path_length);
        CHECK(memcmp(path,again,(r.path_length+1U)*sizeof(*path))==0);
        if (r.path_length>0) {
            memcpy(again,sentinel,sizeof(again));NfCompactResult before;
            memset(&r2,0x5a,sizeof(r2));memcpy(&before,&r2,sizeof(before));
            CHECK(nf_compact_orthogonal_plan(m,c,q,&work,again,r.path_length,&r2)==NF_ROUTE_PLAN_CAPACITY);
            CHECK(memcmp(again,sentinel,sizeof(again))==0);
            CHECK(memcmp(&r2,&before,sizeof(r2))==0);
        }
    } else CHECK(memcmp(path,sentinel,sizeof(path))==0);
}

static void self_test(void)
{
    NfRouteMaze m;NfOrthogonalPlannerConfig c;
    NfOrthogonalPlannerRequest q={0,0,NF_ROUTE_DIR_NORTH};
    CHECK(sizeof(work)==36876);
    for (unsigned i=0;i<240;++i) {
        CHECK(f405_orthogonal_config((uint8_t)(2+i%6),3+i%7,&c));
        fixture(&m,3+i%6,(i%5)*15);
        c.allow_large_turns=(i%2)==0;
        if (i%7==0) m.goals[1][1]=true;
        q.start_heading=(NfRouteDirection)(i%4);
        compare(&m,&c,&q);
    }
    q.start_heading=NF_ROUTE_DIR_NORTH;
    fixture(&m,16,0); CHECK(f405_orthogonal_config(2,4,&c));compare(&m,&c,&q);
    m.goals[0][0]=true;compare(&m,&c,&q);m.goals[0][0]=false;
    uint16_t out[1024];NfCompactResult r;
    m.walls[0][0]&=(uint8_t)~1U;
    CHECK(nf_compact_orthogonal_plan(&m,&c,&q,&work,out,1024,&r)==NF_ROUTE_PLAN_INVALID_MAZE);
    m.walls[0][0]|=1U;m.walls[0][0]|=4U;
    CHECK(nf_compact_orthogonal_plan(&m,&c,&q,&work,out,1024,&r)==NF_ROUTE_PLAN_INVALID_MAZE);
    fixture(&m,16,0);m.width=32;
    CHECK(nf_compact_orthogonal_plan(&m,&c,&q,&work,out,1024,&r)==NF_ROUTE_PLAN_INVALID_ARGUMENT);
    m.width=16;c.half_cell_mm=NAN;
    CHECK(nf_compact_orthogonal_plan(&m,&c,&q,&work,out,1024,&r)==NF_ROUTE_PLAN_INVALID_CONFIG);
    CHECK(f405_orthogonal_config(2,4,&c));
    CHECK(nf_compact_orthogonal_plan(&m,&c,&q,&work,work.heap,1024,&r)==NF_ROUTE_PLAN_INVALID_ARGUMENT);
    c.small_90.alpha_deg_s2=1e-20;
    CHECK(nf_compact_orthogonal_plan(&m,&c,&q,&work,out,1024,&r)==NF_ROUTE_PLAN_OVERFLOW);
    uint8_t saved[256],before[256];
    memset(saved,0xf0,sizeof(saved));memcpy(before,saved,sizeof(saved));
    CHECK(f405_orthogonal_preview(2,4,saved,256,out,1024,&r)==NF_ROUTE_PLAN_NO_PATH);
    CHECK(memcmp(saved,before,sizeof(saved))==0);
    CHECK(f405_orthogonal_preview(2,4,saved,255,out,1024,&r)==NF_ROUTE_PLAN_INVALID_ARGUMENT);
    /* The adapter must reject aliasing before modifying caller-owned data. */
    memset(out,0xa5,sizeof(out));
    uint16_t untouched[1024];memcpy(untouched,out,sizeof(out));
    CHECK(f405_orthogonal_preview(2,4,(const uint8_t *)out,256,out,1024,&r)==NF_ROUTE_PLAN_INVALID_ARGUMENT);
    CHECK(memcmp(untouched,out,sizeof(out))==0);
    memset(&r,0x5a,sizeof(r));NfCompactResult previous;memcpy(&previous,&r,sizeof(r));
    CHECK(f405_orthogonal_preview(2,4,(const uint8_t *)&r,256,out,1024,&r)==NF_ROUTE_PLAN_INVALID_ARGUMENT);
    CHECK(memcmp(&previous,&r,sizeof(r))==0);
    CHECK(!f405_orthogonal_config(2,0,&c));CHECK(!f405_orthogonal_config(8,1,&c));
    fixture(&m,16,0);
    for (unsigned y=0;y<16;++y) for (unsigned x=0;x<16;++x)
        saved[y*16+x]=(uint8_t)(m.walls[y][x]*17U);
    saved[0]|=0x44;saved[1]|=0x11;memcpy(before,saved,sizeof(saved));
    for (unsigned mode=2;mode<=7;++mode) for (unsigned cs=1;cs<=9;++cs) {
        NfRoutePlanStatus s=f405_orthogonal_preview(mode,cs,saved,256,out,1024,&r);
        CHECK(s==NF_ROUTE_PLAN_OK);CHECK(r.path_length>0);CHECK(r.goal_entry_us>0);
        for (size_t j=0;j<r.path_length;++j) CHECK(out[j]<700);
        CHECK(memcmp(before,saved,sizeof(saved))==0);
    }
    /* Deliberate one-sided unknown edge must close both directions. */
    for (unsigned x=0;x<16;++x) saved[x]|=0x80;
    CHECK(f405_orthogonal_preview(2,4,saved,256,out,1024,&r)==NF_ROUTE_PLAN_NO_PATH);
    printf("PASS checks=%u reference_comparisons=%u workspace_bytes=%zu\n",checks,comparisons,sizeof(work));
}

static void matrix(int count,char **files)
{
    unsigned ok=0,no_path=0;
    for (int i=0;i<count;++i) {
        NfRouteMaze m;NfMazeAsciiInfo info;char error[200];
        CHECK(nf_maze_ascii_load(files[i],&m,&info,error,sizeof(error))==NF_MAZE_ASCII_OK);
        CHECK(m.width<=16 && m.height<=16);
        NfOrthogonalPlannerRequest q={info.start_x,info.start_y,NF_ROUTE_DIR_NORTH};
        for (unsigned mode=2;mode<=7;++mode) for (unsigned cs=1;cs<=9;++cs) {
            NfOrthogonalPlannerConfig c;CHECK(f405_orthogonal_config(mode,cs,&c));
            NfCompactResult r;uint16_t path[1024];
            NfRoutePlanStatus status=nf_compact_orthogonal_plan(&m,&c,&q,&work,path,1024,&r);
            CHECK(status==NF_ROUTE_PLAN_OK || status==NF_ROUTE_PLAN_NO_PATH);
            if (cs>=3) compare(&m,&c,&q);
            if (status==NF_ROUTE_PLAN_OK) { ++ok;CHECK(replay(&m,&q,path,&r)); }
            else ++no_path;
            printf("matrix\t%s\t%u\t%u\t%s\t%u\n",files[i],mode,cs,
                nf_route_plan_status_name(status),status==NF_ROUTE_PLAN_OK?r.goal_entry_us:0);
        }
    }
    printf("PASS matrix_mazes=%d configs=%u paths=%u no_path=%u reference_comparisons=%u checks=%u\n",
        count,ok+no_path,ok,no_path,comparisons,checks);
}

int main(int argc,char **argv)
{
    if (argc>=3 && strcmp(argv[1],"--matrix")==0) { matrix(argc-2,argv+2);return 0; }
    if (argc==2 && strcmp(argv[1],"--self-test")==0) { self_test();return 0; }
    if (argc!=4) { fprintf(stderr,"usage: %s --self-test | --matrix maze.maze [...] | maze.maze mode case\n",argv[0]);return 2; }
    unsigned mode,cs;char tail;
    if (sscanf(argv[2],"%u%c",&mode,&tail)!=1 || sscanf(argv[3],"%u%c",&cs,&tail)!=1 || mode<2 || mode>7 || cs<1 || cs>9) return 2;
    NfRouteMaze m;NfMazeAsciiInfo info;char error[200];
    if (nf_maze_ascii_load(argv[1],&m,&info,error,sizeof(error))!=NF_MAZE_ASCII_OK) {
        fprintf(stderr,"%s\n",error);return 2;
    }
    NfOrthogonalPlannerConfig c;CHECK(f405_orthogonal_config(mode,cs,&c));
    NfOrthogonalPlannerRequest q={info.start_x,info.start_y,NF_ROUTE_DIR_NORTH};
    NfCompactResult r;uint16_t path[1024];
    clock_t start=clock();
    NfRoutePlanStatus s=nf_compact_orthogonal_plan(&m,&c,&q,&work,path,1024,&r);
    printf("status=%s mode=%u case=%u workspace=%zu host_ms=%.3f model=nominal\n",
        nf_route_plan_status_name(s),mode,cs,sizeof(work),1000.0*(clock()-start)/CLOCKS_PER_SEC);
    if (s!=NF_ROUTE_PLAN_OK) return 1;
    CHECK(replay(&m,&q,path,&r));
    printf("goal=(%u,%u) entry_us=%u stop_us=%u post_goal_cells=%u expanded=%u edges=%u codes=%u\n",
        r.goal_x,r.goal_y,r.goal_entry_us,r.stop_us,r.post_goal_cells,r.expanded_states,r.relaxed_edges,r.path_length);
    for (size_t i=0;i<=r.path_length;++i) printf("%u%s",path[i],i==r.path_length?"\n":",");
    return 0;
}
