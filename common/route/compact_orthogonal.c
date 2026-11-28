#include "compact_orthogonal.h"

#include <math.h>
#include <string.h>

#define FREE UINT16_MAX
#define DONE (UINT16_MAX - 1U)
#define INF UINT32_MAX
#define PREV_MASK 0xfffU
#define CELLS_SHIFT 12U
#define SIDE_SHIFT 17U

_Static_assert(NF_COMPACT_STATES <= PREV_MASK, "parent encoding");
_Static_assert(sizeof(NfCompactWorkspace) == 36876U, "workspace budget");

/* Class 0 exists only for the one start state. Other classes retain both
 * boundary speed and ownership of the outgoing half cell. Merging the three
 * turn classes would make Dijkstra incorrect when their velocities differ. */
typedef struct {
    uint16_t source, cells;
    uint8_t kind, side, x, y, heading, extension;
    uint32_t score, stop;
} Goal;

typedef struct {
    const NfRouteMaze *maze;
    const NfOrthogonalPlannerConfig *config;
    const NfOrthogonalPlannerRequest *request;
    NfCompactWorkspace *w;
    NfTurnPlan turns[3];
    uint32_t turn_us[3];
    /* Cost depends only on boundary class, next turn and connector length.
     * Cache on the foreground stack: 1536 bytes, at most 384 motion solves.
     * 0 = uncomputed, INF = infeasible, otherwise cost in us plus one. */
    uint32_t connectors[4][3][2U * NF_COMPACT_MAX_SIZE];
    double turn_cross_s[3];
    double start_speed;
    uint16_t start, heap_size;
    uint32_t expanded, relaxed;
    Goal goal;
} Context;

static const int dx[4] = {0, 1, 0, -1};
static const int dy[4] = {1, 0, -1, 0};
static const uint8_t wall[4] = {8, 4, 2, 1};

static bool inside(const NfRouteMaze *m, int x, int y)
{
    return x >= 0 && y >= 0 && x < m->width && y < m->height;
}

static bool move(const NfRouteMaze *m, int *x, int *y, unsigned d)
{
    int nx = *x + dx[d], ny = *y + dy[d];
    if (!inside(m, nx, ny) || (m->walls[*y][*x] & wall[d]) ||
        (m->walls[ny][nx] & wall[(d + 2U) % 4U])) return false;
    *x = nx; *y = ny;
    return true;
}

static bool valid_maze(const NfRouteMaze *m)
{
    bool goal = false;
    for (int y = 0; y < m->height; ++y) {
        for (int x = 0; x < m->width; ++x) {
            if (m->walls[y][x] & 0xf0U) return false;
            goal |= m->goals[y][x];
            for (unsigned d = 0; d < 4; ++d) {
                int nx = x + dx[d], ny = y + dy[d];
                bool present = (m->walls[y][x] & wall[d]) != 0;
                if (!inside(m, nx, ny)) { if (!present) return false; }
                else if (present != ((m->walls[ny][nx] & wall[(d+2)%4]) != 0))
                    return false;
            }
        }
    }
    return goal;
}

static uint16_t state(const Context *c, int x, int y, unsigned d, unsigned k)
{
    return (uint16_t)(((y * c->maze->width + x) * 4U + d) * 3U + k - 1U);
}

static void decode(const Context *c, uint16_t s, int *x, int *y,
                   unsigned *d, unsigned *k)
{
    if (s == c->start) {
        *x = c->request->start_x; *y = c->request->start_y;
        *d = c->request->start_heading; *k = 0; return;
    }
    *k = s % 3U + 1U; s /= 3U;
    *d = s % 4U; s /= 4U;
    *x = s % c->maze->width; *y = s / c->maze->width;
}

static const NfTurnSpec *turn(const Context *c, unsigned kind)
{
    return kind == 1 ? &c->config->small_90 :
           kind == 2 ? &c->config->large_90 : &c->config->large_180;
}

static unsigned out_half(unsigned k) { return k != 1U; }
static unsigned in_half(unsigned k) { return k > 1U; }
static double speed(const Context *c, unsigned k)
{
    return k == 0 ? c->start_speed : turn(c, k)->velocity_mm_s;
}

static bool time_us(double seconds, uint32_t *out)
{
    uint64_t n;
    if (nf_motion_seconds_to_us(seconds, &n) != NF_MOTION_OK || n >= INF - 1U)
        return false;
    *out = (uint32_t)n;
    return true;
}

static bool sum(uint32_t a, uint32_t b, uint32_t *out)
{
    if (a >= INF - b) return false;
    *out = a + b; return true;
}

static bool less(const Context *c, uint16_t a, uint16_t b)
{
    return c->w->distance_us[a] < c->w->distance_us[b] ||
        (c->w->distance_us[a] == c->w->distance_us[b] && a < b);
}

static void swap(Context *c, uint16_t a, uint16_t b)
{
    uint16_t t = c->w->heap[a]; c->w->heap[a] = c->w->heap[b]; c->w->heap[b] = t;
    c->w->position[c->w->heap[a]] = a; c->w->position[c->w->heap[b]] = b;
}

static void push(Context *c, uint16_t s)
{
    uint16_t p = c->w->position[s];
    if (p == FREE) { p = c->heap_size++; c->w->heap[p] = s; c->w->position[s] = p; }
    while (p > 0) {
        uint16_t parent = (uint16_t)((p - 1U) / 2U);
        if (!less(c, s, c->w->heap[parent])) break;
        swap(c, p, parent); p = parent;
    }
}

static uint16_t pop(Context *c)
{
    uint16_t s = c->w->heap[0];
    --c->heap_size;
    if (c->heap_size) {
        c->w->heap[0] = c->w->heap[c->heap_size];
        c->w->position[c->w->heap[0]] = 0;
        uint16_t p = 0;
        for (;;) {
            uint16_t child = (uint16_t)(2U*p + 1U);
            if (child >= c->heap_size) break;
            if (child + 1U < c->heap_size && less(c, c->w->heap[child+1], c->w->heap[child])) ++child;
            if (!less(c, c->w->heap[child], c->w->heap[p])) break;
            swap(c, p, child); p = child;
        }
    }
    c->w->position[s] = DONE;
    return s;
}

static uint8_t extensions(const Context *c, int x, int y, unsigned d)
{
    uint8_t n = 0;
    while (move(c->maze, &x, &y, d)) ++n;
    return n;
}

static void consider(Context *c, const Goal *g)
{
    if (g->score < c->goal.score ||
        (g->score == c->goal.score && g->stop < c->goal.stop)) c->goal = *g;
}

static NfRoutePlanStatus straight_goal(Context *c, uint16_t s, unsigned k,
                                       int x, int y, unsigned d, uint16_t cells)
{
    int half = 2*cells - (int)out_half(k);
    if (half < 0) return NF_ROUTE_PLAN_OK;
    Goal g = {.source=s, .cells=cells, .x=(uint8_t)x, .y=(uint8_t)y,
              .heading=(uint8_t)d, .extension=extensions(c,x,y,d)};
    NfGoalTerminalPlan p;
    if (nf_motion_goal_terminal_plan(&c->config->straight,
            half*c->config->half_cell_mm,
            (1+2*g.extension)*c->config->half_cell_mm, speed(c,k), &p) != NF_MOTION_OK)
        return NF_ROUTE_PLAN_OK;
    uint32_t cross, total;
    if (!time_us(p.goal_cross_time_s,&cross) || !time_us(p.full_plan.total_time_s,&total) ||
        !sum(c->w->distance_us[s],cross,&g.score) || !sum(c->w->distance_us[s],total,&g.stop))
        return NF_ROUTE_PLAN_OVERFLOW;
    consider(c,&g); return NF_ROUTE_PLAN_OK;
}

static NfRoutePlanStatus try_turn(Context *c, uint16_t s, unsigned previous,
                                  int x, int y, unsigned d, uint16_t cells,
                                  unsigned kind, unsigned side)
{
    int half = 2*cells - (int)out_half(previous) - (int)in_half(kind);
    if (half < 0 || (previous == 0 && kind > 1 && half == 0)) return NF_ROUTE_PLAN_OK;
    unsigned dest_dir = (d + (side ? 3U : 1U)) % 4U;
    int tx=x, ty=y;
    if (!move(c->maze,&tx,&ty,dest_dir)) return NF_ROUTE_PLAN_OK;
    if (kind == 3) {
        if (c->maze->goals[ty][tx]) return NF_ROUTE_PLAN_OK;
        dest_dir=(d+2U)%4U;
        if (!move(c->maze,&tx,&ty,dest_dir)) return NF_ROUTE_PLAN_OK;
    }
    const NfTurnSpec *t=turn(c,kind);
    if (!t->enabled) return NF_ROUTE_PLAN_OK;
    NfLinearPlan p;
    uint32_t *cached = &c->connectors[previous][kind-1][half];
    if (*cached == 0U) {
        if (nf_motion_linear_plan(&c->config->straight,half*c->config->half_cell_mm,
                                  speed(c,previous),t->velocity_mm_s,&p) != NF_MOTION_OK) {
            *cached = INF;
        } else {
            uint32_t cost;
            if (!time_us(p.total_time_s,&cost)) return NF_ROUTE_PLAN_OVERFLOW;
            *cached = cost + 1U;
        }
    }
    if (*cached == INF) return NF_ROUTE_PLAN_OK;
    uint32_t connector = *cached - 1U, edge, distance;
    if (!sum(connector,c->turn_us[kind-1],&edge) ||
        !sum(c->w->distance_us[s],edge,&distance)) return NF_ROUTE_PLAN_OVERFLOW;
    if (c->maze->goals[ty][tx]) {
        Goal g={.source=s,.cells=cells,.kind=(uint8_t)kind,.side=(uint8_t)side,
                .x=(uint8_t)tx,.y=(uint8_t)ty,.heading=(uint8_t)dest_dir,
                .extension=extensions(c,tx,ty,dest_dir)};
        if ((kind>1 && !g.extension) || !isfinite(c->turn_cross_s[kind-1])) return NF_ROUTE_PLAN_OK;
        double brake=(2*g.extension+(kind==1))*c->config->half_cell_mm;
        if (nf_motion_linear_plan(&c->config->straight,brake,t->velocity_mm_s,0,&p)!=NF_MOTION_OK)
            return NF_ROUTE_PLAN_OK;
        uint32_t cross, tail, prefix;
        if (!time_us(c->turn_cross_s[kind-1],&cross) || !time_us(p.total_time_s,&tail) ||
            !sum(c->w->distance_us[s],connector,&prefix) || !sum(prefix,cross,&g.score) ||
            !sum(distance,tail,&g.stop)) return NF_ROUTE_PLAN_OVERFLOW;
        consider(c,&g); return NF_ROUTE_PLAN_OK;
    }
    uint16_t dest=state(c,tx,ty,dest_dir,kind);
    if (c->w->position[dest]!=DONE && distance<c->w->distance_us[dest]) {
        c->w->distance_us[dest]=distance;
        c->w->parent[dest]=(uint32_t)s | ((uint32_t)cells<<CELLS_SHIFT) | (side<<SIDE_SHIFT);
        ++c->relaxed; push(c,dest);
    }
    return NF_ROUTE_PLAN_OK;
}

static NfRoutePlanStatus configure(Context *c, uint32_t *start_time)
{
    const NfOrthogonalPlannerConfig *f=c->config;
    if (!isfinite(f->half_cell_mm) || f->half_cell_mm<=0 ||
        !isfinite(f->start_offset_mm) || f->start_offset_mm<=0 || f->start_offset_mm>f->half_cell_mm)
        return NF_ROUTE_PLAN_INVALID_CONFIG;
    for (unsigned k=1;k<=(f->allow_large_turns?3U:1U);++k) {
        const NfTurnSpec *t=turn(c,k);
        if (!isfinite(t->angle_deg) || t->angle_deg<=0 || t->angle_deg>180 ||
            t->velocity_mm_s>f->straight.vmax_mm_s ||
            nf_motion_turn_plan(t,&f->turn_environment,&c->turns[k-1])!=NF_MOTION_OK)
            return NF_ROUTE_PLAN_INVALID_CONFIG;
        if (!time_us(c->turns[k-1].total_time_s,&c->turn_us[k-1])) return NF_ROUTE_PLAN_OVERFLOW;
        c->turn_cross_s[k-1]=c->turns[k-1].total_time_s;
        if (k>1) {
            double lateral;
            if (nf_motion_turn_exit_boundary_cross(t,&c->turns[k-1],f->half_cell_mm,
                    &c->turn_cross_s[k-1],&lateral)!=NF_MOTION_OK || fabs(lateral)>f->half_cell_mm+1e-3)
                c->turn_cross_s[k-1]=NAN;
        }
    }
    NfLinearPlan p;
    if (nf_motion_accelerating_exit_velocity(&f->straight,f->start_offset_mm,0,&c->start_speed)!=NF_MOTION_OK ||
        nf_motion_linear_plan(&f->straight,f->start_offset_mm,0,c->start_speed,&p)!=NF_MOTION_OK)
        return NF_ROUTE_PLAN_INVALID_CONFIG;
    return time_us(p.total_time_s,start_time)?NF_ROUTE_PLAN_OK:NF_ROUTE_PLAN_OVERFLOW;
}

static uint16_t code(unsigned k, unsigned side)
{
    return (uint16_t)(k==1?(side?400:300):(side?600:500)+(k==3?2:1));
}

/* Stage a REVERSED path in the now-idle heap. No second route-sized array.
 * The final stopping half section is owned by run(), never encoded twice. */
static bool emit(Context *c, size_t *n, size_t capacity, uint16_t value)
{
    if (!value) return true;
    if (*n+1>=capacity || *n>=NF_COMPACT_PATH_CAPACITY-1U) return false;
    c->w->heap[(*n)++]=value; return true;
}

static NfRoutePlanStatus reconstruct(Context *c,uint16_t *output,size_t capacity,NfCompactResult *result)
{
    size_t n=0;
    Goal *g=&c->goal;
    int x,y; unsigned d,k;
    decode(c,g->source,&x,&y,&d,&k);
    int terminal;
    if (g->kind==0) terminal=2*(g->cells+g->extension)-(int)out_half(k);
    else terminal=2*g->extension-(g->kind>1);
    if (terminal<0 || terminal>99) return NF_ROUTE_PLAN_CAPACITY;
    if (terminal && !emit(c,&n,capacity,(uint16_t)(200+terminal))) return NF_ROUTE_PLAN_CAPACITY;
    if (g->kind) {
        int h=2*g->cells-(int)out_half(k)-(int)in_half(g->kind);
        if (!emit(c,&n,capacity,code(g->kind,g->side)) ||
            (h && !emit(c,&n,capacity,(uint16_t)(200+h)))) return NF_ROUTE_PLAN_CAPACITY;
    }
    uint16_t current=g->source;
    for (size_t hops=0;current!=c->start;++hops) {
        if (hops>=c->start) return NF_ROUTE_PLAN_INVALID_ARGUMENT;
        uint32_t parent=c->w->parent[current];
        uint16_t previous=(uint16_t)(parent&PREV_MASK);
        unsigned child_kind;
        decode(c,current,&x,&y,&d,&child_kind);
        decode(c,previous,&x,&y,&d,&k);
        unsigned cells=(parent>>CELLS_SHIFT)&31U;
        int half=2*(int)cells-(int)out_half(k)-(int)in_half(child_kind);
        if (!emit(c,&n,capacity,code(child_kind,(parent>>SIDE_SHIFT)&1U)) ||
            (half && !emit(c,&n,capacity,(uint16_t)(200+half)))) return NF_ROUTE_PLAN_CAPACITY;
        current=previous;
    }
    for (size_t i=0;i<n;++i) output[i]=c->w->heap[n-1-i];
    output[n]=0;
    *result=(NfCompactResult){.goal_entry_us=g->score,.stop_us=g->stop,
        .expanded_states=c->expanded,.relaxed_edges=c->relaxed,.path_length=(uint16_t)n,
        .goal_x=g->x,.goal_y=g->y,.goal_heading=g->heading,.post_goal_cells=g->extension};
    return NF_ROUTE_PLAN_OK;
}

static bool overlaps(const void *a,size_t na,const void *b,size_t nb)
{
    uintptr_t x=(uintptr_t)a,y=(uintptr_t)b;
    return x<=y ? y-x<na : x-y<nb;
}

NfRoutePlanStatus nf_compact_orthogonal_plan(const NfRouteMaze *maze,
    const NfOrthogonalPlannerConfig *config,const NfOrthogonalPlannerRequest *request,
    NfCompactWorkspace *workspace,uint16_t *output,size_t capacity,NfCompactResult *result)
{
    if (!maze || !config || !request || !workspace || !output || !result ||
        capacity==0 || capacity>NF_COMPACT_PATH_CAPACITY ||
        maze->width==0 || maze->height==0 || maze->width>NF_COMPACT_MAX_SIZE || maze->height>NF_COMPACT_MAX_SIZE ||
        (unsigned)request->start_heading>=4 || !inside(maze,request->start_x,request->start_y))
        return NF_ROUTE_PLAN_INVALID_ARGUMENT;
    const void *ptrs[]={maze,config,request,workspace,output,result};
    size_t sizes[]={sizeof(*maze),sizeof(*config),sizeof(*request),sizeof(*workspace),capacity*sizeof(*output),sizeof(*result)};
    for (size_t i=0;i<6;++i) for (size_t j=i+1;j<6;++j)
        if (overlaps(ptrs[i],sizes[i],ptrs[j],sizes[j])) return NF_ROUTE_PLAN_INVALID_ARGUMENT;
    if (!valid_maze(maze)) return NF_ROUTE_PLAN_INVALID_MAZE;
    Context c={.maze=maze,.config=config,.request=request,.w=workspace,
        .start=(uint16_t)(maze->width*maze->height*12U),.goal={.score=INF}};
    uint32_t start_time;
    NfRoutePlanStatus status=configure(&c,&start_time);
    if (status!=NF_ROUTE_PLAN_OK) return status;
    if (maze->goals[request->start_y][request->start_x]) {
        output[0]=0; *result=(NfCompactResult){.goal_x=request->start_x,.goal_y=request->start_y,
                                            .goal_heading=(uint8_t)request->start_heading};
        return NF_ROUTE_PLAN_OK;
    }
    for (unsigned i=0;i<=c.start;++i) { workspace->distance_us[i]=INF; workspace->position[i]=FREE; }
    workspace->distance_us[c.start]=start_time; push(&c,c.start);
    while (c.heap_size) {
        uint16_t s=pop(&c);
        if (workspace->distance_us[s]>=c.goal.score) break;
        ++c.expanded;
        int x,y; unsigned d,k; decode(&c,s,&x,&y,&d,&k);
        for (uint16_t cells=0;;++cells) {
            for (unsigned kind=1;kind<=(config->allow_large_turns?3U:1U);++kind) {
                for (unsigned side=0;side<2;++side) {
                    status=try_turn(&c,s,k,x,y,d,cells,kind,side);
                    if (status!=NF_ROUTE_PLAN_OK) return status;
                }
            }
            if (!move(maze,&x,&y,d)) break;
            if (maze->goals[y][x]) {
                status=straight_goal(&c,s,k,x,y,d,(uint16_t)(cells+1U));
                if (status!=NF_ROUTE_PLAN_OK) return status;
                break;
            }
        }
    }
    return c.goal.score==INF?NF_ROUTE_PLAN_NO_PATH:reconstruct(&c,output,capacity,result);
}
