/* Host adapter only. The included planner is unchanged from 4fb45ed.
 * Inclusion gives this adapter access to geometry replay without introducing
 * an exploration API or instrumentation into the firmware planner. */
#include NF_ORACLE_SLALOM_SOURCE
#include "slalom_time_plan_host.h"
#include "params.h"
#include "shortest_run_params.h"
#include <stdarg.h>

#ifndef NF_ORACLE_CLASSIC
#define NF_ORACLE_CLASSIC 0
#endif

typedef struct { char *data; size_t capacity; size_t length; bool failed; } Json;
static void emit(Json *j, const char *format, ...)
{
    if (j->failed) return;
    va_list args;
    va_start(args, format);
    int written = vsnprintf(j->data + j->length, j->capacity - j->length, format, args);
    va_end(args);
    if (written < 0 || (size_t)written >= j->capacity - j->length) j->failed = true;
    else j->length += (size_t)written;
}
static const ShortestRunModeParams_t *mode_params(unsigned mode)
{
    switch (mode) {
    case 2: return &shortestRunModeParams2;
    case 3: return &shortestRunModeParams3;
    case 4: return &shortestRunModeParams4;
    case 5: return &shortestRunModeParams5;
    default: return NULL;
    }
}
static const ShortestRunCaseParams_t *case_params(unsigned mode, unsigned index)
{
    if (index < 1 || index > 9) return NULL;
    switch (mode) {
    case 2: return &shortestRunCaseParamsMode2[index - 1];
    case 3: return &shortestRunCaseParamsMode3[index - 1];
    case 4: return &shortestRunCaseParamsMode4[index - 1];
    case 5: return &shortestRunCaseParamsMode5[index - 1];
    default: return NULL;
    }
}
static NfOrthogonalPlannerConfig orthogonal_config(unsigned mode, unsigned index)
{
    const ShortestRunModeParams_t *m = mode_params(mode);
    const ShortestRunCaseParams_t *c = case_params(mode, index);
    NfOrthogonalPlannerConfig cfg = {0};
    if (m == NULL || c == NULL) return cfg;
    cfg.half_cell_mm = DIST_HALF_SEC;
    cfg.start_offset_mm = DIST_FIRST_SEC;
    cfg.straight = (NfLinearLimits){c->velocity_straight, m->accel_switch_velocity,
        c->acceleration_straight, c->acceleration_straight_dash};
    cfg.turn_environment = (NfTurnEnvironment){NF_ORACLE_CLASSIC ? 0.0 : 2200.0,
        TURN_OMEGA_PROFILE_ROUNDING_SCALE};
    cfg.small_90 = (NfTurnSpec){true,m->velocity_turn90,m->alpha_turn90,90.0,
        m->dist_offset_in,m->dist_offset_out};
    cfg.large_90 = (NfTurnSpec){true,m->velocity_l_turn_90,m->alpha_l_turn_90,90.0,
        m->dist_l_turn_in_90,m->dist_l_turn_out_90};
    cfg.large_180 = (NfTurnSpec){true,m->velocity_l_turn_180,m->alpha_l_turn_180,180.0,
        m->dist_l_turn_in_180,m->dist_l_turn_out_180};
    cfg.allow_large_turns = true;
    return cfg;
}
static void block_edge(NfRouteMaze *maze, unsigned x, unsigned y, unsigned d, bool block)
{
    static const int dx[2] = {0,1}, dy[2] = {1,0};
    unsigned nx = x + dx[d], ny = y + dy[d];
    uint8_t a = k_wall_masks[d], b = k_wall_masks[d+2];
    if (block) { maze->walls[y][x] |= a; maze->walls[ny][nx] |= b; }
    else { maze->walls[y][x] &= (uint8_t)~a; maze->walls[ny][nx] &= (uint8_t)~b; }
}
/* Check the exact required-open topology and centre-line of one action,
 * reusing the prepared trajectory cache. No timing is recomputed. */
static bool slalom_action_open(NfSlalomContext *ctx, const NfSlalomAction *action)
{
    NfSlalomAnchor end = action->start_anchor;
    if (action->kind == NF_SLALOM_ACTION_START_OFFSET) return true;
    for (unsigned i=0; i<action->connector_steps; i++) {
        if (!nf_slalom_advance_connector(ctx->maze,end,action->start_heading,&end)) return false;
    }
    if (action->kind < NF_SLALOM_ACTION_START_OFFSET) {
        NfSlalomAnchor destination;
        NfSlalomHeading8 heading;
        NfTurnTrace trace;
        if (!nf_slalom_turn_destination(ctx->maze,end,action->start_heading,
              action->kind,action->side,&destination,&heading)) return false;
        if (nf_slalom_trace_turn(ctx,end,action->start_heading,action->kind,
              (NfSlalomTurnVariant)action->turn_speed_mode,action->side,
              destination,heading,&trace) != NF_SLALOM_PLAN_OK || !trace.feasible) return false;
    }
    return true;
}
static bool slalom_plan_open(NfSlalomContext *ctx, const NfSlalomRoutePlan *plan)
{
    for (size_t i=0;i<plan->action_count;i++)
        if (!slalom_action_open(ctx,&plan->actions[i])) return false;
    return true;
}
/* Determine a sufficient required-open edge set by testing individual walls,
 * then replay with every other edge blocked. This final replay protects the
 * certificate against alternative/OR guard paths. Fallback is all open edges
 * if the minimal single-edge dependency set is not sufficient. */
static void slalom_requirements(NfRouteMaze *maze, const NfSlalomPlannerConfig *cfg,
                                const NfSlalomRoutePlan *plan,
                                bool required[32][32][2])
{
    NfSlalomContext ctx = {0};
    NfRouteMaze original = *maze;
    ctx.maze = maze; ctx.config = cfg;
    if (nf_slalom_prepare_config(&ctx) != NF_SLALOM_PLAN_OK) {
        nf_slalom_free_turn_trajectories(&ctx); return;
    }
    for (unsigned y=0;y<maze->height;y++) for (unsigned x=0;x<maze->width;x++) {
        for (unsigned d=0;d<2;d++) {
            if ((d==0 && y+1>=maze->height) || (d==1 && x+1>=maze->width) ||
                (maze->walls[y][x]&k_wall_masks[d])) continue;
            block_edge(maze,x,y,d,true);
            required[y][x][d] = !slalom_plan_open(&ctx,plan);
            block_edge(maze,x,y,d,false);
        }
    }
    for (unsigned y=0;y<maze->height;y++) for (unsigned x=0;x<maze->width;x++)
        for (unsigned d=0;d<2;d++)
            if ((d==0 ? y+1<maze->height : x+1<maze->width) && !required[y][x][d])
                block_edge(maze,x,y,d,true);
    if (!slalom_plan_open(&ctx,plan)) {
        for (unsigned y=0;y<maze->height;y++) for (unsigned x=0;x<maze->width;x++)
            for (unsigned d=0;d<2;d++) required[y][x][d] =
                (d==0 ? y+1<maze->height : x+1<maze->width) &&
                !(original.walls[y][x]&k_wall_masks[d]);
    }
    *maze = original;
    nf_slalom_free_turn_trajectories(&ctx);
}
static void emit_edges(Json *out, const NfRouteMaze *maze, bool edges[32][32][2])
{
    bool comma=false;
    emit(out,",\"required_edges\":[");
    for(unsigned y=0;y<maze->height;y++) for(unsigned x=0;x<maze->width;x++)
        for(unsigned d=0;d<2;d++) if(edges[y][x][d]) {
            emit(out,"%s[%u,%u,%u]",comma?",":"",x,y,d); comma=true;
        }
    emit(out,"]");
}
static void emit_point(Json *out, double x, double y, bool *comma)
{
    emit(out,"%s[%.5f,%.5f]",*comma?",":"",x,y); *comma=true;
}
static void emit_slalom_points(Json *out, const NfSlalomPlannerConfig *cfg,
                               NfRouteMaze *maze, const NfSlalomRoutePlan *plan)
{
    NfSlalomContext ctx={0}; bool comma=false;
    ctx.config=cfg; ctx.maze=maze;
    if(nf_slalom_prepare_config(&ctx)!=NF_SLALOM_PLAN_OK) {
        nf_slalom_free_turn_trajectories(&ctx); emit(out,",\"route_points\":[]"); return;
    }
    emit(out,",\"route_points\":[");
    for(size_t i=0;i<plan->action_count;i++) {
        const NfSlalomAction *a=&plan->actions[i];
        emit_point(out,a->start_anchor.half_x*.5-.5,a->start_anchor.half_y*.5-.5,&comma);
        emit_point(out,a->connector_end_anchor.half_x*.5-.5,a->connector_end_anchor.half_y*.5-.5,&comma);
        if(a->kind<NF_SLALOM_ACTION_START_OFFSET) {
            const NfTurnTrajectoryCache *t=&ctx.turn_trajectories[a->turn_speed_mode][a->kind];
            double angle=nf_slalom_heading_math_deg(a->start_heading)*(NF_SLALOM_PI/180.0);
            double scale=2.0*cfg->half_cell_mm;
            for(unsigned n=1;n<=t->intervals;n+=8) {
                NfTurnPose p=t->poses[n];
                if(a->side==NF_ROUTE_SIDE_RIGHT) p.lateral_mm=-p.lateral_mm;
                double x=(p.forward_mm*cos(angle)-p.lateral_mm*sin(angle))/scale;
                double y=(p.forward_mm*sin(angle)+p.lateral_mm*cos(angle))/scale;
                emit_point(out,a->connector_end_anchor.half_x*.5-.5+x,
                    a->connector_end_anchor.half_y*.5-.5+y,&comma);
            }
            emit_point(out,a->end_anchor.half_x*.5-.5,a->end_anchor.half_y*.5-.5,&comma);
        }
    }
    emit(out,"]"); nf_slalom_free_turn_trajectories(&ctx);
}

int nf_exploration_oracle(unsigned width,unsigned height,const uint8_t *walls,
    const uint8_t *goals,unsigned start_x,unsigned start_y,unsigned start_heading,
    unsigned diagonal,unsigned mode,unsigned index,
    unsigned details,char *buffer,size_t capacity)
{
    Json out={buffer,capacity,0,false}; NfRouteMaze maze={0};
    if(width<1||width>32||height<1||height>32||start_x>=width||start_y>=height||start_heading>3||capacity<64) return -1;
    maze.width=(uint8_t)width; maze.height=(uint8_t)height;
    for(unsigned y=0;y<height;y++) for(unsigned x=0;x<width;x++) {
        for(unsigned d=0;d<4;d++) if(walls[y*width+x]&(1U<<d)) maze.walls[y][x]|=k_wall_masks[d];
        maze.goals[y][x]=goals[y*width+x]!=0;
    }
    bool edges[32][32][2]={0};
    if(diagonal && !NF_ORACLE_CLASSIC) {
        NfSlalomPlannerConfig cfg; const NfAuditProfile *profile=NULL; char error[160];
        char profile_name[40];
        snprintf(profile_name,sizeof(profile_name),mode==2?"f413-preorder-mode%u":"f405-mini-mode%u",mode);
        if(!nf_host_slalom_make_config(profile_name,(uint8_t)index,
            NF_SLALOM_ENABLE_SHORTEST_1_TO_5,&cfg,&profile,error,sizeof(error))) {
            emit(&out,"{\"status\":\"invalid_config\",\"error\":\"%s\"}",error); return out.failed?-1:0;
        }
        /* Higher modes retain F413 launch offset and angular-rate cap. */
        cfg.start_offset_mm=DIST_FIRST_SEC; cfg.turn_environment.omega_cap_deg_s=2200.0;
        NfSlalomPlannerRequest request={(uint8_t)start_x,(uint8_t)start_y,
            (NfSlalomHeading8)(2U*start_heading)};
        NfSlalomRoutePlan *plan=calloc(1,sizeof(*plan));
        if(!plan) return -1;
        if (!(details & 2U)) nf_oracle_cache_begin(&maze);
        NfSlalomPlanStatus status=nf_slalom_time_plan(&maze,&cfg,&request,plan);
        nf_oracle_cache_end();
        emit(&out,"{\"status\":\"%s\",\"model\":\"slalom-time\"",nf_slalom_plan_status_name(status));
        if(status==NF_SLALOM_PLAN_OK) {
            emit(&out,",\"goal_entry_s\":%.6f,\"stop_s\":%.6f,\"expanded_states\":%u,\"actions\":%zu",
                plan->goal_entry_us/1e6,plan->stop_us/1e6,plan->expanded_states,plan->action_count);
            if(details & 1U) {
                slalom_requirements(&maze,&cfg,plan,edges); emit_edges(&out,&maze,edges);
                emit_slalom_points(&out,&cfg,&maze,plan);
            }
        }
        emit(&out,"}"); free(plan);
    } else {
        NfOrthogonalPlannerConfig cfg=orthogonal_config(mode,index);
        NfOrthogonalPlannerRequest request={(uint8_t)start_x,(uint8_t)start_y,
            (NfRouteDirection)start_heading};
        NfOrthogonalRoutePlan *plan=calloc(1,sizeof(*plan));
        if(!plan) return -1;
        NfRoutePlanStatus status=nf_orthogonal_time_plan(&maze,&cfg,&request,plan);
        emit(&out,"{\"status\":\"%s\",\"model\":\"orthogonal-time\"",nf_route_plan_status_name(status));
        if(status==NF_ROUTE_PLAN_OK) {
            emit(&out,",\"goal_entry_s\":%.6f,\"stop_s\":%.6f,\"expanded_states\":%u,\"actions\":%zu",
                plan->goal_entry_us/1e6,plan->stop_us/1e6,plan->expanded_states,plan->action_count);
            if(details & 1U) {
                NfRouteValidation validation;
                for(unsigned y=0;y<height;y++) for(unsigned x=0;x<width;x++) for(unsigned d=0;d<2;d++) {
                    if((d==0?y+1>=height:x+1>=width)||(maze.walls[y][x]&k_wall_masks[d])) continue;
                    block_edge(&maze,x,y,d,true);
                    edges[y][x][d]=!nf_orthogonal_route_validate(&maze,&cfg,&request,plan,&validation);
                    block_edge(&maze,x,y,d,false);
                }
                emit_edges(&out,&maze,edges);
                emit(&out,",\"route_points\":["); bool comma=false;
                for(size_t i=0;i<plan->action_count;i++) {
                    NfRouteMotion *a=&plan->actions[i];
                    emit_point(&out,a->start_x,a->start_y,&comma);
                    /* Logical cell path for orthogonal model, not physical turn geometry. */
                    if(a->kind==NF_ROUTE_MOTION_LARGE_180) {
                        int side=a->side==NF_ROUTE_SIDE_RIGHT?1:-1;
                        unsigned heading=((int)a->start_heading+side+4)%4;
                        static const int dx[4]={0,1,0,-1},dy[4]={1,0,-1,0};
                        emit_point(&out,a->start_x+dx[heading],a->start_y+dy[heading],&comma);
                    }
                    emit_point(&out,a->end_x,a->end_y,&comma);
                }
                emit(&out,"]");
            }
        }
        emit(&out,"}"); free(plan);
    }
    return out.failed?-1:0;
}
