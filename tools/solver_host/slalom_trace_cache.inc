/* Host-only exact memoization, injected after the original trace function.
 * A cache lives for exactly one solve, on one calling thread, with fixed maze
 * and config. The key covers every varying trace argument; destination and
 * end heading are checked as well. Failed allocations fall back to stock C. */
typedef struct {
    NfTurnTrace trace;
    NfSlalomAnchor destination;
    NfSlalomHeading8 end_heading;
    NfSlalomPlanStatus status;
    bool valid;
} NfOracleTraceMemo;
static _Thread_local NfOracleTraceMemo *nf_oracle_trace_memo;
static _Thread_local size_t nf_oracle_trace_memo_count;
static void nf_oracle_cache_begin(const NfRouteMaze *maze)
{
    nf_oracle_trace_memo_count = nf_slalom_anchor_count(maze) * 8U *
        NF_SLALOM_TURN_COUNT * NF_SLALOM_TURN_VARIANT_COUNT * 2U;
    nf_oracle_trace_memo = calloc(nf_oracle_trace_memo_count,
                                  sizeof(*nf_oracle_trace_memo));
}
static void nf_oracle_cache_end(void)
{
    free(nf_oracle_trace_memo); nf_oracle_trace_memo = NULL;
    nf_oracle_trace_memo_count = 0;
}
static NfSlalomPlanStatus nf_oracle_cached_trace_turn(
    const NfSlalomContext *context, NfSlalomAnchor source,
    NfSlalomHeading8 heading, NfSlalomActionKind kind,
    NfSlalomTurnVariant variant, NfRouteSide side,
    NfSlalomAnchor destination, NfSlalomHeading8 end_heading, NfTurnTrace *out)
{
    size_t anchor_id;
    if (nf_oracle_trace_memo == NULL || out == NULL ||
        !nf_slalom_anchor_id(context->maze,source,&anchor_id) ||
        (unsigned)heading>=8U || (unsigned)kind>=NF_SLALOM_TURN_COUNT ||
        (unsigned)variant>=NF_SLALOM_TURN_VARIANT_COUNT ||
        (side!=NF_ROUTE_SIDE_RIGHT && side!=NF_ROUTE_SIDE_LEFT))
        return nf_slalom_trace_turn(context,source,heading,kind,variant,side,
                                   destination,end_heading,out);
    size_t key=((((anchor_id*8U+(unsigned)heading)*NF_SLALOM_TURN_COUNT+
        (unsigned)kind)*NF_SLALOM_TURN_VARIANT_COUNT+(unsigned)variant)*2U)+
        (side==NF_ROUTE_SIDE_LEFT?1U:0U);
    if (key>=nf_oracle_trace_memo_count)
        return nf_slalom_trace_turn(context,source,heading,kind,variant,side,
                                   destination,end_heading,out);
    NfOracleTraceMemo *entry=&nf_oracle_trace_memo[key];
    if(entry->valid && (entry->destination.half_x==destination.half_x && entry->destination.half_y==destination.half_y) &&
       entry->end_heading==end_heading) { *out=entry->trace; return entry->status; }
    entry->status=nf_slalom_trace_turn(context,source,heading,kind,variant,side,
                                     destination,end_heading,&entry->trace);
    entry->destination=destination; entry->end_heading=end_heading; entry->valid=true;
    *out=entry->trace; return entry->status;
}
#define nf_slalom_trace_turn nf_oracle_cached_trace_turn
