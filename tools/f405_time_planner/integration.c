/* Exercise the real run_shortest -> solver -> planner -> run call chain.
 * Only hardware/drive primitives are replaced. This validates command
 * dispatch and abort behavior, not closed-loop dynamics or wall correction. */
#define MAIN_C_
#include "global.h"
#include "solver.h"
#include "maze_grid.h"
#include "f405_orthogonal_preview.h"
#include "shortest_run_params.h"
#include "maze_ascii.h"
#include <math.h>
#include <setjmp.h>
#include <string.h>

static unsigned checks, motor_starts, fans, firsts, stops, turns, buffers, loads;
static bool expect_halt, wall_hit;
static jmp_buf halt_env;
static uint16_t saved[16][16];
static double commanded_mm, commanded_seconds;
#define CHECK(x) do { ++checks; if (!(x)) { fprintf(stderr,"FAIL %s:%d: %s\n",__FILE__,__LINE__,#x); exit(1); } } while (0)

void HAL_Delay(uint32_t ms) { (void)ms; }
uint32_t HAL_GetTick(void) { return 0; }
void load_map_from_eeprom(void) { ++loads; memcpy(map,saved,sizeof(map)); }
void led_flash(uint8_t code) {
    if (code==5) { CHECK(expect_halt); longjmp(halt_env,1); }
}
void led_write(bool a,bool b,bool c) { (void)a;(void)b;(void)c; }
void led_wait(void) {}
void buzzer_beep(uint16_t a) { (void)a; }
void drive_variable_reset(void) { speed_now=0; }
void drive_reset_before_run(void) { speed_now=0; }
void drive_enable_motor(void) { ++motor_starts; }
void drive_disable_motor(void) {}
void drive_start(void) {}
void drive_stop(void) {}
void drive_fan(uint16_t duty) { if (duty) ++fans; }
void IMU_GetOffset(void) {}
uint8_t get_base(void) { return 0; }
void sensor_log_start(void) {}
void sensor_log_stop(void) {}
void turn_dir(uint8_t d) { (void)d; }

static void linear(float mm,float exit_speed) {
    CHECK(isfinite(mm) && mm>0);
    CHECK(isfinite(speed_now) && speed_now>=0);
    CHECK(isfinite(exit_speed) && exit_speed>=0);
    CHECK(speed_now+exit_speed>0);
    commanded_mm+=mm;
    commanded_seconds+=2.0*mm/(speed_now+exit_speed);
    speed_now=exit_speed;
}
void first_sectionA(void) {
    ++firsts;
    linear(DIST_FIRST_SEC,sqrtf(2*acceleration_straight*DIST_FIRST_SEC));
}
void half_sectionD(uint16_t v) { CHECK(v==0); ++stops; linear(DIST_HALF_SEC,0); }
void run_straight(float sections,float speed,float wallend) {
    CHECK(wallend==0); linear(sections*DIST_HALF_SEC,speed);
}
bool driveC_wallend(float distance,float speed) {
    ++buffers; speed_now=speed;
    linear(wall_hit?distance*0.5f:distance,speed);
    return wall_hit;
}
static void turn_command(float speed,float in,float out) {
    CHECK(isfinite(speed) && speed>0); CHECK(in>=0 && out>=0);
    ++turns;
    if (in>0) linear(in,speed);
    speed_now=speed;
    if (out>0) linear(out,speed);
}
void turn_R90(uint8_t f) { CHECK(f==1);turn_command(velocity_turn90,dist_offset_in,dist_offset_out); }
void turn_L90(uint8_t f) { turn_R90(f); }
void l_turn_R90(bool next) { (void)next;turn_command(velocity_l_turn_90,dist_l_turn_in_90,dist_l_turn_out_90); }
void l_turn_L90(bool next) { l_turn_R90(next); }
void l_turn_R180(bool next) { (void)next;turn_command(velocity_l_turn_180,dist_l_turn_in_180,dist_l_turn_out_180); }
void l_turn_L180(bool next) { l_turn_R180(next); }
#define DIAGONAL(name) void name(void) { CHECK(!"diagonal dispatch"); }
DIAGONAL(turn_R45_In) DIAGONAL(turn_R45_Out) DIAGONAL(turn_L45_In) DIAGONAL(turn_L45_Out)
DIAGONAL(turn_RV90) DIAGONAL(turn_LV90)
DIAGONAL(turn_R135_In) DIAGONAL(turn_R135_Out) DIAGONAL(turn_L135_In) DIAGONAL(turn_L135_Out)
void run_diagonal(float a,float b) { (void)a;(void)b;CHECK(!"diagonal straight"); }

static void reset_observations(void) {
    motor_starts=fans=firsts=stops=turns=buffers=loads=0;
    commanded_mm=commanded_seconds=0;
    MF.FLAGS=0;
    memset((void *)&g_debug,0,sizeof(g_debug));
}

static void open_maze(void) {
    for (unsigned y=0;y<16;++y) for (unsigned x=0;x<16;++x) {
        unsigned w=(x==0?1:0)|(y==0?2:0)|(x==15?4:0)|(y==15?8:0);
        saved[y][x]=(uint16_t)(0xA500U|w*17U);
    }
    saved[0][0]|=0x44; saved[0][1]|=0x11;
}

static void one_run(unsigned mode,unsigned cs,bool nominal) {
    reset_observations();
    g_angle_accum_mode=nominal;
    /* A real uint16_t map with nonzero upper bits catches accidental casts. */
    uint8_t bytes[256];
    for (unsigned i=0;i<256;++i) bytes[i]=(uint8_t)saved[i/16][i%16];
    uint16_t expected[1024]; NfCompactResult result;
    NfRoutePlanStatus status=f405_orthogonal_plan(mode,cs,nominal,bytes,256,expected,1024,&result);
    CHECK(status==NF_ROUTE_PLAN_OK || status==NF_ROUTE_PLAN_NO_PATH);
    expect_halt=status!=NF_ROUTE_PLAN_OK || result.path_length==0;
    if (setjmp(halt_env)==0) {
        run_shortest(mode,cs);
        CHECK(!expect_halt);
    }
    CHECK(loads==1);
    if (expect_halt) {
        CHECK(motor_starts==0 && fans==0 && firsts==0 && stops==0);
        CHECK(MF.FLAG.FAILED && !MF.FLAG.RUNNING);
        for (unsigned i=0;i<1024;++i) CHECK(path[i]==0);
    } else {
        const ShortestRunModeParams_t *modes[]={&shortestRunModeParams2,&shortestRunModeParams3,
            &shortestRunModeParams4,&shortestRunModeParams5,&shortestRunModeParams6,&shortestRunModeParams7};
        CHECK(motor_starts==1 && fans==(unsigned)(modes[mode-2]->fan_power>0) && firsts==1 && stops==1);
        CHECK(!MF.FLAG.FAILED && !MF.FLAG.RUNNING && speed_now==0);
        CHECK(commanded_mm>0 && isfinite(commanded_seconds));
        CHECK(memcmp(expected,path,(result.path_length+1)*sizeof(path[0]))==0);
        unsigned expected_turns=0;
        for (unsigned i=0;i<result.path_length;++i) if (path[i]>=300) ++expected_turns;
        CHECK(turns==expected_turns);
        CHECK(path_cell[START_Y][START_X]);
        CHECK(path_cell[result.goal_y][result.goal_x]);
        if (nominal) CHECK(angle_turn_90==90 && angle_l_turn_90==90 && angle_l_turn_180==180);
    }
    CHECK(memcmp(map,saved,sizeof(map))==0);
}

int main(int argc,char **argv) {
    open_maze();
    for (unsigned mode=2;mode<=7;++mode) for (unsigned cs=1;cs<=9;++cs) {
        wall_hit=false; one_run(mode,cs,mode!=2 || cs>2);
        wall_hit=true; one_run(mode,cs,mode!=2 || cs>2);
    }
    /* Actual flag controls angle choice even for nonstandard callers. */
    one_run(2,1,true); one_run(3,3,false);
    /* Failure after success cannot reuse the old path or start the machine. */
    for (unsigned y=0;y<16;++y) for (unsigned x=0;x<16;++x) saved[y][x]=0xffff;
    one_run(2,4,true);
    reset_observations();
    memset(path,0x55,sizeof(path));
    CHECK(!solver_build_path(9,1)); CHECK(path[0]==0);
    CHECK(motor_starts==0 && fans==0);
    for (int f=1;f<argc;++f) {
        NfRouteMaze m;NfMazeAsciiInfo info;char error[200];
        CHECK(nf_maze_ascii_load(argv[f],&m,&info,error,sizeof(error))==NF_MAZE_ASCII_OK);
        CHECK(m.width==16 && m.height==16);
        for (unsigned y=0;y<16;++y) for (unsigned x=0;x<16;++x)
            saved[y][x]=(uint16_t)(0xA500U|m.walls[y][x]*17U);
        for (unsigned mode=2;mode<=7;++mode) for (unsigned cs=1;cs<=9;++cs) {
            wall_hit=false; one_run(mode,cs,mode!=2 || cs>2);
        }
    }
    fprintf(stderr,"PASS production integration: checks=%u, historical_mazes=%d; hardware stubbed, no motion\n",checks,argc-1);
    return 0;
}
