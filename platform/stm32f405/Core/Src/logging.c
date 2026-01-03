#include "global.h"
#include "interrupt.h"
#include "logging.h"
#include "stdio.h"

// 現在のログプロファイル（取得内容の切替）
static volatile LogProfile s_log_profile = LOG_PROFILE_OMEGA;
static uint8_t s_wall_end_deriv_decim = 0;

static const char *s_mm_columns_velocity = "timestamp,velocity_interrupt,real_velocity,p_term_velocity,i_term_velocity,d_term_velocity,out_r,out_l";
static const char *s_mm_columns_distance = "timestamp,target_distance,real_distance,p_term_distance,i_term_distance,d_term_distance,out_r,out_l";
static const char *s_mm_columns_omega = "timestamp,omega_interrupt,real_omega,p_term_omega,i_term_omega,d_term_omega,out_r,out_l";
static const char *s_mm_columns_angle = "timestamp,target_angle,real_angle,p_term_angle,i_term_angle,d_term_angle,out_r,out_l";
static const char *s_mm_columns_wall_end_deriv = "timestamp,wall_end_deriv_r,wall_end_deriv_l,unused3,unused4,unused5,unused6,unused7";

static void log_print_mm_columns(const char *cols) {
    printf("#mm_columns=%s\n", cols);
}

void log_set_profile(LogProfile profile) { s_log_profile = profile; }

// 速度ログ（log_buffer）をCSV出力
void log_print_velocity_all(void) {
    printf("=== Micromouse Log Data (CSV Format, VELOCITY) ===\n");
    printf("Total entries: %d\n", log_buffer.count);
    printf("CSV Format: timestamp,param1,param2,param3,param4,param5,param6,param7\n");
    printf("--- CSV Data Start ---\n");
    log_print_mm_columns(s_mm_columns_velocity);

    uint16_t count = log_buffer.count > MAX_LOG_ENTRIES ? MAX_LOG_ENTRIES : log_buffer.count;
    uint16_t start = log_buffer.count > MAX_LOG_ENTRIES ? log_buffer.head : 0;

    for (uint16_t i = 0; i < count; i++) {
        uint16_t idx = (start + i) % MAX_LOG_ENTRIES;
        volatile LogEntry *entry = &log_buffer.entries[idx];
        printf("%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
               entry->timestamp - log_buffer.start_time,
               entry->target_omega,
               entry->actual_omega,
               entry->p_term_omega,
               entry->i_term_omega,
               entry->d_term_omega,
               entry->motor_out_r,
               entry->motor_out_l);
    }

    printf("--- CSV Data End ---\n");
    printf("=== End of Log ===\n");
}

// 距離ログ（log_buffer2）をCSV出力
void log_print_distance_all(void) {
    printf("=== Micromouse Log Data (CSV Format, DISTANCE) ===\n");
    printf("Total entries: %d\n", log_buffer2.count);
    printf("CSV Format: timestamp,param1,param2,param3,param4,param5,param6,param7\n");
    printf("--- CSV Data Start ---\n");
    log_print_mm_columns(s_mm_columns_distance);

    uint16_t count = log_buffer2.count > MAX_LOG_ENTRIES ? MAX_LOG_ENTRIES : log_buffer2.count;
    uint16_t start = log_buffer2.count > MAX_LOG_ENTRIES ? log_buffer2.head : 0;

    for (uint16_t i = 0; i < count; i++) {
        uint16_t idx = (start + i) % MAX_LOG_ENTRIES;
        volatile LogEntry *entry = &log_buffer2.entries[idx];
        printf("%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
               entry->timestamp - log_buffer2.start_time,
               entry->target_omega,
               entry->actual_omega,
               entry->p_term_omega,
               entry->i_term_omega,
               entry->d_term_omega,
               entry->motor_out_r,
               entry->motor_out_l);
    }

    printf("--- CSV Data End ---\n");
    printf("=== End of Log ===\n");
}

// 角速度ログ（log_buffer）をCSV出力
void log_print_omega_all(void) {
    printf("=== Micromouse Log Data (CSV Format, OMEGA) ===\n");
    printf("Total entries: %d\n", log_buffer.count);
    printf("CSV Format: timestamp,param1,param2,param3,param4,param5,param6,param7\n");
    printf("--- CSV Data Start ---\n");
    log_print_mm_columns(s_mm_columns_omega);

    uint16_t count = log_buffer.count > MAX_LOG_ENTRIES ? MAX_LOG_ENTRIES : log_buffer.count;
    uint16_t start = log_buffer.count > MAX_LOG_ENTRIES ? log_buffer.head : 0;

    for (uint16_t i = 0; i < count; i++) {
        uint16_t idx = (start + i) % MAX_LOG_ENTRIES;
        volatile LogEntry *entry = &log_buffer.entries[idx];
        printf("%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
               entry->timestamp - log_buffer.start_time,
               entry->target_omega,
               entry->actual_omega,
               entry->p_term_omega,
               entry->i_term_omega,
               entry->d_term_omega,
               entry->motor_out_r,
               entry->motor_out_l);
    }

    printf("--- CSV Data End ---\n");
    printf("=== End of Log ===\n");
}

// 角度ログ（log_buffer2）をCSV出力
void log_print_angle_all(void) {
    printf("=== Micromouse Log Data (CSV Format, ANGLE) ===\n");
    printf("Total entries: %d\n", log_buffer2.count);
    printf("CSV Format: timestamp,param1,param2,param3,param4,param5,param6,param7\n");
    printf("--- CSV Data Start ---\n");
    log_print_mm_columns(s_mm_columns_angle);

    uint16_t count = log_buffer2.count > MAX_LOG_ENTRIES ? MAX_LOG_ENTRIES : log_buffer2.count;
    uint16_t start = log_buffer2.count > MAX_LOG_ENTRIES ? log_buffer2.head : 0;

    for (uint16_t i = 0; i < count; i++) {
        uint16_t idx = (start + i) % MAX_LOG_ENTRIES;
        volatile LogEntry *entry = &log_buffer2.entries[idx];
        printf("%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
               entry->timestamp - log_buffer2.start_time,
               entry->target_omega,
               entry->actual_omega,
               entry->p_term_omega,
               entry->i_term_omega,
               entry->d_term_omega,
               entry->motor_out_r,
               entry->motor_out_l);
    }

    printf("--- CSV Data End ---\n");
    printf("=== End of Log ===\n");
}

void log_print_wall_end_deriv_all(void) {
    printf("=== Micromouse Log Data (CSV Format, WALL_END_DERIV) ===\n");
    printf("Total entries: %d\n", log_buffer2.count);
    printf("CSV Format: timestamp,param1,param2,param3,param4,param5,param6,param7\n");
    printf("--- CSV Data Start ---\n");
    log_print_mm_columns(s_mm_columns_wall_end_deriv);

    uint16_t count = log_buffer2.count > MAX_LOG_ENTRIES ? MAX_LOG_ENTRIES : log_buffer2.count;
    uint16_t start = log_buffer2.count > MAX_LOG_ENTRIES ? log_buffer2.head : 0;

    for (uint16_t i = 0; i < count; i++) {
        uint16_t idx = (start + i) % MAX_LOG_ENTRIES;
        volatile LogEntry *entry = &log_buffer2.entries[idx];
        printf("%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
               entry->timestamp - log_buffer2.start_time,
               entry->target_omega,
               entry->actual_omega,
               entry->p_term_omega,
               entry->i_term_omega,
               entry->d_term_omega,
               entry->motor_out_r,
               entry->motor_out_l);
    }

    printf("--- CSV Data End ---\n");
    printf("=== End of Log ===\n");
}

LogProfile log_get_profile(void) { return s_log_profile; }

/**
 * @brief ロギングシステムを初期化する
 */
void log_init(void) {
    // 速度/角速度 等（主）
    log_buffer.head = 0;
    log_buffer.count = 0;
    log_buffer.logging_active = 0;
    log_buffer.start_time = 0;

    // 距離/角度 等（副）
    log_buffer2.head = 0;
    log_buffer2.count = 0;
    log_buffer2.logging_active = 0;
    log_buffer2.start_time = 0;
}

/**
 * @brief 現在のプロファイルに基づいて1サンプル分のログを記録する（割り込みから呼ぶ）
 * @note 取得する変数は後で編集予定のため、現状は既存(角速度系)の内容を各プロファイルで共通に記録
 */
void log_capture_tick(void) {
    if (!log_buffer.logging_active) {
        return;
    }

    uint32_t current_time = HAL_GetTick();

    switch (s_log_profile) {

    case LOG_PROFILE_OMEGA:
        // 角速度（主バッファ）
        {
            const float kp_o = MF.FLAG.SUCTION ? KP_OMEGA_FAN_ON : KP_OMEGA_FAN_OFF;
            const float ki_o = MF.FLAG.SUCTION ? KI_OMEGA_FAN_ON : KI_OMEGA_FAN_OFF;
            const float kd_o = MF.FLAG.SUCTION ? KD_OMEGA_FAN_ON : KD_OMEGA_FAN_OFF;
            log_add_entry(
                (uint16_t)log_buffer.count,
                omega_interrupt,
                real_omega,
                kp_o * omega_error,
                ki_o * omega_integral,
                kd_o * omega_error_error,
                (float)out_r,
                (float)out_l,
                current_time
            );
        }
        // 角度（副バッファ）
        if (log_buffer2.logging_active && log_buffer2.count < MAX_LOG_ENTRIES) {
            uint16_t pos2 = log_buffer2.head;
            log_buffer2.entries[pos2].count = (uint16_t)log_buffer2.count;
            // フィールド名は共通だが、角度系を格納する
            log_buffer2.entries[pos2].target_omega = target_angle; // 目標角度
            log_buffer2.entries[pos2].actual_omega = real_angle;   // 実角度
            log_buffer2.entries[pos2].p_term_omega = KP_ANGLE * angle_error;
            log_buffer2.entries[pos2].i_term_omega = KI_ANGLE * angle_integral;
            log_buffer2.entries[pos2].d_term_omega = KD_ANGLE * angle_error_error;
            log_buffer2.entries[pos2].motor_out_r = (float)out_r;
            log_buffer2.entries[pos2].motor_out_l = (float)out_l;
            log_buffer2.entries[pos2].timestamp = current_time;
            log_buffer2.head = (pos2 + 1) % MAX_LOG_ENTRIES;
            log_buffer2.count++;
        }
        break;

    case LOG_PROFILE_VELOCITY:
        // 並進速度（主バッファ）
        log_add_entry(
            (uint16_t)log_buffer.count,
            velocity_interrupt,
            real_velocity,
            KP_VELOCITY * velocity_error,
            KI_VELOCITY * velocity_integral,
            KD_VELOCITY * velocity_error_error,
            (float)out_r,
            (float)out_l,
            current_time
        );

        // 並進距離（副バッファ）
        if (log_buffer2.logging_active && log_buffer2.count < MAX_LOG_ENTRIES) {
            uint16_t pos2 = log_buffer2.head;
            log_buffer2.entries[pos2].count = (uint16_t)log_buffer2.count;
            log_buffer2.entries[pos2].target_omega = target_distance;
            log_buffer2.entries[pos2].actual_omega = real_distance;
            log_buffer2.entries[pos2].p_term_omega = KP_DISTANCE * distance_error;
            log_buffer2.entries[pos2].i_term_omega = KI_DISTANCE * distance_integral;
            log_buffer2.entries[pos2].d_term_omega = KD_DISTANCE * distance_error_error;
            log_buffer2.entries[pos2].motor_out_r = (float)out_r;
            log_buffer2.entries[pos2].motor_out_l = (float)out_l;
            log_buffer2.entries[pos2].timestamp = current_time;
            log_buffer2.head = (pos2 + 1) % MAX_LOG_ENTRIES;
            log_buffer2.count++;
        }
        break;
    
    case LOG_PROFILE_DISTANCE:
        // 並進距離（副バッファ）
        if (log_buffer2.logging_active && log_buffer2.count < MAX_LOG_ENTRIES) {
            uint16_t pos2 = log_buffer2.head;
            log_buffer2.entries[pos2].count = (uint16_t)log_buffer2.count;
            log_buffer2.entries[pos2].target_omega = target_distance;
            log_buffer2.entries[pos2].actual_omega = real_distance;
            log_buffer2.entries[pos2].p_term_omega = KP_DISTANCE * distance_error;
            log_buffer2.entries[pos2].i_term_omega = KI_DISTANCE * distance_integral;
            log_buffer2.entries[pos2].d_term_omega = KD_DISTANCE * distance_error_error;
            log_buffer2.entries[pos2].motor_out_r = (float)out_r;
            log_buffer2.entries[pos2].motor_out_l = (float)out_l;
            log_buffer2.entries[pos2].timestamp = current_time;
            log_buffer2.head = (pos2 + 1) % MAX_LOG_ENTRIES;
            log_buffer2.count++;
        }
        // 並進速度（主バッファ）も併記しておく（閲覧側でVELOCITYを選ばれても空にならないように）
        log_add_entry(
            (uint16_t)log_buffer.count,
            velocity_interrupt,
            real_velocity,
            KP_VELOCITY * velocity_error,
            KI_VELOCITY * velocity_integral,
            KD_VELOCITY * velocity_error_error,
            (float)out_r,
            (float)out_l,
            current_time
        );
        break;
    case LOG_PROFILE_WALL_END_DERIV: {
        const bool is_straight = (!MF.FLAG.SLALOM_R && !MF.FLAG.SLALOM_L);
        if (!MF.FLAG.WALL_END || !is_straight) {
            break;
        }
        s_wall_end_deriv_decim++;
        if ((s_wall_end_deriv_decim & 0x03u) != 0u) {
            break;
        }

        if (log_buffer2.logging_active && log_buffer2.count < MAX_LOG_ENTRIES) {
            uint16_t pos2 = log_buffer2.head;
            log_buffer2.entries[pos2].count = (uint16_t)log_buffer2.count;
            log_buffer2.entries[pos2].target_omega = (float)wall_end_deriv_r;
            log_buffer2.entries[pos2].actual_omega = (float)wall_end_deriv_l;
            log_buffer2.entries[pos2].p_term_omega = 0.0f;
            log_buffer2.entries[pos2].i_term_omega = 0.0f;
            log_buffer2.entries[pos2].d_term_omega = 0.0f;
            log_buffer2.entries[pos2].motor_out_r = 0.0f;
            log_buffer2.entries[pos2].motor_out_l = 0.0f;
            log_buffer2.entries[pos2].timestamp = current_time;
            log_buffer2.head = (pos2 + 1) % MAX_LOG_ENTRIES;
            log_buffer2.count++;
        }
        break;
    }
    case LOG_PROFILE_CUSTOM:
    default:
        break;
    }
}

/**
 * @brief ロギングを開始する
 * @param start_time 開始時間（ms）
 * @note この関数はMF.FLAG.GET_LOG_1フラグも操作します
 */
void log_start(uint32_t start_time) {
    // 主ログ開始
    log_buffer.head = 0;
    log_buffer.count = 0;
    log_buffer.logging_active = 1;
    log_buffer.start_time = start_time;

    // 副ログ開始
    log_buffer2.head = 0;
    log_buffer2.count = 0;
    log_buffer2.logging_active = 1;
    log_buffer2.start_time = start_time;

    s_wall_end_deriv_decim = 0;

    // ロギングフラグを設定
    MF.FLAG.GET_LOG_1 = 1;
    MF.FLAG.GET_LOG_2 = 1;
}

/**
 * @brief ロギングを停止する
 * @note この関数はMF.FLAG.GET_LOG_1フラグも操作します
 */
void log_stop(void) {
    log_buffer.logging_active = 0;
    log_buffer2.logging_active = 0;

    // ロギングフラグをクリア
    MF.FLAG.GET_LOG_1 = 0;
    MF.FLAG.GET_LOG_2 = 0;
}

/**
 * @brief ログにデータを追加する（割り込みハンドラから呼ばれる）
 * @param index サンプルカウント/インデックス
 * @param target_omega 目標角速度
 * @param actual_omega 実際の角速度
 * @param p_term_omega 角速度制御のP項
 * @param i_term_omega 角速度制御のI項
 * @param d_term_omega 角速度制御のD項
 * @param motor_out_r 右モーター出力
 * @param motor_out_l 左モーター出力
 * @param timestamp タイムスタンプ（ms）
 */
void log_add_entry(uint16_t index, float target_omega, float actual_omega,
                  float p_term_omega, float i_term_omega, float d_term_omega,
                  float motor_out_r, float motor_out_l, uint32_t timestamp) {
    
    if (!log_buffer.logging_active || log_buffer.count >= MAX_LOG_ENTRIES) {
        return;
    }
    
    uint16_t position = log_buffer.head;
    
    // データをバッファに格納
    log_buffer.entries[position].count = index;
    log_buffer.entries[position].target_omega = target_omega;
    log_buffer.entries[position].actual_omega = actual_omega;
    log_buffer.entries[position].p_term_omega = p_term_omega;
    log_buffer.entries[position].i_term_omega = i_term_omega;
    log_buffer.entries[position].d_term_omega = d_term_omega;
    log_buffer.entries[position].motor_out_r = motor_out_r;
    log_buffer.entries[position].motor_out_l = motor_out_l;
    log_buffer.entries[position].timestamp = timestamp;
    
    // ヘッドとカウントを更新
    log_buffer.head = (position + 1) % MAX_LOG_ENTRIES;
    log_buffer.count++;
}

/**
 * @brief ログデータをシリアルポートに出力する（micromouse_log_visualizer用CSV形式）
 */
void log_print_all(void) {
    printf("=== Micromouse Log Data (CSV Format) ===\n");
    printf("Total entries: %d\n", log_buffer.count);
    printf("CSV Format: timestamp,target_omega,actual_omega,p_term_omega,i_term_omega,d_term_omega,motor_out_r,motor_out_l\n");
    printf("--- CSV Data Start ---\n");
    {
        const char *cols = s_mm_columns_omega;
        if (s_log_profile == LOG_PROFILE_VELOCITY || s_log_profile == LOG_PROFILE_DISTANCE) {
            cols = s_mm_columns_velocity;
        }
        log_print_mm_columns(cols);
    }
    
    // CSV形式でデータを出力（micromouse_log_visualizer用）
    // 形式: timestamp,target_omega,actual_omega,p_term_omega,i_term_omega,d_term_omega,motor_out_r,motor_out_l
    
    uint16_t count = log_buffer.count > MAX_LOG_ENTRIES ? MAX_LOG_ENTRIES : log_buffer.count;
    uint16_t start = log_buffer.count > MAX_LOG_ENTRIES ? log_buffer.head : 0;
    
    for (uint16_t i = 0; i < count; i++) {
        uint16_t idx = (start + i) % MAX_LOG_ENTRIES;
        volatile LogEntry *entry = &log_buffer.entries[idx];
        
        // CSV形式: timestamp,param1,param2,param3,param4,param5,param6,param7
        // timestamp = entry->timestamp - log_buffer.start_time (相対時間)
        // param1 = target_omega
        // param2 = actual_omega  
        // param3 = p_term_omega
        // param4 = i_term_omega
        // param5 = d_term_omega
        // param6 = motor_out_r
        // param7 = motor_out_l
        printf("%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
               entry->timestamp - log_buffer.start_time,
               entry->target_omega,
               entry->actual_omega,
               entry->p_term_omega,
               entry->i_term_omega,
               entry->d_term_omega,
               entry->motor_out_r,
               entry->motor_out_l);
    }
    
    printf("--- CSV Data End ---\n");
    printf("=== End of Log ===\n");
}

/**
 * @brief ログデータを純粋なCSV形式で出力する（可視化ツール貼り付け用）
 */
void log_print_csv_only(void) {
    // 純粋なCSVデータのみを出力（説明行なし）
    // 可視化ツールのテキストエリアに直接貼り付け可能

    {
        const char *cols = s_mm_columns_omega;
        if (s_log_profile == LOG_PROFILE_VELOCITY || s_log_profile == LOG_PROFILE_DISTANCE) {
            cols = s_mm_columns_velocity;
        }
        log_print_mm_columns(cols);
    }
    
    uint16_t count = log_buffer.count > MAX_LOG_ENTRIES ? MAX_LOG_ENTRIES : log_buffer.count;
    uint16_t start = log_buffer.count > MAX_LOG_ENTRIES ? log_buffer.head : 0;
    
    for (uint16_t i = 0; i < count; i++) {
        uint16_t idx = (start + i) % MAX_LOG_ENTRIES;
        volatile LogEntry *entry = &log_buffer.entries[idx];
        
        // CSV形式: timestamp,param1,param2,param3,param4,param5,param6,param7
        // timestampは相対時間（0始まり）
        printf("%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
               entry->timestamp - log_buffer.start_time,
               entry->target_omega,
               entry->actual_omega,
               entry->p_term_omega,
               entry->i_term_omega,
               entry->d_term_omega,
               entry->motor_out_r,
               entry->motor_out_l);
    }
}
