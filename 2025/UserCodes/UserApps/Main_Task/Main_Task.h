#ifndef __MAIN_TASK_H__
#define __MAIN_TASK_H__

#include "common_inc.h"
// 定义状态枚举
typedef enum {
    STATE_IDLE = 0,
    STATE_MOVE_TO_QRCODE,
    STATE_WAIT_QRCODE,
    STATE_MOVE_TO_MATERIAL,
    STATE_LOAD_MATERIAL,
    STATE_MOVE_TO_PROCESS,
    STATE_CALIBRATE_PROCESS,
    STATE_UNLOAD_PROCESS,
    STATE_LOAD_PROCESS,
    STATE_MOVE_TO_STORAGE,
    STATE_CALIBRATE_STORAGE,
    STATE_UNLOAD_STORAGE,
    STATE_MOVE_TO_STOP
} system_state_t;

void Main_Task();
// 状态处理函数声明
system_state_t state_idle();
system_state_t state_move_to_qrcode();
system_state_t state_wait_qrcode();
system_state_t state_move_to_material();
system_state_t state_load_material();
system_state_t state_move_to_process();
system_state_t state_calibrate_process();
system_state_t state_unload_process();
system_state_t state_load_process();
system_state_t state_move_to_storage();
system_state_t state_calibrate_storage();
system_state_t state_unload_storage();
system_state_t state_move_to_stop();


#endif