#ifndef __MAIN_TASK_H__
#define __MAIN_TASK_H__

#include "common_inc.h"
// 定义状态枚举
typedef enum {
    STATE_IDLE = 0,
    STATE_MOVE_TO_QRCODE = 1,
    STATE_WAIT_QRCODE = 2,
    STATE_MOVE_TO_MATERIAL = 3,
    STATE_LOAD_MATERIAL = 4,
    STATE_MOVE_TO_PROCESS = 5,
    STATE_UNLOAD_PROCESS = 6,
    STATE_LOAD_PROCESS = 7,
    STATE_MOVE_TO_STORAGE = 8,
    STATE_UNLOAD_STORAGE = 9,
    STATE_MOVE_TO_STOP = 10
} system_state_t;

void Main_Task();
// 状态处理函数声明
system_state_t state_idle();
system_state_t state_move_to_qrcode();
system_state_t state_wait_qrcode();
system_state_t state_move_to_material();
system_state_t state_load_material();
system_state_t state_move_to_process();
system_state_t state_unload_process();
system_state_t state_load_process();
system_state_t state_move_to_storage();
system_state_t state_unload_storage();
system_state_t state_move_to_stop();


#endif