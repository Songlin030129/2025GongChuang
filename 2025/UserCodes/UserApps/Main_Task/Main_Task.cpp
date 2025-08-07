#include "Main_Task.h"
#include "LED.h"
#include "Paths.h"
#include "HMI.h"
#include "Motion.h"
#include "LCD.h"
#include "Gimbal_Task.h"
system_state_t global_state = STATE_IDLE;
system_state_t last_state;
float run_enable = 0;
uint8_t run_round = 1;
uint8_t colors[3] = { 0 };
uint32_t start_tick, stop_tick;
uint8_t put_calibrate = 1;
// 状态函数指针数组
typedef system_state_t(*state_func_t)();
state_func_t state_functions[] = {
    state_idle,
    state_move_to_qrcode,
    state_wait_qrcode,
    state_move_to_material,
    state_load_material,
    state_move_to_process,
    state_calibrate_process,
    state_unload_process,
    state_load_process,
    state_move_to_storage,
    state_calibrate_storage,
    state_unload_storage,
    state_move_to_stop
};

uint8_t finished = 0;
uint8_t jaw_finished = 0;
uint8_t lift_finished = 0;
uint8_t rotate_finished = 0;
uint8_t extension_finished = 0;
void Main_Task()
{
    hmi.Init(&huart1);
    lcd.Init(&huart5);
    vTaskDelay(100);

    while (1)
    {
        finished = gimbal.All_Move_Finished();
        jaw_finished = gimbal.Jaw_Finished();
        lift_finished = gimbal.Lift_Finished();
        rotate_finished = gimbal.Rotate_Finished();
        extension_finished = gimbal.Extension_Finished();
        system_state_t next_state = state_functions[global_state]();
        if (next_state != global_state) {
            last_state = global_state;
            global_state = next_state;
        }
        vTaskDelay(20);
    }
}

// 空闲状态
system_state_t state_idle() {
    if (run_enable >= 0.5f) {
        run_enable = 0;
        gimbal.Rotate_Move(gimbal.ROTATE_ANGLE_OUT_2);
        printf("START!!!\r\n");
        printf("moving to qrcode...\r\n");
        start_tick = HAL_GetTick();
        return STATE_MOVE_TO_QRCODE;
    }
    return STATE_IDLE;
}
//移动到二维码区
system_state_t state_move_to_qrcode() {
    uint8_t temp = paths.task_from_start_to_qrcode();
    if (temp == 1) {
        printf("waiting for qrcode...\r\n");
        hmi.Set_Detect_Mode(HMI::DETECT_MODE_QRCODE, 0);
        vTaskDelay(100);
        return STATE_WAIT_QRCODE;
    }
    return STATE_MOVE_TO_QRCODE;
}
//等待识别二维码
system_state_t state_wait_qrcode() {
    if (hmi.qrcode_detected == 1) {

        printf("moving to material...\r\n");

        colors[0] = hmi.first_round_color1;
        colors[1] = hmi.first_round_color2;
        colors[2] = hmi.first_round_color3;
        return STATE_MOVE_TO_MATERIAL;
    }
    return STATE_WAIT_QRCODE;
}
//移动到原料区
system_state_t state_move_to_material()
{
    static uint8_t temp_state = 0;
    if (temp_state == 0) {
        if (run_round == 1) {
            temp_state = 1;
        }
        else if (run_round == 2) {
            temp_state = 2;
        }
    }
    else if (temp_state == 1) {
        uint8_t temp = paths.task_from_qrcode_to_material();
        if (temp == 1) {
            printf("loading in material area...\r\n");
            temp_state = 0;
            return STATE_LOAD_MATERIAL;
        }
    }
    else if (temp_state == 2) {
        uint8_t temp = paths.task_from_storage_to_material();
        if (temp == 1) {
            printf("loading in material area...\r\n");
            temp_state = 0;
            return STATE_LOAD_MATERIAL;
        }
    }
    return STATE_MOVE_TO_MATERIAL;
}
//原料区装载物料
system_state_t state_load_material() {
    static uint8_t temp_state = 0;
    static uint8_t material_index = 1;
    if (temp_state == 0) {
        gimbal.CAMERA_CALIBRATE_X_THRESHOLD = 30.0f;
        gimbal.PID_CAM_X.P = 0.0005f;
        gimbal.PID_CAM_X.DeadZone = 40.0f;
        hmi.Set_Detect_Mode(HMI::DETECT_MODE_MATERIAL, colors[material_index - 1]);
        vTaskDelay(100);
        temp_state = 1;
    }
    else if (temp_state == 1) {
        uint8_t temp = 0;
        if (material_index != 3)
            temp = motion.load_from_material(material_index, 1);
        else if (material_index == 3)
            temp = motion.load_from_material(material_index, 0);
        if (temp == 1) {
            printf("finish load:%d...\r\n", colors[material_index - 1]);
            if (material_index < 3) {
                material_index++;
                temp_state = 0;
            }
            else {
                printf("finish all loads,moving to process...\r\n");
                gimbal.Rotate_Move(gimbal.ROTATE_ANGLE_IN_1);
                material_index = 1; // 重置
                temp_state = 0;
                gimbal.CAMERA_CALIBRATE_X_THRESHOLD = 15.0f;
                gimbal.PID_CAM_X.P = 0.0006f;
                gimbal.PID_CAM_X.DeadZone = 40.0f;
                return STATE_MOVE_TO_PROCESS;
            }
        }
    }
    return STATE_LOAD_MATERIAL;
}
//移动到加工区
system_state_t state_move_to_process() {
    static uint8_t temp_state = 0;
    if (temp_state == 0) {
        uint8_t temp = paths.task_from_material_to_process();
        if (temp == 1) {
            printf("get block:%d...\r\n", colors[0]);
            temp_state = 1;
        }
    }
    else if (temp_state == 1) {
        uint8_t temp = motion.get_from_car(1, 2, 1);
        if (temp == 1) {
            temp_state = 0;
            printf("calibrating in process area...\r\n");
            return STATE_CALIBRATE_PROCESS;
        }
    }
    return STATE_MOVE_TO_PROCESS;
}
//加工区校准底盘
system_state_t state_calibrate_process()
{
    static uint8_t temp_state = 0;
    if (temp_state == 0) {
        temp_state = 1;
        gimbal.Rotate_Move(gimbal.ROTATE_ANGLE_OUT_2);
        gimbal.Extension_Move(gimbal.EXTENSION_DISTANCE_OUT_2);
    }
    else if (temp_state == 1) {
        if (gimbal.All_Move_Finished()) {
            hmi.Set_Detect_Mode(HMI::DETECT_MODE_TARGET, 2);
            vTaskDelay(100);
            chassis.Set_ControlMode(Chassis::CHASSIS_CAMERA_CONTROL);
            temp_state = 2;
        }
    }
    else if (temp_state == 2) {
        if (chassis.Camera_Calibrated()) {
            chassis.Set_ControlMode(Chassis::CHASSIS_POSITION_CONTROL);
            printf("calibrate finished, unloading in process area...\r\n");
            temp_state = 0;
            return STATE_UNLOAD_PROCESS;
        }
    }
    return STATE_CALIBRATE_PROCESS;
}
//加工区卸货
system_state_t state_unload_process()
{
    static uint8_t temp_state = 0;
    static uint8_t process_index = 1;
    if (temp_state == 0) {
        hmi.Set_Detect_Mode(HMI::DETECT_MODE_TARGET, colors[process_index - 1]);
        vTaskDelay(100);
        temp_state = 1;
    }
    else if (temp_state == 1) {
        uint8_t temp = motion.unload_to_ground(colors[process_index - 1], put_calibrate);
        if (temp == 1) {
            printf("unload block:%d...\r\n", colors[process_index - 1]);
            if (process_index < 3) {
                temp_state = 2;
                process_index++;
            }
            else {
                temp_state = 0;
                process_index = 1;
                printf("finish all unload,start loading...\r\n");
                return STATE_LOAD_PROCESS;
            }
        }
    }
    else if (temp_state == 2) {
        uint8_t temp = motion.get_from_car(process_index, colors[process_index - 1], 1);
        if (temp == 1) {
            temp_state = 0;
        }
    }
    return STATE_UNLOAD_PROCESS;
}
//加工区装货
system_state_t state_load_process()
{
    static uint8_t temp_state = 0;
    static uint8_t process_index = 1;
    if (temp_state == 0) {
        hmi.Set_Detect_Mode(HMI::DETECT_MODE_BLOCK, colors[process_index - 1]);
        vTaskDelay(100);
        temp_state = 1;
    }
    else if (temp_state == 1) {
        uint8_t temp = 0;
        if (process_index < 3)
            temp = motion.load_from_ground(process_index, colors[process_index - 1], 1, colors[process_index], 0);
        else if (process_index == 3)
            temp = motion.load_from_ground(process_index, colors[process_index - 1], 0, 0, 0);
        if (temp == 1) {
            printf("load block:%d...\r\n", colors[process_index - 1]);
            if (process_index < 3) {
                process_index++;
                temp_state = 0;
            }
            else {
                temp_state = 0;
                process_index = 1;
                gimbal.Rotate_Move(gimbal.ROTATE_ANGLE_IN_1);
                gimbal.ROTATE_ANGLE_OUT_1 = Gimbal::DEFAULT_ROTATE_ANGLE_OUT_1;
                gimbal.EXTENSION_DISTANCE_OUT_1 = Gimbal::DEFAULT_EXTENSION_DISTANCE_OUT_1;
                gimbal.ROTATE_ANGLE_OUT_2 = Gimbal::DEFAULT_ROTATE_ANGLE_OUT_2;
                gimbal.EXTENSION_DISTANCE_OUT_2 = Gimbal::DEFAULT_EXTENSION_DISTANCE_OUT_2;
                gimbal.ROTATE_ANGLE_OUT_3 = Gimbal::DEFAULT_ROTATE_ANGLE_OUT_3;
                gimbal.EXTENSION_DISTANCE_OUT_3 = Gimbal::DEFAULT_EXTENSION_DISTANCE_OUT_3;
                printf("finished all loads,moving to storage area...\r\n");
                return STATE_MOVE_TO_STORAGE;
            }
        }
    }
    return STATE_LOAD_PROCESS;
}
//移动到暂存区
system_state_t state_move_to_storage()
{
    static uint8_t temp_state = 0;
    if (temp_state == 0) {
        uint8_t temp = paths.task_from_process_to_storage();
        if (temp == 1) {
            printf("get block:%d...\r\n", colors[0]);
            temp_state = 1;
        }
    }
    else if (temp_state == 1) {
        uint8_t temp = 0;
        if (run_round == 1)
            temp = motion.get_from_car(1, 2, 1);
        else if (run_round == 2)
            temp = motion.get_from_car(1, 2, 0);
        if (temp == 1) {
            temp_state = 0;
            printf("calibrating in storage area...\r\n");
            return STATE_CALIBRATE_STORAGE;
        }
    }
    return STATE_MOVE_TO_STORAGE;
}
//暂存区校准底盘
system_state_t state_calibrate_storage()
{
    static uint8_t temp_state = 0;
    if (temp_state == 0) {
        temp_state = 1;
        gimbal.Rotate_Move(gimbal.ROTATE_ANGLE_OUT_2);
        gimbal.Extension_Move(gimbal.EXTENSION_DISTANCE_OUT_2);
    }
    else if (temp_state == 1) {
        if (gimbal.All_Move_Finished()) {
            if (run_round == 1) {
                hmi.Set_Detect_Mode(HMI::DETECT_MODE_TARGET, 2);
                vTaskDelay(100);
                chassis.Set_ControlMode(Chassis::CHASSIS_CAMERA_CONTROL);
            }
            else if (run_round == 2) {
                hmi.Set_Detect_Mode(HMI::DETECT_MODE_BLOCK, 2);
                vTaskDelay(100);
                chassis.Set_ControlMode(Chassis::CHASSIS_CAMERA_CONTROL);
            }
            temp_state = 2;
        }
    }
    else if (temp_state == 2) {
        if (chassis.Camera_Calibrated()) {
            chassis.Set_ControlMode(Chassis::CHASSIS_POSITION_CONTROL);
            printf("calibrate finished, unloading in storage area...\r\n");
            temp_state = 0;
            return STATE_UNLOAD_STORAGE;
        }
    }
    return STATE_CALIBRATE_STORAGE;
}
//暂存区卸货
system_state_t state_unload_storage()
{
    static uint8_t temp_state = 0;
    static uint8_t storage_index = 1;
    if (temp_state == 0) {
        if (run_round == 1)
            hmi.Set_Detect_Mode(HMI::DETECT_MODE_TARGET, colors[storage_index - 1]);
        else if (run_round == 2)
            hmi.Set_Detect_Mode(HMI::DETECT_MODE_BLOCK, colors[storage_index - 1]);
        vTaskDelay(100);
        temp_state = 1;
    }
    else if (temp_state == 1) {
        uint8_t temp = 0;
        if (run_round == 1) {
            if (storage_index < 3)
                temp = motion.unload_to_ground(colors[storage_index - 1], put_calibrate);
            else
                temp = motion.unload_to_ground(colors[storage_index - 1], put_calibrate);
        }
        else if (run_round == 2)
            temp = motion.unload_to_second(colors[storage_index - 1], 0);
        if (temp == 1) {
            printf("unload block:%d...\r\n", colors[storage_index - 1]);
            if (storage_index < 3) {
                temp_state = 2;
                storage_index++;
            }
            else {
                gimbal.ROTATE_ANGLE_OUT_1 = Gimbal::DEFAULT_ROTATE_ANGLE_OUT_1;
                gimbal.EXTENSION_DISTANCE_OUT_1 = Gimbal::DEFAULT_EXTENSION_DISTANCE_OUT_1;
                gimbal.ROTATE_ANGLE_OUT_2 = Gimbal::DEFAULT_ROTATE_ANGLE_OUT_2;
                gimbal.EXTENSION_DISTANCE_OUT_2 = Gimbal::DEFAULT_EXTENSION_DISTANCE_OUT_2;
                gimbal.ROTATE_ANGLE_OUT_3 = Gimbal::DEFAULT_ROTATE_ANGLE_OUT_3;
                gimbal.EXTENSION_DISTANCE_OUT_3 = Gimbal::DEFAULT_EXTENSION_DISTANCE_OUT_3;
                temp_state = 0;
                storage_index = 1;
                if (run_round == 1) {
                    run_round = 2;
                    colors[0] = hmi.second_round_color1;
                    colors[1] = hmi.second_round_color2;
                    colors[2] = hmi.second_round_color3;
                    gimbal.Rotate_Move(gimbal.ROTATE_ANGLE_OUT_2);
                    gimbal.Extension_Move(gimbal.EXTENSION_DISTANCE_OUT_MATERIAL_1);
                    printf("finish round 1, moving to material area...\r\n");
                    return STATE_MOVE_TO_MATERIAL;
                }
                else if (run_round == 2) {
                    printf("finish round 2, moving to start area...\r\n");
                    gimbal.Extension_Move(gimbal.EXTENSION_DISTANCE_IN_2);
                    gimbal.Rotate_Move(gimbal.ROTATE_ANGLE_IN_2);
                    return STATE_MOVE_TO_STOP;
                }
            }
        }
    }
    else if (temp_state == 2) {
        uint8_t temp = 0;
        if (run_round == 1)
            temp = motion.get_from_car(storage_index, colors[storage_index - 1], 1);
        else if (run_round == 2)
            temp = motion.get_from_car(storage_index, colors[storage_index - 1], 0);
        if (temp == 1) {
            temp_state = 0;
        }
    }
    return STATE_UNLOAD_STORAGE;
}
//移动到停止区
system_state_t state_move_to_stop()
{
    uint8_t temp = paths.task_from_storage_to_stop();
    if (temp == 1) {
        printf("FINISHED!!!\r\n");
        stop_tick = HAL_GetTick();
        printf("Used Time:%f\r\n", (stop_tick - start_tick) / 1000.0f);
        return STATE_IDLE;
    }
    return STATE_MOVE_TO_STOP;
}
