#include "Motion.h"
#include "Gimbal_Task.h"
#include "HMI.h"
Motion motion;
void Motion::Init()
{
}
uint8_t Motion::load_from_material(uint8_t _loadDir, uint8_t _is_rotate_out)
{
    uint8_t ret = 0;
    static enum {
        load_stop_state = 0,
        load_ready_state,
        load_calibrate_state1,
        load_calibrate_state2,
        load_huagui_down_state,
        load_jiazhua_close_state,
        load_huagui_up_state,
        load_yuntai_in_state,
        load_huagui_down2_state,
        load_jiazhua_open_state,
        load_huagui_up2_state,
        load_yuntai_out_state,

    } state = load_stop_state;
    if (state == load_stop_state) {
        gimbal.Lift_Move(gimbal.LIFT_DISTANCE_TOP);
        gimbal.Extension_Move(gimbal.EXTENSION_DISTANCE_OUT_MATERIAL_1);
        gimbal.Rotate_Move(gimbal.ROTATE_ANGLE_OUT_2);
        gimbal.Jaw_Move(gimbal.JAW_ANGLE_OPEN);
        state = load_ready_state;
    }
    else if (state == load_ready_state) {
        if (gimbal.All_Move_Finished()) {
            state = load_calibrate_state1;
            gimbal.EXTENSION_MOVE_VEL = 0.1f;
            gimbal.EXTENSION_MOVE_ACC = 0.5f;
            gimbal.Extension_Move(gimbal.EXTENSION_DISTANCE_OUT_MATERIAL_2);
        }
        if (hmi.camera_data_valid) {
            gimbal.EXTENSION_MOVE_VEL = 0.5f;
            gimbal.EXTENSION_MOVE_ACC = 1.5f;
            gimbal.Set_ControlMode(Gimbal::GIMBAL_CAMERA_CONTROL);
            state = load_calibrate_state2;
        }
    }
    else if (state == load_calibrate_state1) {
        if (gimbal.All_Move_Finished()) {
            gimbal.Extension_Move(gimbal.EXTENSION_DISTANCE_OUT_MATERIAL_1);
            state = load_ready_state;
        }
        if (hmi.camera_data_valid) {
            gimbal.EXTENSION_MOVE_VEL = 0.5f;
            gimbal.EXTENSION_MOVE_ACC = 1.5f;
            gimbal.Set_ControlMode(Gimbal::GIMBAL_CAMERA_CONTROL);
            state = load_calibrate_state2;
        }
    }
    else if (state == load_calibrate_state2) {
        if (gimbal.Camera_Calibrated()) {
            state = load_huagui_down_state;
            gimbal.Set_ControlMode(Gimbal::GIMBAL_POSITION_CONTROL);
            vTaskDelay(100);
            gimbal.Lift_Move(gimbal.LIFT_DISTANCE_ZHUANPAN);
        }
    }
    else if (state == load_huagui_down_state) {
        if (gimbal.All_Move_Finished()) {
            state = load_jiazhua_close_state;
            gimbal.Jaw_Move(gimbal.JAW_ANGLE_CLOSE);
        }
    }
    else if (state == load_jiazhua_close_state) {
        if (gimbal.All_Move_Finished()) {
            state = load_huagui_up_state;
            gimbal.Lift_Move(gimbal.LIFT_DISTANCE_TOP);
        }
    }
    else if (state == load_huagui_up_state) {
        if (gimbal.All_Move_Finished()) {
            if (_loadDir == 1) {
                gimbal.Rotate_Move(gimbal.ROTATE_ANGLE_IN_1);
                gimbal.Extension_Move(gimbal.EXTENSION_DISTANCE_IN_1);
            }
            else if (_loadDir == 2) {
                gimbal.Rotate_Move(gimbal.ROTATE_ANGLE_IN_2);
                gimbal.Extension_Move(gimbal.EXTENSION_DISTANCE_IN_2);
            }
            else if (_loadDir == 3) {
                gimbal.Rotate_Move(gimbal.ROTATE_ANGLE_IN_3);
                gimbal.Extension_Move(gimbal.EXTENSION_DISTANCE_IN_3);
            }
            state = load_yuntai_in_state;
        }
    }
    else if (state == load_yuntai_in_state) {
        if (gimbal.All_Move_Finished()) {
            state = load_huagui_down2_state;
            gimbal.Lift_Move(gimbal.LIFT_DISTANCE_ZAIWU_PUT);
        }
    }
    else if (state == load_huagui_down2_state) {
        if (gimbal.All_Move_Finished()) {
            state = load_jiazhua_open_state;
            gimbal.Jaw_Move(gimbal.JAW_ANGLE_OPEN);
        }
    }
    else if (state == load_jiazhua_open_state) {
        if (gimbal.All_Move_Finished()) {
            gimbal.Lift_Move(gimbal.LIFT_DISTANCE_TOP);
            state = load_huagui_up2_state;
        }
    }
    else if (state == load_huagui_up2_state) {
        if (gimbal.All_Move_Finished()) {
            if (_is_rotate_out) {
                gimbal.Rotate_Move(gimbal.ROTATE_ANGLE_OUT_2);
                gimbal.Extension_Move(gimbal.EXTENSION_DISTANCE_OUT_MATERIAL_1);
            }
            state = load_yuntai_out_state;
        }
    }
    else if (state == load_yuntai_out_state) {
        if (gimbal.All_Move_Finished()) {
            state = load_stop_state;
            ret = 1;
        }
    }
    return ret;
}

uint8_t Motion::load_from_ground(uint8_t _loadDir, uint8_t _unloadDir, uint8_t _is_rotate_out, uint8_t _rotate_out_dir, uint8_t _is_calibrate)
{
    uint8_t ret = 0;
    static enum {
        load_stop_state = 0,
        load_ready_state,
        load_calibrate_state,
        load_huagui_down_state,
        load_jiazhua_close_state,
        load_huagui_up_state,
        load_yuntai_in_state,
        load_huagui_down2_state,
        load_jiazhua_open_state,
        load_huagui_up2_state,
        load_yuntai_out_state,

    } state = load_stop_state;
    if (state == load_stop_state) {

        if (_unloadDir == 1) {
            gimbal.Rotate_Move(gimbal.ROTATE_ANGLE_OUT_1);
            gimbal.Extension_Move(gimbal.EXTENSION_DISTANCE_OUT_1);
        }
        else if (_unloadDir == 2) {
            gimbal.Rotate_Move(gimbal.ROTATE_ANGLE_OUT_2);
            gimbal.Extension_Move(gimbal.EXTENSION_DISTANCE_OUT_2);
        }
        else if (_unloadDir == 3) {
            gimbal.Rotate_Move(gimbal.ROTATE_ANGLE_OUT_3);
            gimbal.Extension_Move(gimbal.EXTENSION_DISTANCE_OUT_3);
        }

        gimbal.Lift_Move(gimbal.LIFT_DISTANCE_TOP);
        gimbal.Jaw_Move(gimbal.JAW_ANGLE_OPEN);
        state = load_ready_state;
    }
    else if (state == load_ready_state) {
        if (gimbal.All_Move_Finished()) {
            if (_is_calibrate) {
                gimbal.Set_ControlMode(Gimbal::GIMBAL_CAMERA_CONTROL);
                state = load_calibrate_state;
            }
            else {
                state = load_huagui_down_state;
                gimbal.Lift_Move(gimbal.LIFT_DISTANCE_GROUND);
            }
        }
    }
    else if (state == load_calibrate_state) {
        if (gimbal.Camera_Calibrated()) {
            state = load_huagui_down_state;
            gimbal.Set_ControlMode(Gimbal::GIMBAL_POSITION_CONTROL);
            vTaskDelay(100);
            gimbal.Lift_Move(gimbal.LIFT_DISTANCE_GROUND);
        }
    }
    else if (state == load_huagui_down_state) {
        if (gimbal.All_Move_Finished()) {
            state = load_jiazhua_close_state;
            gimbal.Jaw_Move(gimbal.JAW_ANGLE_CLOSE);
        }
    }
    else if (state == load_jiazhua_close_state) {
        if (gimbal.All_Move_Finished()) {
            state = load_huagui_up_state;
            gimbal.Lift_Move(gimbal.LIFT_DISTANCE_TOP);
        }
    }
    else if (state == load_huagui_up_state) {
        if (gimbal.All_Move_Finished()) {
            if (_loadDir == 1) {
                gimbal.Rotate_Move(gimbal.ROTATE_ANGLE_IN_1);
                gimbal.Extension_Move(gimbal.EXTENSION_DISTANCE_IN_1);
            }
            else if (_loadDir == 2) {
                gimbal.Rotate_Move(gimbal.ROTATE_ANGLE_IN_2);
                gimbal.Extension_Move(gimbal.EXTENSION_DISTANCE_IN_2);
            }
            else if (_loadDir == 3) {
                gimbal.Rotate_Move(gimbal.ROTATE_ANGLE_IN_3);
                gimbal.Extension_Move(gimbal.EXTENSION_DISTANCE_IN_3);
            }
            state = load_yuntai_in_state;
        }
    }
    else if (state == load_yuntai_in_state) {
        if (gimbal.All_Move_Finished()) {
            state = load_huagui_down2_state;
            gimbal.Lift_Move(gimbal.LIFT_DISTANCE_ZAIWU_PUT);
        }
    }
    else if (state == load_huagui_down2_state) {
        if (gimbal.All_Move_Finished()) {
            state = load_jiazhua_open_state;
            gimbal.Jaw_Move(gimbal.JAW_ANGLE_OPEN);
        }
    }
    else if (state == load_jiazhua_open_state) {
        if (gimbal.All_Move_Finished()) {
            gimbal.Lift_Move(gimbal.LIFT_DISTANCE_TOP);
            state = load_huagui_up2_state;
        }
    }
    else if (state == load_huagui_up2_state) {
        if (gimbal.All_Move_Finished()) {
            if (_is_rotate_out) {
                if (_rotate_out_dir == 1) {
                    gimbal.Rotate_Move(gimbal.ROTATE_ANGLE_OUT_1);
                    gimbal.Extension_Move(gimbal.EXTENSION_DISTANCE_OUT_1);
                }
                else if (_rotate_out_dir == 2) {
                    gimbal.Rotate_Move(gimbal.ROTATE_ANGLE_OUT_2);
                    gimbal.Extension_Move(gimbal.EXTENSION_DISTANCE_OUT_2);
                }
                else if (_rotate_out_dir == 3) {
                    gimbal.Rotate_Move(gimbal.ROTATE_ANGLE_OUT_3);
                    gimbal.Extension_Move(gimbal.EXTENSION_DISTANCE_OUT_3);
                }
            }
            state = load_yuntai_out_state;
        }
    }
    else if (state == load_yuntai_out_state) {
        if (gimbal.All_Move_Finished()) {
            state = load_stop_state;
            ret = 1;
        }
    }
    return ret;
}

uint8_t Motion::get_from_car(uint8_t _loadDir, uint8_t _unloadDir)
{
    uint8_t ret = 0;
    static enum {
        get_stop_state = 0,
        get_ready_state,
        get_yuntai_in_state,
        get_huagui_down_state,
        get_jiazhua_close_state,
        get_huagui_up_state,
        get_yuntai_out_state,
    } state = get_stop_state;
    if (state == get_stop_state) {

        gimbal.Lift_Move(gimbal.LIFT_DISTANCE_TOP);
        gimbal.Jaw_Move(gimbal.JAW_ANGLE_OPEN);
        state = get_ready_state;

    }
    else if (state == get_ready_state) {
        if (gimbal.All_Move_Finished()) {
            if (_loadDir == 1) {
                gimbal.Rotate_Move(gimbal.ROTATE_ANGLE_IN_1);
                gimbal.Extension_Move(gimbal.EXTENSION_DISTANCE_IN_1);
            }
            else if (_loadDir == 2) {
                gimbal.Rotate_Move(gimbal.ROTATE_ANGLE_IN_2);
                gimbal.Extension_Move(gimbal.EXTENSION_DISTANCE_IN_2);
            }
            else if (_loadDir == 3) {
                gimbal.Rotate_Move(gimbal.ROTATE_ANGLE_IN_3);
                gimbal.Extension_Move(gimbal.EXTENSION_DISTANCE_IN_3);
            }
            state = get_yuntai_in_state;
        }
    }
    else if (state == get_yuntai_in_state) {
        if (gimbal.All_Move_Finished()) {
            gimbal.Lift_Move(gimbal.LIFT_DISTANCE_ZAIWU_GET);
            state = get_huagui_down_state;
        }
    }
    else if (state == get_huagui_down_state) {
        if (gimbal.All_Move_Finished()) {
            gimbal.Jaw_Move(gimbal.JAW_ANGLE_CLOSE);
            state = get_jiazhua_close_state;
        }
    }
    else if (state == get_jiazhua_close_state) {
        if (gimbal.All_Move_Finished()) {
            gimbal.Lift_Move(gimbal.LIFT_DISTANCE_TOP);
            state = get_huagui_up_state;
        }
    }
    else if (state == get_huagui_up_state) {
        if (gimbal.All_Move_Finished()) {
            if (_unloadDir == 1) {
                gimbal.Rotate_Move(gimbal.ROTATE_ANGLE_OUT_1);
                gimbal.Extension_Move(gimbal.EXTENSION_DISTANCE_OUT_1);
            }
            else if (_unloadDir == 2) {
                gimbal.Rotate_Move(gimbal.ROTATE_ANGLE_OUT_2);
                gimbal.Extension_Move(gimbal.EXTENSION_DISTANCE_OUT_2);
            }
            else if (_unloadDir == 3) {
                gimbal.Rotate_Move(gimbal.ROTATE_ANGLE_OUT_3);
                gimbal.Extension_Move(gimbal.EXTENSION_DISTANCE_OUT_3);
            }
            state = get_yuntai_out_state;
        }
    }
    else if (state == get_yuntai_out_state) {
        if (gimbal.All_Move_Finished()) {
            state = get_stop_state;
            ret = 1;
        }
    }
    return ret;
}

uint8_t Motion::unload_to_ground(uint8_t _unloadDir, uint8_t _is_calibrate, uint8_t _is_rotate_in)
{
    uint8_t ret = 0;
    static enum {
        unload_stop_state = 0,
        unload_ready_state,
        unload_calibrate_state,
        unload_huagui_down_state,
        unload_jiazhua_open_state,
        unload_huagui_up_state,
        unload_yuntai_in_state,
    } state = unload_stop_state;
    if (state == unload_stop_state) {
        gimbal.Lift_Move(gimbal.LIFT_DISTANCE_TOP);
        if (_unloadDir == 1) {
            gimbal.Rotate_Move(gimbal.ROTATE_ANGLE_OUT_1);
            gimbal.Extension_Move(gimbal.EXTENSION_DISTANCE_OUT_1);
        }
        else if (_unloadDir == 2) {
            gimbal.Rotate_Move(gimbal.ROTATE_ANGLE_OUT_2);
            gimbal.Extension_Move(gimbal.EXTENSION_DISTANCE_OUT_2);
        }
        else if (_unloadDir == 3) {
            gimbal.Rotate_Move(gimbal.ROTATE_ANGLE_OUT_3);
            gimbal.Extension_Move(gimbal.EXTENSION_DISTANCE_OUT_3);
        }
        state = unload_ready_state;
    }
    else if (state == unload_ready_state) {
        if (gimbal.All_Move_Finished()) {
            if (_is_calibrate) {
                state = unload_calibrate_state;
                gimbal.Set_ControlMode(Gimbal::GIMBAL_CAMERA_CONTROL);
            }
            else {
                state = unload_huagui_down_state;
                gimbal.Lift_Move(gimbal.LIFT_DISTANCE_GROUND);
            }
        }
    }
    else if (state == unload_calibrate_state) {
        if (gimbal.Camera_Calibrated()) {
            printf("calibrate finished, angle:%f, distance:%f\r\n", gimbal.rotate_angle, gimbal.extension_distance);
            if (_unloadDir == 1) {
                gimbal.ROTATE_ANGLE_OUT_1 = gimbal.rotate_angle;
                gimbal.EXTENSION_DISTANCE_OUT_1 = gimbal.extension_distance;
            }
            else if (_unloadDir == 2) {
                gimbal.ROTATE_ANGLE_OUT_2 = gimbal.rotate_angle;
                gimbal.EXTENSION_DISTANCE_OUT_2 = gimbal.extension_distance;
            }
            else if (_unloadDir == 3) {
                gimbal.ROTATE_ANGLE_OUT_3 = gimbal.rotate_angle;
                gimbal.EXTENSION_DISTANCE_OUT_3 = gimbal.extension_distance;
            }
            gimbal.Set_ControlMode(Gimbal::GIMBAL_POSITION_CONTROL);
            vTaskDelay(100);
            gimbal.Lift_Move(gimbal.LIFT_DISTANCE_GROUND);
            state = unload_huagui_down_state;
        }
    }
    else if (state == unload_huagui_down_state) {
        if (gimbal.All_Move_Finished()) {
            state = unload_jiazhua_open_state;
            gimbal.Jaw_Move(gimbal.JAW_ANGLE_OPEN);
        }
    }
    else if (state == unload_jiazhua_open_state) {
        if (gimbal.All_Move_Finished()) {
            state = unload_huagui_up_state;
            gimbal.Lift_Move(gimbal.LIFT_DISTANCE_TOP);
        }
    }
    else if (state == unload_huagui_up_state) {
        if (gimbal.All_Move_Finished()) {
            if (_is_rotate_in) {
                state = unload_yuntai_in_state;
                gimbal.Rotate_Move(gimbal.ROTATE_ANGLE_IN_1);
            }
            else {
                ret = 1;
                state = unload_stop_state;
            }
        }
    }
    else if (state == unload_yuntai_in_state) {
        if (gimbal.All_Move_Finished()) {
            ret = 1;
            state = unload_stop_state;
        }
    }
    return ret;
}

uint8_t Motion::unload_to_second(uint8_t _unloadDir, uint8_t _is_calibrate)
{
    uint8_t ret = 0;
    static enum {
        unload_stop_state = 0,
        unload_ready_state,
        unload_calibrate_state,
        unload_huagui_down_state,
        unload_jiazhua_open_state,
        unload_huagui_up_state,
    } state = unload_stop_state;
    if (state == unload_stop_state) {
        gimbal.Lift_Move(gimbal.LIFT_DISTANCE_TOP);
        if (_unloadDir == 1) {
            gimbal.Rotate_Move(gimbal.ROTATE_ANGLE_OUT_1);
            gimbal.Extension_Move(gimbal.EXTENSION_DISTANCE_OUT_1);
        }
        else if (_unloadDir == 2) {
            gimbal.Rotate_Move(gimbal.ROTATE_ANGLE_OUT_2);
            gimbal.Extension_Move(gimbal.EXTENSION_DISTANCE_OUT_2);
        }
        else if (_unloadDir == 3) {
            gimbal.Rotate_Move(gimbal.ROTATE_ANGLE_OUT_3);
            gimbal.Extension_Move(gimbal.EXTENSION_DISTANCE_OUT_3);
        }
        state = unload_ready_state;
    }
    else if (state == unload_ready_state) {
        if (gimbal.All_Move_Finished()) {
            if (_is_calibrate) {
                state = unload_calibrate_state;
                gimbal.Set_ControlMode(Gimbal::GIMBAL_CAMERA_CONTROL);
            }
            else {
                state = unload_huagui_down_state;
                gimbal.Lift_Move(gimbal.LIFT_DISTANCE_SECOND);
            }
        }
    }
    else if (state == unload_calibrate_state) {
        if (gimbal.Camera_Calibrated()) {
            printf("calibrate finished, angle:%f, distance:%f\r\n", gimbal.rotate_angle, gimbal.extension_distance);
            gimbal.Set_ControlMode(Gimbal::GIMBAL_POSITION_CONTROL);
            vTaskDelay(100);
            gimbal.Lift_Move(gimbal.LIFT_DISTANCE_SECOND);
            state = unload_huagui_down_state;
        }
    }
    else if (state == unload_huagui_down_state) {
        if (gimbal.All_Move_Finished()) {
            state = unload_jiazhua_open_state;
            gimbal.Jaw_Move(gimbal.JAW_ANGLE_OPEN);
        }
    }
    else if (state == unload_jiazhua_open_state) {
        if (gimbal.All_Move_Finished()) {
            state = unload_huagui_up_state;
            gimbal.Lift_Move(gimbal.LIFT_DISTANCE_TOP);
        }
    }
    else if (state == unload_huagui_up_state) {
        if (gimbal.All_Move_Finished()) {
            ret = 1;
            state = unload_stop_state;
        }
    }
    return ret;
}


