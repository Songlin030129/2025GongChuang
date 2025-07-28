#include "Motion.h"
#include "Gimbal.h"
Motion motion;
void Motion::Init()
{
    // mot5.Init(&hcan1, 5, DIRECTION_NEGATIVE);
    // mot5.SetPosition_TRAP(0, 100, 100, 100, POSITION_MODE_ABSOLUT, MULTI_MODE_ASYNC);
}

uint8_t Motion::load_from_material(uint8_t _loadDir, uint8_t _last)
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
        gimbal.Lift_Move(Gimbal::LIFT_DISTANCE_TOP);
        gimbal.Extension_Move(Gimbal::EXTENSION_DISTANCE_OUT_2);
        gimbal.Rotate_Move(Gimbal::ROTATE_ANGLE_OUT_2);
        gimbal.Jaw_Move(Gimbal::JAW_ANGLE_OPEN);
        state = load_ready_state;
    }
    else if (state == load_ready_state) {
        if (gimbal.Rotate_Finished()) {
            gimbal.gimbal_control_mode = Gimbal::GIMBAL_CAMERA_CONTROL;
            state = load_calibrate_state;
        }
    }
    else if (state == load_calibrate_state) {
        if (gimbal.Camera_Calibrated()) {
            state = load_huagui_down_state;
            gimbal.gimbal_control_mode = Gimbal::GIMBAL_POSITION_CONTROL;
            gimbal.Lift_Move(Gimbal::LIFT_DISTANCE_ZHUANPAN);
        }
    }
    else if (state == load_huagui_down_state) {
        if (gimbal.Lift_Finished()) {
            state = load_jiazhua_close_state;
            gimbal.Jaw_Move(Gimbal::JAW_ANGLE_CLOSE);
        }
    }
    else if (state == load_jiazhua_close_state) {
        if (gimbal.Jaw_Finished()) {
            state = load_huagui_up_state;
            gimbal.Lift_Move(Gimbal::LIFT_DISTANCE_TOP);
        }
    }
    else if (state == load_huagui_up_state) {
        if (gimbal.Lift_Finished()) {
            if (_loadDir == 1) {
                gimbal.Rotate_Move(Gimbal::ROTATE_ANGLE_IN_1);
                gimbal.Extension_Move(Gimbal::EXTENSION_DISTANCE_IN_1);
            }
            else if (_loadDir == 2) {
                gimbal.Rotate_Move(Gimbal::ROTATE_ANGLE_IN_2);
                gimbal.Extension_Move(Gimbal::EXTENSION_DISTANCE_IN_2);
            }
            else if (_loadDir == 3) {
                gimbal.Rotate_Move(Gimbal::ROTATE_ANGLE_IN_3);
                gimbal.Extension_Move(Gimbal::EXTENSION_DISTANCE_IN_3);
            }
            state = load_yuntai_in_state;
        }
    }
    else if (state == load_yuntai_in_state) {
        if (gimbal.Rotate_Finished()) {
            state = load_huagui_down2_state;
            gimbal.Lift_Move(Gimbal::LIFT_DISTANCE_ZAIWU_PUT);
        }
    }
    else if (state == load_huagui_down2_state) {
        if (gimbal.Lift_Finished()) {
            state = load_jiazhua_open_state;
            gimbal.Jaw_Move(Gimbal::JAW_ANGLE_OPEN);
        }
    }
    else if (state == load_jiazhua_open_state) {
        if (gimbal.Jaw_Finished()) {
            gimbal.Lift_Move(Gimbal::LIFT_DISTANCE_TOP);
            state = load_huagui_up2_state;
        }
    }
    else if (state == load_huagui_up2_state) {
        if (gimbal.Lift_Finished()) {
            if (!_last) {
                gimbal.Rotate_Move(Gimbal::ROTATE_ANGLE_OUT_2);
                gimbal.Extension_Move(Gimbal::EXTENSION_DISTANCE_OUT_2);
            }
            state = load_yuntai_out_state;
        }
    }
    else if (state == load_yuntai_out_state) {
        if (gimbal.Rotate_Finished()) {
            state = load_stop_state;
            ret = 1;
        }
    }
    return ret;
}

uint8_t Motion::load_from_ground(uint8_t _loadDir, uint8_t _unloadDir, uint8_t _last)
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
            gimbal.Rotate_Move(Gimbal::ROTATE_ANGLE_OUT_1);
            gimbal.Extension_Move(Gimbal::EXTENSION_DISTANCE_OUT_1);
        }
        else if (_unloadDir == 2) {
            gimbal.Rotate_Move(Gimbal::ROTATE_ANGLE_OUT_2);
            gimbal.Extension_Move(Gimbal::EXTENSION_DISTANCE_OUT_2);
        }
        else if (_unloadDir == 3) {
            gimbal.Rotate_Move(Gimbal::ROTATE_ANGLE_OUT_3);
            gimbal.Extension_Move(Gimbal::EXTENSION_DISTANCE_OUT_3);
        }

        gimbal.Lift_Move(Gimbal::LIFT_DISTANCE_TOP);
        gimbal.Jaw_Move(Gimbal::JAW_ANGLE_OPEN);
        state = load_ready_state;
    }
    else if (state == load_ready_state) {
        if (gimbal.Rotate_Finished()) {
            gimbal.gimbal_control_mode = Gimbal::GIMBAL_CAMERA_CONTROL;
            state = load_calibrate_state;
        }
    }
    else if (state == load_calibrate_state) {
        if (gimbal.Camera_Calibrated()) {
            state = load_huagui_down_state;
            gimbal.gimbal_control_mode = Gimbal::GIMBAL_POSITION_CONTROL;
            gimbal.Lift_Move(Gimbal::LIFT_DISTANCE_GROUND);
        }
    }
    else if (state == load_huagui_down_state) {
        if (gimbal.Lift_Finished()) {
            state = load_jiazhua_close_state;
            gimbal.Jaw_Move(Gimbal::JAW_ANGLE_CLOSE);
        }
    }
    else if (state == load_jiazhua_close_state) {
        if (gimbal.Jaw_Finished()) {
            state = load_huagui_up_state;
            gimbal.Lift_Move(Gimbal::LIFT_DISTANCE_TOP);
        }
    }
    else if (state == load_huagui_up_state) {
        if (gimbal.Lift_Finished()) {
            if (_loadDir == 1) {
                gimbal.Rotate_Move(Gimbal::ROTATE_ANGLE_IN_1);
                gimbal.Extension_Move(Gimbal::EXTENSION_DISTANCE_IN_1);
            }
            else if (_loadDir == 2) {
                gimbal.Rotate_Move(Gimbal::ROTATE_ANGLE_IN_2);
                gimbal.Extension_Move(Gimbal::EXTENSION_DISTANCE_IN_2);
            }
            else if (_loadDir == 3) {
                gimbal.Rotate_Move(Gimbal::ROTATE_ANGLE_IN_3);
                gimbal.Extension_Move(Gimbal::EXTENSION_DISTANCE_IN_3);
            }
            state = load_yuntai_in_state;
        }
    }
    else if (state == load_yuntai_in_state) {
        if (gimbal.Rotate_Finished()) {
            state = load_huagui_down2_state;
            gimbal.Lift_Move(Gimbal::LIFT_DISTANCE_ZAIWU_PUT);
        }
    }
    else if (state == load_huagui_down2_state) {
        if (gimbal.Lift_Finished()) {
            state = load_jiazhua_open_state;
            gimbal.Jaw_Move(Gimbal::JAW_ANGLE_OPEN);
        }
    }
    else if (state == load_jiazhua_open_state) {
        if (gimbal.Jaw_Finished()) {
            gimbal.Lift_Move(Gimbal::LIFT_DISTANCE_TOP);
            state = load_huagui_up2_state;
        }
    }
    else if (state == load_huagui_up2_state) {
        if (gimbal.Lift_Finished()) {
            if (!_last) {
                gimbal.Rotate_Move(Gimbal::ROTATE_ANGLE_OUT_2);
                gimbal.Extension_Move(Gimbal::EXTENSION_DISTANCE_OUT_2);
            }
            state = load_yuntai_out_state;
        }
    }
    else if (state == load_yuntai_out_state) {
        if (gimbal.Rotate_Finished()) {
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
        if (ready_to_unload == 0) {
            gimbal.Lift_Move(Gimbal::LIFT_DISTANCE_TOP);
            gimbal.Jaw_Move(Gimbal::JAW_ANGLE_OPEN);
            state = get_ready_state;
        }
    }
    else if (state == get_ready_state) {
        if (gimbal.Lift_Finished() && gimbal.Jaw_Finished()) {
            if (_loadDir == 1) {
                gimbal.Rotate_Move(Gimbal::ROTATE_ANGLE_IN_1);
                gimbal.Extension_Move(Gimbal::EXTENSION_DISTANCE_IN_1);
            }
            else if (_loadDir == 2) {
                gimbal.Rotate_Move(Gimbal::ROTATE_ANGLE_IN_2);
                gimbal.Extension_Move(Gimbal::EXTENSION_DISTANCE_IN_2);
            }
            else if (_loadDir == 3) {
                gimbal.Rotate_Move(Gimbal::ROTATE_ANGLE_IN_3);
                gimbal.Extension_Move(Gimbal::EXTENSION_DISTANCE_IN_3);
            }
            state = get_yuntai_in_state;
        }
    }
    else if (state == get_yuntai_in_state) {
        if (gimbal.Rotate_Finished()) {
            gimbal.Lift_Move(Gimbal::LIFT_DISTANCE_ZAIWU_GET);
            state = get_huagui_down_state;
        }
    }
    else if (state == get_huagui_down_state) {
        if (gimbal.Lift_Finished()) {
            gimbal.Jaw_Move(Gimbal::JAW_ANGLE_CLOSE);
            state = get_jiazhua_close_state;
        }
    }
    else if (state == get_jiazhua_close_state) {
        if (gimbal.Jaw_Finished()) {
            gimbal.Lift_Move(Gimbal::LIFT_DISTANCE_TOP);
            state = get_huagui_up_state;
        }
    }
    else if (state == get_huagui_up_state) {
        if (gimbal.Lift_Finished()) {
            if (_unloadDir == 1) {
                gimbal.Rotate_Move(Gimbal::ROTATE_ANGLE_OUT_1);
                gimbal.Extension_Move(Gimbal::EXTENSION_DISTANCE_OUT_1);
            }
            else if (_unloadDir == 2) {
                gimbal.Rotate_Move(Gimbal::ROTATE_ANGLE_OUT_2);
                gimbal.Extension_Move(Gimbal::EXTENSION_DISTANCE_OUT_2);
            }
            else if (_unloadDir == 3) {
                gimbal.Rotate_Move(Gimbal::ROTATE_ANGLE_OUT_3);
                gimbal.Extension_Move(Gimbal::EXTENSION_DISTANCE_OUT_3);
            }
            state = get_yuntai_out_state;
        }
    }
    else if (state == get_yuntai_out_state) {
        if (gimbal.Rotate_Finished()) {
            ret = 1;
            ready_to_unload = 1;
            state = get_stop_state;
        }
    }
    return ret;
}

uint8_t Motion::unload_to_ground()
{
    uint8_t ret = 0;
    static enum {
        unload_stop_state,
        unload_ready_state,
        unload_calibrate_state,
        unload_huagui_down_state,
        unload_jiazhua_open_state,
        unload_huagui_up_state,
    } state = unload_stop_state;
    if (state == unload_stop_state) {
        if (ready_to_unload) {
            gimbal.Lift_Move(Gimbal::LIFT_DISTANCE_TOP);
            state = unload_ready_state;
        }
    }
    else if (state == unload_ready_state) {
        if (gimbal.Lift_Finished()) {
            state = unload_calibrate_state;
            gimbal.gimbal_control_mode = Gimbal::GIMBAL_CAMERA_CONTROL;
        }
    }
    else if (state == unload_calibrate_state) {
        if (gimbal.Camera_Calibrated()) {
            gimbal.gimbal_control_mode = Gimbal::GIMBAL_POSITION_CONTROL;
            gimbal.Lift_Move(Gimbal::LIFT_DISTANCE_GROUND);
            state = unload_huagui_down_state;
        }
    }
    else if (state == unload_huagui_down_state) {
        if (gimbal.Lift_Finished()) {
            state = unload_jiazhua_open_state;
            gimbal.Jaw_Move(Gimbal::JAW_ANGLE_OPEN);
        }
    }
    else if (state == unload_jiazhua_open_state) {
        if (gimbal.Jaw_Finished()) {
            state = unload_huagui_up_state;
            gimbal.Lift_Move(Gimbal::LIFT_DISTANCE_TOP);
        }
    }
    else if (state == unload_huagui_up_state) {
        if (gimbal.Lift_Finished()) {
            ready_to_unload = 0;
            ret = 1;
            state = unload_stop_state;
        }
    }
    return ret;
}

uint8_t Motion::unload_to_second()
{
    uint8_t ret = 0;
    static enum {
        unload_stop_state,
        unload_ready_state,
        unload_calibrate_state,
        unload_huagui_down_state,
        unload_jiazhua_open_state,
        unload_huagui_up_state,
    } state = unload_stop_state;
    if (state == unload_stop_state) {
        if (ready_to_unload) {
            gimbal.Lift_Move(Gimbal::LIFT_DISTANCE_TOP);
            state = unload_ready_state;
        }
    }
    else if (state == unload_ready_state) {
        if (gimbal.Lift_Finished()) {
            state = unload_calibrate_state;
            gimbal.gimbal_control_mode = Gimbal::GIMBAL_CAMERA_CONTROL;
        }
    }
    else if (state == unload_calibrate_state) {
        if (gimbal.Camera_Calibrated()) {
            gimbal.gimbal_control_mode = Gimbal::GIMBAL_POSITION_CONTROL;
            gimbal.Lift_Move(Gimbal::LIFT_DISTANCE_SECOND);
            state = unload_huagui_down_state;
        }
    }
    else if (state == unload_huagui_down_state) {
        if (gimbal.Lift_Finished()) {
            state = unload_jiazhua_open_state;
            gimbal.Jaw_Move(Gimbal::JAW_ANGLE_OPEN);
        }
    }
    else if (state == unload_jiazhua_open_state) {
        if (gimbal.Jaw_Finished()) {
            state = unload_huagui_up_state;
            gimbal.Lift_Move(Gimbal::LIFT_DISTANCE_TOP);
        }
    }
    else if (state == unload_huagui_up_state) {
        if (gimbal.Lift_Finished()) {
            ready_to_unload = 0;
            ret = 1;
            state = unload_stop_state;
        }
    }
    return ret;
}
