#include "Motion.h"
#include "Gimbal.h"
Motion motion;
void Motion::Init()
{
    // mot5.Init(&hcan1, 5, DIRECTION_NEGATIVE);
    // mot5.SetPosition_TRAP(0, 100, 100, 100, POSITION_MODE_ABSOLUT, MULTI_MODE_ASYNC);
}

uint8_t Motion::load_from_material(uint8_t _loadDir)
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
        Huagui_Move(HUAGUI_TOP_ANGLE, HUAGUI_ACC, HUAGUI_VELOCITY);
        Gimbal_Out();
        Jiazhua_Open();
        if (_loadDir == 1)
            Zaiwu_Front();
        if (_loadDir == 2)
            Zaiwu_Mid();
        if (_loadDir == 3)
            Zaiwu_Back();
        state = load_ready_state;
    } else if (state == load_ready_state) {
        if (Gimbal_Finished()) {
            gimbal.gimbal_control_mode = Gimbal::GIMBAL_CAMERA_CONTROL;
            state                      = load_calibrate_state;
        }
    } else if (state == load_calibrate_state) {
        if (Gimbal_Calibrated()) {
            state                      = load_huagui_down_state;
            gimbal.gimbal_control_mode = Gimbal::GIMBAL_POSITION_CONTROL;
            Huagui_Move(HUAGUI_ZHUANPAN_ANGLE, HUAGUI_ACC, HUAGUI_VELOCITY);
        }
    } else if (state == load_huagui_down_state) {
        if (Huagui_Finished()) {
            state = load_jiazhua_close_state;
            Jiazhua_Close();
        }
    } else if (state == load_jiazhua_close_state) {
        if (Jiazhua_Finished()) {
            state = load_huagui_up_state;
            Huagui_Move(HUAGUI_TOP_ANGLE, HUAGUI_ACC, HUAGUI_VELOCITY);
        }
    } else if (state == load_huagui_up_state) {
        if (Huagui_Finished()) {
            Gimbal_In();
            state = load_yuntai_in_state;
        }
    } else if (state == load_yuntai_in_state) {
        if (Gimbal_Finished()) {
            state = load_huagui_down2_state;
            Huagui_Move(HUAGUI_ZAIWU_PUT_ANGLE, HUAGUI_ACC, HUAGUI_VELOCITY);
        }
    } else if (state == load_huagui_down2_state) {
        if (Huagui_Finished()) {
            state = load_jiazhua_open_state;
            Jiazhua_Open();
        }
    } else if (state == load_jiazhua_open_state) {
        if (Jiazhua_Finished()) {
            Huagui_Move(HUAGUI_TOP_ANGLE, HUAGUI_ACC, HUAGUI_VELOCITY);
            state = load_huagui_up2_state;
        }
    } else if (state == load_huagui_up2_state) {
        if (Huagui_Finished()) {
            Gimbal_Out();
            state = load_yuntai_out_state;
        }
    } else if (state == load_yuntai_out_state) {
        if (Gimbal_Finished()) {
            state = load_stop_state;
            ret   = 1;
        }
    }
    return ret;
}

uint8_t Motion::load_from_ground(uint8_t _loadDir)
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
        Huagui_Move(HUAGUI_TOP_ANGLE, HUAGUI_ACC, HUAGUI_VELOCITY);
        Gimbal_Out();
        Jiazhua_Open();
        state = load_ready_state;
    } else if (state == load_ready_state) {
        if (Gimbal_Finished()) {
            state                      = load_calibrate_state;
            gimbal.gimbal_control_mode = Gimbal::GIMBAL_CAMERA_CONTROL;
        }
    } else if (state == load_calibrate_state) {
        if (Gimbal_Finished()) {
            state = load_huagui_down_state;
            Huagui_Move(HUAGUI_GROUND_ANGLE, HUAGUI_ACC, HUAGUI_VELOCITY);
            gimbal.gimbal_control_mode = Gimbal::GIMBAL_POSITION_CONTROL;
        }
    } else if (state == load_huagui_down_state) {
        if (Huagui_Finished()) {
            state = load_jiazhua_close_state;
            Jiazhua_Close();
        }
    } else if (state == load_jiazhua_close_state) {
        if (Jiazhua_Finished()) {
            state = load_huagui_up_state;
            Huagui_Move(HUAGUI_TOP_ANGLE, HUAGUI_ACC, HUAGUI_VELOCITY);
        }
    } else if (state == load_huagui_up_state) {
        if (Huagui_Finished()) {
            Gimbal_In();
            state = load_yuntai_in_state;
        }
    } else if (state == load_yuntai_in_state) {
        if (Gimbal_Finished()) {
            state = load_huagui_down2_state;
            Huagui_Move(HUAGUI_ZAIWU_PUT_ANGLE, HUAGUI_ACC, HUAGUI_VELOCITY);
        }
    } else if (state == load_huagui_down2_state) {
        if (Huagui_Finished()) {
            state = load_jiazhua_open_state;
            Jiazhua_Open();
        }
    } else if (state == load_jiazhua_open_state) {
        if (Jiazhua_Finished()) {
            Huagui_Move(HUAGUI_TOP_ANGLE, HUAGUI_ACC, HUAGUI_VELOCITY);
            state = load_huagui_up2_state;
        }
    } else if (state == load_huagui_up2_state) {
        if (Huagui_Finished()) {
            Gimbal_Out();
            Zaiwu_Mid();
            state = load_yuntai_out_state;
        }
    } else if (state == load_yuntai_out_state) {
        if (Gimbal_Finished()) {
            Zaiwu_Mid();
            state = load_stop_state;
            ret   = 1;
        }
    }
    return ret;
}

uint8_t Motion::get_from_car(uint8_t _loadDir)
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
            Huagui_Move(HUAGUI_TOP_ANGLE, HUAGUI_ACC, HUAGUI_VELOCITY);
            Jiazhua_Open();
            state = get_ready_state;
        }
    } else if (state == get_ready_state) {
        if (Huagui_Finished() && Jiazhua_Finished()) {
            Gimbal_In();
            state = get_yuntai_in_state;
        }
    } else if (state == get_yuntai_in_state) {
        if (Gimbal_Finished() && Zaiwu_Finished()) {
            Huagui_Move(HUAGUI_ZAIWU_GET_ANGLE, HUAGUI_ACC, HUAGUI_VELOCITY);
            state = get_huagui_down_state;
        }
    } else if (state == get_huagui_down_state) {
        if (Huagui_Finished()) {
            Jiazhua_Close();
            state = get_jiazhua_close_state;
        }
    } else if (state == get_jiazhua_close_state) {
        if (Jiazhua_Finished()) {
            Huagui_Move(HUAGUI_TOP_ANGLE, HUAGUI_ACC, HUAGUI_VELOCITY);
            state = get_huagui_up_state;
        }
    } else if (state == get_huagui_up_state) {
        if (Huagui_Finished()) {
            Gimbal_Out();
            state = get_yuntai_out_state;
        }
    } else if (state == get_yuntai_out_state) {
        if (Gimbal_Finished()) {
            ret             = 1;
            ready_to_unload = 1;
            state           = get_stop_state;
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
            Huagui_Move(HUAGUI_TOP_ANGLE, HUAGUI_ACC, HUAGUI_VELOCITY);
            Gimbal_Out();
            state = unload_ready_state;
        }
    } else if (state == unload_ready_state) {
        if (Huagui_Finished() && Gimbal_Finished()) {
            state                      = unload_calibrate_state;
            gimbal.gimbal_control_mode = Gimbal::GIMBAL_CAMERA_CONTROL;
        }
    } else if (state == unload_calibrate_state) {
        if (Gimbal_Calibrated()) {
            gimbal.gimbal_control_mode = Gimbal::GIMBAL_POSITION_CONTROL;
            Huagui_Move(HUAGUI_GROUND_ANGLE, HUAGUI_ACC, HUAGUI_VELOCITY);
            state = unload_huagui_down_state;
        }
    } else if (state == unload_huagui_down_state) {
        if (Huagui_Finished()) {
            state = unload_jiazhua_open_state;
            Jiazhua_Open();
        }
    } else if (state == unload_jiazhua_open_state) {
        if (Jiazhua_Finished()) {
            state = unload_huagui_up_state;
            Huagui_Move(HUAGUI_TOP_ANGLE, HUAGUI_ACC, HUAGUI_VELOCITY);
        }
    } else if (state == unload_huagui_up_state) {
        if (Huagui_Finished()) {
            ready_to_unload = 0;
            ret             = 1;
            state           = unload_stop_state;
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
            Huagui_Move(HUAGUI_TOP_ANGLE, HUAGUI_ACC, HUAGUI_VELOCITY);
            Gimbal_Out();
            state = unload_ready_state;
        }
    } else if (state == unload_ready_state) {
        if (Huagui_Finished() && Gimbal_Finished()) {
            state                      = unload_calibrate_state;
            gimbal.gimbal_control_mode = Gimbal::GIMBAL_CAMERA_CONTROL;
        }
    } else if (state == unload_calibrate_state) {
        if (Gimbal_Calibrated()) {
            gimbal.gimbal_control_mode = Gimbal::GIMBAL_POSITION_CONTROL;
            Huagui_Move(HUAGUI_SECOND_ANGLE, HUAGUI_ACC, HUAGUI_VELOCITY);
            state = unload_huagui_down_state;
        }
    } else if (state == unload_huagui_down_state) {
        if (Huagui_Finished()) {
            state = unload_jiazhua_open_state;
            Jiazhua_Open();
        }
    } else if (state == unload_jiazhua_open_state) {
        if (Jiazhua_Finished()) {
            state = unload_huagui_up_state;
            Huagui_Move(HUAGUI_TOP_ANGLE, HUAGUI_ACC, HUAGUI_VELOCITY);
        }
    } else if (state == unload_huagui_up_state) {
        if (Huagui_Finished()) {
            ready_to_unload = 0;
            ret             = 1;
            state           = unload_stop_state;
        }
    }
    return ret;
}
