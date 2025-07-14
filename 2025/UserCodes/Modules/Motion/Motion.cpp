#include "Motion.h"
#include "DMMotor.h"
#include "ZDTMotor.h"
#include "FSUS_Servo.h"

Motion motion;
void Motion::Init()
{
    // mot5.Init(&hcan1, 5, DIRECTION_NEGATIVE);
    // mot5.SetPosition_TRAP(0, 100, 100, 100, POSITION_MODE_ABSOLUT, MULTI_MODE_ASYNC);

    servo_protocol.Init(&huart5);
    servo_camera.init(1, &servo_protocol);
    servo_jiazhua.init(3, &servo_protocol);
    servo_zaiwu.init(2, &servo_protocol);
    if (servo_camera.ping() != 1) printf("servo_camera error\r\n");
    if (servo_jiazhua.ping() != 1) printf("servo_jiazhua error\r\n");
    if (servo_zaiwu.ping() != 1) printf("servo_zaiwu error\r\n");

    dm_gimbal.Init(&hcan2, 0x01, 0x00, CONTROL_MODE_POS_VEL);

    servo_camera.setRawAngleMTurn(CAMERA_IN_ANGLE, 3000);
    servo_jiazhua.setAngle(JIAZHUA_OPEN_ANGLE);
    servo_zaiwu.setAngle(ZAIWU_MID_ANGLE);

    dm_gimbal.Enable();
    dm_gimbal.Control(1, YUNTAI_IN_ANGLE);

}

void Motion::Huagui_Move(float _angle, float _acc, float _vel)
{
    // mot5.SetPosition_TRAP(_angle, _acc, _acc, _vel, POSITION_MODE_ABSOLUT, MULTI_MODE_ASYNC);
    // mot5.SetPosition_TRAP(_angle, _acc, _acc, _vel, POSITION_MODE_ABSOLUT, MULTI_MODE_ASYNC);
    // mot5.SetPosition_TRAP(_angle, _acc, _acc, _vel, POSITION_MODE_ABSOLUT, MULTI_MODE_ASYNC);
    // mot5.SetPosition_TRAP(_angle, _acc, _acc, _vel, POSITION_MODE_ABSOLUT, MULTI_MODE_ASYNC);
}
uint8_t Motion::Huagui_Finished()
{
    // mot5.GetPosition();
    // if (fabs(mot5.Tar_Pos - mot5.Position) <= HUAGUI_FINISHED_THRESHOLD)
    //     return 1;
    // else
        return 0;
}

void Motion::Yuntai_In()
{
    dm_gimbal.Control(YUNTAI_VELOCITY, YUNTAI_IN_ANGLE);
}
void Motion::Yuntai_Out()
{
    dm_gimbal.Control(YUNTAI_VELOCITY, YUNTAI_OUT_ANGLE);
}
uint8_t Motion::Yuntai_Finished()
{
    dm_gimbal.Enable();
    if (fabs(dm_gimbal.Pos_des - dm_gimbal.Position) <= YUNTAI_FINISHED_THRESHOLD)
        return 1;
    else
        return 0;
}

void Motion::Jiazhua_Close()
{
    vTaskDelay(5);
    servo_jiazhua.setAngle(JIAZHUA_CLOSE_ANGLE, JIAZHUA_INTERVAL);
}
void Motion::Jiazhua_Open()
{
    vTaskDelay(5);
    servo_jiazhua.setAngle(JIAZHUA_OPEN_ANGLE, JIAZHUA_INTERVAL);
}
uint8_t Motion::Jiazhua_Finished()
{
    vTaskDelay(5);
    if (fabs(servo_jiazhua.targetAngle - servo_jiazhua.queryAngle()) <= JIAZHUA_FINISHED_THRESHOLD)
        return 1;
    else
        return 0;
}

void Motion::Zaiwu_Front()
{
    vTaskDelay(5);
    servo_zaiwu.setAngle(ZAIWU_FRONT_ANGLE, ZAIWU_INTERVAL);
}
void Motion::Zaiwu_Mid()
{
    vTaskDelay(5);
    servo_zaiwu.setAngle(ZAIWU_MID_ANGLE, ZAIWU_INTERVAL);
}
void Motion::Zaiwu_Back()
{
    vTaskDelay(5);
    servo_zaiwu.setAngle(ZAIWU_BACK_ANGLE, ZAIWU_INTERVAL);
}
uint8_t Motion::Zaiwu_Finished()
{
    vTaskDelay(5);
    if (fabs(servo_zaiwu.targetAngle - servo_zaiwu.queryAngle()) <= ZAIWU_FINISHED_THRESHOLD)
        return 1;
    else
        return 0;
}

void Motion::Camera_In()
{
    vTaskDelay(5);
    servo_camera.setRawAngleMTurn(CAMERA_IN_ANGLE, CAMERA_INTERVAL);
}
void Motion::Camera_Out()
{
    vTaskDelay(5);
    servo_camera.setRawAngleMTurn(CAMERA_OUT_ANGLE, CAMERA_INTERVAL);
}

uint8_t Motion::load_from_material(uint8_t _loadDir)
{
    uint8_t ret = 0;
    static enum
    {
        load_stop_state = 0,
        load_ready_state,
        load_huagui_down_state,
        load_jiazhua_close_state,
        load_huagui_up_state,
        load_yuntai_in_state,
        load_huagui_down2_state,
        load_jiazhua_open_state,
        load_huagui_up2_state,
        load_yuntai_out_state,

    } state = load_stop_state;
    if (state == load_stop_state)
    {
        Huagui_Move(HUAGUI_TOP_ANGLE, HUAGUI_ACC, HUAGUI_VELOCITY);
        Yuntai_Out();
        Jiazhua_Open();
        if (_loadDir == 1)
            Zaiwu_Front();
        if (_loadDir == 2)
            Zaiwu_Mid();
        if (_loadDir == 3)
            Zaiwu_Back();
        state = load_ready_state;
    }
    else if (state == load_ready_state)
    {
        if (Yuntai_Finished())
        {
            state = load_huagui_down_state;
            Huagui_Move(HUAGUI_ZHUANPAN_ANGLE, HUAGUI_ACC, HUAGUI_VELOCITY);
        }
    }
    else if (state == load_huagui_down_state)
    {
        if (Huagui_Finished())
        {
            state = load_jiazhua_close_state;
            Jiazhua_Close();
        }
    }
    else if (state == load_jiazhua_close_state)
    {
        if (Jiazhua_Finished())
        {
            state = load_huagui_up_state;
            Huagui_Move(HUAGUI_TOP_ANGLE, HUAGUI_ACC, HUAGUI_VELOCITY);
        }
    }
    else if (state == load_huagui_up_state)
    {
        if (Huagui_Finished())
        {
            Yuntai_In();
            state = load_yuntai_in_state;
        }
    }
    else if (state == load_yuntai_in_state)
    {
        if (Yuntai_Finished())
        {
            state = load_huagui_down2_state;
            Huagui_Move(HUAGUI_ZAIWU_PUT_ANGLE, HUAGUI_ACC, HUAGUI_VELOCITY);
        }
    }
    else if (state == load_huagui_down2_state)
    {
        if (Huagui_Finished())
        {
            state = load_jiazhua_open_state;
            Jiazhua_Open();
        }
    }
    else if (state == load_jiazhua_open_state)
    {
        if (Jiazhua_Finished())
        {
            Huagui_Move(HUAGUI_TOP_ANGLE, HUAGUI_ACC, HUAGUI_VELOCITY);
            state = load_huagui_up2_state;
        }
    }
    else if (state == load_huagui_up2_state)
    {
        if (Huagui_Finished())
        {
            if (_loadDir == 3) {
                state = load_stop_state;
                Zaiwu_Mid();
                ret = 1;
            }
            else {
                Yuntai_Out();
                Zaiwu_Mid();

                state = load_yuntai_out_state;
            }
        }
    }
    else if (state == load_yuntai_out_state)
    {
        if (Yuntai_Finished())
        {
            state = load_stop_state;
            Zaiwu_Mid();
            ret = 1;
        }
    }
    return ret;
}

uint8_t Motion::load_from_ground(uint8_t _loadDir)
{
    uint8_t ret = 0;
    static enum
    {
        load_stop_state = 0,
        load_ready_state,
        load_huagui_down_state,
        load_jiazhua_close_state,
        load_huagui_up_state,
        load_yuntai_in_state,
        load_huagui_down2_state,
        load_jiazhua_open_state,
        load_huagui_up2_state,
        load_yuntai_out_state,

    } state = load_stop_state;
    if (state == load_stop_state)
    {
        Huagui_Move(HUAGUI_TOP_ANGLE, HUAGUI_ACC, HUAGUI_VELOCITY);
        Yuntai_Out();
        Jiazhua_Open();
        if (_loadDir == 1)
            Zaiwu_Front();
        if (_loadDir == 2)
            Zaiwu_Mid();
        if (_loadDir == 3)
            Zaiwu_Back();
        state = load_ready_state;
    }
    else if (state == load_ready_state)
    {
        if (Yuntai_Finished())
        {
            state = load_huagui_down_state;
            Huagui_Move(HUAGUI_GROUND_ANGLE, HUAGUI_ACC, HUAGUI_VELOCITY);
        }
    }
    else if (state == load_huagui_down_state)
    {
        if (Huagui_Finished())
        {
            state = load_jiazhua_close_state;
            Jiazhua_Close();
        }
    }
    else if (state == load_jiazhua_close_state)
    {
        if (Jiazhua_Finished())
        {
            state = load_huagui_up_state;
            Huagui_Move(HUAGUI_TOP_ANGLE, HUAGUI_ACC, HUAGUI_VELOCITY);
        }
    }
    else if (state == load_huagui_up_state)
    {
        if (Huagui_Finished())
        {
            if (_loadDir != 3) {
                ret = 2;
            }
            Yuntai_In();
            state = load_yuntai_in_state;
        }
    }
    else if (state == load_yuntai_in_state)
    {
        if (Yuntai_Finished())
        {
            state = load_huagui_down2_state;
            Huagui_Move(HUAGUI_ZAIWU_PUT_ANGLE, HUAGUI_ACC, HUAGUI_VELOCITY);
        }
    }
    else if (state == load_huagui_down2_state)
    {
        if (Huagui_Finished())
        {
            state = load_jiazhua_open_state;
            Jiazhua_Open();
        }
    }
    else if (state == load_jiazhua_open_state)
    {
        if (Jiazhua_Finished())
        {
            Huagui_Move(HUAGUI_TOP_ANGLE, HUAGUI_ACC, HUAGUI_VELOCITY);
            state = load_huagui_up2_state;
        }
    }
    else if (state == load_huagui_up2_state)
    {
        if (Huagui_Finished())
        {
            if (_loadDir == 3) {
                state = load_stop_state;
                Zaiwu_Mid();
                ret = 1;
            }
            else {
                Yuntai_Out();
                Zaiwu_Mid();
                state = load_yuntai_out_state;
            }
        }
    }
    else if (state == load_yuntai_out_state)
    {
        if (Yuntai_Finished())
        {
            Zaiwu_Mid();
            state = load_stop_state;
            ret = 3;
        }
    }
    return ret;
}

uint8_t Motion::get_from_car(uint8_t _loadDir)
{
    uint8_t ret = 0;
    static enum
    {
        get_stop_state = 0,
        get_ready_state,
        get_yuntai_in_state,
        get_huagui_down_state,
        get_jiazhua_close_state,
        get_huagui_up_state,
        get_yuntai_out_state,
    } state = get_stop_state;
    if (state == get_stop_state)
    {
        if (ready_to_unload == 0) {
            Huagui_Move(HUAGUI_TOP_ANGLE, HUAGUI_ACC, HUAGUI_VELOCITY);
            Jiazhua_Open();
            if (_loadDir == 1)
                Zaiwu_Front();
            if (_loadDir == 2)
                Zaiwu_Mid();
            if (_loadDir == 3)
                Zaiwu_Back();
            state = get_ready_state;
        }
    }
    else if (state == get_ready_state)
    {
        if (Huagui_Finished() && Jiazhua_Finished())
        {
            Yuntai_In();
            state = get_yuntai_in_state;
        }
    }
    else if (state == get_yuntai_in_state)
    {
        if (Yuntai_Finished() && Zaiwu_Finished())
        {
            Huagui_Move(HUAGUI_ZAIWU_GET_ANGLE, HUAGUI_ACC, HUAGUI_VELOCITY);
            state = get_huagui_down_state;
        }
    }
    else if (state == get_huagui_down_state)
    {
        if (Huagui_Finished())
        {
            Jiazhua_Close();
            state = get_jiazhua_close_state;
        }
    }
    else if (state == get_jiazhua_close_state)
    {
        if (Jiazhua_Finished())
        {
            Huagui_Move(HUAGUI_TOP_ANGLE, HUAGUI_ACC, HUAGUI_VELOCITY);
            state = get_huagui_up_state;
        }
    }
    else if (state == get_huagui_up_state)
    {
        if (Huagui_Finished())
        {
            Yuntai_Out();
            Zaiwu_Mid();
            state = get_yuntai_out_state;
        }
    }
    else if (state == get_yuntai_out_state)
    {
        if (Yuntai_Finished())
        {
            ret = 1;
            ready_to_unload = 1;
            state = get_stop_state;
            Zaiwu_Mid();
        }
    }
    return ret;
}

uint8_t Motion::unload_to_ground()
{
    uint8_t ret = 0;
    static enum
    {
        unload_stop_state,
        unload_ready_state,
        unload_huagui_down_state,
        unload_jiazhua_open_state,
        unload_huagui_up_state,
    } state = unload_stop_state;
    if (state == unload_stop_state)
    {
        if (ready_to_unload) {
            Huagui_Move(HUAGUI_TOP_ANGLE, HUAGUI_ACC, HUAGUI_VELOCITY);
            Yuntai_Out();
            state = unload_ready_state;
        }
    }
    else if (state == unload_ready_state)
    {
        if (Huagui_Finished() && Yuntai_Finished())
        {
            Huagui_Move(HUAGUI_GROUND_ANGLE, HUAGUI_ACC, HUAGUI_VELOCITY);
            state = unload_huagui_down_state;
        }
    }
    else if (state == unload_huagui_down_state)
    {
        if (Huagui_Finished())
        {
            state = unload_jiazhua_open_state;
            Jiazhua_Open();
        }
    }
    else if (state == unload_jiazhua_open_state)
    {
        if (Jiazhua_Finished())
        {
            state = unload_huagui_up_state;
            Huagui_Move(HUAGUI_TOP_ANGLE, HUAGUI_ACC, HUAGUI_VELOCITY);
        }
    }
    else if (state == unload_huagui_up_state)
    {
        if (Huagui_Finished())
        {
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
    static enum
    {
        unload_stop_state,
        unload_ready_state,
        unload_huagui_down_state,
        unload_jiazhua_open_state,
        unload_huagui_up_state,
    } state = unload_stop_state;
    if (state == unload_stop_state)
    {
        if (ready_to_unload) {
            Huagui_Move(HUAGUI_TOP_ANGLE, HUAGUI_ACC, HUAGUI_VELOCITY);
            Yuntai_Out();
            state = unload_ready_state;
        }
    }
    else if (state == unload_ready_state)
    {
        if (Huagui_Finished() && Yuntai_Finished())
        {
            Huagui_Move(HUAGUI_SECOND_ANGLE, HUAGUI_ACC, HUAGUI_VELOCITY);
            state = unload_huagui_down_state;
        }
    }
    else if (state == unload_huagui_down_state)
    {
        if (Huagui_Finished())
        {
            state = unload_jiazhua_open_state;
            Jiazhua_Open();
        }
    }
    else if (state == unload_jiazhua_open_state)
    {
        if (Jiazhua_Finished())
        {
            state = unload_huagui_up_state;
            Huagui_Move(HUAGUI_TOP_ANGLE, HUAGUI_ACC, HUAGUI_VELOCITY);
        }
    }
    else if (state == unload_huagui_up_state)
    {
        if (Huagui_Finished())
        {
            ready_to_unload = 0;
            ret = 1;
            state = unload_stop_state;
        }
    }
    return ret;
}


