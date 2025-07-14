#include "Paths.h"
#define DEBUG_PATH 0
#define FF_GAIN 1.0f
Paths paths;
uint8_t Paths::task_from_1_to_2()
{
    uint8_t ret = 0;
    static uint8_t NAV_STATE = 0; // 状态机索引
    static global_state_t global_state = { 0 };
    switch (NAV_STATE)
    {
    case 0: //********** 初始化状态 **********
        navi_1_to_2.addPath(&path_1_to_2);
        NAV_STATE = 1; // 切换到运行状态
        break;
    case 1: //********** 运行状态 **********

        global_state = navi_1_to_2.path_interpolation();
        chassis.Tar_Pos_X = global_state.x;
        chassis.Tar_Pos_Y = global_state.y;
        chassis.Tar_Angle = global_state.theta;
        chassis.Vel_X_FF = global_state.vel_x * FF_GAIN;
        chassis.Vel_Y_FF = global_state.vel_y * FF_GAIN;

#if DEBUG_PATH
        printf("x:%f, y:%f, theta:%f, w_z:%f, vel:%f, vel_x:%f, vel_y:%f, finish_state:%d, ops_x:%f, ops_y:%f, ops_yaw:%f, ops_vel_x:%f, ops_vel_y:%f\r\n",
            global_state.x, global_state.y, global_state.theta,
            global_state.w_z, global_state.vel, global_state.vel_x, global_state.vel_y,
            global_state.all_paths_interpolation_is_done,
            ops.Pos_X, ops.Pos_Y, ops.Yaw, ops.Vel_X, ops.Vel_Y
        );
#endif

        if (global_state.all_paths_interpolation_is_done == 1)
        {
            ret = 1;
            global_state = { 0 };
            NAV_STATE = 0;
        };
        break;
    default:
        break;
    }
    return ret;
}

uint8_t Paths::task_from_2_to_3()
{
    uint8_t ret = 0;
    static uint8_t NAV_STATE = 0; // 状态机索引
    static global_state_t global_state = { 0 };
    switch (NAV_STATE)
    {
    case 0: //********** 初始化状态 **********
        navi_2_to_3.addPath(&path_2_to_3);
        NAV_STATE = 1; // 切换到运行状态
        break;
    case 1: //********** 运行状态 **********

        global_state = navi_2_to_3.path_interpolation();
        chassis.Tar_Pos_X = global_state.x;
        chassis.Tar_Pos_Y = global_state.y;
        chassis.Tar_Angle = global_state.theta;
        chassis.Vel_X_FF = global_state.vel_x * FF_GAIN;
        chassis.Vel_Y_FF = global_state.vel_y * FF_GAIN;

#if DEBUG_PATH
        printf("x:%f, y:%f, theta:%f, w_z:%f, vel:%f, vel_x:%f, vel_y:%f, finish_state:%d, ops_x:%f, ops_y:%f, ops_yaw:%f, ops_vel_x:%f, ops_vel_y:%f\r\n",
            global_state.x, global_state.y, global_state.theta,
            global_state.w_z, global_state.vel, global_state.vel_x, global_state.vel_y,
            global_state.all_paths_interpolation_is_done,
            ops.Pos_X, ops.Pos_Y, ops.Yaw, ops.Vel_X, ops.Vel_Y
        );
#endif
        if (global_state.all_paths_interpolation_is_done == 1)
        {
            ret = 1;
            global_state = { 0 };
            NAV_STATE = 0;
        };
        break;
    default:
        break;
    }
    return ret;
}

uint8_t Paths::task_from_3_to_7(uint8_t _des_flag)
{
    uint8_t ret = 0;
    static uint8_t NAV_STATE = 0; // 状态机索引
    static global_state_t global_state = { 0 };
    switch (NAV_STATE)
    {
    case 0: //********** 初始化状态 **********
        if (_des_flag == 1) {
            path_3_to_7[3].end_pos = pos_7_1;
        }
        else if (_des_flag == 2) {
            path_3_to_7[3].end_pos = pos_7_2;
        }
        else if (_des_flag == 3) {
            path_3_to_7[3].end_pos = pos_7_3;
        }
        navi_3_to_7.addPaths(path_3_to_7, 4);
        NAV_STATE = 1; // 切换到运行状态
        break;
    case 1: //********** 运行状态 **********

        global_state = navi_3_to_7.path_interpolation();
        chassis.Tar_Pos_X = global_state.x;
        chassis.Tar_Pos_Y = global_state.y;
        chassis.Tar_Angle = global_state.theta;
        chassis.Vel_X_FF = global_state.vel_x * FF_GAIN;
        chassis.Vel_Y_FF = global_state.vel_y * FF_GAIN;

#if DEBUG_PATH
        printf("x:%f, y:%f, theta:%f, w_z:%f, vel:%f, vel_x:%f, vel_y:%f, finish_state:%d, ops_x:%f, ops_y:%f, ops_yaw:%f, ops_vel_x:%f, ops_vel_y:%f\r\n",
            global_state.x, global_state.y, global_state.theta,
            global_state.w_z, global_state.vel, global_state.vel_x, global_state.vel_y,
            global_state.all_paths_interpolation_is_done,
            ops.Pos_X, ops.Pos_Y, ops.Yaw, ops.Vel_X, ops.Vel_Y
        );
#endif
        if (global_state.all_paths_interpolation_is_done == 1)
        {
            ret = 1;
            global_state = { 0 };
            NAV_STATE = 0;
        };
        break;
    default:
        break;
    }
    return ret;
}

uint8_t Paths::task_from_7_to_10(uint8_t _src_flag, uint8_t _des_flag)
{
    uint8_t ret = 0;
    static uint8_t NAV_STATE = 0; // 状态机索引
    static global_state_t global_state = { 0 };
    switch (NAV_STATE)
    {
    case 0: //********** 初始化状态 **********
        if (_src_flag == 1) {
            path_7_to_10[0].start_pos = pos_7_1;
        }
        else if (_src_flag == 2) {
            path_7_to_10[0].start_pos = pos_7_2;
        }
        else if (_src_flag == 3) {
            path_7_to_10[0].start_pos = pos_7_3;
        }
        if (_des_flag == 1) {
            path_7_to_10[2].end_pos = pos_10_1;
        }
        else if (_des_flag == 2) {
            path_7_to_10[2].end_pos = pos_10_2;
        }
        else if (_des_flag == 3) {
            path_7_to_10[2].end_pos = pos_10_3;
        }

        navi_7_to_10.addPaths(path_7_to_10, 3);
        NAV_STATE = 1; // 切换到运行状态
        break;
    case 1: //********** 运行状态 **********

        global_state = navi_7_to_10.path_interpolation();
        chassis.Tar_Pos_X = global_state.x;
        chassis.Tar_Pos_Y = global_state.y;
        chassis.Tar_Angle = global_state.theta;
        chassis.Vel_X_FF = global_state.vel_x * FF_GAIN;
        chassis.Vel_Y_FF = global_state.vel_y * FF_GAIN;

#if DEBUG_PATH
        printf("x:%f, y:%f, theta:%f, w_z:%f, vel:%f, vel_x:%f, vel_y:%f, finish_state:%d, ops_x:%f, ops_y:%f, ops_yaw:%f, ops_vel_x:%f, ops_vel_y:%f\r\n",
            global_state.x, global_state.y, global_state.theta,
            global_state.w_z, global_state.vel, global_state.vel_x, global_state.vel_y,
            global_state.all_paths_interpolation_is_done,
            ops.Pos_X, ops.Pos_Y, ops.Yaw, ops.Vel_X, ops.Vel_Y
        );
#endif
        if (global_state.all_paths_interpolation_is_done == 1)
        {
            ret = 1;
            global_state = { 0 };
            NAV_STATE = 0;
        };
        break;
    default:
        break;
    }
    return ret;
}

uint8_t Paths::task_from_10_to_3(uint8_t _src_flag)
{
    uint8_t ret = 0;
    static uint8_t NAV_STATE = 0; // 状态机索引
    static global_state_t global_state = { 0 };
    switch (NAV_STATE)
    {
    case 0: //********** 初始化状态 **********
        if (_src_flag == 1) {
            path_10_to_3[0].start_pos = pos_10_1;
        }
        else if (_src_flag == 2) {
            path_10_to_3[0].start_pos = pos_10_2;
        }
        else if (_src_flag == 3) {
            path_10_to_3[0].start_pos = pos_10_3;
        }
        navi_10_to_3.addPaths(path_10_to_3, 3);
        NAV_STATE = 1; // 切换到运行状态
        break;
    case 1: //********** 运行状态 **********

        global_state = navi_10_to_3.path_interpolation();
        chassis.Tar_Pos_X = global_state.x;
        chassis.Tar_Pos_Y = global_state.y;
        chassis.Tar_Angle = global_state.theta;
        chassis.Vel_X_FF = global_state.vel_x * FF_GAIN;
        chassis.Vel_Y_FF = global_state.vel_y * FF_GAIN;

#if DEBUG_PATH
        printf("x:%f, y:%f, theta:%f, w_z:%f, vel:%f, vel_x:%f, vel_y:%f, finish_state:%d, ops_x:%f, ops_y:%f, ops_yaw:%f, ops_vel_x:%f, ops_vel_y:%f\r\n",
            global_state.x, global_state.y, global_state.theta,
            global_state.w_z, global_state.vel, global_state.vel_x, global_state.vel_y,
            global_state.all_paths_interpolation_is_done,
            ops.Pos_X, ops.Pos_Y, ops.Yaw, ops.Vel_X, ops.Vel_Y
        );
#endif
        if (global_state.all_paths_interpolation_is_done == 1)
        {
            ret = 1;
            global_state = { 0 };
            NAV_STATE = 0;
        };
        break;
    default:
        break;
    }
    return ret;
}

uint8_t Paths::task_from_10_to_1(uint8_t _src_flag)
{
    uint8_t ret = 0;
    static uint8_t NAV_STATE = 0; // 状态机索引
    static global_state_t global_state = { 0 };
    switch (NAV_STATE)
    {
    case 0: //********** 初始化状态 **********
        if (_src_flag == 1) {
            path_10_to_1[0].start_pos = pos_10_1;
        }
        else if (_src_flag == 2) {
            path_10_to_1[0].start_pos = pos_10_2;
        }
        else if (_src_flag == 3) {
            path_10_to_1[0].start_pos = pos_10_3;
        }

        navi_10_to_1.addPaths(path_10_to_1, 4);
        NAV_STATE = 1; // 切换到运行状态
        break;
    case 1: //********** 运行状态 **********

        global_state = navi_10_to_1.path_interpolation();
        chassis.Tar_Pos_X = global_state.x;
        chassis.Tar_Pos_Y = global_state.y;
        chassis.Tar_Angle = global_state.theta;
        chassis.Vel_X_FF = global_state.vel_x * FF_GAIN;
        chassis.Vel_Y_FF = global_state.vel_y * FF_GAIN;

#if DEBUG_PATH
        printf("x:%f, y:%f, theta:%f, w_z:%f, vel:%f, vel_x:%f, vel_y:%f, finish_state:%d, ops_x:%f, ops_y:%f, ops_yaw:%f, ops_vel_x:%f, ops_vel_y:%f\r\n",
            global_state.x, global_state.y, global_state.theta,
            global_state.w_z, global_state.vel, global_state.vel_x, global_state.vel_y,
            global_state.all_paths_interpolation_is_done,
            ops.Pos_X, ops.Pos_Y, ops.Yaw, ops.Vel_X, ops.Vel_Y
        );
#endif
        if (global_state.all_paths_interpolation_is_done == 1)
        {
            ret = 1;
            global_state = { 0 };
            NAV_STATE = 0;
        };
        break;
    default:
        break;
    }
    return ret;
}




