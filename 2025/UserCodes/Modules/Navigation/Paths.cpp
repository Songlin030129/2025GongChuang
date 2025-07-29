#include "Paths.h"
#define DEBUG_PATH 0
#define FF_GAIN 1.0f
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
Paths paths;
uint8_t Paths::task_from_start_to_qrcode()
{
    uint8_t ret = 0;
    static uint8_t NAV_STATE = 0; // 状态机索引
    static global_state_t global_state = { 0 };
    switch (NAV_STATE)
    {
    case 0: //********** 初始化状态 **********
        navi_start_to_qrcode.addPaths(path_start_to_qrcode, ARRAY_SIZE(path_start_to_qrcode));
        NAV_STATE = 1; // 切换到运行状态
        break;
    case 1: //********** 运行状态 **********

        global_state = navi_start_to_qrcode.path_interpolation();
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

uint8_t Paths::task_from_qrcode_to_material()
{
    uint8_t ret = 0;
    static uint8_t NAV_STATE = 0; // 状态机索引
    static global_state_t global_state = { 0 };
    switch (NAV_STATE)
    {
    case 0: //********** 初始化状态 **********
        navi_qrcode_to_material.addPaths(path_qrcode_to_material, ARRAY_SIZE(path_qrcode_to_material));
        NAV_STATE = 1; // 切换到运行状态
        break;
    case 1: //********** 运行状态 **********

        global_state = navi_qrcode_to_material.path_interpolation();
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

uint8_t Paths::task_from_material_to_process()
{
    uint8_t ret = 0;
    static uint8_t NAV_STATE = 0; // 状态机索引
    static global_state_t global_state = { 0 };
    switch (NAV_STATE)
    {
    case 0: //********** 初始化状态 **********
        navi_material_to_process.addPaths(path_material_to_process, ARRAY_SIZE(path_material_to_process));
        NAV_STATE = 1; // 切换到运行状态
        break;
    case 1: //********** 运行状态 **********

        global_state = navi_material_to_process.path_interpolation();
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

uint8_t Paths::task_from_process_to_storage()
{
    uint8_t ret = 0;
    static uint8_t NAV_STATE = 0; // 状态机索引
    static global_state_t global_state = { 0 };
    switch (NAV_STATE)
    {
    case 0: //********** 初始化状态 **********

        navi_process_to_storage.addPaths(path_process_to_storage, ARRAY_SIZE(path_process_to_storage));
        NAV_STATE = 1; // 切换到运行状态
        break;
    case 1: //********** 运行状态 **********

        global_state = navi_process_to_storage.path_interpolation();
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

uint8_t Paths::task_from_storage_to_material()
{
    uint8_t ret = 0;
    static uint8_t NAV_STATE = 0; // 状态机索引
    static global_state_t global_state = { 0 };
    switch (NAV_STATE)
    {
    case 0: //********** 初始化状态 **********
        navi_storage_to_material.addPaths(path_storage_to_material, ARRAY_SIZE(path_storage_to_material));
        NAV_STATE = 1; // 切换到运行状态
        break;
    case 1: //********** 运行状态 **********

        global_state = navi_storage_to_material.path_interpolation();
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

uint8_t Paths::task_from_storage_to_stop()
{
    uint8_t ret = 0;
    static uint8_t NAV_STATE = 0; // 状态机索引
    static global_state_t global_state = { 0 };
    switch (NAV_STATE)
    {
    case 0: //********** 初始化状态 **********
        navi_storage_to_stop.addPaths(path_storage_to_stop, ARRAY_SIZE(path_storage_to_stop));
        NAV_STATE = 1; // 切换到运行状态
        break;
    case 1: //********** 运行状态 **********

        global_state = navi_storage_to_stop.path_interpolation();
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




