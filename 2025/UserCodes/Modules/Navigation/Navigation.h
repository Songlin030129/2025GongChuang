#ifndef NAVIGATION_H
#define NAVIGATION_H
#include "common_inc.h"
#include <vector>
#include "Chassis.h"
#include <cmath>

#define MAX_PLAN_VEL 0.7f // 最大直线速度
#define MAX_PLAN_W 3.14f   // 最大旋转速度

#define ACC 0.5f    // 直线段加速度
#define W_ACC 3.14f // 自转段角加速度

#define Round_Error 0.001f  // 圆整误差 请勿改小，过小会导致程序出错
#define Round_Error_T 0.01f // 圆整误差 请勿改小，过小会导致程序出错

#define low_constrain(amt, low) ((amt) < (low) ? (amt = low) : (amt = amt))     // 下限 限幅
#define high_constrain(amt, high) ((amt) > (high) ? (amt = high) : (amt = amt)) // 上限 限幅

typedef enum
{
    NONE = 0, // 直线无旋转方向
    CCW = 1,  // 逆时针
    CW = 2,   // 顺时针
} rotate_direction_e;

typedef enum
{
    TRUE = 1,
    FALSE = 0,
} logic_e;

typedef enum
{
    PATH_TYPE_START_END = 0,
    PATH_TYPE_START,
    PATH_TYPE_MID,
    PATH_TYPE_END,
} path_type_e;

typedef enum
{
    PATH_SHAPE_LINE = 0,
    PATH_SHAPE_CIRCLE = 1,
} path_shape_e;

typedef struct
{
    float x;
    float y;
    float theta;
} position_t;

typedef struct
{
    float center_x;
    float center_y;
    float radius;
} arc_t;

// 路径信息
typedef struct
{
    /********************** 输入数据 **********************/
    path_type_e path_type; // 路径类型结构体

    path_shape_e path_shape; // 路径形状

    position_t start_pos; // 起始位置角度

    position_t end_pos; // 终止位置角度

    arc_t arc; // 圆弧圆心坐标半径

    /********************** END 输入数据 **********************/

    float v_start;  // 起点线速度
    float v_target; // 运行过程中最大线速度
    float v_end;    // 终点线速度

    float w_start;  // 起点小车方向角速度
    float w_target; // 运行过程中最大小车方向角速度
    float w_end;    // 终点小车方向角速度
    rotate_direction_e rotate_direction; // 转角方向标志位
    rotate_direction_e arc_direction;  // 圆弧旋转方向

} path_t;

// 插补数据
typedef struct
{
    float x;      // 插补点x (单位:m)
    float y;      // 插补点y (单位:m)
    float theta;  // 插补点theta车头朝向

    float w_z; // 小车自旋 角速度
    float vel; // 小车线速度

    float vel_x;
    float vel_y;

    float path_theta; // 起始点和终止点连线 L 相对于x轴的角度 路径角度
    float total_distance;   // 总共的插补距离 (单位:m)
    float brake_distance;   // 刹车距离 (单位:m)
    float past_distance;    // 已插补过的距离 (单位:m)
    float residue_distance; // 未插补的距离 (单位:m)

    float total_theta;   // 总小车方向转角(rad)
    float brake_theta;   // 小车方向刹车转角(rad)
    float past_theta;    // 已插补过的小车方向角度(rad)
    float residue_theta; // 未插补的小车方向角度(rad)

    float total_arcAngle;   // 总圆弧角度(rad)
    float brake_arcAngle;   // 刹车圆弧角度(rad)
    float past_arcAngle;    // 已插补过的圆弧角度(rad)
    float residue_arcAngle; // 未插补的圆弧角度(rad)

    // 真实的圆弧轨迹半径 在圆弧插补中实时的计算
    // 因为圆弧插补的半径是变化的 用变化值代替固定值减小误差
    float arc_radius_actual;

    logic_e line_interpolation_is_done;  // 直线插补完成标志位
    logic_e arc_interpolation_is_done;   // 小车角度插补完成标志位
    logic_e theta_interpolation_is_done; // 小车角度插补完成标志位
    logic_e both_interpolation_is_done;  // 插补完成标志位
} interpolation_t;

typedef struct
{
    float x;      // 插补点x (单位:m)
    float y;      // 插补点y (单位:m)
    float theta;  // 插补点theta车头朝向

    float w_z; // 小车自旋 角速度
    float vel; // 小车线速度

    float vel_x; // x方向速度
    float vel_y; // y方向速度

    logic_e all_paths_interpolation_is_done;

}global_state_t;

class Navigation
{
public:
    std::vector<path_t*> paths;
    std::vector<interpolation_t*> inp_datas;
    void addPath(path_t* _path);
    void addPaths(path_t* _paths, uint8_t _size);
    global_state_t path_interpolation();

private:
    float pre_v = 0;      // 前插补点速度 直线插补和圆弧插补中用到
    float pre_w_z = 0;    // 前插补点转速 下车转件插补中用到
    float dl = 0, ds = 0; // 微分直线长 微分弧长
    float dtheta = 0;     // 微分角度
    float acc_adjusted;   // 直线 加速度调整值
    float w_acc_adjusted; // 小车方向角 角加速度调整值
    uint32_t Last_Time = 0;
    uint8_t path_index = 0;
};

#endif
