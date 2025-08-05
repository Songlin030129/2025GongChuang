#include "Navigation.h"


void Navigation::addPath(path_t* _path)
{

    path_t* path = _path;
    interpolation_t* inp_data = (interpolation_t*)pvPortMalloc(sizeof(interpolation_t));
    // interpolation_t* inp_data = _inp_data;

    switch (path->path_type)
    {
    case PATH_TYPE_START_END:
        path->v_start = 0;
        path->v_target = MAX_PLAN_VEL;
        path->v_end = 0;
        path->w_start = 0;
        path->w_target = MAX_PLAN_W;
        path->w_end = 0;
        break;
    case PATH_TYPE_START:
        path->v_start = 0;
        path->v_target = MAX_PLAN_VEL;
        path->v_end = MAX_PLAN_VEL;
        path->w_start = 0;
        path->w_target = MAX_PLAN_W;
        path->w_end = 0;
        break;
    case PATH_TYPE_MID:
        path->v_start = MAX_PLAN_VEL;
        path->v_target = MAX_PLAN_VEL;
        path->v_end = MAX_PLAN_VEL;
        path->w_start = 0;
        path->w_target = MAX_PLAN_W;
        path->w_end = 0;
        break;
    case PATH_TYPE_END:
        path->v_start = MAX_PLAN_VEL;
        path->v_target = MAX_PLAN_VEL;
        path->v_end = 0;
        path->w_start = 0;
        path->w_target = MAX_PLAN_W;
        path->w_end = 0;
        break;
    default:
        break;
    }
    switch (path->path_shape)
    {
    case PATH_SHAPE_LINE:
        path->arc.center_x = 0.00f;
        path->arc.center_y = 0.00f;
        path->arc.radius = 0.00f;
        path->arc_direction = NONE;
        break;

    default:
        break;
    }
    if (navi_abs(path->arc.radius) < Round_Error) // ****直线插补参数初始化****
    {
        // 计算轨迹角度 起始点和终止点连线 L 相对于x轴的角度 路径角度
        inp_data->path_theta = atan2(path->end_pos.y - path->start_pos.y, path->end_pos.x - path->start_pos.x);
        // 初始化插补数据 直线插补数据初始化
        inp_data->total_distance = sqrt(pow((path->end_pos.x - path->start_pos.x), 2) + pow((path->end_pos.y - path->start_pos.y), 2)); // 直线插补 直线的总长
        inp_data->brake_distance = 0;
        inp_data->past_distance = 0;
        inp_data->residue_distance = inp_data->total_distance; // 剩余距离
    }
    else // ****圆弧插补参数初始化****
    {
        float temp = (path->start_pos.x - path->arc.center_x) * (path->end_pos.y - path->arc.center_y) - (path->start_pos.y - path->arc.center_y) * (path->end_pos.x - path->arc.center_x);
        if (temp > 0)
        {
            path->arc_direction = CCW;
            // printf("Dir:CCW\r\n");
        }
        else if (temp < 0)
        {
            path->arc_direction = CW;
            // printf("Dir:CW\r\n");
        }
        else
        {
            // printf("error paramenter,can not calculate circle direction!\r\n");
            return;
        }
        // 使用余弦定理根据起点终点弧长计算总转角
        inp_data->total_arcAngle = acos(1 - (pow(path->start_pos.x - path->end_pos.x, 2) + pow(path->start_pos.y - path->end_pos.y, 2)) / (2 * pow(path->arc.radius, 2)));
        inp_data->brake_arcAngle = 0;
        inp_data->past_arcAngle = 0;
        inp_data->residue_arcAngle = inp_data->total_arcAngle;
        // 初始化插补数据 圆弧插补数据初始化
        inp_data->total_distance = (2 * _PI * path->arc.radius) * (inp_data->total_arcAngle / (2 * _PI)); // 圆弧总长
        inp_data->brake_distance = 0;
        inp_data->past_distance = 0;
        inp_data->residue_distance = inp_data->total_distance; // 剩余距离
        inp_data->arc_radius_actual = path->arc.radius * (1 + 0.5 * pow(inp_data->past_arcAngle, 2) -
            pow(inp_data->past_arcAngle, 3) / inp_data->total_arcAngle +
            0.5 * pow(inp_data->past_arcAngle, 4) / pow(inp_data->total_arcAngle, 2));
    }

    // 初始化插补数据 自转插补数据初始化
    inp_data->vel = path->v_start;
    inp_data->w_z = path->w_start;
    inp_data->theta = path->start_pos.theta;
    inp_data->total_theta = navi_abs(path->end_pos.theta - path->start_pos.theta); // 自转角度插补 小车方向角 总转角
    inp_data->brake_theta = 0;
    inp_data->past_theta = 0;
    inp_data->residue_theta = (inp_data->total_theta); // 剩余弧度用绝对值表示

    // 插补标志位 置位
    inp_data->line_interpolation_is_done = FALSE;  // 直线插补完成标志位
    inp_data->theta_interpolation_is_done = FALSE; // 车身转角插补完成标志位
    inp_data->arc_interpolation_is_done = FALSE;   // 圆弧插补完成标志位
    inp_data->both_interpolation_is_done = FALSE;  // 总插补完成标志位

    // 判断自旋方向
    (path->start_pos.theta <= path->end_pos.theta) ? (path->rotate_direction = CW) : (path->rotate_direction = CCW);

    // 判断 起始点 终止点 速度是否超过最大速度 超过则报错
    if ((path->v_start > path->v_target) || (path->w_start > path->w_target) ||
        (path->v_end > path->v_target) || (path->w_end > path->w_target))
    {
        printf("error paramenter! please check");
        return;
    }
    paths.push_back(path);
    inp_datas.push_back(inp_data);
    // printf("add path\r\n");
}

void Navigation::addPaths(path_t* _paths, uint8_t _size)
{
    for (uint8_t i = 0; i < _size; i++)
    {
        addPath(_paths + i);
    }
}

global_state_t Navigation::path_interpolation()
{
    global_state_t ret = {};
    //获取时间间隔Ts
    uint32_t currentTime = HAL_GetTick();
    if (Last_Time == 0)
    {
        Last_Time = currentTime;
        ret.x = paths[path_index]->start_pos.x;
        ret.y = paths[path_index]->start_pos.y;
        ret.theta = paths[path_index]->start_pos.theta;
        return ret;
    }
    float Time_Step = (float)(currentTime - Last_Time) / 1000.0f;
    Last_Time = currentTime;

    if (inp_datas[path_index]->both_interpolation_is_done == TRUE)
        path_index++;
    high_constrain(path_index, paths.size() - 1); // 上限 限幅

    if (navi_abs(paths[path_index]->arc.radius) < Round_Error) // 如果轨迹的半径为0 则进入直线插补
    {
        if (inp_datas[path_index]->residue_distance > Round_Error) // 剩余距离大于0
        {
            dl = 0; // dl先清 0
            pre_v = inp_datas[path_index]->vel;
            inp_datas[path_index]->brake_distance = navi_abs(pow(inp_datas[path_index]->vel, 2) - pow(paths[path_index]->v_end, 2)) / (2 * ACC);
            // 剩余距离大于理论刹车距离 将速度逐步的调整为给定的目标(最大)速度 可以加速、减速、匀速
            if (inp_datas[path_index]->residue_distance - inp_datas[path_index]->brake_distance > Round_Error)
            {
                if (inp_datas[path_index]->vel < paths[path_index]->v_target) // 加速
                {
                    inp_datas[path_index]->vel += ACC * Time_Step;
                    high_constrain(inp_datas[path_index]->vel, paths[path_index]->v_target);
                    dl = navi_abs(pow(inp_datas[path_index]->vel, 2) - pow(pre_v, 2)) / (2 * ACC);
                }
                else if (inp_datas[path_index]->vel > paths[path_index]->v_target) // 减速
                {
                    inp_datas[path_index]->vel -= ACC * Time_Step;
                    low_constrain(inp_datas[path_index]->vel, paths[path_index]->v_target);
                    dl = navi_abs(pow(inp_datas[path_index]->vel, 2) - pow(pre_v, 2)) / (2 * ACC);
                }
                else // 匀速(达到目标速度)
                {
                    inp_datas[path_index]->vel = paths[path_index]->v_target;
                    dl = inp_datas[path_index]->vel * Time_Step;
                }
            }
            // 剩余距离小于理论刹车距离 将速度慢慢调整为 给定的终点速度
            else
            {
                // 更改加速度，使小车可以在终点速度恰好为要求的速度
                acc_adjusted = navi_abs(pow(inp_datas[path_index]->vel, 2) - pow(paths[path_index]->v_end, 2)) / (2 * inp_datas[path_index]->residue_distance);
                if (inp_datas[path_index]->vel < paths[path_index]->v_end) // 加速
                {
                    inp_datas[path_index]->vel += acc_adjusted * Time_Step;
                    high_constrain(inp_datas[path_index]->vel, paths[path_index]->v_end);
                    dl = navi_abs(pow(inp_datas[path_index]->vel, 2) - pow(pre_v, 2)) / (2 * acc_adjusted);
                }
                else if (inp_datas[path_index]->vel > paths[path_index]->v_end) // 减速
                {
                    inp_datas[path_index]->vel -= acc_adjusted * Time_Step;
                    low_constrain(inp_datas[path_index]->vel, paths[path_index]->v_end);
                    dl = navi_abs(pow(inp_datas[path_index]->vel, 2) - pow(pre_v, 2)) / (2 * acc_adjusted);
                }
                else // 匀速(达到目标速度之后)
                {
                    inp_datas[path_index]->vel = paths[path_index]->v_end;
                    dl = inp_datas[path_index]->vel * Time_Step;
                }
            }

            inp_datas[path_index]->past_distance += dl;                                                   // 更新已走过的距离
            inp_datas[path_index]->residue_distance = inp_datas[path_index]->total_distance - inp_datas[path_index]->past_distance; // 更新剩余距离

            if (inp_datas[path_index]->residue_distance < Round_Error) // 判断剩余距离是否为0
            {
                inp_datas[path_index]->past_distance = inp_datas[path_index]->total_distance;
                inp_datas[path_index]->residue_distance = 0;
                inp_datas[path_index]->vel = paths[path_index]->v_end;
                inp_datas[path_index]->line_interpolation_is_done = TRUE; // 直线插补完成标志位
            }

            inp_datas[path_index]->x = inp_datas[path_index]->past_distance * cos(inp_datas[path_index]->path_theta) + paths[path_index]->start_pos.x; // 下一个点的位置 这个给到控制器
            inp_datas[path_index]->y = inp_datas[path_index]->past_distance * sin(inp_datas[path_index]->path_theta) + paths[path_index]->start_pos.y; // 下一个点的位置 这个给到控制器
            inp_datas[path_index]->vel_x = inp_datas[path_index]->vel * cos(inp_datas[path_index]->path_theta);
            inp_datas[path_index]->vel_y = inp_datas[path_index]->vel * sin(inp_datas[path_index]->path_theta);

        } // END inp_datas[path_index]->residue_distance > Round_Error
        else // 剩余距离0
        {
            inp_datas[path_index]->line_interpolation_is_done = TRUE; // 直线插补完成标志位
            // inp_datas[path_index]->x = paths[path_index]->end_pos.x;
            // inp_datas[path_index]->y = paths[path_index]->end_pos.y;

        }

    } // END 直线插补
    else // 如果轨迹的半径不为0则 进入 ****圆弧插补****
    {
        // 圆弧插补 判断依据是 剩余的速度方向角 初始的速度方向角就等于圆弧角度 往前走的话圆弧角度减小 速度方向角也减小
        // 在圆弧插补的注释里面 没有特殊说明的话 用角度 指代速度方向角度 用角速度 指代 速度方向角角速度
        if (inp_datas[path_index]->residue_distance > Round_Error) // 速度方向角 插补
        {
            ds = 0;
            pre_v = inp_datas[path_index]->vel;
            inp_datas[path_index]->brake_distance = navi_abs(pow(inp_datas[path_index]->vel, 2) - pow(paths[path_index]->v_end, 2)) / (ACC * 2);
            // 剩余角度大于理论刹车角度 将速度逐步的调整为给定的目标(最大)速度 可以加速、减速、匀速
            if (inp_datas[path_index]->residue_distance - inp_datas[path_index]->brake_distance > Round_Error)
            {
                if (inp_datas[path_index]->vel < paths[path_index]->v_target) // 加速
                {
                    inp_datas[path_index]->vel += ACC * Time_Step;
                    high_constrain(inp_datas[path_index]->vel, paths[path_index]->v_target);
                    ds = navi_abs(pow(inp_datas[path_index]->vel, 2) - pow(pre_v, 2)) / (2 * ACC);
                }
                else if (inp_datas[path_index]->vel > paths[path_index]->v_target) // 减速
                {
                    inp_datas[path_index]->vel -= ACC * Time_Step;
                    low_constrain(inp_datas[path_index]->vel, paths[path_index]->v_target);
                    ds = navi_abs(pow(inp_datas[path_index]->vel, 2) - pow(pre_v, 2)) / (2 * ACC);
                }
                else // 匀速(达到目标速度)
                {
                    inp_datas[path_index]->vel = paths[path_index]->v_target;
                    ds = inp_datas[path_index]->vel * Time_Step;
                }
            }
            // 剩余角度小于理论刹车角度 将角速度慢慢调整为 给定的终点速度
            else
            {
                // 更改加速度，使小车可以在终点速度恰好为要求的速度
                acc_adjusted = navi_abs(pow(inp_datas[path_index]->vel, 2) - pow(paths[path_index]->v_end, 2)) / (2 * inp_datas[path_index]->residue_distance);
                if (inp_datas[path_index]->vel < paths[path_index]->v_end) // 加速
                {
                    inp_datas[path_index]->vel += acc_adjusted * Time_Step;
                    high_constrain(inp_datas[path_index]->vel, paths[path_index]->v_end);
                    ds = navi_abs(pow(inp_datas[path_index]->vel, 2) - pow(pre_v, 2)) / (2 * acc_adjusted);
                }
                else if (inp_datas[path_index]->vel > paths[path_index]->v_end) // 减速
                {
                    inp_datas[path_index]->vel -= acc_adjusted * Time_Step;
                    low_constrain(inp_datas[path_index]->vel, paths[path_index]->v_end);
                    ds = navi_abs(pow(inp_datas[path_index]->vel, 2) - pow(pre_v, 2)) / (2 * acc_adjusted);
                }
                else // 匀速(达到目标速度之后)
                {
                    inp_datas[path_index]->vel = paths[path_index]->v_end;
                    ds = inp_datas[path_index]->vel * Time_Step;
                }
            }

            inp_datas[path_index]->past_distance += ds;                                                   // 更新已走过的距离
            inp_datas[path_index]->residue_distance = inp_datas[path_index]->total_distance - inp_datas[path_index]->past_distance; // 更新剩余距离

            inp_datas[path_index]->past_arcAngle = (inp_datas[path_index]->past_distance / inp_datas[path_index]->total_distance) * (inp_datas[path_index]->total_arcAngle); // 更新已走过的圆弧角度
            inp_datas[path_index]->residue_arcAngle = inp_datas[path_index]->total_arcAngle - inp_datas[path_index]->past_arcAngle;                             // 更新剩余的圆弧角度

            if (inp_datas[path_index]->residue_distance < Round_Error)
            {
                inp_datas[path_index]->past_distance = inp_datas[path_index]->total_distance;
                inp_datas[path_index]->residue_distance = 0;
                inp_datas[path_index]->vel = paths[path_index]->v_end;
                inp_datas[path_index]->arc_interpolation_is_done = TRUE; // 圆弧插补完成标志
            }
            else
            {
                inp_datas[path_index]->arc_interpolation_is_done = FALSE; // 圆弧插补完成标志
            }

            // 计算现在这个时刻真实的曲率半径(也可以理解为圆弧半径) 连续曲率的四次方曲线
            inp_datas[path_index]->arc_radius_actual = paths[path_index]->arc.radius * (1 + 0.5 * pow(inp_datas[path_index]->past_arcAngle, 2) -
                pow(inp_datas[path_index]->past_arcAngle, 3) / inp_datas[path_index]->total_arcAngle +
                0.5 * pow(inp_datas[path_index]->past_arcAngle, 4) / pow(inp_datas[path_index]->total_arcAngle, 2));

            if (paths[path_index]->arc_direction == CW) // 顺时针
            {
                if ((paths[path_index]->start_pos.x < paths[path_index]->end_pos.x) && (paths[path_index]->start_pos.y > paths[path_index]->end_pos.y))
                {
                    inp_datas[path_index]->x = paths[path_index]->start_pos.x + inp_datas[path_index]->arc_radius_actual * sin(inp_datas[path_index]->past_arcAngle);
                    inp_datas[path_index]->y = paths[path_index]->start_pos.y - inp_datas[path_index]->arc_radius_actual * (1 - cos(inp_datas[path_index]->past_arcAngle));
                    inp_datas[path_index]->vel_x = inp_datas[path_index]->vel * cos(inp_datas[path_index]->past_arcAngle);
                    inp_datas[path_index]->vel_y = -inp_datas[path_index]->vel * sin(inp_datas[path_index]->past_arcAngle);
                }
                else if ((paths[path_index]->start_pos.x < paths[path_index]->end_pos.x) && (paths[path_index]->start_pos.y < paths[path_index]->end_pos.y))
                {
                    inp_datas[path_index]->x = paths[path_index]->start_pos.x + inp_datas[path_index]->arc_radius_actual * (1 - cos(inp_datas[path_index]->past_arcAngle));
                    inp_datas[path_index]->y = paths[path_index]->start_pos.y + inp_datas[path_index]->arc_radius_actual * sin(inp_datas[path_index]->past_arcAngle);
                    inp_datas[path_index]->vel_x = inp_datas[path_index]->vel * sin(inp_datas[path_index]->past_arcAngle);
                    inp_datas[path_index]->vel_y = inp_datas[path_index]->vel * cos(inp_datas[path_index]->past_arcAngle);

                }
                else if ((paths[path_index]->start_pos.x > paths[path_index]->end_pos.x) && (paths[path_index]->start_pos.y < paths[path_index]->end_pos.y))
                {
                    inp_datas[path_index]->x = paths[path_index]->start_pos.x - inp_datas[path_index]->arc_radius_actual * sin(inp_datas[path_index]->past_arcAngle);
                    inp_datas[path_index]->y = paths[path_index]->start_pos.y + inp_datas[path_index]->arc_radius_actual * (1 - cos(inp_datas[path_index]->past_arcAngle));
                    inp_datas[path_index]->vel_x = -inp_datas[path_index]->vel * cos(inp_datas[path_index]->past_arcAngle);
                    inp_datas[path_index]->vel_y = inp_datas[path_index]->vel * sin(inp_datas[path_index]->past_arcAngle);

                }
                else if ((paths[path_index]->start_pos.x > paths[path_index]->end_pos.x) && (paths[path_index]->start_pos.y > paths[path_index]->end_pos.y))
                {
                    inp_datas[path_index]->x = paths[path_index]->start_pos.x - inp_datas[path_index]->arc_radius_actual * (1 - cos(inp_datas[path_index]->past_arcAngle));
                    inp_datas[path_index]->y = paths[path_index]->start_pos.y - inp_datas[path_index]->arc_radius_actual * sin(inp_datas[path_index]->past_arcAngle);
                    inp_datas[path_index]->vel_x = -inp_datas[path_index]->vel * sin(inp_datas[path_index]->past_arcAngle);
                    inp_datas[path_index]->vel_y = -inp_datas[path_index]->vel * cos(inp_datas[path_index]->past_arcAngle);

                }
            }
            else // 逆时针
            {
                if ((paths[path_index]->start_pos.x > paths[path_index]->end_pos.x) && (paths[path_index]->start_pos.y < paths[path_index]->end_pos.y))
                {
                    inp_datas[path_index]->x = paths[path_index]->start_pos.x - inp_datas[path_index]->arc_radius_actual * (1 - cos(inp_datas[path_index]->past_arcAngle));
                    inp_datas[path_index]->y = paths[path_index]->start_pos.y + inp_datas[path_index]->arc_radius_actual * sin(inp_datas[path_index]->past_arcAngle);
                    inp_datas[path_index]->vel_x = -inp_datas[path_index]->vel * sin(inp_datas[path_index]->past_arcAngle);
                    inp_datas[path_index]->vel_y = +inp_datas[path_index]->vel * cos(inp_datas[path_index]->past_arcAngle);

                }
                else if ((paths[path_index]->start_pos.x > paths[path_index]->end_pos.x) && (paths[path_index]->start_pos.y > paths[path_index]->end_pos.y))
                {
                    inp_datas[path_index]->x = paths[path_index]->start_pos.x - inp_datas[path_index]->arc_radius_actual * sin(inp_datas[path_index]->past_arcAngle);
                    inp_datas[path_index]->y = paths[path_index]->start_pos.y - inp_datas[path_index]->arc_radius_actual * (1 - cos(inp_datas[path_index]->past_arcAngle));
                    inp_datas[path_index]->vel_x = -inp_datas[path_index]->vel * cos(inp_datas[path_index]->past_arcAngle);
                    inp_datas[path_index]->vel_y = -inp_datas[path_index]->vel * sin(inp_datas[path_index]->past_arcAngle);

                }
                else if ((paths[path_index]->start_pos.x < paths[path_index]->end_pos.x) && (paths[path_index]->start_pos.y > paths[path_index]->end_pos.y))
                {
                    inp_datas[path_index]->x = paths[path_index]->start_pos.x + inp_datas[path_index]->arc_radius_actual * (1 - cos(inp_datas[path_index]->past_arcAngle));
                    inp_datas[path_index]->y = paths[path_index]->start_pos.y - inp_datas[path_index]->arc_radius_actual * sin(inp_datas[path_index]->past_arcAngle);
                    inp_datas[path_index]->vel_x = inp_datas[path_index]->vel * sin(inp_datas[path_index]->past_arcAngle);
                    inp_datas[path_index]->vel_y = -inp_datas[path_index]->vel * cos(inp_datas[path_index]->past_arcAngle);

                }
                else if ((paths[path_index]->start_pos.x < paths[path_index]->end_pos.x) && (paths[path_index]->start_pos.y < paths[path_index]->end_pos.y))
                {
                    inp_datas[path_index]->x = paths[path_index]->start_pos.x + inp_datas[path_index]->arc_radius_actual * sin(inp_datas[path_index]->past_arcAngle);
                    inp_datas[path_index]->y = paths[path_index]->start_pos.y + inp_datas[path_index]->arc_radius_actual * (1 - cos(inp_datas[path_index]->past_arcAngle));
                    inp_datas[path_index]->vel_x = inp_datas[path_index]->vel * cos(inp_datas[path_index]->past_arcAngle);
                    inp_datas[path_index]->vel_y = inp_datas[path_index]->vel * sin(inp_datas[path_index]->past_arcAngle);

                }
            }
        } // END
        else
        {
            inp_datas[path_index]->arc_interpolation_is_done = TRUE; // 圆弧插补完成标志
            // inp_datas[path_index]->x = paths[path_index]->end_pos.x;
            // inp_datas[path_index]->y = paths[path_index]->end_pos.y;

        }
    } // END 圆弧插补


    // 机器人自转角度插补 无论是在直线还是圆弧都可以进行旋转插补
    if (inp_datas[path_index]->residue_theta > Round_Error_T)
    {
        dtheta = 0;
        pre_w_z = inp_datas[path_index]->w_z;
        inp_datas[path_index]->brake_theta = navi_abs(pow(inp_datas[path_index]->w_z, 2) - pow(paths[path_index]->w_end, 2)) / (W_ACC * 2);
        // 剩余角度大于理论刹车角度 将速度逐步的调整为给定的目标(最大)角速度 可以加速、减速、匀速
        if (inp_datas[path_index]->residue_theta - inp_datas[path_index]->brake_theta > Round_Error)
        {
            if (inp_datas[path_index]->w_z < paths[path_index]->w_target) // 加速
            {
                inp_datas[path_index]->w_z += W_ACC * Time_Step;
                high_constrain(inp_datas[path_index]->w_z, paths[path_index]->w_target);
                dtheta = navi_abs(pow(inp_datas[path_index]->w_z, 2) - pow(pre_w_z, 2)) / (2 * W_ACC);
            }
            else if (inp_datas[path_index]->w_z > paths[path_index]->w_target) // 减速
            {
                inp_datas[path_index]->w_z -= W_ACC * Time_Step;
                low_constrain(inp_datas[path_index]->w_z, paths[path_index]->w_target);
                dtheta = navi_abs(pow(inp_datas[path_index]->w_z, 2) - pow(pre_w_z, 2)) / (2 * W_ACC);
            }
            else // 匀速(达到目标速度)
            {
                inp_datas[path_index]->w_z = paths[path_index]->w_target;
                dtheta = navi_abs(inp_datas[path_index]->w_z) * Time_Step;
            }
        }
        // 剩余角度小于理论刹车角度 将角速度慢慢调整为 给定的终点角速度
        else
        {
            // 更改加速度，使小车可以在终点速度恰好为要求的速度
            w_acc_adjusted = navi_abs(pow(inp_datas[path_index]->w_z, 2) - pow(paths[path_index]->w_end, 2)) / (2 * inp_datas[path_index]->residue_theta);
            if (inp_datas[path_index]->w_z < paths[path_index]->w_end) // 加速
            {
                inp_datas[path_index]->w_z += w_acc_adjusted * Time_Step;
                high_constrain(inp_datas[path_index]->w_z, paths[path_index]->w_end);
                dtheta = navi_abs(pow(inp_datas[path_index]->w_z, 2) - pow(pre_w_z, 2)) / (2 * w_acc_adjusted);
            }
            else if (inp_datas[path_index]->w_z > paths[path_index]->w_end) // 减速
            {
                inp_datas[path_index]->w_z -= w_acc_adjusted * Time_Step;
                low_constrain(inp_datas[path_index]->w_z, paths[path_index]->w_end);
                dtheta = navi_abs(pow(inp_datas[path_index]->w_z, 2) - pow(pre_w_z, 2)) / (2 * w_acc_adjusted);
            }
            else // 匀速(达到目标速度)
            {
                inp_datas[path_index]->w_z = paths[path_index]->w_end;
                dtheta = inp_datas[path_index]->w_z * Time_Step;
            }
        }

        // 更新已经插补过的角度(rad) 已经插补的角度 总角度 当前角度 都是有正有负 有方向的量
        inp_datas[path_index]->past_theta += dtheta;
        inp_datas[path_index]->residue_theta = inp_datas[path_index]->total_theta - inp_datas[path_index]->past_theta; // 剩余角度用的是绝对角度 正值

        if (inp_datas[path_index]->residue_theta <= Round_Error_T)
        {
            inp_datas[path_index]->past_theta = inp_datas[path_index]->total_theta;
            inp_datas[path_index]->residue_theta = 0;
            inp_datas[path_index]->w_z = paths[path_index]->w_end;
            inp_datas[path_index]->theta_interpolation_is_done = TRUE; // 自转角度插补完成 标志
        }
        else // 插补还未结束
        {
            if (paths[path_index]->rotate_direction == CCW)
                inp_datas[path_index]->theta = -inp_datas[path_index]->past_theta + paths[path_index]->start_pos.theta; // 计算当前点 应该转到的角度
            else
            {
                inp_datas[path_index]->theta = inp_datas[path_index]->past_theta + paths[path_index]->start_pos.theta; // 计算当前点 应该转到的角度
            }
        }

    } // END 机器人自转角度插补
    else
    {
        inp_datas[path_index]->theta_interpolation_is_done = TRUE; // 自转角度插补完成 标志
        inp_datas[path_index]->theta = paths[path_index]->end_pos.theta;
    }
    // 判断插补是否结束 直线插补和圆弧插补不可能同时进行 所以 直线+转角 圆弧+转件 完成就认为所有的插补都完成了
    if ((inp_datas[path_index]->line_interpolation_is_done == TRUE || inp_datas[path_index]->arc_interpolation_is_done == TRUE) &&
        (inp_datas[path_index]->theta_interpolation_is_done == TRUE))
    {
        inp_datas[path_index]->both_interpolation_is_done = TRUE;
        if (path_index == paths.size() - 1)
        {
            ret.all_paths_interpolation_is_done = TRUE;
        }

    }
    ret.x = inp_datas[path_index]->x;
    ret.y = inp_datas[path_index]->y;
    ret.theta = inp_datas[path_index]->theta;
    ret.w_z = inp_datas[path_index]->w_z;
    ret.vel = inp_datas[path_index]->vel;
    ret.vel_x = inp_datas[path_index]->vel_x;
    ret.vel_y = inp_datas[path_index]->vel_y;

    if (ret.all_paths_interpolation_is_done == TRUE)
    {
        clearPaths();
    }

    return ret;

}

void Navigation::clearPaths()
{
    // 释放所有动态分配的内存
    for (auto& inp_data : inp_datas) {
        if (inp_data != nullptr) {
            vPortFree(inp_data);  // 释放内存
            inp_data = nullptr;
        }
    }
    inp_datas.clear();
    paths.clear();

    // 重置其他变量
    pre_v = 0;
    pre_w_z = 0;
    dl = 0;
    ds = 0;
    dtheta = 0;
    acc_adjusted = 0;
    w_acc_adjusted = 0;
    Last_Time = 0;
    path_index = 0;
}
