#ifndef _TRAP_TRAJ_H
#define _TRAP_TRAJ_H



/*
    // 创建一个 TrapezoidalTrajectory 实例
    TrapezoidalTrajectory traj;

    // 定义初始和目标条件
    float Xi = 0.0f;   // 初始位置
    float Vi = 0.0f;   // 初始速度
    float Xf = 10.0f;  // 目标位置
    float Vmax = 2.0f; // 最大速度
    float Amax = 1.0f; // 最大加速度
    float Dmax = 1.0f; // 最大减速度

    // 规划梯形轨迹
    bool success = traj.planTrapezoidal(Xf, Xi, Vi, Vmax, Amax, Dmax);
    if (!success) {
        std::cout << "Failed to plan trajectory!" << std::endl;
        return -1;
    }
    while(1)
    {
        TrapezoidalTrajectory::Step_t step = traj.eval(traj.t_);
        if (traj.t_ <= traj.Tf_)
        {
            car.Tar_Pos_X = step.Y;
            car.Vel_X_FF = step.Yd;
            traj.t_ += (float)TIME_STEP / 1000;
        }
    }
*/



class TrapezoidalTrajectory
{
public:
    struct Config_t
    {
        float vel_limit = 2.0f;   // [turn/s]
        float accel_limit = 0.5f; // [turn/s^2]
        float decel_limit = 0.5f; // [turn/s^2]
    };

    struct Step_t
    {
        float Y;
        float Yd;
        float Ydd;
    };

    bool planTrapezoidal(float Xf, float Xi, float Vi,
        float Vmax, float Amax, float Dmax);
    Step_t eval(float t);


    Config_t config_;

    float Xi_;
    float Xf_;
    float Vi_;

    float Ar_;
    float Vr_;
    float Dr_;

    float Ta_;
    float Tv_;
    float Td_;
    float Tf_;

    float yAccel_;

    float t_;
    void reset();
};

#endif