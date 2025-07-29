#include "LPF.h"

LowPassFilter::LowPassFilter(float _Tf)
    : Tf(_Tf), y_prev(0.0f)
{
}

float LowPassFilter::operator()(float x)
{
    //获取时间间隔Ts
    uint32_t currentTime = HAL_GetTick();
    if (Last_Time == 0)
    {
        Last_Time = currentTime;
        return 0;
    }
    Ts = (currentTime - Last_Time) / 1000.0f;
    Last_Time = currentTime;

    float alpha = Tf / (Tf + Ts);
    float y = alpha * y_prev + (1.0f - alpha) * x;
    y_prev = y;
    return y;
}
void LowPassFilter::reset()
{
    y_prev = 0.0f;      // 重置上一次的输出值
    Last_Time = 0;      // 重置时间戳，下次调用时重新开始计时
}