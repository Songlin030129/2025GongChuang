#include "PID.h"

PIDController::PIDController(float P, float I, float D, float ramp, float limit, float deadzone)
    : P(P), I(I), D(D), output_ramp(ramp),
    output_limit(limit),
    DeadZone(deadzone),
    error_prev(0.0f), output_prev(0.0f), integral_prev(0.0f),
    output_value(0.0f), Enable(1)
{
}
// PID controller function
float PIDController::Cal(float error, float ff)
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

    if (Enable == true)
    {
        // if deadzone defined
        // if the error is within the deadzone, set it to zero

        if (DeadZone > 0)
        {
            if (error < DeadZone && error > -DeadZone)
            {
                error = 0;
            }
        }
        Error = error;
        // u(s) = (P + I/s + Ds)e(s)
        // Discrete implementations
        // proportional part
        // u_p  = P *e(k)
        float proportional = P * error;
        // Tustin transform of the integral part
        // u_ik = u_ik_1  + I*Ts/2*(ek + ek_1)
        float integral = integral_prev + I * Ts * 0.5f * (error + error_prev);
        // antiwindup - limit the output
        if (output_limit > 0)
            integral = _constrain(integral, -output_limit, output_limit);
        // Discrete derivation
        // u_dk = D(ek - ek_1)/Ts
        float derivative = D * (error - error_prev) / Ts;

        // sum all the components
        float output = proportional + integral + derivative + ff;
        // antiwindup - limit the output variable
        if (output_limit > 0)
            output = _constrain(output, -output_limit, output_limit);

        // if output ramp defined
        if (output_ramp > 0)
        {
            // limit the acceleration by ramping the output
            float output_rate = (output - output_prev) / Ts;
            if (output_rate > output_ramp)
                output = output_prev + output_ramp * Ts;
            else if (output_rate < -output_ramp)
                output = output_prev - output_ramp * Ts;
        }
        // saving for the next pass
        integral_prev = integral;
        output_prev = output;
        error_prev = error;
        output_value = output;
        return output;
    }
    else
    {
        error_prev = 0;
        output_prev = 0;
        integral_prev = 0;
        output_value = 0;
        return 0;
    }
}

void PIDController::reset()
{
    integral_prev = 0.0f;
    output_prev = 0.0f;
    error_prev = 0.0f;
}
