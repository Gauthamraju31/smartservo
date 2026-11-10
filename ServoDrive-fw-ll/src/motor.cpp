#include "motor.h"
#include "tim.h"

float Motor::CalcDceOutput(float _inputPos, float _inputVel)
{
    float errorPos = _inputPos - dce.setPointPos;
    float errorVel = _inputVel - dce.setPointVel;
    float deltaPos = errorPos - dce.lastError;
    dce.lastError = errorPos;

    // Anti-windup: only accumulate integral when close to target to prevent massive step overshoot
    if (errorPos > -15.0f && errorPos < 15.0f)
    {
        dce.integralPos += errorPos;
        if (dce.integralPos > DCE_INTEGRAL_LIMIT) dce.integralPos = DCE_INTEGRAL_LIMIT;
        else if (dce.integralPos < -DCE_INTEGRAL_LIMIT) dce.integralPos = -DCE_INTEGRAL_LIMIT;
    }
    else
    {
        dce.integralPos = 0.0f;
    }

    dce.integralVel += errorVel;
    if (dce.integralVel > DCE_INTEGRAL_LIMIT) dce.integralVel = DCE_INTEGRAL_LIMIT;
    else if (dce.integralVel < -DCE_INTEGRAL_LIMIT) dce.integralVel = -DCE_INTEGRAL_LIMIT;

    dce.output = dce.kp * errorPos +
                 dce.ki * dce.integralPos + dce.kv * dce.integralVel +
                 dce.kd * deltaPos;

    // Friction deadband compensation:
    // Overcome SG90 gear stiction so horn reaches within 1.0 degree of target.
    // Outside 1.0 degree deadband, add minimum stiction feedforward.
    // Within 1.0 degree deadband, clear output to prevent buzzing/hunting.
    if (errorPos > 1.0f)
    {
        dce.output += 210.0f;
    }
    else if (errorPos < -1.0f)
    {
        dce.output -= 210.0f;
    }
    else
    {
        dce.output = 0.0f;
    }

    if (dce.output > limitTorque) dce.output = limitTorque;
    else if (dce.output < -limitTorque) dce.output = -limitTorque;

    return dce.output;
}


void Motor::SetPwm(int16_t _pwm)
{
    if (isEnabled)
    {
        // Clamp commanded PWM to valid range
        if (_pwm > 1000) _pwm = 1000;
        else if (_pwm < -1000) _pwm = -1000;

        // Slew-rate limiter: maximum delta PWM of 80 per 5ms loop cycle (~60ms full ramp)
        // Eliminates brownout current inrush on the 5V power rail
        const int16_t MAX_PWM_STEP = 80;
        int16_t diff = _pwm - currentPwm;
        if (diff > MAX_PWM_STEP) currentPwm += MAX_PWM_STEP;
        else if (diff < -MAX_PWM_STEP) currentPwm -= MAX_PWM_STEP;
        else currentPwm = _pwm;

        if (currentPwm >= 0)
        {
            LL_TIM_OC_SetCompareCH1(TIM3, 0);
            LL_TIM_OC_SetCompareCH2(TIM3, (uint32_t)currentPwm);
        } else
        {
            LL_TIM_OC_SetCompareCH1(TIM3, (uint32_t)(-currentPwm));
            LL_TIM_OC_SetCompareCH2(TIM3, 0);
        }
    } else
    {
        currentPwm = 0;
        LL_TIM_OC_SetCompareCH1(TIM3, 0);
        LL_TIM_OC_SetCompareCH2(TIM3, 0);
    }
}


void Motor::SetTorqueLimit(float _percent)
{
    if (_percent > 1)_percent = 1;
    else if (_percent < 0)_percent = 0;

    limitTorque = _percent * 1000;
}


void Motor::UpdateVelocity()
{
    velocity = angle - lastAngle;
    lastAngle = angle;
}


void Motor::SetEnable(bool _enable)
{
    isEnabled = _enable;
}

