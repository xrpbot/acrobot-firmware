#include "vel_ctrl.h"

static float vel_des = 0.f;
static float vel_int = 0.f;

static const float K_P_GAIN = 50.f;
static const float K_I_GAIN = 1.f;

void set_vel_des(float v)
{
    vel_des = v;
}

float vel_ctrl_step(float vel_cur)
{
    float Iq_des = K_P_GAIN*(vel_des - vel_cur) + K_I_GAIN*vel_int;
    vel_int += (vel_des - vel_cur);
    if(vel_int > 1000.f) vel_int = 1000.f;
    if(vel_int < -1000.f) vel_int = -1000.f;
    
    return Iq_des;
}
