#include "pumping_ctrl.h"
#include <math.h>

static const float K_P_GAIN = 1000.f;
static const float K_D_GAIN = 50.f;

static const float K_ALPHA = 0.01f;

static float p_gain = 0.f;
static float p_offset = 0.f;
static float p_scale = 0.f;

static float vel_passive_filt = 0.f;

void pumping_set_params(float gain, float offset, float scale)
{
    p_gain = gain;
    p_offset = offset;
    p_scale = scale;
}

float pumping_ctrl_step(float pos_active, float vel_active, float vel_passive)
{
    vel_passive_filt = K_ALPHA*vel_passive + (1.f-K_ALPHA)*vel_passive_filt;
    float pos_des = p_gain*atanf(vel_passive_filt*p_scale) + p_offset;
    float Iq_des = K_D_GAIN*(-vel_active) + K_P_GAIN*(pos_des - pos_active);
    
    return Iq_des;
}
