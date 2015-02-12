#include "trajectory.h"
#include "traj_data.h"
#include "math.h"

const float t_end = 0.1*sizeof(pos_data)/sizeof(pos_data[0]);

float get_des_pos(float t)
{
    if(t <= 0.) return 0.;
    if(t >= t_end) return 0.;
    
    int i = floor(t / tstep);
    const float u = t/tstep - i;
    
    const float y1 = pos_data[i];
    const float y2 = pos_data[i+1];
    
    const float a = vel_data[i]*tstep - (y2 - y1);
    const float b = (y2 - y1) - vel_data[i+1]*tstep;
    
    return (1.-u)*y1 + u*y2 + u*(1.-u)*(a*(1.-u) + b*u);
}

float get_des_vel(float t)
{
    if(t <= 0.) return 0.;
    if(t >= t_end) return 0.;
    
    int i = floor(t / tstep);
    const float u = t/tstep - i;
    
    const float y1 = pos_data[i];
    const float y2 = pos_data[i+1];
    
    const float a = vel_data[i]*tstep - (y2 - y1);
    const float b = (y2 - y1) - vel_data[i+1]*tstep;
    
    return (-y1 + y2 + a - 4.*a*u + 2.*b*u + 3.*a*u*u - 3.*b*u*u)/tstep;
}
