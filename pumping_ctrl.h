#ifndef XRP_PUMPING_CTRL_H
#define XRP_PUMPING_CTRL_H

void pumping_set_params(float gain, float offset, float scale);
float pumping_ctrl_step(float pos_active, float vel_active, float vel_passive);

#endif
