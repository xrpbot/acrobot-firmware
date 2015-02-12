#ifndef XRP_CURRENT_H
#define XRP_CURRENT_H

extern volatile float current_uq;
extern volatile float current_ud;

extern volatile float current_Iq;   // actual Iq
extern volatile float current_Id;   // actual Id

extern volatile float current_Iq_des;
extern volatile float current_Id_des;

extern volatile float current_Iq_int;  // Iq error integral for PI controller
extern volatile float current_Id_int;  // Id error integral for PI controller

void current_ctrl(void);
void set_des_current(float Iq, float Id);

#endif
