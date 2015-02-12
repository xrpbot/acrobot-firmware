#ifndef XRP_SENSORS_H
#define XRP_SENSORS_H

void sensors_init(void);
void sensors_update(void);

int lb_read(void);

extern float pos_active;   // active axis position (in rad)
extern float pos_passive;  // passive axis position (in rad)

extern float vel_active;   // active axis velocity (in rad/s)
extern float vel_passive;  // passive axis velocity (in rad/s)

extern int pos_active_ring; // DEBUG

#endif
