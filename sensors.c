#include "sensors.h"
#include "ssi.h"
#include <stdint.h>
#include <math.h>
#include <libopencm3/stm32/gpio.h>

/* Module-global (exported) variables */
float pos_active;
float pos_passive;

float vel_active;
float vel_passive;

int pos_active_ring; // DEBUG

/* Parameters */
const float K_POS_ACTIVE_CONV_FACTOR = 2.f*3.14159265359f/4096.f/64.f;
const float K_POS_PASSIVE_CONV_FACTOR = 2.f*3.14159265359f/4096.f/64.f;

const float K_POS_ACTIVE_OFFSET = -4.513f;

const float K_SAMPLING_FREQ = 168e6f/2.f/2048.f/10.f;

const float K_VEL_ACTIVE_CONV_FACTOR = 2.f*3.14159265359f/4096.f/64.f * 168e6f/2.f/2048.f/10.f;
                                       //K_POS_ACTIVE_CONV_FACTOR * K_SAMPLING_FREQ;
const float K_VEL_PASSIVE_CONV_FACTOR = 2.f*3.14159265359f/4096.f/64.f * 168e6f/2.f/2048.f/10.f;
                                       //K_POS_PASSIVE_CONV_FACTOR * K_SAMPLING_FREQ;

const int K_RING_OFFSET = 55;

const float K_VEL_SAMPLING_FACTOR = 3.f/2.f;

/* Module-local variables */
int pos_active_raw;
int pos_passive_raw;

int old_pos_active_ring = 0;
int old_pos_active_ctr = 0;
int rev_active = 0;

int old_pos_passive_ring = 0;
int off_passive = 0;

int lb_last = 0;

float old_vel_active, old_vel_passive;

int lb_read(void)
{
    return !!gpio_get(GPIOA, GPIO8);
}

void sensors_init(void)
{
    as5xxx_data_t data;
    ssi_read_active_axis(); // dummy read
    data = ssi_read_active_axis();
    
    old_pos_active_ctr = data.position;
}

void sensors_update(void)
{
    as5xxx_data_t data;
    
    /*** Active axis position/velocity ***/
    data = ssi_read_active_ring();
    pos_active_ring = data.position;
    
    data = ssi_read_active_axis();
    int pos_active_ctr = data.position;
    
    if((pos_active_ctr-old_pos_active_ctr) > 2048) {
        rev_active--;
    }
    if((pos_active_ctr-old_pos_active_ctr) < -2048) {
        rev_active++;
    }
    
    old_pos_active_ctr = pos_active_ctr;
    
    int ring_id = (pos_active_ctr - pos_active_ring/64 + K_RING_OFFSET)/64;
    
    // Calculate active axis position (in rad)
    pos_active_raw = rev_active * (4096*64) + ring_id * 4096 + pos_active_ring;
    pos_active = ((float)pos_active_raw) * K_POS_ACTIVE_CONV_FACTOR + K_POS_ACTIVE_OFFSET;
    
    int vel_active_raw = pos_active_ring - old_pos_active_ring;
    if(vel_active_raw > 2048) {
        vel_active_raw -= 4096;
    }
    if(vel_active_raw < -2048) {
        vel_active_raw += 4096;
    }
    old_pos_active_ring = pos_active_ring;
    
    // Calculate active axis velocity (in rad/s)
    vel_active = ((float)vel_active_raw) * K_VEL_ACTIVE_CONV_FACTOR;
    
    // Compensate for sensor sampling jitter
    if(fabsf(old_vel_active - vel_active * K_VEL_SAMPLING_FACTOR) < fabsf(old_vel_active - vel_active)) {
        vel_active *= K_VEL_SAMPLING_FACTOR;
    }
    old_vel_active = vel_active;
    
    /*** Passive axis position/velocity ***/
    data = ssi_read_passive_ring();
    int pos_passive_ring = data.position;
    
    // Detect segment transition, and update offset accordingly
    int vel_passive_raw = pos_passive_ring - old_pos_passive_ring;
    if(vel_passive_raw > 2048) {
        vel_passive_raw -= 4096;
        off_passive -= 4096;
    }
    if(vel_passive_raw < -2048) {
        vel_passive_raw += 4096;
        off_passive += 4096;
    }
    old_pos_passive_ring = pos_passive_ring;
    
    // Calculate passive axis position (in rad)
    pos_passive = ((float)(pos_passive_ring + off_passive)) * K_POS_PASSIVE_CONV_FACTOR;
    
    // Calculate passive axis velocity (in rad/s)
    vel_passive = ((float)vel_passive_raw) * K_VEL_PASSIVE_CONV_FACTOR;
    
    // Compensate for sensor sampling jitter
    if(fabsf(old_vel_passive - vel_passive * K_VEL_SAMPLING_FACTOR) < fabsf(old_vel_passive - vel_passive)) {
        vel_passive *= K_VEL_SAMPLING_FACTOR;
    }
    old_vel_passive = vel_passive;
    
    // Use light barrier to correct passive axis offset
    int lb_status = lb_read();
    if(lb_last && !lb_status && vel_passive >= 0.5f) {
        off_passive = 0;
    }
    lb_last = lb_status;
}
