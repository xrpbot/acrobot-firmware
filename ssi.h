#ifndef SSI_H
#define SSI_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    int16_t position;
    bool offset_comp_finished;
    bool cordic_overflow;
    bool linearity_alert;
    bool mag_inc;
    bool mag_dec;
    bool valid;
} as5xxx_data_t;

void ssi_setup(void);
as5xxx_data_t ssi_read_motor(void);
as5xxx_data_t ssi_read_lower_axis(void);
as5xxx_data_t ssi_read_center_ring(void);
as5xxx_data_t ssi_read_lower_ring(void);

#endif
