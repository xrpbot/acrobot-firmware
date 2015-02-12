#include "current.h"
#include "quadrature.h"
#include "pwm.h"
#include <libopencm3/stm32/adc.h>
#include <math.h>

// *** Controller constants ***
const float K_ENCODER_OFFSET = 0.60f;     // motor encoder offset
const float K_CURRENT1_OFFSET = 2044.f;   // offset of "U" current sensor in ADC units
const float K_CURRENT2_OFFSET = 2061.f;   // offset of "W" current sensor in ADC units

const float K_IQ_CLAMP = 50000.f;

const float K_PI = M_PI;                  // floating point version of M_PI

// *** Global variables ***
volatile float current_uq;
volatile float current_ud;

volatile float current_Iq;
volatile float current_Id;

volatile float current_Iq_des;
volatile float current_Id_des;

volatile float current_Iq_int;
volatile float current_Id_int;

void set_des_current(float Iq, float Id)
{
    current_Iq_des = Iq;
    current_Id_des = Id;
}

#define K_RINGBUF_SIZE 8
uint16_t pos_ringbuf[K_RINGBUF_SIZE] = { 0 };
int pos_ringbuf_idx = 0;

/* This is called every time the PWM timer updates, i.e. every 24.38 µs.
   
   Note that the motor encoder (AS5040) has a 192 µs latency (approx. 8 cycles),
   which begins to matter at high revolution frequencies.
*/
void current_ctrl(void)
{
    uint16_t pos = quadrature_read_pos_tim3();
    uint16_t last_pos = pos_ringbuf[pos_ringbuf_idx];
    pos_ringbuf[pos_ringbuf_idx] = pos;
    pos_ringbuf_idx++;
    pos_ringbuf_idx %= K_RINGBUF_SIZE;
    
    float velf = pos - last_pos;
    //if(velf > 512.f) velf_fast -= 1024.f;
    //if(velf < -512.f) velf_fast += 1024.f;
    
    float posf = pos;
    posf += velf;
    float phi = posf / 512.f * 7.f * K_PI + K_ENCODER_OFFSET;
    
    float s_phi, c_phi;
    sincosf(phi, &s_phi, &c_phi);
    
    // current control
    float I1 = adc_read_injected(ADC1, 1);
    float I2 = adc_read_injected(ADC2, 1);
    I1 -= K_CURRENT1_OFFSET;
    I2 -= K_CURRENT2_OFFSET;
    
    float Ia = I1;
    float Ib = (I1 + 2.f*I2)/sqrtf(3.f);
    
    float Iq_cur = c_phi*Ia - s_phi*Ib;
    float Id_cur = s_phi*Ia + c_phi*Ib;
    
    float Iq_int = current_Iq_int;  // load variable declared volatile
    float uq = (current_Iq_des - Iq_cur) + 0.1f*Iq_int;
    Iq_int += current_Iq_des - Iq_cur;
    
    if(Iq_int > K_IQ_CLAMP)  Iq_int = K_IQ_CLAMP;
    if(Iq_int < -K_IQ_CLAMP) Iq_int = -K_IQ_CLAMP;
    current_Iq_int = Iq_int;   // store variable
    
    float Id_int = current_Id_int;  // load variable declared volatile
    float ud = (current_Id_des - Id_cur) + 0.1f*Id_int;
    Id_int += current_Id_des - Id_cur;
    
    if(Id_int > 20000.f)  Id_int = 20000.f;
    if(Id_int < -20000.f) Id_int = -20000.f;
    current_Id_int = Id_int;   // store variable
    
    // update debug info
    current_Iq = Iq_cur;
    current_Id = Id_cur;
    
    current_uq = uq;
    current_ud = ud;
    
    // FIXME
    if(uq > 4700.f) uq = 4700.f;
    if(uq < -4700.f) uq = -4700.f;
    
    int32_t ua, ub;
    ua = c_phi*uq + s_phi*ud;
    ub = s_phi*uq - c_phi*ud;
    
    set_pwm_values(ua, ub);
}
