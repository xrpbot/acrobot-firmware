#ifndef XRP_TRANSPORT_H
#define XRP_TRANSPORT_H

#include <stdint.h>
#include <stddef.h>

enum msgstate_t {
    MSGSTATE_WAIT_START, // waiting for start character
    MSGSTATE_READ,       // reading message
    MSGSTATE_WAIT_PROC   // waiting for message to be processed
};

#define MSG_MAX_LEN 64  // maximum length of message

extern volatile enum msgstate_t msg_state;
extern volatile uint8_t msg_data[MSG_MAX_LEN];
extern volatile size_t msg_len;

extern volatile uint32_t err_msg_overlen;   // messages discarded due to excessive length
extern volatile uint32_t err_msg_busy;      // bytes discarded while processing last message
extern volatile uint32_t err_msg_crc;       // messages discarded due to CRC mismatch

void tp_process_byte(uint8_t ch);
uint8_t tp_check_msg(void);
void tp_msg_done(void);

#endif
