#include "transport.h"

uint32_t calc_crc32(const uint8_t *data, size_t data_len);

#define MSG_START    0xF1
#define MSG_ESCAPE   0xF2
#define MSG_END      0xF3

volatile enum msgstate_t msg_state = MSGSTATE_WAIT_START;
volatile uint8_t msg_data[MSG_MAX_LEN];
volatile size_t msg_len;
volatile uint8_t msg_escape = 0;

volatile uint32_t err_msg_overlen = 0;   // messages discarded due to excessive length
volatile uint32_t err_msg_busy = 0;      // bytes discarded while processing last message
volatile uint32_t err_msg_crc = 0;       // messages discarded due to CRC mismatch

/* Code derived from output of pycrc (http://www.tty1.net/pycrc/index_en.html) */
static const uint32_t crc_table[16] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
};

const uint32_t CRC_OK = 0xDEBB20E3;

uint32_t calc_crc32(const uint8_t *data, size_t data_len)
{
    unsigned int tbl_idx;
    
    uint32_t crc = 0xffffffff;
    while (data_len--) {
        tbl_idx = crc ^ (*data >> (0 * 4));
        crc = crc_table[tbl_idx & 0x0f] ^ (crc >> 4);
        tbl_idx = crc ^ (*data >> (1 * 4));
        crc = crc_table[tbl_idx & 0x0f] ^ (crc >> 4);

        data++;
    }
    return crc;
}

void tp_process_byte(uint8_t ch)
{
    switch(msg_state) {
        case MSGSTATE_WAIT_START:
            if(ch == MSG_START) {
                msg_state = MSGSTATE_READ;
                msg_len = 0;
            }
        break;
        case MSGSTATE_READ:
            if(ch == MSG_END) {
                msg_state = MSGSTATE_WAIT_PROC;
            } else if(ch == MSG_ESCAPE) {
                msg_escape = 1;
            } else {
                if(msg_len < MSG_MAX_LEN) {
                    if(msg_escape) {
                        ch ^= 0x80;
                    }
                    msg_escape = 0;
                    msg_data[msg_len] = ch;
                    msg_len++;
                } else {
                    err_msg_overlen++;
                    msg_state = MSGSTATE_WAIT_START;
                }
            }
        break;
        case MSGSTATE_WAIT_PROC:
            err_msg_busy++;
        break;
    
    }
}

uint8_t tp_check_msg(void)
{
    if(msg_state != MSGSTATE_WAIT_PROC) {
        return 0;
    }
    
    if(calc_crc32(msg_data, msg_len) != CRC_OK) {
        err_msg_crc++;
        msg_state = MSGSTATE_WAIT_START;
        return 0;
    }
    
    return 1;
}

void tp_msg_done(void)
{
    if(msg_state == MSGSTATE_WAIT_PROC) {
        msg_state = MSGSTATE_WAIT_START;
    }
}
