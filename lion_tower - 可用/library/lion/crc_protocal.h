#ifndef _CRC_PROTOCAL_H_
#define _CRC_PROTOCAL_H_

#include "main.h"

#define UART_MSG_OFFSET_SOF               0x00
#define UART_MSG_OFFSET_DATA_LENGTH       0x01
#define UART_MSG_OFFSET_SEQ               0x03
#define UART_MSG_OFFSET_CRC8              0x04
#define UART_MSG_OFFSET_CMD_ID            0x05
#define UART_MSG_OFFSET_DATA              0x07

uint8_t get_crc8_check_sum(uint8_t *,uint16_t,uint8_t);
uint16_t verify_crc8_check_sum(uint8_t *, uint16_t);
void append_crc8_check_sum(uint8_t *, uint16_t);

uint16_t get_crc16_check_sum(uint8_t *,uint32_t,uint16_t);
uint32_t verify_crc16_check_sum(uint8_t *, uint32_t );
void append_crc16_check_sum(uint8_t *,uint32_t);

uint16_t uart_msg_pack_handler(uint8_t * tx_buffer, 
                               uint8_t sof, 
                               uint16_t data_length, 
                               uint8_t seq, 
                               uint16_t cmd_id, 
                               const void * data);
#endif
