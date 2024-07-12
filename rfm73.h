/*
 * rfm73.h
 *
 *  Created on: 10 kwi 2024
 *      Author: mariusz
 */

#ifndef RFM73_RFM73_H_
#define RFM73_RFM73_H_

#include <stdint.h>

typedef enum{
	RFM73_POWER_SAVE_NORMAL,
	RFM73_POWER_SAVE_STANDBY,
	RFM73_POWER_SAVE_POWER_DOWN
}RFM73_power_save_t;

typedef enum{
	RFM73_SEND_OK,
	RFM73_SEND_MAX_RETR,
	RFM73_SEND_POWER_DOWN,
	RFM73_SEND_WRONG_MODE,
	RFM73_SEND_PENDING,
	RFM73_SEND_TIMEOUT
}RFM73_send_t;

typedef enum{
	RFM73_RECV_PENDING,
	RFM73_RECV_NONE
}RFM73_recv_t;

typedef enum{
	RFM73_ACK,
	RFM73_ACK_FORCE,
	RFM73_NOACK,
	RFM73_NOACK_FORCE
}RFM73_ack_t;

typedef enum{
	RFM73_MODE_PTX			= 0b00000000,
	RFM73_MODE_PRX			= 0b00000001
}RFM73_mode_t;

typedef enum{
	RFM73_CRC_1B 			= 0b00000000,
	RFM73_CRC_2B 			= 0b00000100
}RFM73_crc_t;

typedef enum{
	RFM73_BITRATE_1MBPS 	= 0b00000000,
	RFM73_BITRATE_2MBPS		= 0b00001000,
	RFM73_BITRATE_250KBPS 	= 0b00100000
}RFM73_bitrate_t;

typedef enum{
	RFM73_POWER_NEG10DBM 	= 0b00000000,
	RFM73_POWER_NEG5DBM 	= 0b00000010,
	RFM73_POWER_0DBM		= 0b00000100,
	RFM73_POWER_5DBM		= 0b00000110
}RFM73_tx_power_t;

typedef enum{
	RFM73_RETR_DELAY_250US  = 0b00000000,
	RFM73_RETR_DELAY_500US  = 0b00010000,
	RFM73_RETR_DELAY_750US  = 0b00100000,
	RFM73_RETR_DELAY_1000US = 0b00110000,
	RFM73_RETR_DELAY_1250US = 0b01000000,
	RFM73_RETR_DELAY_1500US = 0b01010000,
	RFM73_RETR_DELAY_1750US = 0b01100000,
	RFM73_RETR_DELAY_2000US = 0b01110000,
	RFM73_RETR_DELAY_2250US = 0b10000000,
	RFM73_RETR_DELAY_2500US = 0b10010000,
	RFM73_RETR_DELAY_2750US = 0b10100000,
	RFM73_RETR_DELAY_3000US = 0b10110000,
	RFM73_RETR_DELAY_3250US = 0b11000000,
	RFM73_RETR_DELAY_3500US = 0b11010000,
	RFM73_RETR_DELAY_3750US = 0b11100000,
	RFM73_RETR_DELAY_4000US = 0b11110000,
}RFM73_retr_delay_t;

typedef enum{
	RFM73ETR_DISABLED	= 0b00000000,
	RFM73_RETR_1		= 0b00000001,
	RFM73_RETR_2		= 0b00000010,
	RFM73_RETR_3		= 0b00000011,
	RFM73_RETR_4		= 0b00000100,
	RFM73_RETR_5		= 0b00000101,
	RFM73_RETR_6		= 0b00000110,
	RFM73_RETR_7		= 0b00000111,
	RFM73_RETR_8		= 0b00001000,
	RFM73_RETR_9		= 0b00001001,
	RFM73_RETR_10		= 0b00001010,
	RFM73_RETR_11		= 0b00001011,
	RFM73_RETR_12		= 0b00001100,
	RFM73_RETR_13		= 0b00001101,
	RFM73_RETR_14		= 0b00001110,
	RFM73_RETR_15		= 0b00001111,
}RFM73_retr_count_t;

typedef enum{
	RFM73_PIPE0 = 0b00000001,
	RFM73_PIPE1 = 0b00000010,
	RFM73_PIPE2 = 0b00000100,
	RFM73_PIPE3 = 0b00001000,
	RFM73_PIPE4 = 0b00010000,
	RFM73_PIPE5 = 0b00100000,
}RFM73_pipe_t;

typedef void(*callback_rfm_delay_ms)(uint16_t ms);
typedef uint8_t(*callback_rfm_spi_transfer)(uint8_t data);
typedef void(*callback_rfm_ce_set)(void);
typedef void(*callback_rfm_ce_clr)(void);
typedef void(*callback_rfm_csn_set)(void);
typedef void(*callback_rfm_csn_clr)(void);

typedef struct{
	RFM73_mode_t mode;

	uint8_t channel;
	RFM73_crc_t crc;
	RFM73_bitrate_t bitrate;
	RFM73_power_save_t power_save;

	union{
		struct{
			RFM73_tx_power_t power;
			RFM73_retr_delay_t retransmit_delay;
			RFM73_retr_count_t retransmit_count;

			uint8_t addr[5];
		}tx;

		struct{
			RFM73_tx_power_t power;

			RFM73_pipe_t pipe_en;
			uint8_t pipe_addr0[5];
			uint8_t pipe_addr1to5[9];
		}rx;
	};

	callback_rfm_delay_ms _delay_ms;
	callback_rfm_spi_transfer _spi_transfer;
	callback_rfm_ce_set _ce_set;
	callback_rfm_ce_clr _ce_clr;
	callback_rfm_csn_set _csn_set;
	callback_rfm_csn_clr _csn_clr;
}RFM73_t;

void rfm73_power_save(RFM73_t* rfm, RFM73_power_save_t ps_mode);
void rfm73_power_save_wake_up(RFM73_t* rfm);

void rfm73_init(RFM73_t* rfm);
RFM73_send_t rfm73_write_payload(RFM73_t* rfm, uint8_t* data, uint8_t size, RFM73_ack_t ack);
RFM73_send_t rfm73_write_ack_payload(RFM73_t* rfm, uint8_t* data, uint8_t size, uint8_t pipe);
RFM73_recv_t rfm73_read_payload(RFM73_t* rfm, uint8_t* data, uint8_t size, uint8_t* pipe, uint8_t* len);

//PTX
//status = rfm73_write_payload(...);
//if(status == RFM73_SEND_OK){
//	// read ACK data
//	status = rfm73_read_payload(...);
//	if(status == RFM73_RECV_PENDING){
//
//	}
//}

#endif /* RFM73_RFM73_H_ */
