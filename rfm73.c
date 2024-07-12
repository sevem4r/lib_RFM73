/*
 * rfm73.c
 *
 *  Created on: 10 kwi 2024
 *      Author: mariusz
 */

#include <stddef.h>

#include "rfm73.h"

enum RFM73_CMD{
	RFM73_CMD_R_REGISTER 			= 0b00000000,
	RFM73_CMD_W_REGISTER 			= 0b00100000,
	RFM73_CMD_R_RX_PAYLOAD 			= 0b01100001,
	RFM73_CMD_W_TX_PAYLOAD 			= 0b10100000,
	RFM73_CMD_FLUSH_TX 				= 0b11100001,
	RFM73_CMD_FLUSH_RX 				= 0b11100010,
	RFM73_CMD_REUSE_TX_PL			= 0b11100011,
	RFM73_CMD_ACTIVATE				= 0b01010000,
	RFM73_CMD_R_RX_PL_WID			= 0b01100000,
	RFM73_CMD_W_ACK_PAYLOAD			= 0b10101000,
	RFM73_CMD_W_TX_PAYLOAD_NO_ACK	= 0b10110000,
	RFM73_CMD_NOP					= 0b11111111
};

enum RFM73_REG{
	RFM73_REG_CONFIG		= 0x00,
	RFM73_REG_EN_AA			= 0x01,
	RFM73_REG_EN_RXADDR		= 0x02,
	RFM73_REG_SETUP_AW		= 0x03,
	RFM73_REG_SETUP_RETR	= 0x04,
	RFM73_REG_RF_CH			= 0x05,
	RFM73_REG_RF_SETUP		= 0x06,
	RFM73_REG_STATUS		= 0x07,
	RFM73_REG_OBSERVE_TX	= 0x08,
	RFM73_REG_RPD			= 0x09,
	RFM73_REG_RX_ADDR_P0	= 0x0A,
	RFM73_REG_RX_ADDR_P1	= 0x0B,
	RFM73_REG_RX_ADDR_P2	= 0x0C,
	RFM73_REG_RX_ADDR_P3	= 0x0D,
	RFM73_REG_RX_ADDR_P4	= 0x0E,
	RFM73_REG_RX_ADDR_P5	= 0x0F,
	RFM73_REG_TX_ADDR		= 0x10,
	RFM73_REG_RX_PW_P0		= 0x11,
	RFM73_REG_RX_PW_P1		= 0x12,
	RFM73_REG_RX_PW_P2		= 0x13,
	RFM73_REG_RX_PW_P3		= 0x14,
	RFM73_REG_RX_PW_P4		= 0x15,
	RFM73_REG_RX_PW_P5		= 0x16,
	RFM73_REG_FIFO_STATUS	= 0x17,
	RFM73_REG_DYNPD			= 0x1C,
	RFM73_REG_FEATURE		= 0x1D,
	RFM73_REG_MAX
};

enum RFM73_FEATURE{
	RFM73_FEATURE_EN_DPL		= 0b00000100,
	RFM73_FEATURE_EN_ACK_PAY	= 0b00000010,
	RFM73_FEATURE_EN_DYN_ACK	= 0b00000001
};

enum RFM73_STATUS{
	RFM73_STATUS_RX_DR	= 0x40,
	RFM73_STATUS_TX_DS	= 0x20,
	RFM73_STATUS_MAX_RT	= 0x10,
};

enum RFM73_FIFO_STATUS{
	RFM73_FIFO_STATUS_TX_REUSE	= 0x40,
	RFM73_FIFO_STATUS_TX_FULL	= 0x20,
	RFM73_FIFO_STATUS_TX_EMPTY	= 0x10,

	RFM73_FIFO_STATUS_RX_FULL	= 0x02,
	RFM73_FIFO_STATUS_RX_EMPTY	= 0x01
};

static uint8_t rfm73_cmd_write_register(RFM73_t* rfm, uint8_t reg, uint8_t* value, uint8_t size){
	uint8_t status;

	rfm->_csn_clr();
	status = rfm->_spi_transfer(RFM73_CMD_W_REGISTER | (reg & 0b00011111));
	for(uint8_t i = 0; i < size; i++){
		rfm->_spi_transfer(value[i]);
	}
	rfm->_csn_set();

	return status;
}

static uint8_t rfm73_cmd_write_register_inverse(RFM73_t* rfm, uint8_t reg, uint8_t* value, uint8_t size){
	uint8_t status;

	rfm->_csn_clr();
	status = rfm->_spi_transfer(RFM73_CMD_W_REGISTER | (reg & 0b00011111));
	for(uint8_t i = size; i > 0; i--){
		rfm->_spi_transfer(value[i - 1]);
	}
	rfm->_csn_set();

	return status;
}

static uint8_t rfm73_cmd_read_register(RFM73_t* rfm, uint8_t reg, uint8_t* value, uint8_t size){
	uint8_t status;

	rfm->_csn_clr();
	status = rfm->_spi_transfer(RFM73_CMD_R_REGISTER | (reg & 0b00011111));
	for(uint8_t i = 0; i < size; i++){
		value[i] = rfm->_spi_transfer(0x00);
	}
	rfm->_csn_set();

	return status;
}

static uint8_t rfm73_cmd_wirte_tx_payload(RFM73_t* rfm, uint8_t* value, uint8_t size){
	uint8_t status;

	rfm->_csn_clr();
	status = rfm->_spi_transfer(RFM73_CMD_W_TX_PAYLOAD);
	for(uint8_t i = 0; i < size; i++){
		rfm->_spi_transfer(value[i]);
	}
	rfm->_csn_set();

	return status;
}

static uint8_t rfm73_cmd_wirte_tx_payload_noack(RFM73_t* rfm, uint8_t* value, uint8_t size){
	uint8_t status;

	rfm->_csn_clr();
	status = rfm->_spi_transfer(RFM73_CMD_W_TX_PAYLOAD_NO_ACK);
	for(uint8_t i = 0; i < size; i++){
		rfm->_spi_transfer(value[i]);
	}
	rfm->_csn_set();

	return status;
}

static uint8_t rfm73_cmd_read_rx_payload_width(RFM73_t* rfm, uint8_t* value){
	uint8_t status;

	rfm->_csn_clr();
	status = rfm->_spi_transfer(RFM73_CMD_R_RX_PL_WID);
	*value = rfm->_spi_transfer(0x00);
	rfm->_csn_set();

	return status;
}

static uint8_t rfm73_cmd_read_rx_payload(RFM73_t* rfm, uint8_t* value, uint8_t size){
	uint8_t status;

	rfm->_csn_clr();
	status = rfm->_spi_transfer(RFM73_CMD_R_RX_PAYLOAD);
	for(uint8_t i=0; i<size; i++){
		value[i] = rfm->_spi_transfer(0x00);
	}
	rfm->_csn_set();

	return status;
}

static uint8_t rfm73_cmd_write_ack_payload(RFM73_t* rfm, uint8_t* value, uint8_t size, uint8_t pipe){
	uint8_t status;

	rfm->_csn_clr();
	status = rfm->_spi_transfer(RFM73_CMD_W_ACK_PAYLOAD | (pipe & 0b00000111));
	for(uint8_t i = 0; i < size; i++){
		rfm->_spi_transfer(value[i]);
	}
	rfm->_csn_set();

	return status;
}

static uint8_t rfm73_cmd_flush_tx(RFM73_t* rfm){
	uint8_t status;

	rfm->_csn_clr();
	status = rfm->_spi_transfer(RFM73_CMD_FLUSH_TX);
	rfm->_csn_set();

	return status;
}

static uint8_t rfm73_cmd_flush_rx(RFM73_t* rfm){
	uint8_t status;

	rfm->_csn_clr();
	status = rfm->_spi_transfer(RFM73_CMD_FLUSH_RX);
	rfm->_csn_set();

	return status;
}

static uint8_t rfm73_cmd_switch_bank(RFM73_t* rfm, uint8_t bank){
	uint8_t status;
	uint8_t bank_current;

	// current register bank: REG[7]
	status = rfm73_cmd_read_register(rfm, 7, &bank_current, 1);
	bank_current = (bank_current & 0x80) >> 7;

	if(((bank_current) && (bank == 0)) || (((bank_current) == 0) && (bank))){
		// bank toggle: ACTIVATE 0x53
		rfm->_csn_clr();
		status = rfm->_spi_transfer(RFM73_CMD_ACTIVATE);
		status = rfm->_spi_transfer(0x53);
		rfm->_csn_set();
	}

	return status;
}

static uint8_t rfm73_cmd_activate_features(RFM73_t* rfm){
	uint8_t status;

	// ACTIVATE 0x73
	rfm->_csn_clr();
	status = rfm->_spi_transfer(RFM73_CMD_ACTIVATE);
	status = rfm->_spi_transfer(0x73);
	rfm->_csn_set();

	return status;
}

void rfm73_power_save(RFM73_t* rfm, RFM73_power_save_t ps_mode){
	uint8_t value;

	switch(ps_mode){
	case RFM73_POWER_SAVE_STANDBY:{
		rfm->_ce_clr();
		break;
	}

	case RFM73_POWER_SAVE_POWER_DOWN:{
		rfm->_ce_clr();

		rfm73_cmd_read_register(rfm, RFM73_REG_CONFIG, &value, 1);

		// clear PWR_UP bit
		value &= 0xFD;

		rfm73_cmd_write_register(rfm, RFM73_REG_CONFIG, &value, 1);

		break;
	}

	default:
		break;
	}
}

void rfm73_power_save_wake_up(RFM73_t* rfm){
	uint8_t value;

	rfm73_cmd_read_register(rfm, RFM73_REG_CONFIG, &value, 1);

	// set PWR_UP bit
	value = (value & 0xFD) | 0x02;

	rfm73_cmd_write_register(rfm, RFM73_REG_CONFIG, &value, 1);

	rfm->_ce_set();
}

void rfm73_init(RFM73_t* rfm){
	if(rfm->_delay_ms == NULL) return;

	rfm->_csn_set();
	rfm->_delay_ms(200);


	{ // BANK0
		uint8_t data;
		uint8_t ps_mode;

		rfm73_cmd_switch_bank(rfm, 0);

		// 00 - CONFIG
		ps_mode = (rfm->power_save == RFM73_POWER_SAVE_NORMAL || rfm->power_save == RFM73_POWER_SAVE_STANDBY) * 0b00000010;
		data = rfm->crc | ps_mode | rfm->mode | 0b00001000;
		rfm73_cmd_write_register(rfm, RFM73_REG_CONFIG, &data, 1);

		// 01 - EN_AA
		if(rfm->mode == RFM73_MODE_PRX){
			data = rfm->rx.pipe_en;
			rfm73_cmd_write_register(rfm, RFM73_REG_EN_AA, &data, 1);
		}

		// 02 - EN_EXADDR
		if(rfm->mode == RFM73_MODE_PTX){
			data = RFM73_PIPE0;
			rfm73_cmd_write_register(rfm, RFM73_REG_EN_RXADDR, &data, 1);
		}
		else{
			data = rfm->rx.pipe_en;
			rfm73_cmd_write_register(rfm, RFM73_REG_EN_RXADDR, &data, 1);
		}

		// 03 - SETAUP_AW
		data = 0b00000011;
		rfm73_cmd_write_register(rfm, RFM73_REG_SETUP_AW, &data, 1);

		// 04 - SETUP_RETR
		if(rfm->mode == RFM73_MODE_PTX){
			data = rfm->tx.retransmit_delay | rfm->tx.retransmit_count;
			rfm73_cmd_write_register(rfm, RFM73_REG_SETUP_RETR, &data, 1);
		}

		// 05 - RF_CH
		data = rfm->channel;
		rfm73_cmd_write_register(rfm, RFM73_REG_RF_CH, &data, 1);

		// 06 - RF_SETUP
		if(rfm->mode == RFM73_MODE_PTX){
			data = rfm->bitrate | rfm->tx.power | 0b00000001;
			rfm73_cmd_write_register(rfm, RFM73_REG_RF_SETUP, &data, 1);
		}
		else{
			data = rfm->bitrate | rfm->rx.power | 0b00000001;
			rfm73_cmd_write_register(rfm, RFM73_REG_RF_SETUP, &data, 1);
		}

		// 07 - STATUS
		// 08 - OBSERVER_TX
		// 09 - CD

		// 0A - RX_ADDR_P0
		if(rfm->mode == RFM73_MODE_PTX){
			rfm73_cmd_write_register_inverse(rfm, RFM73_REG_RX_ADDR_P0, rfm->tx.addr, 5);
		}
		else{
			rfm73_cmd_write_register_inverse(rfm, RFM73_REG_RX_ADDR_P0, rfm->rx.pipe_addr0, 5);

			// 0A - RX_ADDR_P1
			rfm73_cmd_write_register_inverse(rfm, RFM73_REG_RX_ADDR_P1, rfm->rx.pipe_addr1to5, 5);

			// 0A - RX_ADDR_P2
			rfm73_cmd_write_register(rfm, RFM73_REG_RX_ADDR_P2, &rfm->rx.pipe_addr1to5[5], 1);

			// 0A - RX_ADDR_P3
			rfm73_cmd_write_register(rfm, RFM73_REG_RX_ADDR_P3, &rfm->rx.pipe_addr1to5[6], 1);

			// 0A - RX_ADDR_P4
			rfm73_cmd_write_register(rfm, RFM73_REG_RX_ADDR_P4, &rfm->rx.pipe_addr1to5[7], 1);

			// 0A - RX_ADDR_P5
			rfm73_cmd_write_register(rfm, RFM73_REG_RX_ADDR_P5, &rfm->rx.pipe_addr1to5[8], 1);
		}

		// 10 - TX_ADDR
		if(rfm->mode == RFM73_MODE_PTX){
			rfm73_cmd_write_register_inverse(rfm, RFM73_REG_TX_ADDR, rfm->tx.addr, 5);
		}

		// 11 - RX_PW_P0
		// 12 - RX_PW_P1
		// 13 - RX_PW_P2
		// 14 - RX_PW_P3
		// 15 - RX_PW_P4
		// 16 - RX_PW_P5
		// 17 - FIFO_STATUS
		// 18 - ACK_PLD
		// 19 - TX_PLD
		// 20 - RX_PLD

		// ACTIVATE 0x73 -> activate DYNPLD and FEATURE registers
		rfm73_cmd_read_register(rfm, RFM73_REG_FEATURE, &data, 1);
		if(data == 0){
			rfm73_cmd_activate_features(rfm);
		}

		// 1C - DYNPLD
		data = 63;//rfm->pipe;
		rfm73_cmd_write_register(rfm, RFM73_REG_DYNPD, &data, 1);

		// 1D - FEATURE
		data = RFM73_FEATURE_EN_ACK_PAY | RFM73_FEATURE_EN_DYN_ACK | RFM73_FEATURE_EN_DPL;
		rfm73_cmd_write_register(rfm, RFM73_REG_FEATURE, &data, 1);
	}

	{ // BANK1
		uint32_t data;

		rfm73_cmd_switch_bank(rfm, 1);

		// register 0 to 8 inversed, MSB first
		// 00 - 0x404B01E2
		data = 0x404B01E2;
		rfm73_cmd_write_register_inverse(rfm, 0x00, (uint8_t*)&data, 4);

		// 01 - 0xC04B0000
		data = 0xC04B0000;
		rfm73_cmd_write_register_inverse(rfm, 0x01, (uint8_t*)&data, 4);

		// 02 - 0xD0FC8C02
		data = 0xD0FC8C02;
		rfm73_cmd_write_register_inverse(rfm, 0x02, (uint8_t*)&data, 4);

		// 03 - 0x99003941
		data = 0x99003941;
		rfm73_cmd_write_register_inverse(rfm, 0x03, (uint8_t*)&data, 4);

		// 04 - 0xD9BE860B
		data = 0xD9BE860B; //0xD9BE860B;
		rfm73_cmd_write_register_inverse(rfm, 0x04, (uint8_t*)&data, 4);

		// 05 - 0x24067FA6
		data = 0x24067FA6;
		rfm73_cmd_write_register_inverse(rfm, 0x05, (uint8_t*)&data, 4);

		// 06
		// 07
		// 08
		// 09
		// 0A
		// 0B

		// 0C - 0x05731200
		data = 0x05731200;
		rfm73_cmd_write_register(rfm, 0x0C, (uint8_t*)&data, 4);

		// 0D - 0x0080B436
		data = 0x0080B436;
		rfm73_cmd_write_register(rfm, 0x0D, (uint8_t*)&data, 4);

		// 0E - 0xFFEF7DF208082082041041
		uint8_t data_array[11] = { 0xFF, 0xEF, 0x7D, 0xF2, 0x08, 0x08, 0x20, 0x82, 0x04, 0x10, 0x41 };
		rfm73_cmd_write_register(rfm, 0x0D, data_array, 11);

		rfm73_cmd_switch_bank(rfm, 0);
	}

	if(rfm->power_save == RFM73_POWER_SAVE_NORMAL){
		rfm->_ce_set();
	}
}

RFM73_send_t rfm73_write_payload(RFM73_t* rfm, uint8_t* data, uint8_t size, RFM73_ack_t ack){
	uint8_t config;
	uint8_t fifo_status;
	uint8_t timeout = 100;

	rfm73_cmd_read_register(rfm, RFM73_REG_CONFIG, &config, 1);

	// check if RFM73 in power down mode
	if((config & 0b00000010) == 0){
		return RFM73_SEND_POWER_DOWN;
	}

	// check if PTX
	if((config & 0b00000001) == 1){
		return RFM73_SEND_WRONG_MODE;
	}

	// check if tx fifo full
	rfm73_cmd_read_register(rfm, RFM73_REG_FIFO_STATUS, &fifo_status, 1);
	if((fifo_status & RFM73_FIFO_STATUS_TX_FULL)){
		if(ack == RFM73_ACK_FORCE || ack == RFM73_NOACK_FORCE){
			rfm73_cmd_flush_tx(rfm);
		}
		else{
			return RFM73_SEND_PENDING;
		}
	}

	// send
	if(ack == RFM73_ACK || ack == RFM73_ACK_FORCE){
		rfm73_cmd_wirte_tx_payload(rfm, data, size);
	}
	else{
		rfm73_cmd_wirte_tx_payload_noack(rfm, data, size);
	}

	// check if message sent
	while(timeout--){
		uint8_t status;

		rfm73_cmd_read_register(rfm, RFM73_REG_STATUS, &status, 1);

		if(status & RFM73_STATUS_TX_DS){
			uint8_t data = RFM73_STATUS_TX_DS;
			rfm73_cmd_write_register(rfm, RFM73_REG_STATUS, &data, 1);

			return RFM73_SEND_OK;
		}

		if(status & RFM73_STATUS_MAX_RT){
			uint8_t data = RFM73_STATUS_MAX_RT;
			rfm73_cmd_write_register(rfm, RFM73_REG_STATUS, &data, 1);

			return RFM73_SEND_MAX_RETR;
		}
	}

	return RFM73_SEND_TIMEOUT;
}

RFM73_send_t rfm73_write_ack_payload(RFM73_t* rfm, uint8_t* data, uint8_t size, uint8_t pipe){
	uint8_t config;

	rfm73_cmd_read_register(rfm, RFM73_REG_CONFIG, &config, 1);

	// check if RFM73 in power down mode
	if((config & 0b00000010) == 0){
		return RFM73_SEND_POWER_DOWN;
	}

	// check if PRX
	if((config & 0b00000001) == 0){
		return RFM73_SEND_WRONG_MODE;
	}

	// send
	rfm73_cmd_write_ack_payload(rfm, data, size, pipe);

	return RFM73_SEND_OK;
}

RFM73_recv_t rfm73_read_payload(RFM73_t* rfm, uint8_t* data, uint8_t size, uint8_t* pipe, uint8_t* len){
	uint8_t status, fifo_status;

	rfm73_cmd_read_register(rfm, RFM73_REG_STATUS, &status, 1);
	rfm73_cmd_read_register(rfm, RFM73_REG_FIFO_STATUS, &fifo_status, 1);

	if(status & RFM73_STATUS_RX_DR){
		if((fifo_status & RFM73_FIFO_STATUS_RX_EMPTY) == 0){
			*pipe = (status & 0b00001110) >> 1;

			rfm73_cmd_read_rx_payload_width(rfm, len);

			if(*len <= 32){
				rfm73_cmd_read_rx_payload(rfm, data, size);

				if(size < *len){
					rfm73_cmd_flush_rx(rfm);
				}
			}
			else{
				rfm73_cmd_flush_rx(rfm);
			}

			return RFM73_RECV_PENDING;
		}
		else{
			status = RFM73_STATUS_RX_DR;
			rfm73_cmd_write_register(rfm, RFM73_REG_STATUS, &status, 1);
		}
	}

	return RFM73_RECV_NONE;
}
