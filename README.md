# Usage example

```c
#include <avr/io.h>
#include <util/delay.h>

#include "rfm73.h"

#define CE   		(1<<PB1)
#define CE_DDR		DDRB
#define CE_PORT		PORTB

#define CSN   		(1<<PB2)
#define CSN_DDR		DDRB
#define CSN_PORT	PORTB

static void _rfm73_delay_ms(uint16_t _ms){
  while(_ms--){
    _delay_ms(1);
  }
}

static uint8_t _rfm73_spi_transfer(uint8_t _data){
  return spiSoft_transfer(_data);
}

static void _rfm73_ce_set(void){
  CE_PORT |= CE;
}

static void _rfm73_ce_clr(void){
  CE_PORT &= ~CE;
}

static void _rfm73_csn_set(void){
  CSN_PORT |= CSN;
}

static void _rfm73_csn_clr(void){
  CSN_PORT &= ~CSN;
}

RFM73_t radio = {
  .mode = RFM73_MODE_PTX,
  
  .channel = 24,
  .crc = RFM73_CRC_2B,
  .bitrate = RFM73_BITRATE_250KBPS,
  .power_save = RFM73_POWER_SAVE_POWER_DOWN,
  
  .tx.power = RFM73_POWER_5DBM,
  .tx.retransmit_delay = RFM73_RETR_DELAY_1000US,
  .tx.retransmit_count = RFM73_RETR_15,
  
  .tx.addr = {0xA2, 0xA2, 0xA2, 0xA2, 0xA2},

  ._delay_ms = _rfm73_delay_ms,
  ._spi_transfer = _rfm73_spi_transfer,
  ._ce_set = _rfm73_ce_set,
  ._ce_clr = _rfm73_ce_clr,
  ._csn_set = _rfm73_csn_set,
  ._csn_clr = _rfm73_csn_clr,
};

int main(void){
  CE_DDR |= CE;
  CSN_DDR |= CSN;

  rfm73_init(&radio);

  while(1){

  }
}
```
