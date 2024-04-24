
#include "stm32f1xx_hal.h"   
#include "nRF24L01.h"
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

typedef enum { 
	RF24_PA_m18dB = 0,
	RF24_PA_m12dB,
	RF24_PA_m6dB,
	RF24_PA_0dB,
	RF24_PA_ERROR 
}rf24_pa_dbm_e ;
typedef enum { 
	RF24_1MBPS = 0,
	RF24_2MBPS,
	RF24_250KBPS
}rf24_datarate_e;
typedef enum { 
	RF24_CRC_DISABLED = 0,
	RF24_CRC_8,
	RF24_CRC_16
}rf24_crclength_e;
static const uint8_t NRF24_ADDR_REGS[7] = {
		REG_RX_ADDR_P0,
		REG_RX_ADDR_P1,
		REG_RX_ADDR_P2,
		REG_RX_ADDR_P3,
		REG_RX_ADDR_P4,
		REG_RX_ADDR_P5,
		REG_TX_ADDR
};
static const uint8_t RF24_RX_PW_PIPE[6] = {
		REG_RX_PW_P0, 
		REG_RX_PW_P1,
		REG_RX_PW_P2,
		REG_RX_PW_P3,
		REG_RX_PW_P4,
		REG_RX_PW_P5
};
void NRF24_DelayMicroSeconds(uint32_t uSec);

void NRF24_csn(int mode);
void NRF24_ce(int level);
uint8_t NRF24_read_register(uint8_t reg);
void NRF24_read_registerN(uint8_t reg, uint8_t *buf, uint8_t len);
void NRF24_write_register(uint8_t reg, uint8_t value);
void NRF24_write_registerN(uint8_t reg, const uint8_t* buf, uint8_t len);
void NRF24_write_payload(const void* buf, uint8_t len);
void NRF24_read_payload(void* buf, uint8_t len);
void NRF24_flush_tx(void);
void NRF24_flush_rx(void);
uint8_t NRF24_get_status(void);
void NRF24_begin(GPIO_TypeDef *nrf24PORT, uint16_t nrfCSN_Pin, uint16_t nrfCE_Pin, SPI_HandleTypeDef nrfSPI);
void NRF24_startListening(void);
void NRF24_stopListening(void);
bool NRF24_write( const void* buf, uint8_t len );
bool NRF24_available(void);
bool NRF24_read( void* buf, uint8_t len );
void NRF24_openWritingPipe(uint64_t address);
void NRF24_openReadingPipe(uint8_t number, uint64_t address);
void NRF24_setRetries(uint8_t delay, uint8_t count);
void NRF24_setChannel(uint8_t channel);
void NRF24_setPayloadSize(uint8_t size);
uint8_t NRF24_getPayloadSize(void);
uint8_t NRF24_getDynamicPayloadSize(void);
void NRF24_enableAckPayload(void);
void NRF24_enableDynamicPayloads(void);
void NRF24_disableDynamicPayloads(void);
bool NRF24_isNRF_Plus(void) ;
void NRF24_setAutoAck(bool enable);
void NRF24_setAutoAckPipe( uint8_t pipe, bool enable ) ;
void NRF24_setPALevel( rf24_pa_dbm_e level ) ;
rf24_pa_dbm_e NRF24_getPALevel( void ) ;
bool NRF24_setDataRate(rf24_datarate_e speed);
rf24_datarate_e NRF24_getDataRate( void );
void NRF24_setCRCLength(rf24_crclength_e length);
rf24_crclength_e NRF24_getCRCLength(void);
void NRF24_disableCRC( void ) ;
void NRF24_powerUp(void) ;
void NRF24_powerDown(void);
bool NRF24_availablePipe(uint8_t* pipe_num);
void NRF24_startWrite( const void* buf, uint8_t len );
void NRF24_writeAckPayload(uint8_t pipe, const void* buf, uint8_t len);
bool NRF24_isAckPayloadAvailable(void);
void NRF24_whatHappened(bool *tx_ok,bool *tx_fail,bool *rx_ready);
bool NRF24_testCarrier(void);
bool NRF24_testRPD(void) ;
void NRF24_resetStatus(void);
void NRF24_ACTIVATE_cmd(void);
uint8_t NRF24_GetAckPayloadSize(void);
