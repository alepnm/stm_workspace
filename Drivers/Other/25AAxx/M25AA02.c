/*

*/
#include <stdlib.h>
#include "M25AA02.h"


/* **************************** DEFINES, MACRO ***************************** */
#define M25AA02E48

#ifdef M25AA02E48
#define M25AA02_UID_NODE_ADDRESS            0xFA
#else
#define M25AA02_UID_NODE_ADDRESS            0xF8
#endif

#define MEMORY_SIZE                         192
#define PAGE_SIZE                           16


#define WRITE_STATUS_INSTRUCTION            0x01    // 0b00000001
#define WRITE_INSTRUCTION                   0x02    // 0b00000010
#define READ_INSTRUCTION                    0x03    // 0b00000011
#define WRITE_DISABLE_INSTRUCTION           0x04    // 0b00000100
#define READ_STATUS_INSTRUCTION             0x05    // 0b00000101
#define WRITE_ENABLE_INSTRUCTION            0x06    // 0b00000110


/* Status register bits */
#define M25AA02_STATUS_WIP                  0x01    // bit WIP - Write In Process (RO)
#define M25AA02_STATUS_WEL                  (0x01)<<1    // bit WEL - Write Enable Latch (RO)

#define M25AA02_STATUS_BP_MASK              0x0C        // mask for BP bits
#define M25AA02_STATUS_BP0                  (0x01)<<2    // bit BP0 - Block0 Protection (RW)
#define M25AA02_STATUS_BP1                  (0x01)<<3    // bit BP1 - Block1 Protection (RW)


#define CS_LOW          HAL_GPIO_WritePin(M25AA_CS_GPIO_Port, M25AA_CS_Pin, GPIO_PIN_RESET);
#define CS_HIGH         HAL_GPIO_WritePin(M25AA_CS_GPIO_Port, M25AA_CS_Pin, GPIO_PIN_SET);


/* ****************************    GLOBALS     ***************************** */
extern SPI_HandleTypeDef hspi2;

/* ****************************    PRIVATES    ***************************** */
static SPI_HandleTypeDef* hspi = &hspi2;

/* *************************** LOCAL FUNCTIONS ***************************** */
static uint8_t _WriteEnable( void );
static uint8_t _WriteDisable( void );

static uint8_t SpiTX(uint8_t* pData, uint8_t len);
static uint8_t SpiRX(uint8_t* pData, uint8_t len);

/* **************************** IMPLEMENTATION ***************************** */
/* Skaitom Status Registra */
HAL_StatusTypeDef M25AA02_PullStatusRegister( uint8_t* status ) {

    HAL_StatusTypeDef result = HAL_OK;
    uint8_t data_tx = (uint8_t)READ_STATUS_INSTRUCTION;

    CS_LOW

    if( SpiTX(&data_tx, 1) != HAL_OK || SpiRX(status, 1) != HAL_OK ) {
        result = HAL_ERROR;
    }

    CS_HIGH

    return result;
}


/* Rasom Status registra */
HAL_StatusTypeDef M25AA02_PushStatusRegister( uint8_t* status ) {

    HAL_StatusTypeDef result = HAL_OK;
    uint8_t data_tx = (uint8_t)WRITE_STATUS_INSTRUCTION;

    (void)_WriteEnable();

    CS_LOW

    if( SpiTX(&data_tx, 1) != HAL_OK || SpiTX(status, 1) != HAL_OK ) {
        result = HAL_ERROR;
    }

    CS_HIGH

    return result;
}


/*  */
bool M25AA02_GetWriteFlag( ) {

    uint8_t status;

    (void)M25AA02_PullStatusRegister(&status);

    return (bool)( status & M25AA02_STATUS_WIP );
}


/*  */
HAL_StatusTypeDef M25AA02_GetID( uint8_t *buffer ) {

    HAL_StatusTypeDef result = HAL_OK;

    if( buffer == NULL ) return HAL_ERROR;

    uint8_t data_tx = (uint8_t)READ_INSTRUCTION;

    CS_LOW

    if( (result = SpiTX(&data_tx, 1) ) != HAL_OK ) goto lp80;

    data_tx = M25AA02_UID_NODE_ADDRESS;

    if( (result = SpiTX(&data_tx, 1) ) != HAL_OK ) goto lp80;

#ifdef M25AA02E48
    result = SpiRX(buffer, 6);
#else
    result = SpiRX(buffer, 8);
#endif

lp80:
    CS_HIGH

    return result;
}


/*  */
HAL_StatusTypeDef M25AA02_Read( uint8_t addr, uint8_t* buffer, int len ) {

    HAL_StatusTypeDef result = HAL_OK;

    if( addr > MEMORY_SIZE-1 || buffer == NULL || !len || len > MEMORY_SIZE ) return HAL_ERROR;

    uint8_t cmd = (uint8_t)READ_INSTRUCTION;

    CS_LOW

    if( SpiTX(&cmd, 1) != HAL_OK || SpiTX(&addr, 1) != HAL_OK || SpiRX(buffer, len) != HAL_OK ) {
        result = HAL_ERROR;
    }

    CS_HIGH

    return result;
}


/* rasymas is buferio N baitu */
HAL_StatusTypeDef M25AA02_Write( uint8_t addr, uint8_t* buffer, uint8_t len ) {

    HAL_StatusTypeDef result = HAL_OK;
    uint8_t cmd = (uint8_t)WRITE_INSTRUCTION;

    if( addr > MEMORY_SIZE-1 || buffer == NULL || !len || len > MEMORY_SIZE ) return HAL_ERROR;

    uint8_t wr_size = 0;
    uint8_t offset_in_page = addr & (PAGE_SIZE-1);

    if( offset_in_page+len > PAGE_SIZE ) {

        while ( len > 0 ) {

            wr_size = PAGE_SIZE - offset_in_page;

            if(len < wr_size) wr_size = len;

            (void)_WriteEnable();

            CS_LOW

            if( SpiTX(&cmd, 1) != HAL_OK || SpiTX(&addr, 1) != HAL_OK || SpiTX(buffer, wr_size) != HAL_OK ) {

                CS_HIGH

                HAL_Delay(7);

                return HAL_ERROR;
            }

            CS_HIGH

            while( M25AA02_GetWriteFlag() != RESET );

            offset_in_page = 0;
            buffer += wr_size;
            addr += wr_size;
            len -= wr_size;

            HAL_Delay(7);
        }

        return result;
    }

    (void)_WriteEnable();

    CS_LOW

    if( SpiTX(&cmd, 1) != HAL_OK || SpiTX(&addr, 1) != HAL_OK || SpiTX(buffer, len) != HAL_OK ) {
        result = HAL_ERROR;
    }

    CS_HIGH

    while( M25AA02_GetWriteFlag() != RESET );

    HAL_Delay(7);

    return result;
}



/* Isvalom EEPROM */
HAL_StatusTypeDef M25AA02_Clear() {

    HAL_StatusTypeDef result = HAL_OK;
    uint8_t* data = (uint8_t*)malloc(sizeof(uint8_t)*(PAGE_SIZE));
    uint16_t i = 0;
    uint8_t addr = 0;

    do {

        *(data+i) = 0xFF;

    } while(++i < PAGE_SIZE);

    while(addr < MEMORY_SIZE) {

        if( (result = M25AA02_Write( addr, data, PAGE_SIZE )) != HAL_OK ) break;
        addr += PAGE_SIZE;
    }

    free(data);

    return result;
}



/* vykdom pries kiek viena irasyma */
static HAL_StatusTypeDef _WriteEnable() {

    HAL_StatusTypeDef result = HAL_OK;
    uint8_t cmd = (uint8_t)WRITE_ENABLE_INSTRUCTION;

    CS_LOW

    result = SpiTX(&cmd, 1);

    CS_HIGH

    return result;
}


/* vykdyti nereikia - po sekmingos write komandos atsistato automatiskai*/
static HAL_StatusTypeDef _WriteDisable() {

    HAL_StatusTypeDef result = HAL_OK;
    uint8_t cmd = (uint8_t)WRITE_DISABLE_INSTRUCTION;

    CS_LOW

    result = SpiTX(&cmd, 1);

    CS_HIGH

    return result;
}


inline static HAL_StatusTypeDef SpiTX(uint8_t* pData, uint8_t len) {

    HAL_StatusTypeDef result = HAL_OK;

    if( ( result = HAL_SPI_Transmit(hspi, pData, len, 10) )!= HAL_OK ) return result;
    while( hspi->State == HAL_SPI_STATE_BUSY );
    return HAL_OK;
}

inline static HAL_StatusTypeDef SpiRX(uint8_t* pData, uint8_t len) {

    HAL_StatusTypeDef result = HAL_OK;

    if( ( result = HAL_SPI_Receive(hspi, pData, len, 10) ) != HAL_OK ) return result;
    while( hspi->State == HAL_SPI_STATE_BUSY );
    return HAL_OK;
}
