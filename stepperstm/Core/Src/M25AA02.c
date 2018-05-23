/*

*/


#include "M25AA02.h"


/* **************************** DEFINES, MACRO ***************************** */
#define M25AA02E48

#ifdef M25AA02E48
    #define M25AA02_UID_NODE_ADDRESS        0xFA
#else
    #define M25AA02_UID_NODE_ADDRESS        0xF8
#endif


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


#define M25AA02_CS_LOW      HAL_GPIO_WritePin(M25AA_CS_GPIO_Port, M25AA_CS_Pin, GPIO_PIN_RESET);
#define M25AA02_CS_HIGH     HAL_GPIO_WritePin(M25AA_CS_GPIO_Port, M25AA_CS_Pin, GPIO_PIN_SET);


/* ****************************    GLOBALS     ***************************** */
extern SPI_HandleTypeDef hspi1;

/* ****************************    PRIVATES    ***************************** */
SPI_HandleTypeDef* hspi = &hspi1;

/* *************************** LOCAL FUNCTIONS ***************************** */
static bool _WriteEnable( void );
static bool _WriteDisable( void );

static bool SpiTX(uint8_t* pData, uint16_t size);
static bool SpiRX(uint8_t* pData, uint16_t size);

/* **************************** IMPLEMENTATION ***************************** */
/* Skaitom Status Registra
*/
bool M25AA02_readStatus( uint8_t* status ) {

    bool result = true;
    uint8_t data_tx = (uint8_t)READ_STATUS_INSTRUCTION;

    M25AA02_CS_LOW

    if( SpiTX(&data_tx, 1) == false ) result = false;
    if( SpiRX(status, 1) == false ) result = false;

    M25AA02_CS_HIGH

    return result;
}


/* Rasom Status registra
*/
bool M25AA02_writeStatus( uint8_t* status ) {

    bool result = true;
    uint8_t data_tx = (uint8_t)WRITE_STATUS_INSTRUCTION;

    M25AA02_CS_LOW

    if( SpiTX(&data_tx, 1) == false ) result = false;
    if( SpiRX(status, 1) == false ) result = false;

    M25AA02_CS_HIGH

    return result;
}


/*
*/
FlagStatus M25AA02_GetWriteFlag( ) {

    FlagStatus status;

    (void)M25AA02_readStatus((uint8_t*)&status);
    while( hspi->State == HAL_SPI_STATE_BUSY );

    return (FlagStatus)( status & ( (uint8_t)M25AA02_STATUS_WIP ) );
}


/* Nustatom rasyma i EEPROM
*/
bool M25AA02_SetWriteStatus( FunctionalState state ) {

    bool result = true;
    uint8_t data_tx = (uint8_t)WRITE_ENABLE_INSTRUCTION;

    if( state == DISABLE ) data_tx = (uint8_t)WRITE_DISABLE_INSTRUCTION;

    _WriteEnable();

    M25AA02_CS_LOW

    if( SpiTX(&data_tx, 1) == false ) result = false;

    M25AA02_CS_HIGH

    while( M25AA02_GetWriteFlag() == SET );

    return result;
}



/*veikia
*/
bool M25AA02_GetID( uint8_t *buffer ) {

    bool result = true;

    if(buffer == NULL) return false;

    uint8_t data_tx = (uint8_t)READ_INSTRUCTION;

    M25AA02_CS_LOW

    if( SpiTX(&data_tx, 1) == false ) result = false;

    data_tx = M25AA02_UID_NODE_ADDRESS;

    if( SpiTX(&data_tx, 1) == false ) result = false;

#ifdef M25AA02E48
    if( SpiRX(buffer, 6) == false ) result = false;
#else
    if( SpiRX(buffer, 8) == false ) result = false;
#endif

    while( hspi->State == HAL_SPI_STATE_BUSY );

    M25AA02_CS_HIGH

    return result;
}


/*
*/
bool  M25AA02_readRegister( uint8_t addr, uint8_t* data ) {

    return M25AA02_readNRegisters( addr, data, 1 );
}

/*veikia
*/
bool M25AA02_readNRegisters( uint8_t addr, uint8_t* buffer, int len ) {

    bool result = true;

    if(buffer == NULL) return false;

    uint8_t cmd = (uint8_t)READ_INSTRUCTION;

    M25AA02_CS_LOW

    if( SpiTX(&cmd, 1) == false ) result = false;
    if( SpiTX(&addr, 1) == false ) result = false;
    if( SpiRX(buffer, len) == false ) result = false;

    M25AA02_CS_HIGH

    return result;
}


/*
*/
bool M25AA02_writeRegister( uint8_t addr, uint8_t value ) {

    bool result = true;
    uint8_t cmd = (uint8_t)WRITE_INSTRUCTION;

    _WriteEnable();

    M25AA02_CS_LOW

    if( SpiTX(&cmd, 1) == false ) result = false;
    if( SpiTX(&addr, 1) == false ) result = false;
    if( SpiTX(&value, 1) == false ) result = false;

    M25AA02_CS_HIGH

    while( M25AA02_GetWriteFlag() == SET );

    return result;
}


/*
*/
bool M25AA02_writePage( uint8_t addr, uint8_t* buffer, uint8_t len ) {

    bool result = true;

    if( !len || (uint8_t)(((uint8_t)addr & 15) + len) > 16 ) return false;

    uint8_t cmd = (uint8_t)WRITE_INSTRUCTION;

    _WriteEnable();

    M25AA02_CS_LOW

    if( SpiTX(&cmd, 1) == false ) result = false;
    if( SpiTX(&addr, 1) == false ) result = false;
    if( SpiTX(buffer, len) == false ) result = false;

    M25AA02_CS_HIGH

    while( M25AA02_GetWriteFlag() == SET );

    return result;
}


/* Isvalom EEPROM
*/
bool M25AA02_clearData(){

    uint8_t i = 0;

    do{
        (void)M25AA02_writeRegister( i++, 0xFF );
    }while( i < 0xC0 );

    return true;
}



/*
*/
static bool _WriteEnable(){

    bool result = true;
    uint8_t cmd = (uint8_t)WRITE_ENABLE_INSTRUCTION;

    M25AA02_CS_LOW

    if( SpiTX(&cmd, 1) == false ) result = false;;

    M25AA02_CS_HIGH

    return result;
}


/*
*/
static bool _WriteDisable(){

    bool result = true;
    uint8_t cmd = (uint8_t)WRITE_DISABLE_INSTRUCTION;

    M25AA02_CS_LOW

    if( SpiTX(&cmd, 1) == false ) result = false;

    M25AA02_CS_HIGH

    return result;
}


inline static bool SpiTX(uint8_t* pData, uint16_t size){
    if( HAL_SPI_Transmit(hspi, pData, size, 10) != HAL_OK ) return false;
    while( hspi->State == HAL_SPI_STATE_BUSY );
    return true;
}

inline static bool SpiRX(uint8_t* pData, uint16_t size){
    if( HAL_SPI_Receive(hspi, pData, size, 10) != HAL_OK ) return false;
    while( hspi->State == HAL_SPI_STATE_BUSY );
    return true;
}
