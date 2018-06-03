/*

*/


#include "M25AA02.h"
#include "mcu_port.h"



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


/* ****************************    GLOBALS     ***************************** */
/* ****************************    PRIVATES    ***************************** */
/* *************************** LOCAL FUNCTIONS ***************************** */
static bool _WriteEnable( void );
static bool _WriteDisable( void );
/* **************************** IMPLEMENTATION ***************************** */

/* Skaitom Status Registra */
bool M25AA02_readStatus( uint8_t* status ) {

    bool result = true;
    uint8_t data_tx = (uint8_t)READ_STATUS_INSTRUCTION;

    //M25AA02_CS_LOW
    //if( SpiTransmit(&data_tx, 1) == false ) result = false;
    //if( SpiReceive(status, 1) == false ) result = false;
    //M25AA02_CS_HIGH

    SpiTransmitReceive(&data_tx, status, 1);

    return result;
}


/* Rasom Status registra */
bool M25AA02_writeStatus( uint8_t* status ) {

    bool result = true;
    uint8_t data_tx = (uint8_t)WRITE_STATUS_INSTRUCTION;

    //M25AA02_CS_LOW
    //if( SpiTransmit(&data_tx, 1) == false ) result = false;
    //if( SpiReceive(status, 1) == false ) result = false;
    //M25AA02_CS_HIGH

    SpiTransmitReceive(&data_tx, status, 1);

    return result;
}


/*  */
bool M25AA02_GetWriteFlag( ) {

    uint8_t status;

    (void)M25AA02_readStatus(&status);

    return (bool)( status & ( (uint8_t)M25AA02_STATUS_WIP ) );
}


/* Nustatom rasyma i EEPROM */
bool M25AA02_SetWriteStatus( FunctionalState state ) {

    bool result = true;
    uint8_t data_tx = (uint8_t)WRITE_ENABLE_INSTRUCTION;

    if( state == DISABLE ) data_tx = (uint8_t)WRITE_DISABLE_INSTRUCTION;

    _WriteEnable();

    if( SpiTransmit(&data_tx, 1) == false ) result = false;
    while( M25AA02_GetWriteFlag() == SET );

    return result;
}



/*  */
bool M25AA02_GetID( uint8_t *buffer ) {

    bool result = true;

    if(buffer == NULL) return false;

    uint8_t data_tx = (uint8_t)READ_INSTRUCTION;

    if( SpiTransmit(&data_tx, 1) == false ) result = false;

    data_tx = M25AA02_UID_NODE_ADDRESS;

    if( SpiTransmit(&data_tx, 1) == false ) result = false;

#ifdef M25AA02E48
    if( SpiReceive(buffer, 6) == false ) result = false;
#else
    if( SpiReceive(buffer, 8) == false ) result = false;
#endif

    return result;
}


/*  */
bool  M25AA02_readRegister( uint8_t addr, uint8_t* data ) {

    return M25AA02_readNRegisters( addr, data, 1 );
}

/*  */
bool M25AA02_readNRegisters( uint8_t addr, uint8_t* buffer, int len ) {

    bool result = true;

    if(buffer == NULL) return false;

    uint8_t cmd = (uint8_t)READ_INSTRUCTION;

    if( SpiTransmit(&cmd, 1) == false ) result = false;
    if( SpiTransmit(&addr, 1) == false ) result = false;
    if( SpiReceive(buffer, len) == false ) result = false;

    return result;
}


/*  */
bool M25AA02_writeRegister( uint8_t addr, uint8_t value ) {

    bool result = true;
    uint8_t cmd = (uint8_t)WRITE_INSTRUCTION;

    _WriteEnable();

    if( SpiTransmit(&cmd, 1) == false ) result = false;
    if( SpiTransmit(&addr, 1) == false ) result = false;
    if( SpiTransmit(&value, 1) == false ) result = false;

    while( M25AA02_GetWriteFlag() == SET );

    return result;
}


/*  */
bool M25AA02_writePage( uint8_t addr, uint8_t* buffer, uint8_t len ) {

    bool result = true;

    if( !len || (uint8_t)(((uint8_t)addr & 15) + len) > 16 ) return false;

    uint8_t cmd = (uint8_t)WRITE_INSTRUCTION;

    _WriteEnable();

    if( SpiTransmit(&cmd, 1) == false ) result = false;
    if( SpiTransmit(&addr, 1) == false ) result = false;
    if( SpiTransmit(buffer, len) == false ) result = false;

    while( M25AA02_GetWriteFlag() == SET );

    return result;
}


/* Isvalom EEPROM */
bool M25AA02_clearData(){

    uint8_t i = 0;

    do{
        (void)M25AA02_writeRegister( i++, 0xFF );
    }while( i < 0xC0 );

    return true;
}



/*  */
static bool _WriteEnable(){

    bool result = true;
    uint8_t cmd = (uint8_t)WRITE_ENABLE_INSTRUCTION;

    if( SpiTransmit(&cmd, 1) == false ) result = false;;

    return result;
}


/*  */
static bool _WriteDisable(){

    bool result = true;
    uint8_t cmd = (uint8_t)WRITE_DISABLE_INSTRUCTION;

    if( SpiTransmit(&cmd, 1) == false ) result = false;

    return result;
}
