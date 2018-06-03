/*

*/

#ifndef MICROCHIP_25AA02_H
#define MICROCHIP_25AA02_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32f0xx_hal.h"


bool M25AA02_readStatus( uint8_t* status );
bool M25AA02_writeStatus( uint8_t* status );
bool M25AA02_GetID( uint8_t *buffer );

bool M25AA02_SetWriteStatus( FunctionalState state );
bool M25AA02_GetWriteFlag( void );

bool M25AA02_readRegister( uint8_t addr, uint8_t* data );
bool M25AA02_readNRegisters( uint8_t addr, uint8_t *buffer, int len );
bool M25AA02_writeRegister( uint8_t addr, uint8_t value );
bool M25AA02_writePage( uint8_t addr, uint8_t* buffer, uint8_t len );
bool M25AA02_clearData( void );

#endif // MICROCHIP_25AA02_H
