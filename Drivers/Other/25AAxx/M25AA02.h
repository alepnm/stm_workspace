/*

*/

#ifndef MICROCHIP_25AA02_H
#define MICROCHIP_25AA02_H

#include <stdbool.h>
#include <stdint.h>


uint8_t M25AA02_PullStatusRegister( uint8_t* status );
uint8_t M25AA02_PushStatusRegister( uint8_t* status );
uint8_t M25AA02_GetID( uint8_t *buffer );

bool M25AA02_GetWriteFlag( void );

uint8_t M25AA02_Read( uint8_t addr, uint8_t *buffer, int len );
uint8_t M25AA02_Write( uint8_t addr, uint8_t* buffer, uint8_t len );
uint8_t M25AA02_Clear( void );

#endif // MICROCHIP_25AA02_H
