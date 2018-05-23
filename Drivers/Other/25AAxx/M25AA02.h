/*

*/

#ifndef MICROCHIP_25AA02_H
#define MICROCHIP_25AA02_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32f0xx_hal.h"


HAL_StatusTypeDef M25AA02_PullStatusRegister( uint8_t* status );
HAL_StatusTypeDef M25AA02_PushStatusRegister( uint8_t* status );
HAL_StatusTypeDef M25AA02_GetID( uint8_t *buffer );

bool M25AA02_GetWriteFlag( void );

HAL_StatusTypeDef M25AA02_Read( uint8_t addr, uint8_t *buffer, int len );
HAL_StatusTypeDef M25AA02_Write( uint8_t addr, uint8_t* buffer, uint8_t len );
HAL_StatusTypeDef M25AA02_Clear( void );

#endif // MICROCHIP_25AA02_H
