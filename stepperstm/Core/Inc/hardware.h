/******************************************************************************
* Project Name		: Stepper Motor Control
* File Name		    : hardware.h
* Version 		    : x.x
* Device Used		: STM32F051C8x
* Software Used		: EmBitz 1.11
* Compiler    		: ARM GCC 4.9-2015-q1-update
* Related Hardware	:
*
* Owner             : Ventmatika Inc.
*******************************************************************************/

#ifndef UART_H_INCLUDED
#define UART_H_INCLUDED

#include "mbport.h"

bool HW_UartStart(UART_HandleTypeDef* port, uint32_t ulBaudRate, uint8_t ucDataBits, eMBParity eParity);
bool HW_UartStop(UART_HandleTypeDef* port);
INLINE bool HW_UartErrHandler( UART_HandleTypeDef* port );

bool HW_PortTimerInit(TIM_HandleTypeDef* timer);
INLINE bool HW_PortTimerStart( TIM_HandleTypeDef* timer );

uint16_t HW_GetAdcValue( ADC_ChannelConfTypeDef* sConfig );

void HW_PwmTimerInit( void );
void HW_SetAnalogOut( float volts );
void HW_SetDAC_Value( float dac );
void HW_SystemReset( void );

#endif /* UART_H_INCLUDED */
