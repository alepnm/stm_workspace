/*
  * FreeModbus Libary: LPC214X Port
  * Copyright (C) 2007 Tiago Prado Lone <tiago@maxwellbohr.com.br>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: portserial.c,v 1.1 2007/04/24 23:15:18 wolti Exp $
 */

/* ----------------------- Modbus includes ----------------------------------*/
#include "mbport.h"
//#include "hardware.h"

/* Externs */
extern UART_HandleTypeDef* pMbPort;

/* ----------------------- extern functions ---------------------------------*/
extern bool UartStart( UART_HandleTypeDef* port, uint32_t ulBaudRate, uint8_t ucDataBits, uint8_t eParity );

/* ----------------------- static functions ---------------------------------*/
//static void prvvUARTTxReadyISR( void );
//static void prvvUARTRxISR( void );

/* ----------------------- Start implementation -----------------------------*/
void vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
    if( xRxEnable ) {
        __HAL_UART_ENABLE_IT( pMbPort, UART_IT_RXNE );
    } else {
        __HAL_UART_DISABLE_IT( pMbPort, UART_IT_RXNE );
    }

    if( xTxEnable ) {
#if( RS485_DE_HW_ENABLE == 0 )
        SLAVE_RS485_SEND_MODE;
#endif
        __HAL_UART_ENABLE_IT( pMbPort, UART_IT_TXE );
    } else {
        __HAL_UART_DISABLE_IT( pMbPort, UART_IT_TXE );
    }
}

/*  */
BOOL xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
    UNUSED(ucPORT);

    if( UartStart(pMbPort, (uint32_t)ulBaudRate, (uint8_t)ucDataBits, eParity) != true ){
        _Error_Handler(__FILE__, __LINE__);
    }

    vMBPortSerialEnable( TRUE, FALSE );

    return TRUE;
}

/*  */
BOOL xMBPortSerialPutByte( CHAR ucByte )
{
    pMbPort->Instance->TDR = ucByte;

    return TRUE;
}

/*  */
BOOL xMBPortSerialGetByte( CHAR * pucByte )
{
    *pucByte = pMbPort->Instance->RDR;

    return TRUE;
}

/*
 * Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call
 * xMBPortSerialPutByte( ) to send the character.
 */
//static void prvvUARTTxReadyISR( void )
//{
//    pxMBFrameCBTransmitterEmpty(  );
//}

/*
 * Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.
 */
//static void prvvUARTRxISR( void )
//{
//    pxMBFrameCBByteReceived(  );
//}

