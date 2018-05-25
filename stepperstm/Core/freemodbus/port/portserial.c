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
extern UART_HandleTypeDef huart1;

/* ----------------------- extern functions ---------------------------------*/
extern bool UartStart( UART_HandleTypeDef* port, uint32_t ulBaudRate, uint8_t ucDataBits, uint8_t eParity );
/* ----------------------- static functions ---------------------------------*/
static bool UartErr_Handler( UART_HandleTypeDef* port );
//static void prvvUARTTxReadyISR( void );
//static void prvvUARTRxISR( void );

/* ----------------------- Start implementation -----------------------------*/
void vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
    if( xRxEnable ) {
        __HAL_UART_ENABLE_IT( &huart1, UART_IT_RXNE );
    } else {
        __HAL_UART_DISABLE_IT( &huart1, UART_IT_RXNE );
    }

    if( xTxEnable ) {
#if( RS485_DE_HW_ENABLE == 0 )
        SLAVE_RS485_SEND_MODE;
#endif
        __HAL_UART_ENABLE_IT( &huart1, UART_IT_TXE );
    } else {
        __HAL_UART_DISABLE_IT( &huart1, UART_IT_TXE );
    }
}

/*  */
BOOL xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
    UNUSED(ucPORT);

    if( UartStart( 0, (uint32_t)ulBaudRate, (uint8_t)ucDataBits, eParity) != RESULT_OK ) Error_Handler();

    vMBPortSerialEnable( TRUE, FALSE );

    return TRUE;
}

/*  */
BOOL xMBPortSerialPutByte( CHAR ucByte )
{
    huart1.Instance->TDR = ucByte;

    return TRUE;
}

/*  */
BOOL xMBPortSerialGetByte( CHAR * pucByte )
{
    *pucByte = huart1.Instance->RDR;

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

void MbPortTransmitter_IRQHandler( void ) {

    if( __HAL_UART_GET_FLAG( &huart1, UART_FLAG_TC ) != RESET &&
            __HAL_UART_GET_IT_SOURCE( &huart1, UART_IT_TC ) != RESET ) {

#if( RS485_DE_HW_ENABLE == 0 )
        SLAVE_RS485_RECEIVE_MODE;
#endif
        __HAL_UART_DISABLE_IT( &huart1, UART_IT_TC );
    }

    if( __HAL_UART_GET_FLAG( &huart1, UART_FLAG_TXE ) != RESET &&
            __HAL_UART_GET_IT_SOURCE( &huart1, UART_IT_TXE ) != RESET ) {

        __HAL_UART_ENABLE_IT( &huart1, UART_IT_TC );

        (void)pxMBFrameCBTransmitterEmpty();    //prvvUARTTxReadyISR( );
    }

    if( __HAL_UART_GET_FLAG( &huart1, UART_FLAG_RXNE ) != RESET &&
            __HAL_UART_GET_IT_SOURCE( &huart1, UART_IT_RXNE ) != RESET ) {

        (void)pxMBFrameCBByteReceived();    //prvvUARTRxISR( );
    }

    UartErr_Handler(&huart1);
}


INLINE bool UartErr_Handler( UART_HandleTypeDef* port ) {

    if( __HAL_UART_GET_FLAG(port, UART_FLAG_PE) != RESET &&
            __HAL_UART_GET_IT_SOURCE(port, UART_IT_PE) != RESET ) {
        __HAL_UART_CLEAR_PEFLAG(port);

        port->ErrorCode |= HAL_UART_ERROR_PE;
    }

    if( __HAL_UART_GET_IT_SOURCE(port, UART_IT_ERR) != RESET ) {
        if( __HAL_UART_GET_FLAG(port, UART_FLAG_FE) != RESET ) {
            __HAL_UART_CLEAR_FEFLAG(port);
            port->ErrorCode |= HAL_UART_ERROR_FE;
        }

        if( __HAL_UART_GET_FLAG(port, UART_FLAG_NE) != RESET ) {
            __HAL_UART_CLEAR_NEFLAG(port);
            port->ErrorCode |= HAL_UART_ERROR_NE;
        }

        if( __HAL_UART_GET_FLAG(port, UART_FLAG_ORE) != RESET ) {
            __HAL_UART_CLEAR_OREFLAG(port);
            port->ErrorCode |= HAL_UART_ERROR_ORE;
        }
    }

    return true;
}
