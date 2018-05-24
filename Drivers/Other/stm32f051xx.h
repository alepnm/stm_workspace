#ifndef STM32F051XX_H_INCLUDED
#define STM32F051XX_H_INCLUDED

#include "stm32f0xx_hal.h"

#define Delay_ms(x) HAL_Delay(x)

#define M25AA02_CS_LOW          HAL_GPIO_WritePin(M25AA_CS_GPIO_Port, M25AA_CS_Pin, GPIO_PIN_RESET);
#define M25AA02_CS_HIGH         HAL_GPIO_WritePin(M25AA_CS_GPIO_Port, M25AA_CS_Pin, GPIO_PIN_SET);

uint8_t SpiTransmit(uint8_t* pData, uint8_t len);
uint8_t SpiReceive(uint8_t* pData, uint8_t len);




inline bool UartErr_Handler( UART_HandleTypeDef* port ) {

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



#endif /* STM32F051XX_H_INCLUDED */
