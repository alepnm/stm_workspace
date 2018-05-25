
#include "stm32f051xx.h"
#include "mcu_port.h"
#include "port.h"

/* ****************************    GLOBALS     ***************************** */
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;

/* ****************************    PRIVATES    ***************************** */
/* ************************** EXTERNAL FUNKCIJOS *************************** */
extern bool CheckBaudrateValue(uint16_t baudrate);
extern bool (*pxMBFrameCBByteReceived)( void );
extern bool (*pxMBFrameCBTransmitterEmpty)( void );


/* ************************************************************************* */
uint8_t SpiTransmit(uint8_t* pData, uint8_t len) {

    uint8_t result = RESULT_OK;

    result = HAL_SPI_Transmit(&hspi1, pData, len, 10);
    while( hspi1.State == HAL_SPI_STATE_BUSY );

    return result;
}

uint8_t SpiReceive(uint8_t* pData, uint8_t len) {

    uint8_t result = RESULT_OK;

    result = HAL_SPI_Receive(&hspi1, pData, len, 10);
    while( hspi1.State == HAL_SPI_STATE_BUSY );

    return result;
}

/* 0-NONE, 1-ODD, 2-EVEN*/
uint8_t UartStart( uint8_t uart, uint32_t ulBaudRate, uint8_t ucDataBits, uint8_t eParity ) {

    UNUSED(uart);

    if ( HAL_UART_DeInit(&huart1) != HAL_OK ) return RESULT_ERR;

    if( CheckBaudrateValue(ulBaudRate) == false ) ulBaudRate = 19200;

    huart1.Init.BaudRate = ulBaudRate;

    if( ucDataBits == (9U) ) huart1.Init.WordLength = UART_WORDLENGTH_9B;
    else huart1.Init.WordLength = UART_WORDLENGTH_8B;

    huart1.Init.StopBits = UART_STOPBITS_1;

    switch(eParity) {
    case PARITY_ODD:
        huart1.Init.Parity = UART_PARITY_ODD;
        break;
    case PARITY_EVEN:
        huart1.Init.Parity = UART_PARITY_EVEN;
        break;
    default:
        huart1.Init.Parity = UART_PARITY_NONE;
    }

    if ( HAL_UART_Init(&huart1) != HAL_OK ) return RESULT_ERR;

    return RESULT_OK;
}

/*  */
uint8_t UartStop( uint8_t uart ) {

    UNUSED(uart);

    __HAL_UART_DISABLE_IT( &huart1, UART_IT_RXNE );
    __HAL_UART_DISABLE_IT( &huart1, UART_IT_TXE );
    __HAL_UART_DISABLE_IT( &huart1, UART_IT_TC );

    if ( HAL_UART_DeInit(&huart1) != HAL_OK ) return RESULT_ERR;

    return RESULT_OK;
}


