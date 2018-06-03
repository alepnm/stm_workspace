
#include "stm32f051xx.h"
#include "port.h"

enum { PARITY_NONE = 0x00U, PARITY_ODD, PARITY_EVEN };
#define MBBAURATE_DEF 19200

/* ****************************    GLOBALS     ***************************** */
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;

/* ****************************    PRIVATES    ***************************** */

extern bool CheckBaudrateValue(uint16_t baudrate);
extern inline bool UartErr_Handler( UART_HandleTypeDef* port );


uint8_t SpiTransmit(uint8_t* pData, uint8_t len) {

    HAL_StatusTypeDef result = HAL_OK;

    L6470_CS_LOW

    result = HAL_SPI_Transmit(&hspi1, pData, len, 10);
    while( hspi1.State == HAL_SPI_STATE_BUSY );

    L6470_CS_HIGH

    return (uint8_t)result;
}

uint8_t SpiReceive(uint8_t* pData, uint8_t len) {

    HAL_StatusTypeDef result = HAL_OK;

    L6470_CS_LOW

    result = HAL_SPI_Receive(&hspi1, pData, len, 10);
    while( hspi1.State == HAL_SPI_STATE_BUSY );

    L6470_CS_HIGH

    return (uint8_t)result;
}

uint8_t SpiTransmitReceive(uint8_t* pDataTx, uint8_t* pDataRx, uint8_t len) {

    HAL_StatusTypeDef result = HAL_OK;

    L6470_CS_LOW

    result = HAL_SPI_TransmitReceive(&hspi1, pDataTx, pDataRx, len, 10);
    while( hspi1.State == HAL_SPI_STATE_BUSY );

    L6470_CS_HIGH

    return (uint8_t)result;
}



/* 0-NONE, 1-ODD, 2-EVEN*/
bool UartStart( UART_HandleTypeDef* port, uint32_t ulBaudRate, uint8_t ucDataBits, uint8_t eParity ) {

    if (HAL_UART_DeInit(port) != HAL_OK) return false;

    if( CheckBaudrateValue(ulBaudRate) == false ) ulBaudRate = MBBAURATE_DEF;

    port->Init.BaudRate = ulBaudRate;

    if( ucDataBits == (9U) ) port->Init.WordLength = UART_WORDLENGTH_9B;
    else port->Init.WordLength = UART_WORDLENGTH_8B;

    port->Init.StopBits = UART_STOPBITS_1;

    switch(eParity) {
    case PARITY_ODD:
        port->Init.Parity = UART_PARITY_ODD;
        break;
    case PARITY_EVEN:
        port->Init.Parity = UART_PARITY_EVEN;
        break;
    default:
        port->Init.Parity = UART_PARITY_NONE;
    }

    if (HAL_UART_Init(port) != HAL_OK) return false;

    return true;
}

/*  */
bool UartStop( UART_HandleTypeDef* port ) {

    __HAL_UART_DISABLE_IT( port, UART_IT_RXNE );
    __HAL_UART_DISABLE_IT( port, UART_IT_TXE );
    __HAL_UART_DISABLE_IT( port, UART_IT_TC );

    if (HAL_UART_DeInit(port) != HAL_OK) return false;

    return true;
}




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

        //(void)pxMBFrameCBTransmitterEmpty();    //prvvUARTTxReadyISR( );
    }

    if( __HAL_UART_GET_FLAG( &huart1, UART_FLAG_RXNE ) != RESET &&
            __HAL_UART_GET_IT_SOURCE( &huart1, UART_IT_RXNE ) != RESET ) {

        //(void)pxMBFrameCBByteReceived();    //prvvUARTRxISR( );
    }

    UartErr_Handler(&huart1);
}



