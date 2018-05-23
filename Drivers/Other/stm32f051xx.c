
#include "stm32f0xx_hal.h"


/* ****************************    GLOBALS     ***************************** */
extern SPI_HandleTypeDef hspi1;

/* ****************************    PRIVATES    ***************************** */
static SPI_HandleTypeDef* phspi = &hspi1;


uint8_t SpiTX(uint8_t* pData, uint8_t len) {

    HAL_StatusTypeDef result = HAL_OK;

    result = HAL_SPI_Transmit(phspi, pData, len, 10);
    while( phspi->State == HAL_SPI_STATE_BUSY );

    return (uint8_t)result;
}

uint8_t SpiRX(uint8_t* pData, uint8_t len) {

    HAL_StatusTypeDef result = HAL_OK;

    result = HAL_SPI_Receive(phspi, pData, len, 10);
    while( phspi->State == HAL_SPI_STATE_BUSY );

    return (uint8_t)result;
}
