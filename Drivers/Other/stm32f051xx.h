#ifndef STM32F051XX_H_INCLUDED
#define STM32F051XX_H_INCLUDED

#include "stm32f0xx_hal.h"

#define __enter_critical() {uint32_t irq; irq = __get_PRIMASK();
#define __exit_critical() __set_PRIMASK(irq);}
#define ATOMIC_SECTION(X) __enter_critical(); {X}; __exit_critical();

#define Delay_ms(x) HAL_Delay(x)

#define LED_ON                  GPIO_PIN_RESET
#define LED_OFF                 GPIO_PIN_SET

#define STATUS_LED_ON()         HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET)
#define STATUS_LED_OFF()        HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET)
#define STATUS_LED_TOGGLE()     HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin)
#define FAULT_LED_ON()          HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_RESET)
#define FAULT_LED_OFF()         HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_SET)
#define FAULT_LED_TOGGLE()      HAL_GPIO_TogglePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin)

#define COOLER_ON()             HAL_GPIO_WritePin(COOLER_GPIO_Port, COOLER_Pin, GPIO_PIN_SET)
#define COOLER_OFF()            HAL_GPIO_WritePin(COOLER_GPIO_Port, COOLER_Pin, GPIO_PIN_RESET)
#define RELAY_ON()              HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_SET)
#define RELAY_OFF()             HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_RESET)

#define L6470_EMERGENCY_STOP    HAL_GPIO_WritePin(L6470_RST_GPIO_Port, L6470_RST_Pin, GPIO_PIN_RESET); while(1);

#define DI0_STATE()             HAL_GPIO_ReadPin(DI0_GPIO_Port, DI0_Pin)
#define DI1_STATE()             HAL_GPIO_ReadPin(DI1_GPIO_Port, DI1_Pin)
#define DI2_STATE()             HAL_GPIO_ReadPin(DI2_GPIO_Port, DI2_Pin)
#define DI3_STATE()             HAL_GPIO_ReadPin(DI3_GPIO_Port, DI3_Pin)

#define M25AA02_CS_LOW          HAL_GPIO_WritePin(M25AA_CS_GPIO_Port, M25AA_CS_Pin, GPIO_PIN_RESET);
#define M25AA02_CS_HIGH         HAL_GPIO_WritePin(M25AA_CS_GPIO_Port, M25AA_CS_Pin, GPIO_PIN_SET);

/* SPI */
uint8_t SpiTransmit(uint8_t* pData, uint8_t len);
uint8_t SpiReceive(uint8_t* pData, uint8_t len);

/* UART */
uint8_t UartStart( uint8_t uart, uint32_t ulBaudRate, uint8_t ucDataBits, uint8_t eParity );
uint8_t UartStop( uint8_t uart );



#endif /* STM32F051XX_H_INCLUDED */
