/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "stm32f0xx.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
#define RS485_DE_HW_ENABLE 0

#define RELAY_Pin GPIO_PIN_13
#define RELAY_GPIO_Port GPIOC
#define DI3_Pin GPIO_PIN_14
#define DI3_GPIO_Port GPIOC
#define VBUS_Pin GPIO_PIN_0
#define VBUS_GPIO_Port GPIOA
#define SPEED_CONTROL_Pin GPIO_PIN_1
#define SPEED_CONTROL_GPIO_Port GPIOA
#define HALL_S_Pin GPIO_PIN_2
#define HALL_S_GPIO_Port GPIOA
#define HALL_S_EXTI_IRQn EXTI2_3_IRQn
#define STCK_Pin GPIO_PIN_3
#define STCK_GPIO_Port GPIOA
#define DI0_Pin GPIO_PIN_0
#define DI0_GPIO_Port GPIOB
#define DI1_Pin GPIO_PIN_1
#define DI1_GPIO_Port GPIOB
#define DI2_Pin GPIO_PIN_2
#define DI2_GPIO_Port GPIOB
#define L6470_RST_Pin GPIO_PIN_10
#define L6470_RST_GPIO_Port GPIOB
#define L6470_CS_Pin GPIO_PIN_11
#define L6470_CS_GPIO_Port GPIOB
#define M25AA_CS_Pin GPIO_PIN_12
#define M25AA_CS_GPIO_Port GPIOB
#define HC165_CS_Pin GPIO_PIN_13
#define HC165_CS_GPIO_Port GPIOB
#define STATUS_LED_Pin GPIO_PIN_14
#define STATUS_LED_GPIO_Port GPIOB
#define FAULT_LED_Pin GPIO_PIN_15
#define FAULT_LED_GPIO_Port GPIOB
#define HCCTRL_Pin GPIO_PIN_11
#define HCCTRL_GPIO_Port GPIOA
#define USART1_DE_Pin GPIO_PIN_12
#define USART1_DE_GPIO_Port GPIOA
#define HC165_LATCH_Pin GPIO_PIN_6
#define HC165_LATCH_GPIO_Port GPIOF
#define COOLER_Pin GPIO_PIN_7
#define COOLER_GPIO_Port GPIOF
#define BEEPER_Pin GPIO_PIN_8
#define BEEPER_GPIO_Port GPIOB
#define PWM_Pin GPIO_PIN_9
#define PWM_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
#define LO16(x) (uint16_t)( x & 0x0000FFFF )
#define HI16(x) (uint16_t)((x & 0xFFFF0000 ) >> 16)

#define __enter_critical() {uint32_t irq; irq = __get_PRIMASK();
#define __exit_critical() __set_PRIMASK(irq);}
#define ATOMIC_SECTION(X) __enter_critical(); {X}; __exit_critical();


#define DUMMY                   (0U)

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




/* VBUS kontrole */
#define FLT_HW_VBUS_Pos         (3U)
#define FLT_HW_VBUS_Msk         (0x03U << FLT_HW_VBUS_Pos)
#define FLT_HW_VBUS             FLT_HW_VBUS_Msk
#define FLT_HW_VBUS_LOW         (0x01U << FLT_HW_VBUS_Pos)
#define FLT_HW_VBUS_HIGH        (0x02U << FLT_HW_VBUS_Pos)

/* variklis */
#define FLT_HW_MOTOR_Pos        (5U)
#define FLT_HW_MOTOR_Msk        (0x01U << FLT_HW_MOTOR_Pos)
#define FLT_HW_MOTOR            FLT_HW_MOTOR_Msk

/* holo daviklis */
#define FLT_HW_HS_Pos           (6U)
#define FLT_HW_HS_Msk           (0x01U << FLT_HW_HS_Pos)
#define FLT_HW_HS               FLT_HW_HS_Msk

/* L6470 STATUS register TH_WRN flagas */
#define FLT_HW_TH_WRN_Pos       (7U)
#define FLT_HW_TH_WRN_Msk       (0x01U << FLT_HW_TH_WRN_Pos)
#define FLT_HW_TH_WRN           FLT_HW_TH_WRN_Msk

/* L6470 STATUS register TH_SD flagas  */
#define FLT_HW_TH_SHUTDOWN_Pos  (8U)
#define FLT_HW_TH_SHUTDOWN_Msk  (0x01U << FLT_HW_TH_SHUTDOWN_Pos)
#define FLT_HW_TH_SHUTDOWN      FLT_HW_TH_SHUTDOWN_Msk

/* L6470 STATUS register OCD flagas */
#define FLT_HW_OCD_Pos          (9U)
#define FLT_HW_OCD_Msk          (0x01U << FLT_HW_OCD_Pos)
#define FLT_HW_OCD              FLT_HW_OCD_Msk

/* L6470 STATUS register ULVO flagas */
#define FLT_HW_ULVO_Pos         (10U)
#define FLT_HW_ULVO_Msk         (0x01U << FLT_HW_ULVO_Pos)
#define FLT_HW_ULVO             FLT_HW_ULVO_Msk


typedef struct{
    __IO uint32_t       CurrentTimestamp;
    uint32_t            WTime_sec;
}SYSTIME_TypeDef;


/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
