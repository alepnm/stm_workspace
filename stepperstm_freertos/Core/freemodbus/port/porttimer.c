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
 * File: $Id: porttimer.c,v 1.1 2007/04/24 23:15:18 wolti Exp $
 */


/* ----------------------- Platform includes --------------------------------*/
/* ----------------------- Modbus includes ----------------------------------*/
#include "mbport.h"
#include "hardware.h"


extern TIM_HandleTypeDef* pMbPortTimer;

/* Statics */
static USHORT usT35TimeOut50us;

/* ----------------------- static functions ---------------------------------*/

/* ----------------------- Start implementation -----------------------------*/
BOOL
xMBPortTimersInit( USHORT usTimeOut50us )
{
    /* backup T35 ticks */
    usT35TimeOut50us = usTimeOut50us;

    pMbPortTimer->Init.Prescaler = (uint32_t)(SystemCoreClock / 48000U) - 1;
    pMbPortTimer->Init.Period = (uint32_t)( usT35TimeOut50us - 1 );
    pMbPortTimer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    pMbPortTimer->Init.CounterMode = TIM_COUNTERMODE_UP;

    if( HAL_TIM_Base_Init(pMbPortTimer) != HAL_OK ) return FALSE;
    if( HAL_TIM_OnePulse_Start_IT(pMbPortTimer, TIM_CHANNEL_ALL) != HAL_OK ) return FALSE;

    __HAL_TIM_ENABLE_IT(pMbPortTimer, TIM_IT_UPDATE);

    return TRUE;
}


inline void vMBPortTimersEnable( )
{
    if( HW_PortTimerStart(pMbPortTimer) != true ){
        _Error_Handler(__FILE__, __LINE__);
    }

    //HAL_GPIO_WritePin(COOLER_GPIO_Port, COOLER_Pin, GPIO_PIN_SET);
}

inline void vMBPortTimersDisable( )
{
    if( HAL_TIM_Base_Stop(pMbPortTimer) != HAL_OK ) {
        _Error_Handler(__FILE__, __LINE__);
    }
}


/* Create an ISR which is called whenever the timer has expired. This function
 * must then call pxMBPortCBTimerExpired( ) to notify the protocol stack that
 * the timer has expired.
 */
//static void prvvTIMERExpiredISR( void ) {
//
//    ( void )pxMBPortCBTimerExpired( );
//
//}

void MbPortPortTimer_IRQHandler( void )
{
    if(__HAL_TIM_GET_FLAG(pMbPortTimer, TIM_FLAG_UPDATE) != RESET && __HAL_TIM_GET_IT_SOURCE(pMbPortTimer, TIM_IT_UPDATE) != RESET) {

        __HAL_TIM_CLEAR_IT(pMbPortTimer, TIM_IT_UPDATE);

        ( void )pxMBPortCBTimerExpired( );    //prvvTIMERExpiredISR();

        //HAL_GPIO_WritePin(COOLER_GPIO_Port, COOLER_Pin, GPIO_PIN_RESET);
    }
}
