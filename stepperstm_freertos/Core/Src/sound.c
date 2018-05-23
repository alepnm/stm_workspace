/**
  ******************************************************************************
  * File Name          : sound.c
  * Description        :
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "sound.h"

/* Defines, macro, typedefs --------------------------------------------------*/

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef* pBeeperTimer;

/* Global variables ----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static SND_HandleTypeDef Sounder = {
    .tone = 1,
    .freq = 1000,
    .volume = LOW
};

/* Private functions ---------------------------------------------------------*/
static void SoundTimerInit( void );
static void SoundTimerStart( void );
static void SoundTimerStop( void );



/**
*/
void SoundInit( bool ena ) {

    Sounder.tone = 0;
    Sounder.volume = LOW;
    Sounder.freq = 1000;

    SoundTimerInit();
}


/** garso stiprumo nustatymas
 */
void SoundSetVolume( SND_VolTypeDef volume ) {
    Sounder.volume = volume;
    SoundTimerInit();
}


/** garso daznio nustatymas
 */
void SoundSetFreq( uint16_t freq ) {
    Sounder.freq = freq;
    SoundTimerInit();
}


/** startuojam garsa
 */
void SoundStart( uint8_t tone ) {
    Sounder.tone = tone;
}


/* vykdom is SysTick interaptu kas 1ms */
void SoundHandler() {

    static uint8_t stage, cnt = (1u), begin = true;

    if( Sounder.tone == 0 ) return;


    if(begin) {
        begin = false;
        stage = (0u);
    } else {
        if(--cnt) return;
    }


    if(Sounder.tone == (1u)) {

        SoundTimerStart();

        switch(stage) {
        case 0:
            cnt = (30u);
            break;
        default:
            goto stop;
        }
        stage++;
        return;
    }
    if(Sounder.tone == (2u)) {

        SoundTimerStart();

        switch(stage) {
        case 0:
            cnt = (50u);
            SoundTimerStart();
            break;
        case 1:
            cnt = (50u);
            SoundTimerStop();
            break;
        case 2:
            cnt = (50u);
            SoundTimerStart();
            break;
        case 3:
            cnt = (50u);
            SoundTimerStop();
            break;
        case 4:
        default:
            goto stop;
        }
        stage++;
        return;
    }
    if(Sounder.tone == (3u)) {

        switch(stage) {
        case 0:
            cnt = (50u);
            SoundTimerStart();
            break;
        case 1:
            cnt = (50u);
            SoundTimerStop();
            break;
        case 2:
            cnt = (50u);
            SoundTimerStart();
            break;
        case 3:
            cnt = (50u);
            SoundTimerStop();
            break;
        case 4:
            cnt = (50u);
            SoundTimerStart();
            break;
        case 5:
        default:
            //Sounder.freq = F1KHZ;
            goto stop;
        }
        if(stage < 5) {
            stage++;
        } else {
            stage = 5;
        }
        return;
    }
    if(Sounder.tone == (4u)) {

        SoundTimerStart();

        switch(stage) {
        case 0:
            cnt = (40u);
            SoundSetFreq(1000);
            break;
        case 1:
            cnt = (40u);
            SoundSetFreq(1000-25);
            break;
        case 2:
            cnt = (40u);
            SoundSetFreq(1000-50);
            break;
        case 3:
            cnt = (40u);
            SoundSetFreq(1000-75);
            break;
        case 4:
            cnt = (40u);
            SoundSetFreq(1000-100);
            break;
        case 5:
            cnt = (40u);
            SoundSetFreq(1000-125);
            break;
        case 6:
            cnt = (40u);
            SoundSetFreq(1000-150);
            break;
        case 7:
            cnt = (40u);
            SoundSetFreq(1000-175);
            break;
        case 8:
        default:
            SoundSetFreq(1000);
            goto stop;
        }
        stage++;
        return;
    }
stop:
    SoundTimerStop();
    stage = (0u);
    Sounder.tone = (0u);
    begin = true;
}



/**
  */
static void SoundTimerInit() {

    TIM_OC_InitTypeDef sConfigOC = {
        .OCMode = TIM_OCMODE_PWM1,
        .OCPolarity = TIM_OCPOLARITY_HIGH,
        .OCNPolarity = TIM_OCNPOLARITY_HIGH,
        .OCFastMode = TIM_OCFAST_DISABLE,
        .OCIdleState = TIM_OCIDLESTATE_RESET,
        .OCNIdleState = TIM_OCNIDLESTATE_RESET
    };

    pBeeperTimer->Init.Prescaler = (SystemCoreClock / 1000000) - 1;
    pBeeperTimer->Init.CounterMode = TIM_COUNTERMODE_UP;
    pBeeperTimer->Init.Period = (1000000 / Sounder.freq) - 1;
    pBeeperTimer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    pBeeperTimer->Init.RepetitionCounter = 0;
    pBeeperTimer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if( HAL_TIM_Base_Init(pBeeperTimer) != HAL_OK ){
            Error_Handler();
    }

    sConfigOC.Pulse = (pBeeperTimer->Init.Period * Sounder.volume) / 100;

    if( HAL_TIM_PWM_ConfigChannel(pBeeperTimer, &sConfigOC, TIM_CHANNEL_1 ) != HAL_OK){
            Error_Handler();
    }
}

/**
  */
static void SoundTimerStart() {
    if( HAL_TIM_PWM_Start(pBeeperTimer, TIM_CHANNEL_1) != HAL_OK ) {
        Error_Handler();
    }
}

/**
  */
static void SoundTimerStop() {
    if( HAL_TIM_PWM_Stop(pBeeperTimer, TIM_CHANNEL_1) != HAL_OK ) {
        Error_Handler();
    }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
