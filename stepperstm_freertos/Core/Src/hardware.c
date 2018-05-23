/******************************************************************************
* Project Name		: Stepper Motor Control
* File Name		    : hardware.c
* Version 		    : x.x
* Device Used		: STM32F051C8x
* Software Used		: EmBitz 1.11
* Compiler    		: ARM GCC 4.9-2015-q1-update
* Related Hardware	:
*
* Owner             : Ventmatika Inc.
*******************************************************************************/

#include "hardware.h"
#include "smc.h"
#include "user_mb_app.h"

/* ------------------------------ Externs ---------------------------------- */
extern ADC_HandleTypeDef        hadc;
extern DAC_HandleTypeDef        hdac1;
extern TIM_HandleTypeDef*       pPwmTimer;

static HAL_StatusTypeDef ADC_ConfigChannel(ADC_HandleTypeDef* hadc, ADC_ChannelConfTypeDef* sConfig);


/*  */
bool HW_UartStart( UART_HandleTypeDef* port, uint32_t ulBaudRate, uint8_t ucDataBits, eMBParity eParity ) {

    if (HAL_UART_DeInit(port) != HAL_OK) return false;

    if( SMC_CheckBaudrateValue(ulBaudRate) == false ) ulBaudRate = MBBAURATE_DEF;

    port->Init.BaudRate = ulBaudRate;

    if( ucDataBits == (9U) ) port->Init.WordLength = UART_WORDLENGTH_9B;
    else port->Init.WordLength = UART_WORDLENGTH_8B;

    port->Init.StopBits = UART_STOPBITS_1;

    switch(eParity) {
    case MB_PAR_ODD:
        port->Init.Parity = UART_PARITY_ODD;
        break;
    case MB_PAR_EVEN:
        port->Init.Parity = UART_PARITY_EVEN;
        break;
    default:
        port->Init.Parity = UART_PARITY_NONE;
    }

    if (HAL_UART_Init(port) != HAL_OK) return false;

    return true;
}

/*  */
bool HW_UartStop( UART_HandleTypeDef* port ) {

    __HAL_UART_DISABLE_IT( port, UART_IT_RXNE );
    __HAL_UART_DISABLE_IT( port, UART_IT_TXE );
    __HAL_UART_DISABLE_IT( port, UART_IT_TC );

    if (HAL_UART_DeInit(port) != HAL_OK) return false;

    return true;
}

/*  */
INLINE bool HW_UartErrHandler( UART_HandleTypeDef* port ) {

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


/*  */
bool HW_PortTimerInit( TIM_HandleTypeDef* timer ) {

    timer->Init.Prescaler = (uint32_t)(SystemCoreClock / 48000U) - 1;
    timer->Init.Period = 0;
    timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    timer->Init.CounterMode = TIM_COUNTERMODE_UP;

    if ( HAL_TIM_Base_Init(timer) != HAL_OK ) return false;
    if ( HAL_TIM_OnePulse_Start_IT(timer, TIM_CHANNEL_ALL) != HAL_OK ) return false;

    return true;
}

/*  */
INLINE bool HW_PortTimerStart( TIM_HandleTypeDef* timer ) {

    __HAL_TIM_SET_COUNTER(timer, 0);

    if ( HAL_TIM_Base_Start(timer) != HAL_OK ) return false;

    __HAL_TIM_ENABLE_IT(timer, TIM_IT_UPDATE);

    return true;
}

/*  */
uint16_t HW_GetAdcValue( ADC_ChannelConfTypeDef* sConfig ) {

    uint16_t val = 0;
    uint8_t i = 0;

    if( ADC_ConfigChannel(&hadc, sConfig) != HAL_OK ) {
        _Error_Handler(__FILE__, __LINE__);
    }

    for(i = 0; i < 2; i++) {

        if( HAL_ADC_Start(&hadc) != HAL_OK ) {
            _Error_Handler(__FILE__, __LINE__);
        }

        while(HAL_ADC_PollForConversion(&hadc, 0) != HAL_OK);

        val += (uint16_t)HAL_ADC_GetValue(&hadc);
    }

    (void)HAL_ADC_Stop(&hadc);

    return (uint16_t)(val>>1);
}


/* Pataisyta HAL_ADC_ConfigChannel funkcija is HAL bibliotekos */
static HAL_StatusTypeDef ADC_ConfigChannel( ADC_HandleTypeDef* hadc, ADC_ChannelConfTypeDef* sConfig ) {

    HAL_StatusTypeDef tmp_hal_status = HAL_OK;

    volatile uint32_t wait_loop_index = 0U;

    /* Check the parameters */
    assert_param(IS_ADC_ALL_INSTANCE(hadc->Instance));
    assert_param(IS_ADC_CHANNEL(sConfig->Channel));
    assert_param(IS_ADC_RANK(sConfig->Rank));

    if (! IS_ADC_SAMPLE_TIME(hadc->Init.SamplingTimeCommon)) {
        assert_param(IS_ADC_SAMPLE_TIME(sConfig->SamplingTime));
    }

    __HAL_LOCK(hadc);

    if (ADC_IS_CONVERSION_ONGOING_REGULAR(hadc) == RESET) {
        hadc->Instance->CHSELR = ADC_CHSELR_CHANNEL(sConfig->Channel);

        if (! IS_ADC_SAMPLE_TIME(hadc->Init.SamplingTimeCommon)) {

            if (sConfig->SamplingTime != ADC_GET_SAMPLINGTIME(hadc)) {
                hadc->Instance->SMPR &= ~(ADC_SMPR_SMP);
                hadc->Instance->SMPR |= ADC_SMPR_SET(sConfig->SamplingTime);
            }
        }

        if(ADC_IS_CHANNEL_INTERNAL(sConfig->Channel)) {
            ADC->CCR |= ADC_CHANNEL_INTERNAL_PATH(sConfig->Channel);
            if (sConfig->Channel == ADC_CHANNEL_TEMPSENSOR) {
                wait_loop_index = ( 10 * (SystemCoreClock / 1000000U));
                while(wait_loop_index != 0U) {
                    wait_loop_index--;
                }
            }
        }
    } else {

        SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_CONFIG);
        tmp_hal_status = HAL_ERROR;
    }

    __HAL_UNLOCK(hadc);

    return tmp_hal_status;
}


/* PWM formavimo taimerio inicializavimas */
void HW_PwmTimerInit( void ){

    TIM_OC_InitTypeDef sConfigOC = {
        .OCMode = TIM_OCMODE_PWM1,
        .Pulse = 500,
        .OCPolarity = TIM_OCPOLARITY_HIGH,
        .OCNPolarity = TIM_OCNPOLARITY_HIGH,
        .OCFastMode = TIM_OCFAST_DISABLE,
        .OCIdleState = TIM_OCIDLESTATE_RESET,
        .OCNIdleState = TIM_OCNIDLESTATE_RESET
    };


    pPwmTimer->Instance = TIM17;
    pPwmTimer->Init.Prescaler = ( HAL_RCC_GetPCLK1Freq() / 1000000 ) - 1;;
    pPwmTimer->Init.CounterMode = TIM_COUNTERMODE_UP;
    pPwmTimer->Init.Period = 1000;
    pPwmTimer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    pPwmTimer->Init.RepetitionCounter = 0;
    pPwmTimer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(pPwmTimer) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    if (HAL_TIM_PWM_Init(pPwmTimer) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    if (HAL_TIM_PWM_ConfigChannel(pPwmTimer, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    if (HAL_TIM_PWM_Start(pPwmTimer, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
}



/* Formuoja 0-10V isejime itampa, atitinkamos parametrui reiksmes
Parametrai: volts -> 0-10 (Voltai)
*/
void HW_SetAnalogOut( float volts ){

    volts = (volts < 0) ? 0 : volts;
    volts = (volts > 10.00) ? 10.00 : volts;

    __HAL_TIM_SET_COMPARE( pPwmTimer, TIM_CHANNEL_1, volts * 75.5 );
}


/* Formuoja DAC isejime itampa
Parametrai: val -> 0-100 (procentai)
*/
void HW_SetDAC_Value( float dac ){

    dac = (dac < 0) ? 0 : dac;
    dac = (dac > 100) ? 100 : dac;

    if( HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, (uint32_t)(dac * 2.55) ) != HAL_OK ) Error_Handler();
}





/*  */
void HW_SystemReset( void ){
    NVIC_SystemReset();
}
