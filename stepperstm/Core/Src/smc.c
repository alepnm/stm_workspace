/******************************************************************************
* Project Name		: Stepper Motor Control
* File Name		    : stp.c
* Version 		    : x.x
* Device Used		: STM32F051C8x
* Software Used		: EmBitz 1.11
* Compiler    		: ARM GCC 4.9-2015-q1-update
* Related Hardware	:
*
* Owner             : Ventmatika Inc.
*******************************************************************************/

#include "smc.h"
#include "hardware.h"
#include "M25AA02.h"
#include "l6470.h"
#include "user_mb_app.h"


#define TEMP110_CAL_ADDR                ( (uint16_t*) ((uint32_t) 0x1FFFF7C2) )
#define TEMP30_CAL_ADDR                 ( (uint16_t*) ((uint32_t) 0x1FFFF7B8) )
#define VDD_CALIB                       ( (uint16_t) (330) )
#define VDD_APPLI                       ( (uint16_t) (300) )

/* ******************************** EXTERNS ******************************** */
extern ADC_HandleTypeDef        hadc;
extern DAC_HandleTypeDef        hdac1;
extern SPI_HandleTypeDef        hspi1;
extern TIM_HandleTypeDef*       pPwmTimer;

extern SmcHandle_TypeDef        SMC_Control;
extern SYSTIME_TypeDef          SysTimers;

extern const MotorParamSet*     MotorPresets[];
extern volatile uint32_t*       Timestamp;

/* ******************************** GLOBALS ******************************** */
MbPortParams_TypeDef MbPort = {
	.MbAddr = &usRegHoldingBuf[HR_MBADDR],

	.UART = {
			.Baudrate = &usRegHoldingBuf[HR_MBBAUDRATE],
			.Parity = &usRegHoldingBuf[HR_MBPARITY],
			.StopBits = &usRegHoldingBuf[HR_MBSTOPBITS]
	}
};

/* ******************************** STATICS ******************************** */
static MotorParamSet*   pUserSet = NULL;
static uint32_t*        pWTime = &SysTimers.WTime_sec;
static uint32_t         OverheatStopTimer = 0;
static uint32_t         ScrollCounter = 0;
static bool             CoolerOnBit = false;


/* **************************** EXTERN FUNCTIONS *************************** */
extern void SoundInit( bool ena );



bool SMC_Init ( void ){

    bool result = true;

/* MCU portu, registru inicializavimas */
    HAL_GPIO_WritePin(L6470_SS_GPIO_Port, L6470_SS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(M25AA_CS_GPIO_Port, M25AA_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(HC165_SS_GPIO_Port, HC165_SS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(HC165_LATCH_GPIO_Port, HC165_LATCH_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(HCCTRL_GPIO_Port, HCCTRL_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(L6470_RST_GPIO_Port, L6470_RST_Pin, GPIO_PIN_SET);

    STATUS_LED_OFF();
    FAULT_LED_OFF();

    RELAY_OFF();
    COOLER_OFF();

    while(HAL_ADCEx_Calibration_Start(&hadc) != HAL_OK);

    if( HAL_DAC_Start(&hdac1, DAC_CHANNEL_1) != HAL_OK ){
        _Error_Handler(__FILE__, __LINE__);
    }

/* SPI irenginiu inicializavimas */
//    M25AA02_SetWriteStatus( DISABLE );
    SMC_ReadDipSwitch( );
    L6470_Init();


/* Kiti moduliai */
    HW_PwmTimerInit();

    SoundInit( true );


/* MODBUS registru inicializavimas */
    /* COILS registrai */
    xMbSetCoil( CO_RELAY_ON , RESET );
    xMbSetCoil( CO_COOLER_ON, RESET );
    xMbSetCoil( CO_DRV_REBOOT, RESET );
    xMbSetCoil( CO_COMPENSATION_NLHT, SET );

    /* DISCREET INPUT registrai */
    xMbSetDInput( DI_TH_ALARM, RESET );
    xMbSetDInput( DI_OCD_ALARM, RESET );
    xMbSetDInput( DI_HALL_STATE, RESET );
    xMbSetDInput( DI_HALL_FAULT_FLAG, RESET );

    /* HOLDING registrai */
    usRegHoldingBuf[HR_ROT_SPEED] =                 0;
    usRegHoldingBuf[HR_MICROSTEPS] =                MICROSTEPS_DEF;
    usRegHoldingBuf[HR_HS_TO_VALUE] =               HS_TO_VALUE_DEF;
    usRegHoldingBuf[HR_TRANSMISSION_RATIO] =        TRANSMISSION_RATIO_DEF;

    usRegHoldingBuf[HR_SCROLL_RPM] =                SCROLL_RPM_DEF;
    usRegHoldingBuf[HR_SCROLL_OFF_CYCLE_TIME] =     SCROLL_OFF_CYCLE_TIME_DEF;
    usRegHoldingBuf[HR_SCROLL_ON_CYCLE_TIME] =      SCROLL_ON_CYCLE_TIME_DEF;
    usRegHoldingBuf[HR_SCROLL_SYNC] =               SCROLL_SYNC_DEF;

    usRegHoldingBuf[HR_MBADDR] =                    MBADDR_DEF;
    usRegHoldingBuf[HR_MBBAUDRATE] =                MBBAURATE_DEF;
    usRegHoldingBuf[HR_MBPARITY] =                  MBPARITY_DEF;
    usRegHoldingBuf[HR_MBSTOPBITS] =                MBSTOPBITS_DEF;

    usRegHoldingBuf[HR_OVH_TIMEOUT] =               OVH_TIMEOUT_DEF;

    usRegHoldingBuf[HR_USERSET_STEPS_PER_REV] =     USERSET_STEPS_PER_REV_DEF;
    usRegHoldingBuf[HR_USERSET_KVAL_RUN] =          USERSET_KVAL_RUN_PROC_DEF;
    usRegHoldingBuf[HR_USERSET_KVAL_ACC] =          USERSET_KVAL_ACC_PROC_DEF;
    usRegHoldingBuf[HR_USERSET_KVAL_DEC] =          USERSET_KVAL_DEC_PROC_DEF;
    usRegHoldingBuf[HR_USERSET_KVAL_HOLD] =         USERSET_KVAL_HOLD_PROC_DEF;
    usRegHoldingBuf[HR_USERSET_TRES_OCD] =          USERSET_TRES_OCD_MA_DEF;
    usRegHoldingBuf[HR_USERSET_TRES_STALL] =        USERSET_TRES_STALL_MA_DEF;
    usRegHoldingBuf[HR_USERSET_SPEED_ACC] =         USERSET_SPEED_ACC_DEF;
    usRegHoldingBuf[HR_USERSET_SPEED_DEC] =         USERSET_SPEED_DEC_DEF;


    /* INPUT registrai */
    usRegInputBuf[IR_SPREQ_VALUE] =                 0x00;
    usRegInputBuf[IR_VBUS_VALUE] =                  0x00;
    usRegInputBuf[IR_CURR_RPM] =                    0x00;
    usRegInputBuf[IR_FAULT_CODE] =                  0x00;
    usRegInputBuf[IR_HS_TO_COUNTER] =               usRegHoldingBuf[HR_HS_TO_VALUE] * 6;
    usRegInputBuf[IR_WTIMEHI] =                     LO16(SysTimers.WTime_sec);
    usRegInputBuf[IR_WTIMELO] =                     HI16(SysTimers.WTime_sec);

    SMC_Control.StrData.pName =                     ucSlaveIdBuf;
    SMC_Control.StrData.pVersion =                  ucSlaveIdBuf + 4;
    SMC_Control.StrData.pId =                       ucSlaveIdBuf + 20;

    /* vartotojo preseto inicializavimas */
    pUserSet = (MotorParamSet*)SMC_GetPresetPtr(0x07);
    pUserSet->StepsPerRev =                         usRegHoldingBuf[HR_USERSET_STEPS_PER_REV];
    pUserSet->Kval.RunValue =                       usRegHoldingBuf[HR_USERSET_KVAL_RUN];
    pUserSet->Kval.AccValue =                       usRegHoldingBuf[HR_USERSET_KVAL_ACC];
    pUserSet->Kval.DecValue =                       usRegHoldingBuf[HR_USERSET_KVAL_DEC];
    pUserSet->Kval.HoldValue =                      usRegHoldingBuf[HR_USERSET_KVAL_HOLD];
    pUserSet->Treshold.OcdValue =                   usRegHoldingBuf[HR_USERSET_TRES_OCD];
    pUserSet->Treshold.StallValue =                 usRegHoldingBuf[HR_USERSET_TRES_STALL];
    pUserSet->Speed.Acceleration =                  usRegHoldingBuf[HR_USERSET_SPEED_ACC];
    pUserSet->Speed.Deceleration =                  usRegHoldingBuf[HR_USERSET_SPEED_DEC];

    SMC_Control.MotorData.pCurrentMotorPreset =     SMC_GetPresetPtr( SMC_Control.DipSwitch.Option.MotorType );

    if( SMC_MotorConfig(SMC_Control.MotorData.pCurrentMotorPreset) == false ) _Error_Handler(__FILE__, __LINE__);

    return result;
}


/* Modbus duomenu atnaujinimas */
void SMC_DataUpdate( void ){

    __enter_critical();

    /* tikrinam modbus registru reiksmes - jai jos pasikeite, grazinam i nustatyta diapazona */
    if( usRegHoldingBuf[HR_MBADDR] < MB_ADDRESS_MIN || usRegHoldingBuf[HR_MBADDR] > MB_ADDRESS_MAX ) usRegHoldingBuf[HR_MBADDR] = MBADDR_DEF;
    if( SMC_CheckBaudrateValue( usRegHoldingBuf[HR_MBBAUDRATE] ) == false ) usRegHoldingBuf[HR_MBBAUDRATE] = MBBAURATE_DEF;

    /* tikrinam variklio apsuku nusatymo ribas nuo -IR_MAX_PRM iki +IR_MAX_RPM */
    if( usRegHoldingBuf[HR_ROT_SPEED] != 0 ){
        if( (int16_t)usRegHoldingBuf[HR_ROT_SPEED] < 0 - usRegInputBuf[IR_MAX_RPM] ) usRegHoldingBuf[HR_ROT_SPEED] = 0 - usRegInputBuf[IR_MAX_RPM];
        if( (int16_t)usRegHoldingBuf[HR_ROT_SPEED] > usRegInputBuf[IR_MAX_RPM] ) usRegHoldingBuf[HR_ROT_SPEED] = usRegInputBuf[IR_MAX_RPM];
    }

    usRegHoldingBuf[HR_MICROSTEPS] &= 0x00FF;
    if( SMC_CheckUStepValue( (uint8_t)usRegHoldingBuf[HR_MICROSTEPS] ) == false ) usRegHoldingBuf[HR_MICROSTEPS] = MICROSTEPS_DEF;

    /*  */
    if( usRegHoldingBuf[HR_HS_TO_VALUE] > 600 ) usRegHoldingBuf[HR_HS_TO_VALUE] = HS_TO_VALUE_DEF;

    if( usRegHoldingBuf[HR_SCROLL_RPM] > usRegInputBuf[IR_MAX_RPM] ) usRegHoldingBuf[HR_SCROLL_RPM] = usRegInputBuf[IR_MAX_RPM];

    if( usRegHoldingBuf[HR_SCROLL_OFF_CYCLE_TIME] < 30 ) usRegHoldingBuf[HR_SCROLL_OFF_CYCLE_TIME] = 30;
    if( usRegHoldingBuf[HR_SCROLL_OFF_CYCLE_TIME] > 600 ) usRegHoldingBuf[HR_SCROLL_OFF_CYCLE_TIME] = SCROLL_OFF_CYCLE_TIME_DEF;

    if( usRegHoldingBuf[HR_SCROLL_ON_CYCLE_TIME] < 10 ) usRegHoldingBuf[HR_SCROLL_ON_CYCLE_TIME] = 10;
    if( usRegHoldingBuf[HR_SCROLL_ON_CYCLE_TIME] > 60 ) usRegHoldingBuf[HR_SCROLL_ON_CYCLE_TIME] = SCROLL_ON_CYCLE_TIME_DEF;

    if( usRegHoldingBuf[HR_SCROLL_SYNC] > 10 ) usRegHoldingBuf[HR_SCROLL_SYNC] = SCROLL_SYNC_DEF;

    /*  */
    if( usRegHoldingBuf[HR_USERSET_STEPS_PER_REV] > 200 ) usRegHoldingBuf[HR_USERSET_STEPS_PER_REV] = USERSET_STEPS_PER_REV_DEF;

    if( usRegHoldingBuf[HR_USERSET_KVAL_RUN] > MAX_KVAL_VALUE_DEF ) usRegHoldingBuf[HR_USERSET_KVAL_RUN] = MAX_KVAL_VALUE_DEF;
    if( usRegHoldingBuf[HR_USERSET_KVAL_ACC] > MAX_KVAL_VALUE_DEF ) usRegHoldingBuf[HR_USERSET_KVAL_ACC] = MAX_KVAL_VALUE_DEF;
    if( usRegHoldingBuf[HR_USERSET_KVAL_DEC] > MAX_KVAL_VALUE_DEF ) usRegHoldingBuf[HR_USERSET_KVAL_DEC] = MAX_KVAL_VALUE_DEF;
    if( usRegHoldingBuf[HR_USERSET_KVAL_HOLD] > MAX_KVAL_HOLD_VALUE_DEF ) usRegHoldingBuf[HR_USERSET_KVAL_RUN] = MAX_KVAL_HOLD_VALUE_DEF;

    if( usRegHoldingBuf[HR_USERSET_TRES_OCD] < MIN_TRES_OCD_MA_DEF ) usRegHoldingBuf[HR_USERSET_TRES_OCD] = MIN_TRES_OCD_MA_DEF;
    if( usRegHoldingBuf[HR_USERSET_TRES_OCD] > MAX_TRES_OCD_MA_DEF ) usRegHoldingBuf[HR_USERSET_TRES_OCD] = MAX_TRES_OCD_MA_DEF;

    if( usRegHoldingBuf[HR_USERSET_TRES_STALL] < 50 ) usRegHoldingBuf[HR_USERSET_TRES_STALL] = 50;
    if( usRegHoldingBuf[HR_USERSET_TRES_STALL] > 2500 ) usRegHoldingBuf[HR_USERSET_TRES_STALL] = USERSET_TRES_STALL_MA_DEF;

    if( usRegHoldingBuf[HR_MIN_RPM] < RPM_MIN_DEF ) usRegHoldingBuf[HR_MIN_RPM] = RPM_MIN_DEF;
    if( usRegHoldingBuf[HR_MAX_RPM] > usRegInputBuf[IR_MAX_RPM] ) usRegHoldingBuf[HR_MAX_RPM] = usRegInputBuf[IR_MAX_RPM];

    if( usRegHoldingBuf[HR_USERSET_SPEED_ACC] < 10 ) usRegHoldingBuf[HR_USERSET_SPEED_ACC] = 10;
    if( usRegHoldingBuf[HR_USERSET_SPEED_ACC] > 500 ) usRegHoldingBuf[HR_USERSET_SPEED_ACC] = USERSET_SPEED_ACC_DEF;

    if( usRegHoldingBuf[HR_USERSET_SPEED_DEC] < 10 ) usRegHoldingBuf[HR_USERSET_SPEED_DEC] = 10;
    if( usRegHoldingBuf[HR_USERSET_SPEED_DEC] > 500 ) usRegHoldingBuf[HR_USERSET_SPEED_DEC] = USERSET_SPEED_DEC_DEF;

    /* atnaujinam UserSet parametrus */
    pUserSet->StepsPerRev = usRegHoldingBuf[HR_USERSET_STEPS_PER_REV];
    pUserSet->Kval.RunValue = usRegHoldingBuf[HR_USERSET_KVAL_RUN];
    pUserSet->Kval.AccValue = usRegHoldingBuf[HR_USERSET_KVAL_ACC];
    pUserSet->Kval.DecValue = usRegHoldingBuf[HR_USERSET_KVAL_DEC];
    pUserSet->Kval.HoldValue = usRegHoldingBuf[HR_USERSET_KVAL_HOLD];
    pUserSet->Treshold.OcdValue = usRegHoldingBuf[HR_USERSET_TRES_OCD];
    pUserSet->Treshold.StallValue = usRegHoldingBuf[HR_USERSET_TRES_STALL];
    pUserSet->Speed.Acceleration = usRegHoldingBuf[HR_USERSET_SPEED_ACC];
    pUserSet->Speed.Deceleration = usRegHoldingBuf[HR_USERSET_SPEED_DEC];

    usRegHoldingBuf[HR_DAC_OUT_VAL] = ( usRegHoldingBuf[HR_DAC_OUT_VAL] > 100 ) ? 100 : usRegHoldingBuf[HR_DAC_OUT_VAL];
    usRegHoldingBuf[HR_PWM_OUT_VAL] = ( usRegHoldingBuf[HR_PWM_OUT_VAL] > 1000 ) ? 1000 : usRegHoldingBuf[HR_PWM_OUT_VAL];

    /* irasom wtime */
    usRegInputBuf[IR_WTIMEHI] = LO16(SysTimers.WTime_sec);
    usRegInputBuf[IR_WTIMELO] = HI16(SysTimers.WTime_sec);

    /* irasom holo daviklio busena */
    xMbSetDInput( DI_HALL_STATE, (uint8_t)HAL_GPIO_ReadPin(HALL_S_GPIO_Port, HALL_S_Pin) );

    xMbSetDInput( DI_HALL_FAULT_FLAG, (uint8_t)( usRegInputBuf[IR_FAULT_CODE] && FLT_HW_HS ) );
    xMbSetDInput( DI_TH_ALARM, (uint8_t)( usRegInputBuf[IR_FAULT_CODE] && FLT_HW_TH_SHUTDOWN ) );

    if( (SMC_Control.MotorData.Status & STATUS_MOT_STATUS) == STATUS_MOT_STATUS_STOPPED ) xMbSetDInput( DI_MOTOR_STATE, FALSE );
    else xMbSetDInput( DI_MOTOR_STATE, TRUE );

    xMbSetDInput( DI_OCD_ALARM, (uint8_t)!READ_BIT( SMC_Control.MotorData.Status, STATUS_OCD ) );

    /* skaitom ADC_IN registra L6470 ir ji irasom */
    usRegInputBuf[IR_ADCIN_VALUE] = (uint16_t)GetParam(REG_ADC_OUT);

    __exit_critical();
}



/* Grazina pointeri i aktyvu preseta */
const MotorParamSet* SMC_GetPresetPtr(uint8_t preset_id) {
    return MotorPresets[preset_id];
}


/*  */
void SMC_ReadDipSwitch( void ){

    HAL_GPIO_WritePin(HC165_LATCH_GPIO_Port, HC165_LATCH_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(HC165_LATCH_GPIO_Port, HC165_LATCH_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(HC165_SS_GPIO_Port, HC165_SS_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(HCCTRL_GPIO_Port, HCCTRL_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(HCCTRL_GPIO_Port, HCCTRL_Pin, GPIO_PIN_SET);

    (void)HAL_SPI_Receive(&hspi1, &(SMC_Control.DipSwitch.Data), 1, 10);
    HAL_GPIO_WritePin(HC165_SS_GPIO_Port, HC165_SS_Pin, GPIO_PIN_SET);


    /* issaugojam nuskaitytus parametrus modbus registre */
    xMbSetDInput( DI_SW1_STATE, SMC_Control.DipSwitch.Data & 0x01 );
    xMbSetDInput( DI_SW2_STATE, SMC_Control.DipSwitch.Data>>1 & 0x01 );
    xMbSetDInput( DI_SW3_STATE, SMC_Control.DipSwitch.Data>>2 & 0x01 );
    xMbSetDInput( DI_SW4_STATE, SMC_Control.DipSwitch.Data>>3 & 0x01 );
    xMbSetDInput( DI_SW5_STATE, SMC_Control.DipSwitch.Data>>4 & 0x01 );
    xMbSetDInput( DI_SW6_STATE, SMC_Control.DipSwitch.Data>>5 & 0x01 );
    xMbSetDInput( DI_SW7_STATE, SMC_Control.DipSwitch.Data>>6 & 0x01 );
    xMbSetDInput( DI_SW8_STATE, SMC_Control.DipSwitch.Data>>7 & 0x01 );


    uint8_t inverted = SMC_Control.DipSwitch.Data^0xFF;

    /* jai esam STOP rezime nustatom parametrus */
    SMC_Control.DipSwitch.Option.MotorType = ( inverted & 0x07 );
    usRegInputBuf[IR_MAX_RPM] = ( READ_BIT(inverted, 0x01<<3) == FALSE ) ? 150 : 200;
    SMC_Control.DipSwitch.Option.Scrolling = ( inverted>>4 & 0x01 );
    SMC_Control.DipSwitch.Option.HallSensor = ( inverted>>5 & 0x01 );

    SMC_Control.ControlMode =  ( inverted>>6 & 0x03 );
}



/*   */
void SMC_ReadDigitalInputs( void ){

    xMbSetDInput( DI_DI0_STATE, DI0_STATE() );
    xMbSetDInput( DI_DI1_STATE, DI1_STATE() );
    xMbSetDInput( DI_DI2_STATE, DI2_STATE() );
    xMbSetDInput( DI_DI3_STATE, DI3_STATE() );

}



/* Suvidurkintus rezultatus sudedam i tam skirtus registrus */
void SMC_ReadAnalogInputs( void ){
    ADC_ChannelConfTypeDef sConfig = {
        .Channel = ADC_CHANNEL_0,
        .Rank = ADC_RANK_CHANNEL_NUMBER,
        .SamplingTime = ADC_SAMPLETIME_55CYCLES_5
    };

    static uint8_t stage = 0, n_spreq = 0, n_vbus = 0, n_itemp = 0;
    static uint32_t sum_spreq = 0, sum_vbus = 0, sum_itemp = 0;

    __enter_critical();

    switch(stage) {
    case 0:

        if(n_vbus++ < 64) sum_vbus += HW_GetAdcValue(&sConfig);
        else {

            SMC_Control.ADC_Vals.Vbus = (uint16_t)(sum_vbus>>6);
            usRegInputBuf[IR_VBUS_VALUE] = SMC_Control.ADC_Vals.Vbus * 0.822;   // verciam voltais  ( formatas V*100 )
            n_vbus = sum_vbus = 0;
        }

        stage = 1;
        break;
    case 1:

        sConfig.Channel = ADC_CHANNEL_1;

        uint16_t adc = HW_GetAdcValue(&sConfig);

        /* filtruojam triuksma ir vidurkinam ADC reiksme */
        if( SMC_Control.ADC_Vals.SpReq < adc - 20 || SMC_Control.ADC_Vals.SpReq > adc + 20 ) {

            if(n_spreq++ < 8) sum_spreq += adc;
            else {

                SMC_Control.ADC_Vals.SpReq = (uint16_t)(sum_spreq>>3);
                usRegInputBuf[IR_SPREQ_VALUE] = SMC_Control.ADC_Vals.SpReq * 0.235;   // verciam voltais  ( formatas V*100 )
                n_spreq = sum_spreq = 0;
            }
        }

        stage = 2;
        break;
    case 2:

        sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
        sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;

        if(n_itemp++ < 8) sum_itemp += HW_GetAdcValue(&sConfig);
        else {

            SMC_Control.ADC_Vals.McuTemp  = (int32_t) (sum_itemp>>3);

            int32_t temperature = ((SMC_Control.ADC_Vals.McuTemp * VDD_APPLI / VDD_CALIB) - (int32_t) *TEMP30_CAL_ADDR );
            temperature = temperature * (int32_t)(110 - 30);
            usRegInputBuf[IR_MCUTEMP] = (uint16_t)(temperature / (int32_t)(*TEMP110_CAL_ADDR - *TEMP30_CAL_ADDR) + 30);

            n_itemp = sum_itemp = 0;
        }

        stage = 0;
        break;
    }

    __exit_critical();
}



/*
*/
uint8_t SMC_ScrollingHandler( void ){

    static int8_t sync_counter = 0;
    static SMC_StateTypeDef state = FSM_STATE_STOP;
    static uint16_t last_time;

    /* jai Scrolling funkcija isjungta, arba nera nustatytas jos apsukos iseinam */
    if( SMC_Control.DipSwitch.Option.Scrolling == DISABLE || usRegHoldingBuf[HR_SCROLL_RPM] == 0 ) {

        ScrollCounter = *Timestamp + (usRegHoldingBuf[HR_SCROLL_OFF_CYCLE_TIME] * 1000);
        sync_counter = 0;
        state = FSM_STATE_STOP;
        return false;
    }

    if( ScrollCounter >= *Timestamp) {

        if( state == FSM_STATE_STOP ) {

            if( last_time != usRegHoldingBuf[HR_SCROLL_OFF_CYCLE_TIME] ) {
                /* jai pasikeite scrollingo nustatymai modbase, resetinam taimeri */
                ScrollCounter = *Timestamp + (usRegHoldingBuf[HR_SCROLL_OFF_CYCLE_TIME] * 1000);

                last_time = usRegHoldingBuf[HR_SCROLL_OFF_CYCLE_TIME];
            }

            return false;

        } else {

            if( last_time != usRegHoldingBuf[HR_SCROLL_ON_CYCLE_TIME] ) {
                /* jai pasikeite scrollingo nustatymai modbase, resetinam taimeri */
                ScrollCounter = *Timestamp + (usRegHoldingBuf[HR_SCROLL_ON_CYCLE_TIME] * 1000);

                last_time = usRegHoldingBuf[HR_SCROLL_ON_CYCLE_TIME];
            }

            return true;
        }
    }

    bool dir = SMC_Control.MotorData.Status & STATUS_DIR;

    switch(state) {

    case FSM_STATE_STOP:

        ScrollCounter = *Timestamp + ( usRegHoldingBuf[HR_SCROLL_ON_CYCLE_TIME] * 1000 );

        /* jai ScrolSync naudojam, atliekam ja */
        if( usRegHoldingBuf[HR_SCROLL_SYNC] > 0 ) {
            if( sync_counter-- <= 0 ) {
                sync_counter = usRegHoldingBuf[HR_SCROLL_SYNC]-1;
                dir = !dir;

                /* CW kriptimi darom vienu prasukimu daugiau */
                if( dir == CW ) sync_counter++;
            }
        }

        L6470_run( dir, ConvertRpmToStepsPerSec( usRegHoldingBuf[HR_SCROLL_RPM] ) );

        state = FSM_STATE_SCROLLING;

        break;
    case FSM_STATE_SCROLLING:

        ScrollCounter = *Timestamp + ( usRegHoldingBuf[HR_SCROLL_OFF_CYCLE_TIME] * 1000 );

        L6470_softFree();
        state = FSM_STATE_STOP;

        break;
    default:
        break;
    }

    return ( state == FSM_STATE_STOP ) ? false : true;
}


/*  */
uint8_t SMC_HallSensorHandler( void ){

    static uint32_t delay;

    /* jai Hall Sensor funkcija isjungta, arba nera nustatytas holo daviklio taimautas iseinam */
    if( SMC_Control.DipSwitch.Option.HallSensor == DISABLE || usRegHoldingBuf[HR_HS_TO_VALUE] == 0x00 ) {

        usRegInputBuf[IR_HS_TO_COUNTER] = 60;
        CLEAR_BIT( usRegInputBuf[IR_FAULT_CODE], FLT_HW_HS );
        return true;
    }

    if( delay > *Timestamp) return true;

    delay = *Timestamp + 1000;

    if( usRegInputBuf[IR_HS_TO_COUNTER] > 0 ) usRegInputBuf[IR_HS_TO_COUNTER]--;

    if( usRegInputBuf[IR_HS_TO_COUNTER] == 0 ) {
        /* holo daviklio klaida */
        SET_BIT( usRegInputBuf[IR_FAULT_CODE], FLT_HW_HS );
        return false;
    }

    return true;
}



/* Ausintuvo hendleris
Kuleris aktyvuojamas L6470 overheat signalu arba Modbus CO_COOLER_ON registru
T >= 20ms, main
*/
void SMC_CoolerController( void ){

    /* jai draiverio perkaitimas, aktyvuojam kuleri, jai ne - pagal COOLER Modbus bituka */
    if( READ_BIT( SMC_Control.MotorData.Status, STATUS_TH_WRN) == RESET ) {

        OverheatStopTimer = *Timestamp + usRegHoldingBuf[HR_OVH_TIMEOUT] * 1000; // uztaisom perkaitimo apsaugos taimeri
        CoolerOnBit = true;
    } else {

        /* jai nustatytas kulerio MODBUS bitas */
        if( xMbGetCoil( CO_COOLER_ON ) != FALSE ) {
            CoolerOnBit = true;
        } else {

            /* jai baigesi OverheatStopTimer taimerio laikas ir is draiverio negaunam Overheat alarma, numetam alarmo bita */
            if( OverheatStopTimer < *Timestamp ) {
                CoolerOnBit = false;
            }
        }
    }
}


/* Reles handleris
Alarm rele aktyvuojama esant kritinei klaidai arba Modbus CO_RELAY_ON registru
Kritines klaidos:
1.
2.
3.
4.

T >= 20ms, main
*/
void SMC_RelayController( void ){

    if( xMbGetCoil( CO_RELAY_ON ) != FALSE ) RELAY_ON();
    else RELAY_OFF();
}


/* Status ir Fault indikatoriu valdiklis.
Draiverio ledas rodo draiverio Overheat ir Overcurrent alarmus
*/
void SMC_LedsController( void ){

    static uint32_t delay;
    uint16_t timeout = 0;

    switch(SMC_Control.SMC_State) {

    case FSM_STATE_FAULT:

        /* FAULT ledas greitai mirkcioja. Klaidos matome modbus registre */
        if(delay < *Timestamp) {

            delay = *Timestamp + 100;

            //HAL_GPIO_TogglePin( FAULT_LED_GPIO_Port, FAULT_LED_Pin );
            FAULT_LED_TOGGLE();
        }

        /* STATUS leda panaudojam klaidos kodo parodymui */
        //HAL_GPIO_WritePin( STATUS_LED_GPIO_Port, STATUS_LED_Pin, LED_OFF);
        STATUS_LED_OFF();

        break;
    default:

        /* gesinam FAULT leda */
        //HAL_GPIO_WritePin( FAULT_LED_GPIO_Port, FAULT_LED_Pin, LED_OFF );
        FAULT_LED_OFF();


        if( HAL_GPIO_ReadPin(HALL_S_GPIO_Port, HALL_S_Pin) == GPIO_PIN_SET) {
            /* jai holo daviklis aktyvus, uzdegam STATUS leda */
            //HAL_GPIO_WritePin( STATUS_LED_GPIO_Port, STATUS_LED_Pin, LED_ON );
            STATUS_LED_ON();
            delay = *Timestamp + 100;   // <-- sumazinam uzdelsima, kad greiciau uzgestu
        }

        if(delay < *Timestamp) {

            if( ( SMC_Control.MotorData.Status & STATUS_MOT_STATUS ) == STATUS_MOT_STATUS_CONST_SPD ) {
                /* kai variklis sukasi, STATUS ledu mirkciojam kas 500 ms */
                timeout = 500;
            } else {
                /* kai variklis stovi, STATUS ledu mirkciojam leciau - kas 5000 ms */
                timeout = 5000;
            }

            if( HAL_GPIO_ReadPin(STATUS_LED_GPIO_Port, STATUS_LED_Pin) == LED_OFF ) {

                //HAL_GPIO_WritePin( STATUS_LED_GPIO_Port, STATUS_LED_Pin, LED_ON );
                STATUS_LED_ON();

                timeout = 50;
            } else {

                //HAL_GPIO_WritePin( STATUS_LED_GPIO_Port, STATUS_LED_Pin, LED_OFF );
                STATUS_LED_OFF();
            }

            delay = *Timestamp + timeout;
        }

        break;
    }
}



/* Chekinam klaidos */
void SMC_Check_Faults( void ){

//    CLEAR_BIT( usRegInputBuf[IR_FAULT_CODE], FLT_HW_ULVO);
//    CLEAR_BIT( usRegInputBuf[IR_FAULT_CODE], FLT_HW_OCD );
//    CLEAR_BIT( usRegInputBuf[IR_FAULT_CODE], FLT_HW_TH_WRN );
//    CLEAR_BIT( usRegInputBuf[IR_FAULT_CODE], FLT_HW_TH_SHUTDOWN );


    /* tikrinam L6470 draiverio alarmus */

    /* tikrinam UNDERVOLTAGE flaga */
    if( READ_BIT( SMC_Control.MotorData.Status, STATUS_UVLO ) == RESET ) {
        SET_BIT( usRegInputBuf[IR_FAULT_CODE], FLT_HW_ULVO );
    } else {
        CLEAR_BIT( usRegInputBuf[IR_FAULT_CODE], FLT_HW_ULVO );
    }

    /* tikrinam OVERCURRENT flaga */
    if( READ_BIT( SMC_Control.MotorData.Status, STATUS_OCD ) == RESET ){
        //SET_BIT( usRegInputBuf[IR_FAULT_CODE], FLT_HW_OCD );
    }else{
        //CLEAR_BIT( usRegInputBuf[IR_FAULT_CODE], FLT_HW_OCD );
    }

    /* tikrinam THERMAL_WARNING flaga */
    if( READ_BIT( SMC_Control.MotorData.Status, STATUS_TH_WRN ) == RESET ){
        //SET_BIT( usRegInputBuf[IR_FAULT_CODE], FLT_HW_TH_WRN );
    }else{
        //CLEAR_BIT( usRegInputBuf[IR_FAULT_CODE], FLT_HW_TH_WRN );
    }

    /* tikrinam THERMAL_SHUTDOWN flaga */
    if( READ_BIT( SMC_Control.MotorData.Status, STATUS_TH_SD ) == RESET ){
        SET_BIT( usRegInputBuf[IR_FAULT_CODE], FLT_HW_TH_SHUTDOWN );
    }else{
        CLEAR_BIT( usRegInputBuf[IR_FAULT_CODE], FLT_HW_TH_SHUTDOWN );
    }


    /* STPSTM plokstes faultai */
    CLEAR_BIT( usRegInputBuf[IR_FAULT_CODE], FLT_HW_VBUS );
    CLEAR_BIT( usRegInputBuf[IR_FAULT_CODE], FLT_HW_VBUS_LOW );
    CLEAR_BIT( usRegInputBuf[IR_FAULT_CODE], FLT_HW_VBUS_HIGH );

    if( usRegInputBuf[IR_VBUS_VALUE] < 800 ) {
        /* nera draiverio maitinimo VBUS - < 8 voltu*/
        SET_BIT( usRegInputBuf[IR_FAULT_CODE], FLT_HW_VBUS );
    }else   if( usRegInputBuf[IR_VBUS_VALUE] > 800 && usRegInputBuf[IR_VBUS_VALUE] < 2200 ) {
                /* zemas draiverio maitinimas VBUS - nuo 8 iki 22 voltu*/
                SET_BIT( usRegInputBuf[IR_FAULT_CODE], FLT_HW_VBUS_LOW );
            }else   if( usRegInputBuf[IR_VBUS_VALUE] > 3600 ) {
                        /* aukstas draiverio maitinimas VBUS - virs 36 voltu */
                        SET_BIT( usRegInputBuf[IR_FAULT_CODE], FLT_HW_VBUS_HIGH );
                    }
}


/* chekinam bodreito reiksme - ar standartine? */
bool SMC_CheckBaudrateValue(uint32_t baudrate){

    const uint32_t baudrates[6U] = { 4800U, 9600U, 19200U, 38400U, 57600U, 115200U };
    uint8_t i = 0;

    while( baudrate != baudrates[i++] ) {
        if( i >= ( sizeof(baudrates)/sizeof(baudrate) ) ) return false;
    }

    return true;
}


/* chekinam mikrostepo reiksme - ar standartine? */
bool SMC_CheckUStepValue(uint8_t uStep){

    const uint8_t uStepTable[8U] = { 1U, 2U, 4U, 8U, 16U, 32U, 64U, 128U };
    uint8_t i = 0;

    while( uStep != uStepTable[i++] ) {
        if( i >= ( sizeof(uStepTable)/sizeof(uStep) ) ) return false;
    }

    return true;
}


/* Sisteminiai taimeriai */
void SMC_SysTimeCounterUpdate( void ){

    static uint32_t time = 0u;

    *Timestamp = HAL_GetTick();

    if( time <= *Timestamp ) {
        time = *Timestamp + 1000u;
        (*pWTime)++;
    }
}



/* Draiverio konfiguravimas aktyviam presetui */
bool SMC_MotorConfig( const MotorParamSet* preset ){

    __enter_critical();

    /* L6470 registru inicializavimas */
    L6470_setMicroSteps( usRegHoldingBuf[HR_MICROSTEPS] );

    SetParam( REG_KVAL_RUN, PROC_8BIT(preset->Kval.RunValue) );
    SetParam( REG_KVAL_ACC, PROC_8BIT(preset->Kval.AccValue) );
    SetParam( REG_KVAL_DEC, PROC_8BIT(preset->Kval.DecValue) );
    SetParam( REG_KVAL_HOLD, PROC_8BIT(preset->Kval.HoldValue) );

    L6470_setMinSpeed( ConvertRpmToStepsPerSec(RPM_MIN_DEF) );
    L6470_setMaxSpeed( ConvertRpmToStepsPerSec(usRegInputBuf[IR_MAX_RPM]) );

    L6470_setThresholdSpeed( ConvertRpmToStepsPerSec(300) );

    L6470_setAcc( preset->Speed.Acceleration );
    L6470_setDec( preset->Speed.Deceleration );

    L6470_setOverCurrent( preset->Treshold.OcdValue );
    L6470_setStallCurrent( preset->Treshold.StallValue );

    __exit_critical();

    return true;
}

/*  */
bool SMC_MotorStart( void ){

    L6470_run( (byte)SMC_Control.MotorData.RotDirSetting, ConvertRpmToStepsPerSec( (float)SMC_Control.MotorData.RotSpeedSetting) );

    return true;
}

/*  */
bool SMC_MotorReverse( void ){

    SMC_Control.MotorData.RotDirSetting = !SMC_Control.MotorData.RotDirSetting;

    SMC_MotorStart( );

    return true;
}

/*  */
bool SMC_MotorStartParam( DIR_TypeDef dir, float speed ){

    L6470_run( (byte)dir, ConvertRpmToStepsPerSec( speed ) );

    return true;
}

/*  */
bool SMC_MotorStop( void ){

    L6470_softFree( );

    return true;
}

/*  */
bool SMC_GetMotorStatusRegister( void ){

    SMC_Control.MotorData.Status = L6470_getStatus();

    return true;
}

/*  */
bool SMC_GetMotorCurrentSpeed( void ){

    /* apskaiciuojam momentini variklio greiti RPMais */
    float speed = L6470_getSpeed( ) * 0.01490116;

    usRegInputBuf[IR_CURR_RPM] = ( speed * 60 ) / SMC_Control.MotorData.pCurrentMotorPreset->StepsPerRev;

    /* koreguojam ismatuota momentini greiti, nes REG_SPEED persipildo prie ~148 aps/min  */
    if( SMC_Control.MotorData.RotSpeedSetting > 140 && usRegInputBuf[IR_CURR_RPM] < 60 ) usRegInputBuf[IR_CURR_RPM] += 150;

    return true;
}
