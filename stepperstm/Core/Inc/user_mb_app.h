/******************************************************************************
* Project Name		: Stepper Motor Control
* File Name			: user_mb_app.h
* Version 			: 1.0
* Device Used		: CY8C4245AXI-483
* Software Used		: PSoC Creator 4.0
* Compiler    		: ARM GCC 4.9-2015-q1-update
* Related Hardware	:
*
* Owner             : Ventmatika Inc.
*******************************************************************************/

#ifndef	USER_APP
#define USER_APP
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbconfig.h"

/* -----------------------Slave Defines -------------------------------------*/
#define     DISCRETE_INPUT_START        0
#define     DISCRETE_INPUT_NDISCRETES   64
#define     COIL_START                  0
#define     COIL_NCOILS                 64
#define     REG_INPUT_START             0
#define     REG_INPUT_NREGS             16
#define     REG_HOLDING_START           0
#define     REG_HOLDING_NREGS           64


/* COILS */
//#define     CO_START                    ( 1u )  // start(true)/stop(false)
//#define     CO_DIRECTION                ( 2u )  // sukimosi kriptis: CW(true)/CCW(false)
#define     CO_DRV_REBOOT               ( 8u )  // draiverio reinicializacija(true)

/* DESCREETS */
#define     DI_OVH_ALARM                ( 1u )  // draiverio perkaitimo flagas
#define     DI_OVERCURR_ALARM           ( 2u )  // draiverio sroves perkrovimo flagas
#define     DI_HALL_FAULT_FLAG          ( 3u )  // Hall daviklio avarijos flagas (0 - klaidos nera)
#define     DI_HALL_STATE               ( 16u ) // dubliuoja HALL daviklio iejimo busena
#define     DI_DI0_STATE                ( 17u ) // iejimo DI0 busena
#define     DI_DI1_STATE                ( 18u ) // iejimo DI1 busena
#define     DI_DI2_STATE                ( 19u ) // iejimo DI2 busena
#define     DI_SW1_STATE                ( 32u )	// MODBUS low->enable/high->disable
#define     DI_SW2_STATE                ( 33u )	// Hall sensor enable/disable
#define     DI_SW3_STATE                ( 34u )	// Scrolling enable/disable
#define     DI_SW4_STATE                ( 35u )
#define     DI_SW5_STATE                ( 36u )
#define     DI_SW6_STATE                ( 37u )	// SW6-SW8 variklio tipo pasirinkimas
#define     DI_SW7_STATE                ( 38u )	//
#define     DI_SW8_STATE                ( 39u )	//

/* INPUTS */
#define     IR_SP_REQ_VALUE             ( 1u )  // sukimosi greicio potenciometro reiksme ( formatas (0-10V)*100 )
#define     IR_VBUS_VALUE               ( 2u )  // VBUS itampos reiksme ( formatas V*100 )
#define     IR_CURR_RPM                 ( 3u )  // aktyvi RPM reiksme, nuskaityta is draiverio L6470 per spi ( formatas aps/min )
#define     IR_FAULT_CODE               ( 4u )  // sistemos klaidos kodas ( )
#define     IR_SMC_STATE                ( 5u )  // SMC statusas (veikia/stop/scrolling ir t.t.)
#define     IR_ADCIN_VALUE              ( 6u )  // draiverio L6470 maitinimo itampos reiksme, nuskaityta is draiverio per spi ( draiverio ADCIN registras - formatas 0-31 )
#define     IR_MAX_RPM                  ( 7u )  // maksimalus valdiklio RPMai
#define     IR_HS_TO_COUNTER            ( 8u )  // Hall sensorio timeout counteris ( sekundes )
#define     IR_WTIMEHI                  ( 9u )  // WTIME hi registras
#define     IR_WTIMELO                  ( 10u ) // WTIME lo registras
#define     IR_INTTEMP_VALUE            ( 11u ) // procesoriaus vidinio temperaturos daviklio reiksme ( ADC reiksme )
#define     IR_MOTOR_STATE              ( 13u ) // variklio busena, nuskaityta is draiverio L6470 per spi ( registras STATUS bitai 5-6 (MOT_STATUS) )



/* HOLDINGS */
#define     HR_MBADDR                   ( 3u ) // Modbus adresas
#define     HR_MBPARITY                 ( 4u ) // Parity
#define     HR_MBSTOPBITS               ( 5u ) // Stopbitai
#define     HR_MBBAUDRATE               ( 6u ) // Bodreitas
#define     HR_TESTMODE                 ( 8u ) // testinis rezimas ( jai irasom TEST_MODE_CODE (4949h) - aktyvuojasi testinis rezimas, 0000h - darbinis rezimas )

#define     HR_ROT_SPEED                ( 10u ) // variklio apsukos nustatymui ( 0 - variklis isjungtas, RPM )
#define     HR_MICROSTEPS               ( 11u ) // mikrostepu skaicius
#define     HR_TRANSMISSION_RATIO       ( 12u ) // variklio asies ir rotoriaus diametru santykis
#define     HR_HS_TO_VALUE              ( 14u ) // Hall sensorio timeout reiksme sekundemis ( 0 - Hall Sensor isjungtas ). Nustatoma MAX sukimosi greiciui.
#define     HR_OVH_TIMEOUT              ( 15u ) // overheat taimautas

#define     HR_SCROLL_RPM               ( 19u )  // SCROLLING rezimo RPMai ( 0 - Scrolling isjungtas )
#define     HR_SCROLL_OFF_CYCLE_TIME    ( 20u )  // SCROLLING rezimo OFF ciklo laikas sekundemis
#define     HR_SCROLL_ON_CYCLE_TIME     ( 21u )  // SCROLLING rezimo ON ciklo laikas sekundemis
#define     HR_SCROLL_SYNC              ( 22u )  // ScrollSync ciklu skaicius ( kas kelinta Scrolling cikla keiciam sukimosi kripti. 0 - funkcija isjungta )

#define     HR_USERSET_STEPS_PER_REV    ( 24u )
#define     HR_USERSET_KVAL_RUN         ( 25u ) // RUN rezimo powerfaktorius (procentai 0 - 100)
#define     HR_USERSET_KVAL_ACC         ( 26u ) // ACC rezimo powerfaktorius (procentai 0 - 100)
#define     HR_USERSET_KVAL_DEC         ( 27u ) // DEC rezimo powerfaktorius (procentai 0 - 100)
#define     HR_USERSET_KVAL_HOLD        ( 28u ) // HOLD rezimo powerfaktorius (procentai 0 - 100)
#define     HR_USERSET_TRES_OCD         ( 29u ) // overcurrent treshold parametras (0 - 6)
#define     HR_USERSET_TRES_STALL       ( 30u ) // STALL treshold parametras (0 - 4)
#define     HR_USERSET_RPM_MIN          ( 31u ) // minimalus variklio RPMai
#define     HR_USERSET_RPM_MAX          ( 32u ) // maksimalus variklio RPMai
#define     HR_USERSET_SPEED_ACC        ( 33u )
#define     HR_USERSET_SPEED_DEC        ( 34u )

#define     HR_DAC_OUT_VAL              ( 40u ) // DAC reiksme
#define     HR_PWM_OUT_VAL              ( 41u ) // PWM reiksme ( 0 - 1000 )
#define     HR_HALL_PULSE_COUNTER       ( 42u ) // holo daviklio suveikimo kounteris


/* DEFAULTS */
#define     MBADDR_DEF                  ( 10u )   //0x0A
#define     MBPARITY_DEF                MB_PAR_NONE
#define     MBBAURATE_DEF               ( 19200u )
#define     MBSTOPBITS_DEF              ( 1u )

#define     ROTSPEED_DEF                ( 0u )        // RPM
#define     MICROSTEPS_DEF              (128u )     // mikrostepu
#define     HS_TO_VALUE_DEF             ( 10u )     // sekundes
#define     TRANSMISSION_RATIO_DEF      ( 10u )     //
#define     OVH_TIMEOUT_DEF             ( 30u )     // sekundes

#define     USERSET_STEPS_PER_REV_DEF   ( 200u )
#define     USERSET_KVAL_RUN_PROC_DEF   ( 16u )         // %
#define     USERSET_KVAL_ACC_PROC_DEF   ( 21u )         // %
#define     USERSET_KVAL_DEC_PROC_DEF   ( 16u )         // %
#define     USERSET_KVAL_HOLD_PROC_DEF  ( 5u )          // %
#define     USERSET_TRES_OCD_MA_DEF     ( 2000u )        // mA
#define     USERSET_TRES_STALL_MA_DEF   ( 1800u )        // mA
#define     USERSET_RPM_MIN_DEF         ( 1u )          // RPM
#define     USERSET_RPM_MAX_DEF         ( 200u )        // RPM
#define     USERSET_SPEED_ACC_DEF       ( 20u )         // steps/s^2
#define     USERSET_SPEED_DEC_DEF       ( 100u )        // steps/s^2


#define     SCROLL_RPM_DEF              ( 20u )         // RPM
#define     SCROLL_OFF_CYCLE_TIME_DEF   ( 180u )         // sekundes
#define     SCROLL_ON_CYCLE_TIME_DEF    ( 30u )         // sekundes
#define     SCROLL_SYNC_DEF             ( 3u )


/* kiti defainai */
#define		TESTMODE_KEY				( 0x4949 )


/* extern data */
extern UCHAR    ucDiscInputBuf[];
extern UCHAR    ucCoilBuf[];
extern USHORT   usRegInputBuf[];
extern USHORT   usRegHoldingBuf[];

extern uint8_t  ucSlaveIdBuf[];


#endif
