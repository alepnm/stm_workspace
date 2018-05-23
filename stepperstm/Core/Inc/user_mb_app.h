/******************************************************************************
* Project Name		: Stepper Motor Control
* File Name			: user_mb_app.h
* Version 			: 1.0
* Device Used		: STM32F051C8T6
* Software Used		: EmBitz 1.11
* Compiler    		: ARM GCC Compiller (EmBitz - bare -metal)
* Related Hardware	:
*
* Owner             : Ventmatika Inc.
*******************************************************************************/

#ifndef	USER_APP
#define USER_APP
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"

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
#define     CO_RELAY_ON                 ( 5u )
#define     CO_COOLER_ON                ( 6u )
#define     CO_DRV_REBOOT               ( 8u )  // draiverio reinicializacija(true)
#define     CO_COMPENSATION_NLHT        ( 9u )  // silumos perdavimo kompensacija

/* DESCREETS */
#define     DI_TH_ALARM                 ( 1u )  // draiverio perkaitimo flagas (L6470)
#define     DI_OCD_ALARM                ( 2u )  // draiverio sroves perkrovimo flagas (L6470)
#define     DI_HALL_FAULT_FLAG          ( 4u )  // Hallo daviklio avarijos flagas (0 - klaidos nera)
#define		DI_MOTOR_STATE				( 5u )	// variklio busena
#define     DI_HALL_STATE               ( 16u ) // dubliuoja HALL daviklio iejimo busena
#define     DI_DI0_STATE                ( 17u ) // iejimo DI0 busena
#define     DI_DI1_STATE                ( 18u ) // iejimo DI1 busena
#define     DI_DI2_STATE                ( 19u ) // iejimo DI2 busena
#define     DI_DI3_STATE                ( 20u ) // iejimo DI2 busena
#define     DI_SW1_STATE                ( 32u )	// SW1-SW2 MODBUS/0-10/StepClock/...
#define     DI_SW2_STATE                ( 33u )	//
#define     DI_SW3_STATE                ( 34u )	// MAX RPM 150/200
#define     DI_SW4_STATE                ( 35u ) // Scrolling enable/disable
#define     DI_SW5_STATE                ( 36u ) // Hall sensor enable/disable
#define     DI_SW6_STATE                ( 37u )	// SW6-SW8 variklio tipo pasirinkimas
#define     DI_SW7_STATE                ( 38u )	//
#define     DI_SW8_STATE                ( 39u )	//

/* INPUTS */
#define     IR_FAULT_CODE               ( 1u )  // sistemos klaidos kodas ( )
#define     IR_CURR_RPM                 ( 2u )  // aktyvi RPM reiksme, nuskaityta is draiverio L6470 per spi ( formatas aps/min )
#define     IR_MAX_RPM                  ( 3u )  // maksimalus valdiklio RPMai ( pasirenkama su DIPSWITCH )
#define     IR_HS_TO_COUNTER            ( 4u )  // Hall sensorio timeout counteris ( sekundes )
#define     IR_WTIMEHI                  ( 6u )  // WTIME hi registras
#define     IR_WTIMELO                  ( 7u )  // WTIME lo registras
#define     IR_ADCIN_VALUE              ( 10u ) // draiverio L6470 maitinimo itampos reiksme, nuskaityta is draiverio per spi ( draiverio ADCIN registras - formatas 0-31 )
#define     IR_SPREQ_VALUE              ( 11u ) // sukimosi greicio potenciometro reiksme ( formatas (0-10V)*100 )
#define     IR_VBUS_VALUE               ( 12u ) // VBUS itampos reiksme ( formatas V*100 )
#define     IR_MCUTEMP                  ( 13u ) // MCU vidine temperatura ( laipsniai C )


/* HOLDINGS */
#define     HR_MBADDR                   ( 3u )      // Modbus adresas
#define     HR_MBPARITY                 ( 4u )      // Parity
#define     HR_MBSTOPBITS               ( 5u )      // Stopbitai
#define     HR_MBBAUDRATE               ( 6u )      // Bodreitas
#define     HR_TESTMODE                 ( 8u )      // testinis rezimas ( jai irasom TEST_MODE_CODE (4949h) - aktyvuojasi testinis rezimas, 0000h - darbinis rezimas )
#define     HR_MICROSTEPS               ( 10u )     // mikrostepu skaicius
#define     HR_OVH_TIMEOUT              ( 12u )     // overheat taimautas

#define     HR_SCROLL_RPM               ( 15u )     // SCROLLING rezimo RPMai ( 0 - Scrolling isjungtas )
#define     HR_SCROLL_OFF_CYCLE_TIME    ( 16u )     // SCROLLING rezimo OFF ciklo laikas sekundemis
#define     HR_SCROLL_ON_CYCLE_TIME     ( 17u )     // SCROLLING rezimo ON ciklo laikas sekundemis
#define     HR_SCROLL_SYNC              ( 18u )     // ScrollSync ciklu skaicius ( kas kelinta Scrolling cikla keiciam sukimosi kripti. 0 - funkcija isjungta )

#define     HR_HS_TO_VALUE              ( 20u )     // Hall sensorio timeout reiksme sekundemis ( 0 - Hall Sensor isjungtas ). Nustatoma MAX sukimosi greiciui.

#define     HR_ROT_SPEED                ( 21u )     // variklio apsukos nustatymui ( RPM, 0 - variklis isjungtas; neigiama reiksme - CCW, teigiama - CW )
#define     HR_MIN_RPM                  ( 22u )     // maksimalios apsukos (RPM)  !Tik Modbus rezime!
#define     HR_MAX_RPM                  ( 23u )     // maksimalios apsukos (RPM)  !Tik Modbus rezime!
#define     HR_TRANSMISSION_RATIO       ( 24u )     // variklio asies ir rotoriaus diametru santykis

#define     HR_USERSET_STEPS_PER_REV    ( 26u )
#define     HR_USERSET_KVAL_RUN         ( 27u )     // RUN rezimo powerfaktorius (procentai 0 - MAXKVAL_VALUE_DEF)
#define     HR_USERSET_KVAL_ACC         ( 28u )     // ACC rezimo powerfaktorius (procentai 0 - MAXKVAL_VALUE_DEF)
#define     HR_USERSET_KVAL_DEC         ( 29u )     // DEC rezimo powerfaktorius (procentai 0 - MAXKVAL_VALUE_DEF)
#define     HR_USERSET_KVAL_HOLD        ( 30u )     // HOLD rezimo powerfaktorius (procentai 0 - MAXKVAL_HOLD_VALUE_DEF)
#define     HR_USERSET_TRES_OCD         ( 31u )     // overcurrent treshold parametras (0 - 6)
#define     HR_USERSET_TRES_STALL       ( 32u )     // STALL treshold parametras (0 - 4)
#define     HR_USERSET_SPEED_ACC        ( 33u )
#define     HR_USERSET_SPEED_DEC        ( 34u )

#define     HR_DAC_OUT_VAL              ( 40u )     // DAC reiksme
#define     HR_PWM_OUT_VAL              ( 41u )     // PWM reiksme ( 0 - 1000 )
#define     HR_HALL_PULSE_COUNTER       ( 42u )     // holo daviklio suveikimo kounteris (veikia, kai aktyvuotas holo daviklis)


/* DEFAULTS */
#define     MBADDR_DEF                  ( 10u )     //0x0A
#define     MBPARITY_DEF                MB_PAR_NONE
#define     MBBAURATE_DEF               ( 19200u )
#define     MBSTOPBITS_DEF              ( 1u )

#define     RPM_MIN_DEF                 ( 1u )      // minimalus sukimosi greitis (RPM)
#define     RPM_MAX_DEF                 ( 200u )    // maksimalus sukimosi greitis (RPM)
#define     MICROSTEPS_DEF              ( 16u )     // mikrostepu
#define     MIN_TRES_OCD_MA_DEF         ( 500u )    // minimali sroves reiksme, mA
#define     MAX_TRES_OCD_MA_DEF         ( 4000u )   // maksimali sroves reiksme, mA
#define     MAX_KVAL_VALUE_DEF          ( 60u )
#define     MAX_KVAL_HOLD_VALUE_DEF     ( 20u )
#define     HS_TO_VALUE_DEF             ( 10u )     // holo daviklio taimaut (sekundes)
#define     TRANSMISSION_RATIO_DEF      ( 10u )     // variklio ir rotoriaus diametru santykis
#define     OVH_TIMEOUT_DEF             ( 30u )     // sekundes

#define     USERSET_STEPS_PER_REV_DEF   ( 200u )    //
#define     USERSET_KVAL_RUN_PROC_DEF   ( 22u )     // %
#define     USERSET_KVAL_ACC_PROC_DEF   ( 24u )     // %
#define     USERSET_KVAL_DEC_PROC_DEF   ( 20u )     // %
#define     USERSET_KVAL_HOLD_PROC_DEF  ( 5u )      // %
#define     USERSET_TRES_OCD_MA_DEF     ( 2500u )   // mA
#define     USERSET_TRES_STALL_MA_DEF   ( 2500u )   // mA
#define     USERSET_SPEED_ACC_DEF       ( 100u )    // steps/s^2
#define     USERSET_SPEED_DEC_DEF       ( 100u )    // steps/s^2


#define     SCROLL_RPM_DEF              ( 20u )     // RPM
#define     SCROLL_OFF_CYCLE_TIME_DEF   ( 180u )    // sekundes
#define     SCROLL_ON_CYCLE_TIME_DEF    ( 30u )     // sekundes
#define     SCROLL_SYNC_DEF             ( 3u )


/* kiti defainai */
#define		TESTMODE_KEY				( 0x4949 )


/* extern data */
extern UCHAR    ucDiscInputBuf[];
extern UCHAR    ucCoilBuf[];
extern USHORT   usRegInputBuf[];
extern USHORT   usRegHoldingBuf[];

extern uint8_t  ucSlaveIdBuf[];


uint8_t xMbGetCoil( uint16_t usBitOffset );
void xMbSetCoil( uint16_t usBitOffset, uint8_t ucValue );
uint8_t xMbGetDInput( uint16_t usBitOffset );
void xMbSetDInput( uint16_t usBitOffset, uint8_t ucValue );

uint8_t xMbGetNCoils( uint16_t usBitOffset, uint8_t ucNBits );
void xMbSetNCoils( uint16_t usBitOffset, uint8_t ucNBits, uint8_t ucValue );
uint8_t xMbGetNDInputs( uint16_t usBitOffset, uint8_t ucNBits );
void xMbSetNDInputs( uint16_t usBitOffset, uint8_t ucNBits, uint8_t ucValue );


#endif
