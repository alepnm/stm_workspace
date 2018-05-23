#ifndef STP_H_INCLUDED
#define STP_H_INCLUDED

/******************************************************************************
* Project Name		: Stepper Motor Control
* File Name		    : stp.h
* Version 		    : x.x
* Device Used		: STM32F051C8x
* Software Used		: EmBitz 1.11
* Compiler    		: ARM GCC 4.9-2015-q1-update
* Related Hardware	:
*
* Owner             : Ventmatika Inc.
*******************************************************************************/

#include <stdbool.h>
#include <stdint.h>

typedef enum{ CCW = 0, CW = !CCW }DIR_TypeDef;

typedef enum { FSM_MODE_MANUAL = 0, FSM_MODE_MODBUS, FSM_MODE_STEPCLOCK, FSM_MODE_TESTMODE } SMC_ModeTypeDef;
typedef enum { FSM_STATE_STOP = 0, FSM_STATE_NORMAL, FSM_STATE_FAULT, FSM_STATE_ACCELERATE, FSM_STATE_DECELERATE, FSM_STATE_SCROLLING } SMC_StateTypeDef;


/* Varikliu presetai */
typedef struct{
    uint8_t ID;                 // preseto ID
    uint8_t StepsPerRev;        // variklio stepu per apsisukima
    struct{
        uint8_t RunValue;           // KVAL_RUN reiksme, %
        uint8_t AccValue;           // KVAL_ACC reiksme, %
        uint8_t DecValue;           // KVAL_DEC reiksme, %
        uint8_t HoldValue;          // KVAL_HOLD reiksme, %
    }Kval;
    struct{
        uint16_t OcdValue;          // OCD_TH reiksme, mA
        uint16_t StallValue;        // STALL_TH reiksme, mA
    }Treshold;
    struct{
        uint16_t Acceleration;      //
        uint16_t Deceleration;      //
    }Speed;
}MotorParamSet;


typedef struct{

    SMC_ModeTypeDef 	                ControlMode;                // DIP switch nustatomu opciju reiksme : valdymo rezimas
    SMC_StateTypeDef                    SMC_State;                  //

    struct{
        uint8_t*                		pName;                      // valdiklio pavadinimas
        uint8_t*                		pVersion;                   // SW versija
        uint8_t*                		pId;                        // valdiklio UID (25AA048)
    }StrData;

    struct{
        const MotorParamSet*  	        pCurrentMotorPreset;        // pointeris i naudojamo variklio parametru preseta
        uint16_t                        Status;                     // variklio busena, nuskaityta is draiverio L6470 per spi ( registras STATUS bitai 5-6 (MOT_STATUS) )
        uint16_t                     	RotSpeedSetting;            // nustatytas sukimosi greitis Normal rezime
        DIR_TypeDef                    	RotDirSetting;              // nustatyta sukimosi kriptys Normal rezime
    }MotorData;

    struct{
        uint8_t 			    		Data;                       // is DIP switch nuskaityta reiksme
        struct{
            uint8_t 	        		HallSensor;                 // holo daviklis yra/nera
            uint8_t         		    Scrolling;                  // Scroll rezimas naudojamas/nenaudojamas
            uint8_t 		    		MotorType;                  // naudojamo variklio tipas (0-7)
        }Option;
    }DipSwitch;

    struct{
        uint16_t                        Vbus;                       // VBUS ADC reiksme
        uint16_t                        SpReq;                      // SP_REQ ADC reiksme
        uint16_t                        McuTemp;                    // MCU temperatusa ADC reiksme
    }ADC_Vals;

}SmcHandle_TypeDef;


//#pragma anon_unions
/* @todo (demo#1#): Apgalvoti modbus duomenu struktura! */
#pragma pack(push,1)
typedef struct {
    uint16_t*			MbAddr;
    struct {
        uint16_t*    	Baudrate;
        uint16_t*    	Parity;
        uint16_t*    	StopBits;
    } UART;
} MbPortParams_TypeDef;
#pragma pack(pop)


bool    SMC_Init ( void );
void    SMC_DataUpdate( void );
const   MotorParamSet* SMC_GetPresetPtr( uint8_t preset_id );
void    SMC_ReadDipSwitch( void );
void    SMC_ReadDigitalInputs( void );
void    SMC_ReadAnalogInputs( void );
uint8_t SMC_ScrollingHandler( void );
uint8_t SMC_HallSensorHandler( void );
void    SMC_CoolerController( void );
void    SMC_RelayController( void );
void    SMC_LedsController( void );
void    SMC_Check_Faults( void );
bool    SMC_CheckBaudrateValue( uint32_t baudrate );
bool    SMC_CheckUStepValue( uint8_t uStep );

bool    SMC_MotorConfig( const MotorParamSet* preset );
bool    SMC_MotorStart( void );
bool    SMC_MotorReverse( void );
bool    SMC_MotorStop( void );
bool    SMC_MotorStartParam( DIR_TypeDef dir, float speed );
bool    SMC_GetMotorStatusRegister( void );
bool    SMC_GetMotorCurrentSpeed( void );

void    SMC_SysTimeCounterUpdate( void );

#endif /* STP_H_INCLUDED */
