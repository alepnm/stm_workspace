/*
*/

#include <math.h>
#include <stdlib.h>
#include "stm32f0xx_hal.h"
#include "l6470.h"
#include "smc.h"


#define L6470_CS_LOW      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
#define L6470_CS_HIGH     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);

extern SPI_HandleTypeDef* hspi1;
extern SmcHandle_TypeDef SMC_Control;

/* private functions */
static long convert(unsigned long val);
static unsigned long Param(unsigned long value, byte bit_len);
static byte Xfer(byte data);
static unsigned long ParamHandler(byte param, unsigned long value);


void L6470_Init(void) {

    L6470_resetDev( );

    HAL_Delay(100);

    L6470_hardFree( );   // isejima konfiginam i auksta impedansa, nes reikia keisti WH tipo registrus CONFIG ir STEP_MODE

    /* isejima FLAG konfiguruojam indikacijai blokavimo po perkaitimo, arba perkatimo arba sroves perkrovimo */
    SetParam( REG_ALARM_EN, ALARM_EN_THERMAL_WARNING | ALARM_EN_OVERCURRENT );

    /* PWM daznis 93.8 kHz */
    SetParam( REG_CONFIG, CONFIG_PWM_DIV_1 | CONFIG_PWM_MUL_2 | CONFIG_SR_290V_us | CONFIG_OC_SD_ENABLE | CONFIG_VS_COMP_ENABLE | CONFIG_EXT_24MHZ_OSCOUT_INVERT );

    SetParam( REG_STEP_MODE, (!SYNC_EN) | SYNC_SEL_32 | STEP_SEL_1_16 );

    (void)L6470_getStatus( );

    L6470_hardStop( ); //ijungiam draiveri
}



/**
*/
bool L6470_isBusy(void) {
    int status = L6470_getStatus();
    return !( status & STATUS_BUSY );
}


/**
*/
void L6470_setMicroSteps(int microSteps) {
    byte stepVal;

    for(stepVal = 0; stepVal < 8; stepVal++) {
        if(microSteps == 1) break;
        microSteps = microSteps >> 1;
    }

    Xfer(CMD_HARD_HIZ);     //

    SetParam(REG_STEP_MODE, (!SYNC_EN) | SYNC_SEL_1 | stepVal);
}


/**
*/
void L6470_setThresholdSpeed(float thresholdSpeed) {
    if(thresholdSpeed == 0.0) SetParam(REG_FS_SPD, 0x3FF);
    else SetParam(REG_FS_SPD, FSCalc(thresholdSpeed));
}


/**
*/
void L6470_setCurrent(int current) {

}


/**
*/
void L6470_setMaxSpeed(int speed) {
    SetParam(REG_MAX_SPEED, MaxSpdCalc(speed));
}


/**
*/
void L6470_setMinSpeed(int speed) {

    SetParam(REG_MIN_SPEED, MinSpdCalc(speed));

    //uint16_t  data = ( MinSpdCalc(speed) | SPEED_LSPD_OPT );
    //SetParam(REG_MIN_SPEED, data);
}



/**
*/
void L6470_setAcc(float acceleration) {

    unsigned long accelerationBYTES = AccCalc(acceleration);

    SetParam(REG_ACC, accelerationBYTES);
}


/**
*/
void L6470_setDec(float deceleration) {

    unsigned long decelerationBYTES = DecCalc(deceleration);

    SetParam(REG_DEC, decelerationBYTES);
}


/**
*/
long L6470_getPos(void) {

    unsigned long position = GetParam(REG_ABS_POS);

    return convert(position);
}


/**
*/
float L6470_getSpeed(void) {
    return (float) GetParam(REG_SPEED);
}


/**
*/
void L6470_setOverCurrent(float ma_current) {

    float ocvalue = 0.00266667*ma_current;
    float decimal = ocvalue - (uint8_t)ocvalue;

    if(decimal > (float)0.5) ocvalue += 1;

    if((uint8_t)ocvalue > 0x0F) ocvalue = 0x0F;

    SetParam(REG_OCD_TH, (uint8_t)ocvalue);
}


/**
*/
void L6470_setStallCurrent(float ma_current) {

    int8_t STHValue = (int8_t)floor(ma_current / 31.25);

    if(STHValue > 0x80) STHValue = 0x80;
    if(STHValue < 0) STHValue = 0;

    SetParam((byte)REG_STALL_TH, STHValue);
}


/**
*/
void L6470_SetLowSpeedOpt(bool enable) {

    Xfer(CMD_SET_PARAM | REG_MIN_SPEED);

    if (enable) Param(0x1000, 13);
    else Param(DUMMY, 13);
}


/**
*/
void L6470_run(byte dir, float spd) {
    unsigned long speedVal = SpdCalc(spd);

    Xfer(CMD_RUN | dir);

    if (speedVal > 0xFFFFF) speedVal = 0xFFFFF;

    Xfer((byte)(speedVal >> 16));
    Xfer((byte)(speedVal >> 8));
    Xfer((byte)(speedVal));

}


/**
*/
void L6470_StepClock(byte dir) {
    Xfer(CMD_STEP_CLOCK | dir);
}


/**
*/
void L6470_move(long n_step) {
    byte dir;

    long n_stepABS = abs(n_step);

    if(n_step >= 0) dir =  FWD;
    else dir =  REV;

    Xfer(CMD_MOVE | dir); //set direction

    if (n_stepABS > 0x3FFFFF) n_step = 0x3FFFFF;

    Xfer((byte)(n_stepABS >> 16));
    Xfer((byte)(n_stepABS >> 8));
    Xfer((byte)(n_stepABS));
}


/**
*/
void L6470_goTo(long pos) {
    Xfer(CMD_GOTO);

    if (pos > 0x3FFFFF) pos = 0x3FFFFF;

    Xfer((byte)(pos >> 16));
    Xfer((byte)(pos >> 8));
    Xfer((byte)(pos));
}

/**
*/
void L6470_goTo_DIR(byte dir, long pos) {
    Xfer(CMD_GOTO_DIR);

    if (pos > 0x3FFFFF) pos = 0x3FFFFF;

    Xfer((byte)(pos >> 16));
    Xfer((byte)(pos >> 8));
    Xfer((byte)(pos));
}

/**
*/
void L6470_goUntil(byte act, byte dir, unsigned long spd) {
    Xfer(CMD_GO_UNTIL | act | dir);

    if (spd > 0x3FFFFF) spd = 0x3FFFFF;

    Xfer((byte)(spd >> 16));
    Xfer((byte)(spd >> 8));
    Xfer((byte)(spd));
}

/**
*/
void L6470_releaseSW(byte act, byte dir) {
    Xfer(CMD_RELEASE_SW | act | dir);
}


/**
*/
void L6470_goHome(void) {
    Xfer(CMD_GO_HOME);
}


/**
*/
void L6470_goMark(void) {
    Xfer(CMD_GO_MARK);
}


/**
*/
void L6470_setMark(long value) {
    Xfer(REG_MARK);

    if (value > 0x3FFFFF) value = 0x3FFFFF;
    if (value < -0x3FFFFF) value = -0x3FFFFF;

    Xfer((byte)(value >> 16));
    Xfer((byte)(value >> 8));
    Xfer((byte)(value));
}


/**
*/
void L6470_setMark_currPos(void) {
    long value = L6470_getPos();

    Xfer(REG_MARK);

    if (value > 0x3FFFFF) value = 0x3FFFFF;
    if (value < -0x3FFFFF) value = -0x3FFFFF;

    Xfer((byte)(value >> 16));
    Xfer((byte)(value >> 8));
    Xfer((byte)(value));
}


/**
*/
void L6470_setAsHome(void) {
    Xfer(CMD_RESET_POS);
}


/**
*/
void L6470_resetDev(void) {
    Xfer(CMD_RESET_DEVICE);
}


/**
Minkstas variklio sustojimas (su paletejimu) su laikymo funkcija
*/
void L6470_softStop(void) {
    Xfer(CMD_SOFT_STOP);
}


/**
Greitas variklio sustojimas su laikymo funkcija
*/
void L6470_hardStop(void) {
    Xfer(CMD_HARD_STOP);
}


/**
Minkstas variklio sustojimas (su paletejimu) be laikymo funkcijos
*/
void L6470_softFree(void) {
    Xfer(CMD_SOFT_HIZ);
}


/**
Greitas variklio sustojimas be laikymo funkcijos
*/
void L6470_hardFree(void) {
    Xfer(CMD_HARD_HIZ);
}


/**
*/
int L6470_getStatus(void) {
    int temp = 0;

    Xfer(CMD_GET_STATUS);

    temp = Xfer(DUMMY)<<8;
    temp |= Xfer(DUMMY);

    return temp;
}



/* Perskaiciojam steps per second^2 reiksme i ACC registro reiksme
*/
unsigned long AccCalc(float stepsPerSecPerSec) {

    float temp = stepsPerSecPerSec * 0.137438;

    if( (unsigned long)temp > 0x00000FFF) temp = 0x00000FFF;

    return (unsigned long)temp;
}


/*  Perskaiciojam steps per second^2 reiksme i DEC registro reiksme
*/
unsigned long DecCalc(float stepsPerSecPerSec) {

    float temp = stepsPerSecPerSec * 0.137438;

    if( (unsigned long)temp > 0x00000FFF) temp = 0x00000FFF;

    return (unsigned long)temp;
}


/*  Perskaiciojam steps per second reiksme i MAX_SPEED registro reiksme
*/
unsigned long MaxSpdCalc(float stepsPerSec) {

    float temp = stepsPerSec * .065536;

    if( (unsigned long)temp > 0x000003FF) temp = 0x000003FF;

    return (unsigned long)temp;
}


/*  Perskaiciojam steps per second reiksme i MIN_SPEED registro reiksme
*/
unsigned long MinSpdCalc(float stepsPerSec) {

    float temp = stepsPerSec * 4.1943;

    if( (unsigned long)temp > 0x00000FFF) temp = 0x00000FFF;

    return (unsigned long)temp;
}


/*  Perskaiciojam steps per second reiksme i FS_SPD registro reiksme
*/
unsigned long FSCalc(float stepsPerSec) {

    float temp = (stepsPerSec * 0.065536) - 0.5;

    if( (unsigned long)temp > 0x000003FF) temp = 0x000003FF;

    return (unsigned long)temp;
}


/* Perskaiciojam steps per second reiksme i INT_SPEED registro reiksme
*/
unsigned long IntSpdCalc(float stepsPerSec) {

    float temp = stepsPerSec * 4.1943;

    if( (unsigned long)temp > 0x00003FFF) temp = 0x00003FFF;

    return (unsigned long)temp;
}


/* Perskaiciojam steps per second reiksme i SPEED registro reiksme
*/
unsigned long SpdCalc(float stepsPerSec) {

    float temp = stepsPerSec * 67.1089;

    if( (unsigned long)temp > 0x000FFFFF) temp = 0x000FFFFF;

    return (unsigned long)temp;
}



/* Perskaiciuojam RPMus i steps per second reiksme
*/
float ConvertRpmToStepsPerSec( float rpm ) {
    return ( rpm * ( SMC_Control.MotorData.pCurrentMotorPreset->StepsPerRev - 1 ) / 60 );
}




/**
*/
unsigned long Param(unsigned long value, byte bit_len) {

    unsigned long ret_val=0;

    byte byte_len = bit_len/8;

    if (bit_len%8 > 0) byte_len++;

    unsigned long mask = 0xffffffff >> (32 - bit_len);

    if (value > mask) value = mask;

    if (byte_len == 3) {
        ret_val |= (long)(Xfer((byte)(value>>16))) << 16;
    }
    if (byte_len >= 2) {
        ret_val |= (long)(Xfer((byte)(value>>8))) << 8;
    }
    if (byte_len >= 1) {
        ret_val |= Xfer((byte)value);
    }

    return (ret_val & mask);
}


/**
*/
static byte Xfer(byte data_tx) {

    byte data_rx;

    L6470_CS_LOW
    (void)HAL_SPI_TransmitReceive(hspi1, &data_tx, &data_rx, 1, 1000);
    L6470_CS_HIGH

    return data_rx;
}


/**
*/
void SetParam(byte param, unsigned long value) {

    Xfer(CMD_SET_PARAM | param);

    ParamHandler(param, value);
}

/**
*/
unsigned long GetParam(byte param) {

    Xfer(CMD_GET_PARAM | param);

    return ParamHandler(param, DUMMY);
}


/**
*/
static long convert(unsigned long val) {

    int MSB = val >> 21;

    val = val << 11;
    val = val >> 11;

    if(MSB == 1) val = val | 0xFFE00000;    //0b11111111111000000000000000000000;
    return val;
}


/**
*/
static unsigned long ParamHandler(byte param, unsigned long value) {

    unsigned long ret_val = 0;

    switch (param) {
    case REG_ABS_POS:
        ret_val = Param(value, 22);
        break;
    case REG_EL_POS:
        ret_val = Param(value, 9);
        break;
    case REG_MARK:
        ret_val = Param(value, 22);
        break;
    case REG_SPEED:
        ret_val = Param(DUMMY, 20);
        break;
    case REG_ACC:
        ret_val = Param(value, 12);
        break;
    case REG_DEC:
        ret_val = Param(value, 12);
        break;
    case REG_MAX_SPEED:
        ret_val = Param(value, 10);
        break;
    case REG_MIN_SPEED:
        ret_val = Param(value, 12);
        break;
    case REG_FS_SPD:
        ret_val = Param(value, 10);
        break;
    case REG_KVAL_HOLD:
        ret_val = Xfer((byte)value);
        break;
    case REG_KVAL_RUN:
        ret_val = Xfer((byte)value);
        break;
    case REG_KVAL_ACC:
        ret_val = Xfer((byte)value);
        break;
    case REG_KVAL_DEC:
        ret_val = Xfer((byte)value);
        break;
    case REG_INT_SPD:
        ret_val = Param(value, 14);
        break;
    case REG_ST_SLP:
        ret_val = Xfer((byte)value);
        break;
    case REG_FN_SLP_ACC:
        ret_val = Xfer((byte)value);
        break;
    case REG_FN_SLP_DEC:
        ret_val = Xfer((byte)value);
        break;
    case REG_K_THERM:
        ret_val = Xfer((byte)value & 0x0F);
        break;
    case REG_ADC_OUT:
        ret_val = Xfer(DUMMY);
        break;
    case REG_OCD_TH:
        ret_val = Xfer((byte)value & 0x0F);
        break;
    case REG_STALL_TH:
        ret_val = Xfer((byte)value & 0x7F);
        break;
    case REG_STEP_MODE:
        ret_val = Xfer((byte)value);
        break;
    case REG_ALARM_EN:
        ret_val = Xfer((byte)value);
        break;
    case REG_CONFIG:
        ret_val = Param(value, 16);
        break;
    case REG_STATUS:
        ret_val = Param(DUMMY, 16);
        break;
    default:
        ret_val = Xfer((byte)(value));
        break;
    }

    return ret_val;
}



/*  */
float Speed_0_10V_Compensation(float val) {

    if(val < 11) return 0.00;
    if(val > 95) return 100;

    return val;
}



/*
Parametras: variklio SET'as (0-...)
Rezultatas: steps per seconds
*/
float SpeedCompensationCalc(float set) {

    /* skaiciojam RPMus procentais */
    float rpm_proc = 5.25641026E-08 * pow(set, 5) - 1.07459207E-05 * pow(set, 4) + 0.00085973 * pow(set, 3) - 0.02403497 * pow(set, 2) + 0.29662471 * set - 0.10629371;

    /* perskaiciojam RPMais */
    float rpm = rpm_proc * 200 / 100;

    /* grazinam stepus per sekunde */
    return ( rpm * SMC_Control.MotorData.pCurrentMotorPreset->StepsPerRev / 60 );
}





