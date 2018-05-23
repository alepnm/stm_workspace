
#include "smc.h"
#include "user_mb_app.h"


/* Varikliu presetai */
/*
OCD nustatymo metodika:
1. Nustatom OcdValue pagal variklio darbine srove
2. Nustatom vidutini sukimosi greiti
3. Nuo mazesnes reikmes link didesnes keliant KVAL surandam toki jo reiksme, kai variklis pradeda trukcioti. Tai variklio darbinis KVAL esamai maitinimo itampai
4. Keliam OcdValue ~0.5A. Tai reiskia, kad OCD apsauga suveikines srovei virsinant darbine ~0.5A


Kuo didesne variklio apviju darbine itampa, to didesni KVAL renkames.

KVAL ~22 -> 2.80V
KVAL ~32 -> 3.30V
KVAL ~42 -> 5.40V

*/


// bandomasis presetas
//static const MotorParamSet ParamSet0 = {
//    .ID = 0x00, .StepsPerRev = 200,
//    .Kval = { .RunValue = 80, .AccValue = 80, .DecValue = 80, .HoldValue = 5 },
//    .Treshold = { .OcdValue = 500, .StallValue = 500 },
//    .Speed = { .Acceleration = 100, .Deceleration = 100 }
//};


// 23HS22-1504S variklio presetas (5.4V/1.5A)
static const MotorParamSet ParamSet0 = {
    .ID = 0x00, .StepsPerRev = 200,
    .Kval = { .RunValue = 36, .AccValue = 38, .DecValue = 34, .HoldValue = 5 },
    .Treshold = { .OcdValue = 3000, .StallValue = 3000 },
    .Speed = { .Acceleration = 100, .Deceleration = 100 }
};


// 57BYGH748AA variklio presetas (3A) - ok
static const MotorParamSet ParamSet1 = {
    .ID = 0x01, .StepsPerRev = 200,
    .Kval = { .RunValue = 32, .AccValue = 34, .DecValue = 30, .HoldValue = 5 },
    .Treshold = { .OcdValue = MAX_TRES_OCD_MA_DEF, .StallValue = 3500 },
    .Speed = { .Acceleration = 100, .Deceleration = 100 }
};

// MICROCON SX34-2740N variklio presetas (2.5A)
static const MotorParamSet ParamSet2 = {
    .ID = 0x02, .StepsPerRev = 200,
    .Kval = { .RunValue = 40, .AccValue = 40, .DecValue = 40, .HoldValue = 5 },
    .Treshold = { .OcdValue = MAX_TRES_OCD_MA_DEF, .StallValue = 3500 },
    .Speed = { .Acceleration = 100, .Deceleration = 100 }
};


// 57BYGH201AA variklio presetas (2.8V/2A) - ok
static const MotorParamSet ParamSet3 = {
    .ID = 0x03, .StepsPerRev = 200,
    .Kval = { .RunValue = 22, .AccValue = 24, .DecValue = 20, .HoldValue = 5 },
    .Treshold = { .OcdValue = 2500, .StallValue = 2500 },
    .Speed = { .Acceleration = 100, .Deceleration = 100 }
};

// MATSUSHITA KP39HM4-016 variklio presetas (12V/0.08A) - ok
static const MotorParamSet ParamSet4 = {
    .ID = 0x04, .StepsPerRev = 100,
    .Kval = { .RunValue = 100, .AccValue = 100, .DecValue = 100, .HoldValue = 5 },
    .Treshold = { .OcdValue = 400, .StallValue = 400 },
    .Speed = { .Acceleration = 100, .Deceleration = 100 }
};

//--------
static const MotorParamSet ParamSet5 = {
    .ID = 0x05, .StepsPerRev = 200,
    .Kval = { .RunValue = 22, .AccValue = 22, .DecValue = 22, .HoldValue = 5 },
    .Treshold = { .OcdValue = 1000, .StallValue = 1000 },
    .Speed = { .Acceleration = 100, .Deceleration = 100 }
};

//--------
static const MotorParamSet ParamSet6 = {
    .ID = 0x06, .StepsPerRev = 200,
    .Kval = { .RunValue = 22, .AccValue = 22, .DecValue = 22, .HoldValue = 5 },
    .Treshold = { .OcdValue = 1000, .StallValue = 1000 },
    .Speed = { .Acceleration = 100, .Deceleration = 100 }
};

// 7-as setas ( UserSet ) userio - vartotojas parametrus nustato modbase
static MotorParamSet UserSet = {
    .ID = 0x07, .StepsPerRev = 0,
    .Kval = { .RunValue = 0, .AccValue = 0, .DecValue = 0, .HoldValue = 0 },
    .Treshold = { .OcdValue = 1000, .StallValue = 1000 },
    .Speed = { .Acceleration = 0, .Deceleration = 0 }
};


/* Varikliu presetu pointeriai */
const MotorParamSet* MotorPresets[8] = { &ParamSet0, &ParamSet1, &ParamSet2, &ParamSet3, &ParamSet4, &ParamSet5, &ParamSet6, &UserSet };
