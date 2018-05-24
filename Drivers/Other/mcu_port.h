#ifndef PERIPH_H_INCLUDED
#define PERIPH_H_INCLUDED

enum { RESULT_OK = 0x00U, RESULT_ERR, RESULT_BUSY, RESULT_TIMEOUT, RESULT_BAD_PARAM };

#define STM32F051xx

#if defined STM32F051xx
    #include "stm32f051xx.h"
#endif





#endif /* PERIPH_H_INCLUDED */
