#ifndef __MIC_H
#define __MIC_H
#include "main.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"
#include "arm_math.h"
#include "pdm_fir.h"
#include "audio_output.h"

void mic_init(void);

#endif
