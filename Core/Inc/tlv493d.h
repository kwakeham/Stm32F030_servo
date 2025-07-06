#ifndef TLV493D_H
#define TLV493D_H

#include "stm32f0xx_hal.h"

/* ------------- user API ------------- */
void     TLV493D_Init        (void);     /* call once at start-up           */
uint16_t TLV493D_ReadAngleDeg(void);     /* 0 … 359   (integer degrees)     */

#endif /* TLV493D_H */
