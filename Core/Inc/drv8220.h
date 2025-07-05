#ifndef DRV8220_H
#define DRV8220_H

#include "stm32f0xx_hal.h"

/* ---------------- user-visible API ---------------- */
void     DRV8220_Init          (void);            /* call once at start-up  */
void     DRV8220_Sleep         (uint8_t on);      /* 0 = wake, 1 = sleep    */
void     DRV8220_SetSpeed      (int16_t pwm);     /* -1000 … +1000          */
int16_t  DRV8220_GetSpeed      (void);            /* signed duty -1000…1000 */

#endif /* DRV8220_H */
