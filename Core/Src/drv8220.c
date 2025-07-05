#include "drv8220.h"

/* ===========================================================
 *  GPIO MAPPING
 * =========================================================== */
#define NSLEEP_GPIO_PORT    GPIOA
#define NSLEEP_PIN          GPIO_PIN_0

#define MODE_GPIO_PORT      GPIOA
#define MODE_PIN            GPIO_PIN_1      /* left as input w/ PU */

#define PHASE_GPIO_PORT     GPIOB
#define PHASE_PIN           GPIO_PIN_1

/* ===========================================================
 *  STATIC STATE
 * =========================================================== */
static TIM_HandleTypeDef htim14;
static int16_t           current_pwm = 0;

/* ===========================================================
 *  LOW-LEVEL HELPERS
 * =========================================================== */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef io = {0};

    /* nSLEEP (PA0) -------------------------------------------------- */
    io.Pin   = NSLEEP_PIN;
    io.Mode  = GPIO_MODE_OUTPUT_PP;
    io.Speed = GPIO_SPEED_FREQ_HIGH;
    io.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(NSLEEP_GPIO_PORT, &io);

    /* MODE (PA1) : leave as input, rely on external pull-up -------- */
    io.Pin  = MODE_PIN;
    io.Mode = GPIO_MODE_INPUT;
    io.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(MODE_GPIO_PORT, &io);

    /* PHASE (PB1) --------------------------------------------------- */
    io.Pin   = PHASE_PIN;
    io.Mode  = GPIO_MODE_OUTPUT_PP;
    io.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(PHASE_GPIO_PORT, &io);
}

/* ----- 20 kHz PWM on TIM14_CH1 (PA4, AF4) -------------------------- */
static void MX_TIM14_Init(void)
{
    __HAL_RCC_TIM14_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* —— GPIO: PA4 → AF4 TIM14_CH1 —— */
    GPIO_InitTypeDef io = {
        .Pin       = GPIO_PIN_4,
        .Mode      = GPIO_MODE_AF_PP,
        .Pull      = GPIO_NOPULL,
        .Speed     = GPIO_SPEED_FREQ_HIGH,
        .Alternate = GPIO_AF4_TIM14
    };
    HAL_GPIO_Init(GPIOA, &io);

    /* —— Timer base —— */
    /* 48 MHz / (PSC+1) / (ARR+1)  = 20 000 Hz  */
    htim14.Instance               = TIM14;
    htim14.Init.Prescaler         = 0;                 /* 48 MHz */
    htim14.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim14.Init.Period            = 2399;              /* 20 kHz → 2400 ticks */
    htim14.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_PWM_Init(&htim14);

    /* —— PWM channel —— */
    TIM_OC_InitTypeDef ch = {0};
    ch.OCMode       = TIM_OCMODE_PWM1;
    ch.Pulse        = 0;           /* start disabled */
    ch.OCPolarity   = TIM_OCPOLARITY_HIGH;
    ch.OCFastMode   = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim14, &ch, TIM_CHANNEL_1);

    HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
}

/* ===========================================================
 *  PUBLIC API
 * =========================================================== */
void DRV8220_Init(void)
{
    MX_GPIO_Init();
    MX_TIM14_Init();

    /* Wake the driver, stop motor */
    HAL_GPIO_WritePin(NSLEEP_GPIO_PORT, NSLEEP_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PHASE_GPIO_PORT,  PHASE_PIN, GPIO_PIN_RESET);
}

void DRV8220_Sleep(uint8_t on)
{
    if (on)
        HAL_GPIO_WritePin(NSLEEP_GPIO_PORT, NSLEEP_PIN, GPIO_PIN_RESET);
    else
        HAL_GPIO_WritePin(NSLEEP_GPIO_PORT, NSLEEP_PIN, GPIO_PIN_SET);
}

static inline void pwm_set_raw(uint16_t duty)
{
    __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, duty);   /* 0…2399 */
}

/* pwm ∈ [-1000, +1000]  →  Phase + duty */
void DRV8220_SetSpeed(int16_t pwm)
{
    if (pwm > 1000)  pwm = 1000;
    if (pwm < -1000) pwm = -1000;
    current_pwm = pwm;

    if (pwm >= 0)
    {
        HAL_GPIO_WritePin(PHASE_GPIO_PORT, PHASE_PIN, GPIO_PIN_SET);     /* forward */
        pwm_set_raw((uint16_t)( (uint32_t)pwm * 2399 / 1000 ));
    }
    else
    {
        HAL_GPIO_WritePin(PHASE_GPIO_PORT, PHASE_PIN, GPIO_PIN_RESET);   /* reverse */
        pwm_set_raw((uint16_t)( (uint32_t)(-pwm) * 2399 / 1000 ));
    }
}

int16_t DRV8220_GetSpeed(void)
{
    return current_pwm;
}
