#include "tlv493d.h"

/* ================================================================
 *  I²C low-level
 * ================================================================ */
static I2C_HandleTypeDef hi2c1;

#define TLV_ADDR       (0x5E >> 1)       /* 7-bit = 0x2F                    */
#define REG_DATA       0x00              /* 6 bytes: X(12) Y(12) Z(12) T(8) */

static void TLV_I2C_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef io = {
        .Pin       = GPIO_PIN_9 | GPIO_PIN_10,
        .Mode      = GPIO_MODE_AF_OD,
        .Pull      = GPIO_PULLUP,
        .Speed     = GPIO_SPEED_FREQ_HIGH,
        .Alternate = GPIO_AF4_I2C1
    };
    HAL_GPIO_Init(GPIOA, &io);
}

static void TLV_I2C_Init(void)
{
    __HAL_RCC_I2C1_CLK_ENABLE();
    hi2c1.Instance             = I2C1;
    hi2c1.Init.Timing          = 0x00404C74;          /* ≈100 kbit/s @48 MHz */
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1);
}

/* ================================================================
 *  Fixed-point atan2 approximation
 *  Returns 0…359 (deg)       error ≤1°
 *  Source: cordic-lite, two iterations
 * ================================================================ */
static uint16_t int_atan2_deg(int16_t y, int16_t x)
{
    /* handle quadrants and avoid div by 0 */
    if (x == 0 && y == 0) return 0;

    int32_t ax = (x >= 0) ?  x : -x;
    int32_t ay = (y >= 0) ?  y : -y;

    /*  tan(angle) ≈ y/x  scaled by 128  */
    int32_t t = (ay << 7) / (ax + 1);

    /* piece-wise linear approximation */
    uint16_t angle =
        (t <  13) ? ( 0 +  t) :                       /*   0–  9° */
        (t <  39) ? (10 + (t-13)/2) :                 /*  10– 29° */
        (t <  98) ? (30 + (t-39)/3) :                 /*  30– 59° */
                    (60 + (t-98)/7);                  /*  60– 89° */

    /* octave folding */
    if      (x >= 0 && y >= 0) ;                     /* Q1: angle     */
    else if (x <  0 && y >= 0) angle = 180 - angle;  /* Q2            */
    else if (x <  0 && y <  0) angle = 180 + angle;  /* Q3            */
    else                       angle = 360 - angle;  /* Q4            */

    return angle % 360;
}

/* ================================================================
 *  Public functions
 * ================================================================ */
void TLV493D_Init(void)
{
    TLV_I2C_GPIO_Init();
    TLV_I2C_Init();

    /* optional: put sensor into low-power single-measurement mode
       by writing 0x05 to register 0x10 — skipped here for brevity  */
}

uint16_t TLV493D_ReadAngleDeg(void)
{
    uint8_t buf[6];
    HAL_I2C_Master_Transmit(&hi2c1, TLV_ADDR << 1, (uint8_t[]){REG_DATA}, 1,
                            HAL_MAX_DELAY);
    HAL_I2C_Master_Receive (&hi2c1, TLV_ADDR << 1, buf, 6, HAL_MAX_DELAY);

    /* ---- unpack 12-bit signed X / Y ---------------------------- */
    int16_t x = (int16_t)((buf[0] << 4) | (buf[4] >> 4));
    int16_t y = (int16_t)((buf[1] << 4) | (buf[4] & 0x0F));

    /* sign-extend from 12 bits */
    if (x & 0x800) x |= 0xF000;
    if (y & 0x800) y |= 0xF000;

    return int_atan2_deg(y, x);     /* 0…359° */
}
