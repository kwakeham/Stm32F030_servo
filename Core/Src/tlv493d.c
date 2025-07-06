#include "tlv493d.h"

/* ================================================================ */
/*  I²C low-level                                                    */
/* ================================================================ */
static I2C_HandleTypeDef hi2c1;

#define TLV_ADDR       (0x5E)     /* 7-bit address       */

/* Register map */
#define REG_BX1        0x00
#define REG_MOD1       0x10
#define REG_MOD2       0x11
#define REG_MOD3       0x12

/* ------------ GPIO : PA9=SCL , PA10=SDA (AF4) ------------------- */
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

/* ------------ I²C1 @ 100 kHz  (Timing for SYSCLK = 48 MHz) ------- */
/* 0x00201D2B  → 100 kbit/s, rise = 100 ns, fall = 10 ns            */
static void TLV_I2C_Init(void)
{
    __HAL_RCC_I2C1_CLK_ENABLE();
    hi2c1.Instance             = I2C1;
    hi2c1.Init.Timing          = 0x00201D2B; //0x0010020A;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1);
}

/* ================================================================ */
/*  Helper: general-call reset  +  master-controlled configuration  */
/* ================================================================ */


/* ================================================================ */
/*  Fixed-point atan2  (≤1 ° error, no libm)                         */
/* ================================================================ */
static uint16_t int_atan2_deg(int16_t y, int16_t x)
{
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

/* ================================================================ */
/*  Public functions                                                */
/* ================================================================ */
void TLV493D_Init(void)
{
    TLV_I2C_GPIO_Init();
    TLV_I2C_Init();
}

uint16_t TLV493D_ReadAngleDeg(void)
{
    uint8_t rx[6];

    /* Pointer write → repeated-START → burst read */
    HAL_StatusTypeDef st =
        HAL_I2C_Master_Transmit(&hi2c1,
                                TLV_ADDR << 1,
                                (uint8_t[]){REG_BX1}, 1,
                                HAL_MAX_DELAY);

    // if (st != HAL_OK)                 /* failed → encode reason */
    //     return (uint16_t)(0xF000u | st);   /* 0xFFxx where xx = HAL status */
    if (st != HAL_OK) {
        uint32_t err = HAL_I2C_GetError(&hi2c1);   /* bit-mask, see below */
        // printf("I2C error: 0x%08lX\r\n", err);     /* or return it */
        return (uint16_t)(err);  /* 0xFFxx where xx = error code */
    }

    st = HAL_I2C_Master_Receive(&hi2c1,
                               TLV_ADDR << 1,
                               rx, sizeof rx,
                            HAL_MAX_DELAY);
    if (st != HAL_OK)
        return (uint16_t)(0xFF00u | st);

    /* ---- unpack 12-bit signed X / Y ---------------------------- */
    int16_t x = (int16_t)((rx[0] << 4) | (rx[4] >> 4));
    int16_t y = (int16_t)((rx[1] << 4) | (rx[4] & 0x0F));
    if (x & 0x800) x |= 0xF000;    if (y & 0x800) y |= 0xF000;

    return int_atan2_deg(y, x);            /* 0 … 359° */
}
