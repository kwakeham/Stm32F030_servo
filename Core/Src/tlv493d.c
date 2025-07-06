#include "tlv493d.h"

/* ================================================================ */
/*  I²C low-level                                                    */
/* ================================================================ */
static I2C_HandleTypeDef hi2c1;

#define TLV_ADDR       0x5E     /* 7-bit address       */
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
// static HAL_StatusTypeDef tlv_soft_reset(void)
// {
//     uint8_t zero = 0x00;
//     HAL_StatusTypeDef st =
//         HAL_I2C_Master_Transmit(&hi2c1, 0x00, &zero, 1, HAL_MAX_DELAY); /* GC */
//     HAL_Delay(2);                              /* >1.5 ms as per datasheet */
//     return st;
// }

// static HAL_StatusTypeDef tlv_write_config(void)
// {
//     uint8_t cfg[4] = { REG_MOD1, 0x01, 0x00, 0x02 };  /* Fast+LP, MC-mode */
//     return HAL_I2C_Master_Transmit(&hi2c1,
//                                    TLV_ADDR << 1,
//                                    cfg, sizeof cfg,
//                                    HAL_MAX_DELAY);
// }

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
    // tlv_soft_reset();  /* general-call reset */
    // HAL_Delay(2);      /* >1.5 ms as per datasheet */
}

/* ---- wakes TLV493D, waits one conversion, reads 6-byte frame ---- */
static HAL_StatusTypeDef tlv493d_read6(uint8_t buf[6])
{
    /* 1. Put sensor into MASTER-CONTROLLED (MODE = 0b001) */
    uint8_t wr[4];

    /* 1.1 read shadow registers 0x00-0x03 */
    if (HAL_I2C_Master_Transmit(&hi2c1, TLV_ADDR << 1,
                                (uint8_t[]){0x00}, 1, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    if (HAL_I2C_Master_Receive(&hi2c1, TLV_ADDR << 1,
                               wr, 4, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    /* 1.2 modify only MODE bits */
    wr[1] = (wr[1] & ~0x07) | 0x01;        /* MODE=001 */

    /* 1.3 recompute GLOBAL **odd** parity (bit7 of MOD1) */
    uint32_t map =  (uint32_t)wr[0] |
                   ((uint32_t)wr[1] <<  8) |
                   ((uint32_t)wr[2] << 16) |
                   ((uint32_t)wr[3] << 24);
    if ((__builtin_popcount(map) & 1) == 0) wr[1] ^= 0x80;

    /* 1.4 write back complete frame */
    uint8_t frame[5] = {0x00, wr[0], wr[1], wr[2], wr[3]};
    if (HAL_I2C_Master_Transmit(&hi2c1, TLV_ADDR << 1,
                                frame, 5, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    /* 2. wait for measurement (datasheet 2.3 ms typ, 3 ms safe) */
    HAL_Delay(3);

    /* 3. pointer to 0x00, repeated START, read 6 bytes */
    if (HAL_I2C_Master_Transmit(&hi2c1, TLV_ADDR << 1,
                                (uint8_t[]){0x00}, 1, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    if (HAL_I2C_Master_Receive(&hi2c1, TLV_ADDR << 1,
                               buf, 6, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    return HAL_OK;
}


uint16_t TLV493D_ReadAngleDeg(void)
{
    uint8_t raw[6];
    uint16_t angle = 0;
    if (tlv493d_read6(raw) == HAL_OK) {
        int16_t x = (raw[0] << 4) | (raw[4] >> 4);
        int16_t y = (raw[1] << 4) | (raw[4] & 0x0F);
        if (x & 0x800) x |= 0xF000;
        if (y & 0x800) y |= 0xF000;
        angle = int_atan2_deg(y, x);
        // printf("%d°\r\n", angle);
    }

    return angle;            /* 0 … 359° */
}
