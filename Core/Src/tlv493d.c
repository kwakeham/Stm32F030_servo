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

/* 8-bit bus addresses (include R/W bit) */
#define TLV_ADDR_W   0xBC        /* write byte */
#define TLV_ADDR_R   0xBD        /* read  byte */

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

static HAL_StatusTypeDef tlv_send_config(void)
{
    /* 0x00 pointer, then 0x83 0x00 0x60  (global parity already odd) */
    const uint8_t cfg[4] = { 0x00, 0x83, 0x00, 0x60 };

    HAL_StatusTypeDef st =
        HAL_I2C_Master_Transmit(&hi2c1, TLV_ADDR_W,   /* 8-bit write addr */
                                (uint8_t*)cfg, sizeof cfg,
                                HAL_MAX_DELAY);

    return st;
}

static HAL_StatusTypeDef tlv_send_sleep(void)
{
    /* 0x00 pointer, then 0x83 0x00 0x60  (global parity already odd) */
    const uint8_t cfg[4] = { 0x00, 0x00, 0x00, 0x20 };

    HAL_StatusTypeDef st =
        HAL_I2C_Master_Transmit(&hi2c1, TLV_ADDR_W,   /* 8-bit write addr */
                                (uint8_t*)cfg, sizeof cfg,
                                HAL_MAX_DELAY);

    return st;
}

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
    tlv_send_config();  /* send initial configuration */
    // tlv_soft_reset();  /* general-call reset */
    // HAL_Delay(2);      /* >1.5 ms as per datasheet */
}

/* ---- wakes TLV493D, waits one conversion, reads 6-byte frame ---- */


/* ---- no more “<< 1” anywhere ---- */
static HAL_StatusTypeDef tlv493d_read6(uint8_t buf[6])
{
    if (tlv_send_config() != HAL_OK)
    {
        uint32_t err = HAL_I2C_GetError(&hi2c1);
        printf("I2C ERR 0x %lx \r\n", err);
        return HAL_ERROR;
    }

    HAL_Delay(3);  /* wait one conversion (≥2.3 ms) */

    if (HAL_I2C_Master_Receive (&hi2c1, TLV_ADDR_R, buf, 6, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    tlv_send_sleep();  /* put TLV493D to sleep */

    return HAL_OK;
}



uint16_t TLV493D_ReadAngleDeg(void)
{
    uint8_t raw[6];
    uint16_t angle = 0;
    if (tlv493d_read6(raw) == HAL_OK) {
        int16_t x = ((raw[0] << 4) | (raw[4] >> 4));
        int16_t y = ((raw[1] << 4) | (raw[4] & 0x0F));
        if (x & 0x800) x |= 0xF000;
        if (y & 0x800) y |= 0xF000;
        // angle = int_atan2_deg(y, x);
        printf("x=%d, y=%d\r\n", x, y);
        // printf("%d°\r\n", angle);
    }

    return angle;            /* 0 … 359° */
}
