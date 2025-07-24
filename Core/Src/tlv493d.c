#include "tlv493d.h"

/* ================================================================ */
/*  I²C low-level                                                    */
/* ================================================================ */
static I2C_HandleTypeDef hi2c1;
static uint16_t angle_filt;

#define TLV_ADDR       0x5E     /* 7-bit address       */
/* Register map */
#define REG_BX1        0x00
#define REG_MOD1       0x10
#define REG_MOD2       0x11
#define REG_MOD3       0x12

/* 8-bit bus addresses (include R/W bit) */
// #define TLV_ADDR_W   0xBC        /* write byte */
// #define TLV_ADDR_R   0xBD        /* read  byte */
#define TLV_ADDR_W   0x3E        /* write byte */
#define TLV_ADDR_R   0x3F        /* read  byte */

/* ------------ GPIO : PA9=SCL , PA10=SDA (AF4) ------------------- */
static void TLV_I2C_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef io = {
        .Pin   = GPIO_PIN_9 | GPIO_PIN_10,
        .Pull  = GPIO_PULLUP,
        .Speed = GPIO_SPEED_FREQ_HIGH
    };

    /* ----------------------------------------------------------
     * 1.  Temporary OUTPUT-OD for bus-recovery
     * ---------------------------------------------------------- */
    io.Mode = GPIO_MODE_OUTPUT_OD;
    HAL_GPIO_Init(GPIOA, &io);

    /* nine SCL pulses with SDA released → clear “bus busy”  */
    for (int i = 0; i < 9; ++i) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
        HAL_Delay(1);                       /* ≈5–10 µs ok */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
        HAL_Delay(1);
    }

    /* ----------------------------------------------------------
     * 2.  Re-configure pins for the I²C peripheral
     * ---------------------------------------------------------- */
    io.Mode      = GPIO_MODE_AF_OD;
    io.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOA, &io);              /* overwrites MODE bits */
}

static void i2c_recover(void)          /* PA9=SCL, PA10=SDA */
{
    GPIO_InitTypeDef io = { .Pin = GPIO_PIN_9|GPIO_PIN_10,
                            .Mode = GPIO_MODE_OUTPUT_OD,
                            .Pull = GPIO_PULLUP, .Speed = GPIO_SPEED_FREQ_HIGH };
    __HAL_RCC_GPIOA_CLK_ENABLE();
    HAL_GPIO_Init(GPIOA, &io);

    for (int i = 0; i < 9; ++i) {              /* 9 clocks, SDA released */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
        HAL_Delay(1);
    }
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
static HAL_StatusTypeDef tlv_soft_reset(void)
{
    uint8_t zero = 0x00;
    HAL_StatusTypeDef st =
        HAL_I2C_Master_Transmit(&hi2c1, 0x00, &zero, 1, HAL_MAX_DELAY); /* GC */
    HAL_Delay(2);                              /* >1.5 ms as per datasheet */
    return st;
}

static HAL_StatusTypeDef tlv_send_config(void)
{
    /* 0x00 pointer, then 0x83 0x00 0x60  (global parity already odd) */
    const uint8_t cfg[4] = {0x00, 0x02, 0x00, 0x60 };

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
// static uint16_t int_atan2_deg(int16_t y, int16_t x)
// {
//     if (x == 0 && y == 0) return 0;

//     int32_t ax = x >= 0 ?  x : -x;
//     int32_t ay = y >= 0 ?  y : -y;

//     /* tan(θ) ≈ y/x, scaled by 128 */
//     int32_t t;
//     if (ax == 0)
//         t = 301;                        /* force exactly 89° bucket */
//     else {
//         t = (ay << 7) / ax;             /* 0 … large */
//         if (t > 301) t = 301;           /* clamp to 89° equivalent */
//     }

//     /* piece-wise LUT */
//     uint16_t angle =
//         (t <  13) ? ( 0 +  t) :
//         (t <  39) ? (10 + (t-13)/2) :
//         (t <  98) ? (30 + (t-39)/3) :
//                     (60 + (t-98)/7);    /* now maxes out at 89° */

//     /* quadrant folding */
//     if      (x >= 0 && y >= 0) ;                     /* Q1 */
//     else if (x <  0 && y >= 0) angle = 180 - angle;  /* Q2 */
//     else if (x <  0 && y <  0) angle = 180 + angle;  /* Q3 */
//     else                       angle = 360 - angle;  /* Q4 */

//     return angle;                                    /* 0 … 359 */
// }

static uint16_t int_atan2_deg(int16_t y, int16_t x) {
    // Fixed-point atan2 approximation returning angle in degrees from 0–359
    // Accuracy is coarse but fast, suitable for embedded systems.

    const uint8_t atan_table[8] = { 0, 14, 28, 45, 63, 84, 108, 135 };
    uint16_t angle;
    int16_t abs_y = (y >= 0) ? y : -y;
    int16_t abs_x = (x >= 0) ? x : -x;
    int16_t ratio;

    if (abs_x == 0 && abs_y == 0)
        return 0;  // undefined, default to 0°

    if (abs_x > abs_y) {
        ratio = (abs_y << 7) / abs_x;  // scale to 0–128
        angle = atan_table[(ratio * 7) >> 7];
    } else {
        ratio = (abs_x << 7) / abs_y;  // scale to 0–128
        angle = 90 - atan_table[(ratio * 7) >> 7];
    }

    // Adjust angle based on quadrant
    if (x >= 0 && y >= 0) {
        return angle;                 // Q1: 0–89°
    } else if (x < 0 && y >= 0) {
        return 180 - angle;           // Q2: 90–179°
    } else if (x < 0 && y < 0) {
        return 180 + angle;           // Q3: 180–269°
    } else { // (x >= 0 && y < 0)
        return 360 - angle;           // Q4: 270–359°
    }
}

uint16_t fast_atan2_deg(int16_t y, int16_t x) {
    // Approximate atan2(y,x) in degrees [0–359]
    // Max error ~1°, no LUTs, no floats
    // Source: Rajan et al., 2001 / Cleaned up versions of atan2 approximations

    const int32_t ONE_EIGHTY = 180;
    const int32_t NINETY = 90;
    const int32_t RAD_TO_DEG_SCALE = 57;  // approximation of (180 / PI)

    if (x == 0 && y == 0)
        return 0; // undefined, default to 0

    int32_t abs_y = y >= 0 ? y : -y;
    int32_t abs_x = x >= 0 ? x : -x;

    int32_t angle;
    if (abs_x > abs_y) {
        int32_t r = (abs_y * 1000) / abs_x;
        angle = ((45 * r) + 500) / 1000;
    } else {
        int32_t r = (abs_x * 1000) / abs_y;
        angle = 90 - ((45 * r + 500) / 1000);
    }

    // Quadrant correction
    if (x >= 0 && y >= 0) {
        return angle;
    } else if (x < 0 && y >= 0) {
        return 180 - angle;
    } else if (x < 0 && y < 0) {
        return 180 + angle;
    } else { // x >= 0 && y < 0
        return 360 - angle;
    }
}


/* ================================================================ */
/*  Public functions                                                */
/* ================================================================ */
void TLV493D_Init(void)
{
    TLV_I2C_GPIO_Init();
    TLV_I2C_Init();
    tlv_soft_reset();  /* general-call reset */
    HAL_Delay(2);      /* >1.5 ms as per datasheet */
    tlv_send_config();  /* send initial configuration */
    HAL_Delay(2);      /* >1.5 ms as per datasheet */
}

static HAL_StatusTypeDef tlv493d_read6(uint8_t buf[6])
{
    if (HAL_I2C_Master_Receive (&hi2c1, TLV_ADDR_R, buf, 6, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

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
        angle = fast_atan2_deg(y, x);
        angle_filt = angle_filt - (angle_filt >> 2)  /* (1-α) part, α = 1/4 */
           + angle;                         /* + α·new */
        uint16_t temp_angle_filt = angle_filt >> 2;
        // printf("x=%d, y=%d angle = %d, filt = %d \r\n", x, y, angle, temp_angle_filt);
        // uart_puti32(temp_angle_filt);
        // uart_puts("°\r\n");
        // printf("%d°\r\n", angle);
    } else
    {
        uint32_t err = HAL_I2C_GetError(&hi2c1);
        // printf("I2C ERR 0x %lx \r\n", err);
        return HAL_ERROR;
    }

    return angle;            /* 0 … 359° */
}
