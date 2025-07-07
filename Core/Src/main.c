/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart1;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint32_t rc_us = 1500;           // latest width in µs
volatile uint8_t  rc_new;         /* flag set in ISR             */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
void MX_TIM3_Init(void);
int _write(int file, char *ptr, int len);
void TIM3_IRQHandler(void);
static inline int16_t rc_us_to_pwm(int32_t us);
void uart_puts(const char *s);
void uart_putu32(uint32_t v);
void uart_puti32(int32_t v);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static int systick_is_running(void)
{
    return (SysTick->CTRL & SysTick_CTRL_ENABLE_Msk) != 0;
}

#define DEBUG_PIN   GPIO_PIN_5
#define DEBUG_PORT  GPIOA
static void wait3(void)
{
    HAL_GPIO_WritePin(DEBUG_PORT, DEBUG_PIN, GPIO_PIN_SET); /* ↑ */
    HAL_Delay(3);                                           /* 3 ms */
    HAL_GPIO_WritePin(DEBUG_PORT, DEBUG_PIN, GPIO_PIN_RESET);/* ↓ */
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();

  DRV8220_Init();  // Initialize the DRV8220 motor driver
  TLV493D_Init();  // Initialize the TLV493D magnetic angle sensor

  /* USER CODE BEGIN 2 */
  printf("\r\n*** USART1 ready on PA2/PA3 @115200 ***\r\n");
  /* USER CODE END 2 */

  if (systick_is_running())
    printf("SysTick ON  –  HAL_Delay() will block\r\n");
  else
      printf("SysTick OFF –  HAL_Delay() returns immediately!\r\n");

  wait3();  /* wait 3 ms */
  // extern volatile uint32_t uwTick;
  // uint32_t t0 = uwTick;
  // HAL_Delay(3);
  // uint32_t t1 = uwTick;
  // uart_putu32(t1 - t0);    /* should print 3 (±1) */
  // uart_puts("\r\n");
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // /* USER CODE END WHILE */
    // uint32_t width = rc_us;       /* atomic copy */
    // int16_t pwm = rc_us_to_pwm(width);
    // DRV8220_SetSpeed(pwm);                  /* drive the motor  */
    // // printf("Tick: %lu pwm: %lu ms\r\n", HAL_GetTick(), width);
    // printf("%lu µs  →  %d\n", width, pwm);
    // HAL_Delay(500);
    if (rc_new)               /* do we have fresh data?      */
    {
        rc_new = 0;           /* clear BEFORE processing      */
        uint32_t width = rc_us;  /* atomic copy                  */
        int16_t  pwm = rc_us_to_pwm(width);
        DRV8220_SetSpeed(pwm);

        uint32_t angle = TLV493D_ReadAngleDeg();  /* read angle sensor */

        // uart_puts(" angle: ");
        // uart_putu32(angle);
        // uart_puts("\r\n");

    }

        /* go to sleep until the next capture interrupt (≈20 ms) */
    __WFI();                  /* or HAL_PWR_EnterSLEEPMode()  */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  // /*Configure GPIO pins : PA0 PA1 */
  // GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  // GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  // GPIO_InitStruct.Pull = GPIO_NOPULL;
  // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  // HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_USART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // /*Configure GPIO pin : PA4 */
  // GPIO_InitStruct.Pin = GPIO_PIN_4;
  // GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  // GPIO_InitStruct.Pull = GPIO_NOPULL;
  // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  // GPIO_InitStruct.Alternate = GPIO_AF4_TIM14;
  // HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitTypeDef io = {0};
  io.Pin   = GPIO_PIN_5;              /* PA5 */
  io.Mode  = GPIO_MODE_OUTPUT_PP;     /* push-pull output */
  io.Pull  = GPIO_NOPULL;
  io.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &io);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);   /* start low */

  // /*Configure GPIO pin : PB1 */
  // GPIO_InitStruct.Pin = GPIO_PIN_1;
  // GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  // GPIO_InitStruct.Pull = GPIO_NOPULL;
  // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  // HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA10 */
  // GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
  // GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  // GPIO_InitStruct.Pull = GPIO_NOPULL;
  // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  // GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  // HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* ------------- GPIO (PA6, PA7 → TIM3 CH1/CH2) ---------------- */

void MX_TIM3_Init(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();

    htim3.Instance           = TIM3;
    htim3.Init.Prescaler     = (HAL_RCC_GetHCLKFreq() / 1000000) - 1; // 1 µs
    htim3.Init.CounterMode   = TIM_COUNTERMODE_UP;
    htim3.Init.Period        = 0xFFFF;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_IC_Init(&htim3);

    /* PWM-input: CH1 = rising (direct), CH2 = falling (indirect) */
    TIM_IC_InitTypeDef ic = {
        .ICPrescaler = TIM_ICPSC_DIV1,
        .ICFilter    = 0
    };
    ic.ICPolarity   = TIM_INPUTCHANNELPOLARITY_RISING;
    ic.ICSelection  = TIM_ICSELECTION_DIRECTTI;
    HAL_TIM_IC_ConfigChannel(&htim3, &ic, TIM_CHANNEL_1);

    ic.ICPolarity   = TIM_INPUTCHANNELPOLARITY_FALLING;
    ic.ICSelection  = TIM_ICSELECTION_INDIRECTTI;
    HAL_TIM_IC_ConfigChannel(&htim3, &ic, TIM_CHANNEL_2);

    /* reset counter on rising edge */
    TIM_SlaveConfigTypeDef slave = {
        .SlaveMode   = TIM_SLAVEMODE_RESET,
        .InputTrigger= TIM_TS_TI1FP1
    };
    HAL_TIM_SlaveConfigSynchro(&htim3, &slave);

    /* start capture with interrupt on CH2 (falling) */
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);

    HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

/* ------------- IRQ: save the width --------------------------- */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3 &&
        htim->Channel   == HAL_TIM_ACTIVE_CHANNEL_2)
    {
        rc_us = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
        if (rc_us < 800 || rc_us > 2200)   // sanity clamp
            rc_us = 1500;
        rc_new = 1;  // set flag to indicate new value available
    }
}

/* stm32f0xx_it.c */
void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim3);   /* hands control to HAL */
}


int _write(int file, char *ptr, int len)
{
    (void)file;                       /* stdout/stderr both go to UART */
    HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

static void MX_USART1_UART_Init(void)
{
    __HAL_RCC_USART1_CLK_ENABLE();          /* ❶ peripheral clock */

    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 115200;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;   /* TX + RX both enabled */
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);

    /* Optional: enable interrupt-driven TX/RX */
    // __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
    // HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
    // HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/* Map RC-servo input (µs) → signed duty            */
/* 1000 µs = full-reverse (-1000)                    */
/* 1500 µs = neutral (0)                             */
/* 2000 µs = full-forward (+1000)                    */
static inline int16_t rc_us_to_pwm(int32_t us)
{
    /* Linear scaling: 500 µs span ↔ 1000 units      */
    int32_t pwm = (us - 1500) * 2;       /* (us-1500) × (1000/500) */

    /* Clamp to the endpoints in case of noise       */
    if (pwm >  1000) pwm =  1000;
    if (pwm < -1000) pwm = -1000;

    /* Optional: add a small dead-band around center */
    if (pwm > -20 && pwm < 20) pwm = 0;

    return (int16_t)pwm;
}

void uart_puts(const char *s)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)s, strlen(s), HAL_MAX_DELAY);
}

void uart_putu32(uint32_t v)
{
    char buf[11];            /* enough for 4 294 967 295 */
    int  i = 10;
    buf[i--] = '\0';
    do { buf[i--] = '0' + (v % 10); } while ((v /= 10));
    HAL_UART_Transmit(&huart1, (uint8_t*)&buf[i+1], 10-i, HAL_MAX_DELAY);
}

void uart_puti32(int32_t v)
{
    /*  "-2147483648"  + '\0'  = 12 bytes  */
    char buf[12];
    int  i = 11;
    buf[i--] = '\0';

    /* Special-case INT32_MIN because –INT32_MIN overflows */
    if (v == INT32_MIN) {
        const char *minstr = "-2147483648";
        HAL_UART_Transmit(&huart1, (uint8_t*)minstr, 11, HAL_MAX_DELAY);
        return;
    }

    /* Handle sign */
    uint32_t u = v;
    if (v < 0) {
        u = (uint32_t)(-v);   /* make positive */
    }

    /* Convert to decimal */
    do {
        buf[i--] = '0' + (u % 10);
        u /= 10;
    } while (u);

    if (v < 0) {
        buf[i--] = '-';
    }

    HAL_UART_Transmit(&huart1, (uint8_t*)&buf[i+1], 11 - i, HAL_MAX_DELAY);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
    printf("Error occurred!\r\n");
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
