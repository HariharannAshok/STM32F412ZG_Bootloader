/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : User Application for Bootloader Test (NUCLEO-F412ZG)
  ******************************************************************************
  */
#include "main.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"
#include <string.h>
#include <stdio.h>

/* -------------------------------------------------------------------------- */
/* Private function prototypes                                                */
/* -------------------------------------------------------------------------- */
void SystemClock_Config(void);

/* App base for VTOR (must match linker script: FLASH ORIGIN = 0x08008000) */
#define APP_BASE_ADDR  0x08008000U

int main(void)
{
  /* 1) Initialize HAL (sets default SysTick @ 1 ms based on current HCLK) */
  HAL_Init();

  /* 2) Ensure the vector table points to our app BEFORE enabling interrupts */
  if (SCB->VTOR != APP_BASE_ADDR)
  {
    SCB->VTOR = APP_BASE_ADDR;
    __DSB(); __ISB();
  }

  /* 3) Re-enable interrupts (bootloader disabled them before jump) */
  __enable_irq();

  /* 4) Configure the system clock for the user app */
  SystemClock_Config();

  /* 5) Make sure SysTick matches the new HCLK (some HALs do this for you; this is safe anyway) */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000U);
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  /* Optional: also ensure SysTick IRQ priority is sane (CubeMX usually does this in HAL_Init) */
  HAL_NVIC_SetPriority(SysTick_IRQn, TICK_INT_PRIORITY, 0);

  /* 6) Initialize peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  // MX_USB_OTG_FS_PCD_Init();   // Optional, enable only if you actually need USB

  /* Small startup delay for terminal connection stability */
  HAL_Delay(10);

  const char *msg = "\r\n[APP] User Application Running!\r\n";
  HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);

  /* Turn ON LED1 (PB0) and blink continuously */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);

  /* --- Optional one-shot diagnostics (uncomment if needed) --- */
  /*
  uint32_t s = SysTick->CTRL;
  char dbg[120];
  snprintf(dbg, sizeof(dbg),
           "[APP] SysTick CTRL=0x%08lX (EN=%lu TI=%lu CS=%lu) HCLK=%lu P1=%lu P2=%lu\r\n",
           s, (s>>0)&1UL, (s>>1)&1UL, (s>>2)&1UL,
           HAL_RCC_GetHCLKFreq(), HAL_RCC_GetPCLK1Freq(), HAL_RCC_GetPCLK2Freq());
  HAL_UART_Transmit(&huart3, (uint8_t*)dbg, strlen(dbg), HAL_MAX_DELAY);
  */

  while (1)
  {
    HAL_GPIO_TogglePin(GPIOB, LD1_Pin);
    HAL_Delay(500);

    const char *alive = "[APP] Alive\r\n";
    HAL_UART_Transmit(&huart3, (uint8_t *)alive, strlen(alive), HAL_MAX_DELAY);
  }
}

/* -------------------------------------------------------------------------- */
/* System Clock Configuration                                                 */
/* -------------------------------------------------------------------------- */
void SystemClock_Config(void)
{
  /* NUCLEO-F412ZG default: HSE BYPASS (8 MHz MCO from ST-LINK) → PLL */
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState       = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM       = 8;
  RCC_OscInitStruct.PLL.PLLN       = 384;
  RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV4;  /* SYSCLK = 384/4 = 96 MHz */
  RCC_OscInitStruct.PLL.PLLQ       = 8;              /* 48 MHz for USB FS if needed */

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    Error_Handler();

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK |
                                     RCC_CLOCKTYPE_SYSCLK |
                                     RCC_CLOCKTYPE_PCLK1 |
                                     RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;   /* 96 MHz */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;     /* 48 MHz (<= 50 MHz limit) */
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;     /* 96 MHz */

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
    Error_Handler();
}

/* -------------------------------------------------------------------------- */
/* Error Handler                                                              */
/* -------------------------------------------------------------------------- */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
    /* Blink LD3 (Error LED) if desired */
    HAL_GPIO_TogglePin(GPIOB, LD3_Pin);
    HAL_Delay(200);
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file; (void)line;
}
#endif
