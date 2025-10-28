/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Bootloader Main Program (NUCLEO-F412ZG)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"
#include "crc.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

/* ---------------- Bootloader config ---------------- */
#define APP_START_ADDRESS      0x08008000U
#define APP_START_SECTOR       FLASH_SECTOR_2      /* sectors 2..11 used for app */
#define RX_BUFFER_SIZE         256
#define FIRST_BYTE_TIMEOUT_MS  30000               /* wait long for first byte (30s) */
#define IDLE_TIMEOUT_MS        2000                /* end of transfer if idle (2 s) */

/* Device memory ranges for sanity checks (STM32F412ZG) */
#define RAM_START              0x20000000U
#define RAM_END                0x20040000U         /* 256 KB SRAM total */
#define FLASH_START            0x08000000U
#define FLASH_END              0x08100000U         /* 1 MB flash */
#define APP_MIN_ADDR           (APP_START_ADDRESS)
#define APP_MAX_ADDR           (FLASH_END)

/* Prototypes --------------------------------------------------------------- */
void SystemClock_Config(void);
static void Bootloader_Erase_AppArea(void);
static HAL_StatusTypeDef Bootloader_Flash_Write(uint32_t address, const uint8_t *data, uint32_t length);
static void Bootloader_UART_Receive(void);
static void Bootloader_JumpToApplication(void);
static bool Bootloader_AppVectorsLookSane(uint32_t base);
static void JumpToApplication(uint32_t app_addr);

/* Simple prints over UART3 (blocking) */
static void bl_print(const char *s)
{
  HAL_UART_Transmit(&huart3, (uint8_t*)s, strlen(s), HAL_MAX_DELAY);
}
static void bl_print_hex32(const char *label, uint32_t v)
{
  char buf[64];
  snprintf(buf, sizeof(buf), "%s0x%08lX\r\n", label, (unsigned long)v);
  bl_print(buf);
}

/* ------------------------------------------------------------------------- */
/* Entry                                                                     */
/* ------------------------------------------------------------------------- */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART3_UART_Init();          /* USART3 = ST-Link VCP: PD8/PD9 @115200 */
  MX_USB_OTG_FS_PCD_Init();       /* optional, safe to leave */
  MX_CRC_Init();

  bl_print("[BL] Bootloader UART Hello World\r\n");
  bl_print("[BL] Send 'U' to start UART update (10s timeout)...\r\n");

  uint8_t cmd = 0;
  if (HAL_UART_Receive(&huart3, &cmd, 1, 10000) == HAL_OK && cmd == 'U')
  {
    bl_print("[BL] Update requested. Erasing app area...\r\n");
    Bootloader_UART_Receive();
  }
  else
  {
    bl_print("[BL] No update command received.\r\n");
    Bootloader_JumpToApplication();
  }

  while (1) { }
}

/* ------------------------------------------------------------------------- */
/* Helpers                                                                   */
/* ------------------------------------------------------------------------- */

static void Bootloader_Erase_AppArea(void)
{
  FLASH_EraseInitTypeDef eraseInit;
  uint32_t sectorError = 0;

  eraseInit.TypeErase    = FLASH_TYPEERASE_SECTORS;
  eraseInit.Sector       = FLASH_SECTOR_2;     /* start at sector 2 (0x08008000) */
  eraseInit.NbSectors    = 10;                 /* erase sectors 2..11 inclusive */
  eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;

  HAL_FLASH_Unlock();
  HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&eraseInit, &sectorError);
  HAL_FLASH_Lock();

  if (st == HAL_OK) bl_print("[BL] Flash erase complete\r\n");
  else              bl_print("[BL] Flash erase FAILED!\r\n");
}

static HAL_StatusTypeDef Bootloader_Flash_Write(uint32_t address, const uint8_t *data, uint32_t length)
{
  /* Optional safety: don't write past flash end */
  if ((address < APP_MIN_ADDR) || (address + length) > APP_MAX_ADDR)
  {
    bl_print("[BL] ERROR: Write out of range\r\n");
    return HAL_ERROR;
  }

  HAL_StatusTypeDef status = HAL_OK;
  HAL_FLASH_Unlock();

  static uint32_t byte_counter = 0;

  for (uint32_t i = 0; i < length; i += 4)
  {
    uint32_t word = 0xFFFFFFFFU;
    uint32_t chunk = (length - i) >= 4 ? 4U : (length - i);
    memcpy(&word, &data[i], chunk);

    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + i, word);
    if (status != HAL_OK)
    {
      HAL_UART_Transmit(&huart3, (uint8_t *)"E", 1, HAL_MAX_DELAY);
      break;
    }

    byte_counter += chunk;
    if (byte_counter >= 1024U)    /* every 1KB */
    {
      // HAL_UART_Transmit(&huart3, (uint8_t *)".", 1, HAL_MAX_DELAY);
      byte_counter = 0U;
    }
  }

  HAL_FLASH_Lock();
  return status;
}

/* ------------------------------------------------------------------------- */
/* UART Firmware Receive (Idle timeout based, no EOF marker needed)          */
/* ------------------------------------------------------------------------- */
static void Bootloader_UART_Receive(void)
{
    uint8_t  rxBuf[RX_BUFFER_SIZE];
    uint32_t appAddress = APP_START_ADDRESS;
    uint32_t totalBytes = 0;
    uint32_t rxCount    = 0;

    bl_print("[BL] Ready. Send your .bin now.\r\n");

    /* Erase user app area */
    Bootloader_Erase_AppArea();

    bl_print("[BL] Receiving...\r\n");

    /* Wait for first byte (long timeout to give user time to send) */
    uint8_t byte;
    if (HAL_UART_Receive(&huart3, &byte, 1, FIRST_BYTE_TIMEOUT_MS) != HAL_OK)
    {
        bl_print("[BL] No data arrived. Abort.\r\n");
        return;
    }

    /* Got first byte — start collecting data */
    rxBuf[rxCount++] = byte;
    totalBytes++;
    uint32_t lastByteTick = HAL_GetTick();

    while (1)
    {
        /* Try to read next byte with short timeout (non-blocking loop) */
        if (HAL_UART_Receive(&huart3, &byte, 1, 10) == HAL_OK)
        {
            rxBuf[rxCount++] = byte;
            totalBytes++;
            lastByteTick = HAL_GetTick();

            /* If buffer is full, write it to flash */
            if (rxCount >= RX_BUFFER_SIZE)
            {
                Bootloader_Flash_Write(appAddress, rxBuf, rxCount);
                appAddress += rxCount;
                rxCount = 0;

                /* Progress feedback every ~8KB */
                if ((totalBytes % 8192U) == 0U)
                {
                    char msg[64];
                    snprintf(msg, sizeof(msg), "\r\n[BL] %lu bytes...\r\n",
                             (unsigned long)totalBytes);
                    bl_print(msg);
                }
            }
        }
        else
        {
            /* Check idle timeout: no bytes received recently */
            if ((HAL_GetTick() - lastByteTick) > IDLE_TIMEOUT_MS)
            {
                /* Write any remaining data in the buffer */
                if (rxCount > 0)
                {
                    Bootloader_Flash_Write(appAddress, rxBuf, rxCount);
                    appAddress += rxCount;
                    rxCount = 0;
                }

                bl_print("\r\n[BL] End of stream detected (idle timeout)\r\n");
                break;
            }
        }
    }

    /* Transfer complete — summary */
    char msg[128];
    snprintf(msg, sizeof(msg),
             "[BL] Firmware receive complete. Total: %lu bytes\r\n",
             (unsigned long)totalBytes);
    bl_print(msg);

    /* Print vector table info */
    uint32_t sp = *(volatile uint32_t *)(APP_START_ADDRESS + 0);
    uint32_t pc = *(volatile uint32_t *)(APP_START_ADDRESS + 4);
    bl_print_hex32("[BL] Vector[0] (SP) = ", sp);
    bl_print_hex32("[BL] Vector[1] (PC) = ", pc);

    /* Jump to the new application */
    Bootloader_JumpToApplication();
}

/* ------------------------------------------------------------------------- */
/* App sanity check                                                          */
/* ------------------------------------------------------------------------- */
static bool Bootloader_AppVectorsLookSane(uint32_t base)
{
  uint32_t sp = *(__IO uint32_t*)base;
  uint32_t pc = *(__IO uint32_t*)(base + 4U);

  bool sp_ok = (sp >= RAM_START) && (sp <= RAM_END);
  bool pc_ok = (pc >= APP_MIN_ADDR) && (pc < APP_MAX_ADDR);
  return sp_ok && pc_ok;
}

/* ------------------------------------------------------------------------- */
/* Policy wrapper: validate + jump                                           */
/* ------------------------------------------------------------------------- */
static void Bootloader_JumpToApplication(void)
{
  if (Bootloader_AppVectorsLookSane(APP_START_ADDRESS))
  {
    bl_print("[BL] Jumping to user app...\r\n");
    JumpToApplication(APP_START_ADDRESS);
  }
  else
  {
    bl_print("[BL] No valid user app found.\r\n");
  }
}

/* ------------------------------------------------------------------------- */
/* Mechanics: perform the actual jump                                        */
/* ------------------------------------------------------------------------- */
typedef void (*pFunction)(void);

static void JumpToApplication(uint32_t app_addr)
{
  /* 1) Stop SysTick */
  SysTick->CTRL = 0;   /* disable */
  SysTick->LOAD = 0;
  SysTick->VAL  = 0;

  /* 2) Disable all interrupts */
  __disable_irq();

  /* 3) Clear pending IRQs (F4 has up to 240; clear first 8 words = 256 IRQs) */
  for (uint32_t i = 0; i < 8U; i++) {
    NVIC->ICER[i] = 0xFFFFFFFFU;
    NVIC->ICPR[i] = 0xFFFFFFFFU;
  }

  /* 4) Deinit peripherals used by bootloader */
  HAL_UART_DeInit(&huart3);
  HAL_PCD_DeInit(&hpcd_USB_OTG_FS);
  HAL_DeInit(); /* HAL timebase, etc. */

  /* 5) Reset clock tree to a known default (HSI ON, PLLs OFF) so app can reconfig */
  RCC->CR |= RCC_CR_HSION;
  while ((RCC->CR & RCC_CR_HSIRDY) == 0U) { }
  RCC->CFGR = 0x00000000U;
  RCC->CR &= ~(RCC_CR_PLLON);
  while (RCC->CR & RCC_CR_PLLRDY) { }
  /* Optional: disable HSE if you prefer app to enable it explicitly
     RCC->CR &= ~(RCC_CR_HSEON);
  */

  /* 6) Set vector table to application and set MSP */
  uint32_t app_stack = *(__IO uint32_t*)app_addr;
  uint32_t app_reset = *(__IO uint32_t*)(app_addr + 4U);

  SCB->VTOR = app_addr;   /* relocate vector table to app */
  __DSB(); __ISB();

  __set_MSP(app_stack);

  /* 7) Jump to Reset_Handler of the app */
  pFunction appEntry = (pFunction)app_reset;
  appEntry();

  /* Should never return */
  while (1) { }
}

/* ------------------------------------------------------------------------- */
void SystemClock_Config(void)
{
  /* NUCLEO-F412 typically uses HSE BYPASS from ST-LINK (8 MHz MCO) */
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
  RCC_OscInitStruct.PLL.PLLQ       = 8;              /* USB FS = 48 MHz (ok) */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|
                                     RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  /* <= 50 MHz (here 48 MHz) */
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  /* 96 MHz */
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) Error_Handler();
}

/* ------------------------------------------------------------------------- */
void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { (void)file; (void)line; }
#endif
