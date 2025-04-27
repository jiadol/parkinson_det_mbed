/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "stm32l475e_iot01.h"          // BSP Core
#include "stm32l475e_iot01_accelero.h" // BSP Accelerometer

#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "arm_math.h"

#define LED1_PORT GPIOA
#define LED1_PIN_N 5
#define LED1_PIN (1UL << LED1_PIN_N)

static void SystemClock_Config(void);
static void GPIO_Init_Manual(void);

int main(void)
{
    HAL_Init();                 // 保证 SysTick 正常
    SystemClock_Config();       // 时钟
    GPIO_Init_Manual();             // LED
    BSP_LED_Init(LED2);   // Initialize BSP LED
    
    __enable_irq();             // 别忘开中断

    while (1)
    {
        BSP_LED_Toggle(LED2);
        HAL_Delay(500);         // 如果这里阻塞，必是 tick 没跑
    }
}

/* SystemClock_Config --------------------------------------------------------*/
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0}; // Needed for I2C clock source selection

    // Enable Power Control clock
    __HAL_RCC_PWR_CLK_ENABLE();
    // Set voltage scaling range 1
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1); // Use HAL function

    // Enable LSE Oscillator (assuming it's needed, e.g., for RTC)
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6; // 4 MHz
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 40;            // 4 * 40 / 2 = 80 MHz
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7; // Not used directly for SysClk in this config
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2; // Not used directly for SysClk
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2; // PLL R Clock output = 80MHz
    // Initializes the CPU, AHB and APB buses clocks
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // Select PLL as system clock source
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;        // HCLK = SYSCLK = 80MHz
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;         // PCLK1 = HCLK = 80MHz
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;         // PCLK2 = HCLK = 80MHz

    // Configure I2C2 clock source
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
    PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1; // Select PCLK1 (80MHz) as I2C2 clock source

    // Optional: Configure MSI Auto calibration
    HAL_RCCEx_EnableMSIPLLMode();

    // Update the SystemCoreClock variable
    SystemCoreClockUpdate();
}

/* GPIO_Init_Manual ----------------------------------------------------------*/
static void GPIO_Init_Manual(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio_init_structure = {0};
    gpio_init_structure.Pin = LED1_PIN;
    gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_structure.Pull = GPIO_NOPULL;
    gpio_init_structure.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED1_PORT, &gpio_init_structure);
    HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_RESET);
}

void SysTick_Handler(void)
{
    HAL_IncTick();              // 递增 uwTick
    HAL_SYSTICK_IRQHandler();   // （可选）给 HAL 留的钩子
}
