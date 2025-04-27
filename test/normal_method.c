/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "stm32l475e_iot01.h"          // BSP Core
#include "stm32l475e_iot01_accelero.h" // BSP Accelerometer

#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "arm_math.h"

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
    STATE_IDLE,
    STATE_SAMPLING,
    STATE_PROCESSING
} ApplicationState_t;

/* Private define ------------------------------------------------------------*/
#define LSM6DSL_ACCEL_SENSITIVITY_2G 0.061f

#define FFT_SIZE 128
#define BUFFER_DURATION_S 3.0f
#define SAMPLING_FREQUENCY_HZ (FFT_SIZE / BUFFER_DURATION_S) // ~3 seconds buffer (128 samples / 43Hz = ~2.97s)
#define SENSOR_BUFFER_SIZE FFT_SIZE
#define SAMPLE_INTERVAL_MS (1000 / SAMPLING_FREQUENCY_HZ)

#define FFT_INPUT_BUFFER_SIZE FFT_SIZE
#define FFT_OUTPUT_BUFFER_SIZE FFT_SIZE

#define TREMOR_LOW_HZ 3.0f
#define TREMOR_HIGH_HZ 5.0f
#define DYSKINESIA_LOW_HZ 5.0f
#define DYSKINESIA_HIGH_HZ 7.0f
#define FREQUENCY_RESOLUTION ((float)SAMPLING_FREQUENCY_HZ / (float)FFT_SIZE)

#define LED1_PORT GPIOA
#define LED1_PIN_N 5
#define LED1_PIN (1UL << LED1_PIN_N)

#define MAGNITUDE_THRESHOLD_TREMOR 100.0f
#define MAGNITUDE_THRESHOLD_DYSKINESIA 100.0f
#define INTENSITY_SCALE_FACTOR 1000.0f

/* Private variables ---------------------------------------------------------*/
volatile ApplicationState_t app_state = STATE_IDLE;
volatile uint32_t sample_count = 0;
volatile uint32_t processing_needed = 0;

// --- Buffers ---
float32_t sensor_raw_buffer[SENSOR_BUFFER_SIZE];     // Buffer for sensor data (e.g., Z-axis accel in mg)
float32_t fft_input_buffer[FFT_INPUT_BUFFER_SIZE];   // FFT Input buffer
float32_t fft_output_buffer[FFT_OUTPUT_BUFFER_SIZE]; // Will hold magnitude spectrum
float32_t hann_window_buffer[FFT_SIZE];              // Buffer for the Hanning window coefficients

// --- Peripheral Handles & Instances ---
arm_rfft_fast_instance_f32 fft_instance;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void GPIO_Init_Manual(void);
static void TIM6_Init(void);
static void ProcessSensorData(void);
static void IndicateResults(float32_t tremor_magnitude, float32_t dyskinesia_magnitude);
void Error_Handler(const char *message);

/* Main Function -------------------------------------------------------------*/
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    BSP_LED_Init(LED2); // Initialize BSP LED
    GPIO_Init_Manual(); // For LED1 (PA5) if still used
    TIM6_Init();

    if (BSP_ACCELERO_Init() != ACCELERO_OK) // Initialize Accelerometer using BSP
    {
        Error_Handler("Accelerometer Init Failed");
    }

    if (arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE) != ARM_MATH_SUCCESS)
    {
        Error_Handler("FFT Init Failed");
    }

    NVIC_SetPriority(TIM6_DAC_IRQn, 6); // 设置中断优先级
    NVIC_EnableIRQ(TIM6_DAC_IRQn);      // 使能 NVIC 中断通道
    TIM6->CR1 |= TIM_CR1_CEN;           // 使能 Timer 6 计数器 <--- 检查是否被遗漏或注释掉
    TIM6->DIER |= TIM_DIER_UIE;         // 使能更新中断      <--- 检查是否被遗漏或注释掉

    app_state = STATE_SAMPLING;

    while (1)
    {
        if (processing_needed == 1)
        {
            __disable_irq();

            processing_needed = 0;        /* clear flag      */
            app_state = STATE_PROCESSING; /* <- NEW LINE     */

            memcpy(fft_input_buffer, sensor_raw_buffer,
                   SENSOR_BUFFER_SIZE * sizeof(float32_t));
            sample_count = 0;

            __enable_irq(); /* ISR runs again, but stays
                               in the "else" branch        */

            ProcessSensorData();

            app_state = STATE_SAMPLING; /* ready for next batch */
        }
        else
        {
            // memory ordering
            __DSB();
            __WFI();
            __ISB();
        }
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
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler("Osc Config Failed");
    }

    // Initializes the CPU, AHB and APB buses clocks
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // Select PLL as system clock source
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;        // HCLK = SYSCLK = 80MHz
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;         // PCLK1 = HCLK = 80MHz
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;         // PCLK2 = HCLK = 80MHz

    // Configure Flash Latency based on HCLK speed (80MHz requires 4 Wait States)
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler("Clock Config Failed");
    }

    // Configure I2C2 clock source
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
    PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1; // Select PCLK1 (80MHz) as I2C2 clock source
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler("Periph Clock Config Failed");
    }

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

/* TIM6_Init -----------------------------------------------------------------*/
static void TIM6_Init(void)
{
    __HAL_RCC_TIM6_CLK_ENABLE();

    TIM6->CR1 &= ~TIM_CR1_CEN;

    uint32_t timer_clock = SystemCoreClock;
    // Adjust clock source based on APB1 prescaler (from SystemClock_Config)
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
    {
        timer_clock *= 2; // Basic timers (TIM6/7) clock is x2 PCLK1 if PPRE1 is not /1
    }

    uint32_t prescaler = 7999; // Gives 10kHz counter clock @ 80MHz (or 40MHz if PPRE1 > 1)
    uint32_t period = (uint32_t)(((timer_clock / (prescaler + 1)) / SAMPLING_FREQUENCY_HZ) - 1);

    TIM6->PSC = prescaler;
    TIM6->ARR = period;

    TIM6->EGR |= TIM_EGR_UG;
    TIM6->SR &= ~TIM_SR_UIF;
}

/* ProcessSensorData ---------------------------------------------------------*/
static void ProcessSensorData(void)
{
    // --- 1. Apply Hanning Window using CMSIS-DSP ---
    arm_hanning_f32(hann_window_buffer, FFT_INPUT_BUFFER_SIZE);
    arm_mult_f32(fft_input_buffer, hann_window_buffer, fft_input_buffer, FFT_INPUT_BUFFER_SIZE);

    // --- 2. Perform Real Fast Fourier Transform (RFFT) ---
    arm_rfft_fast_f32(&fft_instance, fft_input_buffer, fft_output_buffer, 0); // 0 = forward FFT

    // --- 3. Calculate Magnitude Spectrum ---
    arm_cmplx_mag_f32(fft_output_buffer, fft_output_buffer, FFT_SIZE / 2);

    // --- 4. Analyze Frequency Spectrum ---
    float32_t max_tremor_mag = 0.0f;
    float32_t max_dyskinesia_mag = 0.0f;

    // Iterate through magnitude results (index k corresponds to frequency k * FREQUENCY_RESOLUTION)
    // Skip DC component (index 0).
    for (uint16_t k = 1; k < FFT_SIZE / 2; k++)
    {
        float32_t current_freq = (float32_t)k * FREQUENCY_RESOLUTION;

        // Check Tremor Range (3-5 Hz)
        if (current_freq >= TREMOR_LOW_HZ && current_freq <= TREMOR_HIGH_HZ)
        {
            if (fft_output_buffer[k] > max_tremor_mag)
            {
                max_tremor_mag = fft_output_buffer[k];
            }
        }
        // Check Dyskinesia Range (5-7 Hz)
        else if (current_freq >= DYSKINESIA_LOW_HZ && current_freq <= DYSKINESIA_HIGH_HZ)
        {
            if (fft_output_buffer[k] > max_dyskinesia_mag)
            {
                max_dyskinesia_mag = fft_output_buffer[k];
            }
        }
    }

    // --- 5. Indicate Results ---
    IndicateResults(max_tremor_mag, max_dyskinesia_mag);
}

/* IndicateResults -----------------------------------------------------------*/
static void IndicateResults(float32_t tremor_magnitude, float32_t dyskinesia_magnitude)
{
    bool tremor_detected = (tremor_magnitude > MAGNITUDE_THRESHOLD_TREMOR);
    bool dyskinesia_detected = (dyskinesia_magnitude > MAGNITUDE_THRESHOLD_DYSKINESIA);

    if (dyskinesia_detected)
    {
        HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_RESET);
        BSP_LED_On(LED2);
    }
    else if (tremor_detected)
    {
        HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_SET);
        BSP_LED_Off(LED2);
    }
    else
    {
        HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_RESET);
        BSP_LED_Off(LED2);
    }
}

/* Interrupt Service Routines ----------------------------------------------*/
void TIM6_DAC_IRQHandler(void)
{
    if ((TIM6->SR & TIM_SR_UIF) && (TIM6->DIER & TIM_DIER_UIE))
    {
        TIM6->SR &= ~TIM_SR_UIF;

        if (app_state == STATE_SAMPLING && sample_count < SENSOR_BUFFER_SIZE)
        {

            int16_t accel_data[3];
            BSP_ACCELERO_AccGetXYZ(accel_data);
            sensor_raw_buffer[sample_count] = (float32_t)accel_data[2] * LSM6DSL_ACCEL_SENSITIVITY_2G;
            sample_count++;
            if (sample_count >= SENSOR_BUFFER_SIZE)
            {
                processing_needed = 1;
            }
        }
    }
}

/* Error Handler -------------------------------------------------------------*/
void Error_Handler(const char *message)
{
    (void)message;
    __disable_irq();

    // Basic GPIO Init as fallback
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef gpio_init_structure = {0};
    gpio_init_structure.Pin = LED1_PIN;
    gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_structure.Pull = GPIO_NOPULL;
    gpio_init_structure.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED1_PORT, &gpio_init_structure);
    gpio_init_structure.Pin = LED2_PIN;
    HAL_GPIO_Init(LED2_GPIO_PORT, &gpio_init_structure);

    // 2 leds blinking when error
    while (1)
    {
        HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_PIN, GPIO_PIN_RESET);
        HAL_Delay(200);
        HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_PIN, GPIO_PIN_SET);
        HAL_Delay(200);
    }
}

#ifdef USE_FULL_ASSERT
/* assert_failed -------------------------------------------------------------*/
void assert_failed(uint8_t *file, uint32_t line)
{
    Error_Handler("Assert Failed");
}
#endif /* USE_FULL_ASSERT */

// Proxy SysTick_Handler
void SysTick_Handler(void)
{
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
}