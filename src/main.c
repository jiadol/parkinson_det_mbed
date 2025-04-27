/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "stm32l475e_iot01.h"
#include "stm32l475e_iot01_accelero.h"

#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "arm_math.h"

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
    STATE_SAMPLING,
    STATE_PROCESSING
} ApplicationState_t;

/* Private define ------------------------------------------------------------*/
// Sensor Sensitivity
#define LSM6DSL_ACCEL_SENSITIVITY_2G 0.061f

// FFT and Buffer Configuration
#define FFT_SIZE 64                                                 // tested: 64 / 128
#define BUFFER_DURATION_S 3.0f                                      // Target buffer duration (used for initial freq calc)
#define TARGET_SAMPLING_FREQUENCY_HZ (FFT_SIZE / BUFFER_DURATION_S) // Target sampling frequency (~42.67 Hz)
#define SENSOR_BUFFER_SIZE FFT_SIZE                                 // Circular buffer size matches FFT size

// --- Streaming Configuration ---
#define FFT_TRIGGER_OVERLAP_PERCENT 20                                                 // Percentage overlap (e.g., 50 means FFT runs when FFT_SIZE/2 new samples arrive)
#define FFT_TRIGGER_NEW_SAMPLES (FFT_SIZE * (100 - FFT_TRIGGER_OVERLAP_PERCENT) / 100) // Samples needed to trigger FFT (e.g., 128 * 50 / 100 = 64)
#if FFT_TRIGGER_NEW_SAMPLES < 1
#error "FFT_TRIGGER_NEW_SAMPLES must be at least 1"
#endif
// --- End Streaming Configuration ---

#define FFT_INPUT_BUFFER_SIZE FFT_SIZE
#define FFT_OUTPUT_BUFFER_SIZE FFT_SIZE

// Frequency Band Definitions
#define TREMOR_LOW_HZ 3.0f
#define TREMOR_HIGH_HZ 5.0f
#define DYSKINESIA_LOW_HZ 5.0f
#define DYSKINESIA_HIGH_HZ 7.0f
// Frequency resolution depends on the *actual* calculated sampling frequency
#define FREQUENCY_RESOLUTION (ACTUAL_SAMPLING_FREQUENCY_HZ / (float)FFT_SIZE)

// LED Definitions
#define LED1_PORT GPIOA
#define LED1_PIN_N 5
#define LED1_PIN (1UL << LED1_PIN_N)
// LED2 uses BSP definition LED2

// Detection Thresholds (NEEDS TUNING!)
#define MAGNITUDE_THRESHOLD_TREMOR 100.0f
#define MAGNITUDE_THRESHOLD_DYSKINESIA 100.0f
#define INTENSITY_SCALE_FACTOR 1000.0f

/* Private variables ---------------------------------------------------------*/
volatile ApplicationState_t app_state = STATE_SAMPLING; // Start sampling
volatile uint16_t stream_write_idx = 0;                 // Circular buffer write index
volatile uint16_t new_sample_count = 0;                 // Counter for new samples since last FFT
volatile bool processing_needed = false;                // Flag to trigger processing

// --- Buffers ---
float32_t sensor_raw_buffer[SENSOR_BUFFER_SIZE];     // Circular buffer
float32_t fft_input_buffer[FFT_INPUT_BUFFER_SIZE];   // Linear buffer for FFT input
float32_t fft_output_buffer[FFT_OUTPUT_BUFFER_SIZE]; // FFT magnitude output
float32_t hann_window_buffer[FFT_SIZE];              // Hanning window coefficients

// --- Peripheral Handles & Instances ---
arm_rfft_fast_instance_f32 fft_instance;
TIM_HandleTypeDef htim6; // Use HAL Timer handle

// --- Actual Sampling Frequency ---
float32_t ACTUAL_SAMPLING_FREQUENCY_HZ = 0.0f; // Calculated in TIM6_Init

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void GPIO_Init_Manual(void);
static void TIM6_Init(void);
static void PrepareFFTInputFromCircularBuffer(void); // New function
static void ProcessSensorData(void);
static void IndicateResults(float32_t tremor_magnitude, float32_t dyskinesia_magnitude);
void Error_Handler(const char *message);

//=============================================================================
// Main Function
//=============================================================================
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    BSP_LED_Init(LED2);
    GPIO_Init_Manual();
    TIM6_Init(); // Calculates ACTUAL_SAMPLING_FREQUENCY_HZ

    if (BSP_ACCELERO_Init() != ACCELERO_OK)
    {
        Error_Handler("Accelerometer Init Failed");
    }
    // Optional: BSP_ACCELERO_SetODR(...) if needed

    if (arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE) != ARM_MATH_SUCCESS)
    {
        Error_Handler("FFT Init Failed");
    }

    arm_hanning_f32(hann_window_buffer, FFT_SIZE); // Pre-calculate window

    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 0); // Set suitable interrupt priority
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
    {
        Error_Handler("Timer Start Failed");
    } // Start Timer & Interrupt

    app_state = STATE_SAMPLING;

    while (1)
    {
        if (processing_needed)
        {
            app_state = STATE_PROCESSING;

            // Prepare the linear FFT input buffer from the circular buffer
            PrepareFFTInputFromCircularBuffer();

            // Process the prepared data (window, FFT, analyze, indicate)
            ProcessSensorData();

            // Safely clear the flag
            __disable_irq();
            processing_needed = false;
            __enable_irq();

            app_state = STATE_SAMPLING; // Ready for next trigger
        }
        else
        {
            // Wait for interrupt in low-power mode
            __DSB();
            __WFI();
            __ISB();
        }
    }
}

//=============================================================================
// System Clock Configuration
//=============================================================================
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6; // 4 MHz
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 40; // 4 * 40 / 2 = 80 MHz
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler("Osc Config Failed");
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler("Clock Config Failed");
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
    PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler("Periph Clock Config Failed");
    }

    HAL_RCCEx_EnableMSIPLLMode();
    SystemCoreClockUpdate();
}

//=============================================================================
// Manual GPIO Initialization (for LED1)
//=============================================================================
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

//=============================================================================
// Timer 6 Initialization (Calculates Actual Frequency)
//=============================================================================
static void TIM6_Init(void)
{
    __HAL_RCC_TIM6_CLK_ENABLE();

    htim6.Instance = TIM6;

    uint32_t timer_clock = HAL_RCC_GetPCLK1Freq(); // Get APB1 Timer Clock
    // Adjust if APB1 prescaler is not /1 (Basic timers get x2 PCLK1)
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
    {
        timer_clock *= 2;
    }

    uint32_t prescaler = 7999; // Example: 10kHz counter clock @ 80MHz PCLK1
    uint32_t counter_clock = timer_clock / (prescaler + 1);
    uint32_t period = (uint32_t)(((float32_t)counter_clock / TARGET_SAMPLING_FREQUENCY_HZ) + 0.5f) - 1;

    // Calculate and store the actual sampling frequency
    ACTUAL_SAMPLING_FREQUENCY_HZ = (float32_t)counter_clock / (period + 1);

    htim6.Init.Prescaler = prescaler;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = period;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
    {
        Error_Handler("Timer Init Failed");
    }
}

//=============================================================================
// Prepare FFT Input Buffer from Circular Buffer
//=============================================================================
static void PrepareFFTInputFromCircularBuffer(void)
{
    // Safely read the current write index from the ISR
    __disable_irq();
    uint16_t current_write_idx = stream_write_idx;
    __enable_irq();

    // Calculate start index in circular buffer (FFT_SIZE samples before current write index)
    int16_t start_idx = (int16_t)current_write_idx - (int16_t)FFT_SIZE;
    if (start_idx < 0)
    {
        start_idx += SENSOR_BUFFER_SIZE;
    } // Handle wrap-around

    // Copy data from circular buffer to linear FFT input buffer
    uint16_t read_idx = (uint16_t)start_idx;
    for (uint16_t i = 0; i < FFT_SIZE; i++)
    {
        // Read from circular buffer (potential optimization: use memcpy if no wrap)
        fft_input_buffer[i] = sensor_raw_buffer[read_idx];
        read_idx = (read_idx + 1) % SENSOR_BUFFER_SIZE; // Increment read index with wrap-around
    }
}

//=============================================================================
// Process Sensor Data
//=============================================================================
static void ProcessSensorData(void)
{
    // --- 1. Apply Hanning Window ---
    arm_mult_f32(fft_input_buffer, hann_window_buffer, fft_input_buffer, FFT_INPUT_BUFFER_SIZE);

    // --- 2. Perform RFFT ---
    arm_rfft_fast_f32(&fft_instance, fft_input_buffer, fft_output_buffer, 0);

    // --- 3. Calculate Magnitude ---
    arm_cmplx_mag_f32(fft_output_buffer, fft_output_buffer, FFT_SIZE / 2);

    // --- 4. Analyze Frequencies ---
    float32_t max_tremor_mag = 0.0f;
    float32_t max_dyskinesia_mag = 0.0f;

    for (uint16_t k = 1; k < FFT_SIZE / 2; k++) // Skip DC
    {
        float32_t current_freq = (float32_t)k * FREQUENCY_RESOLUTION;
        if (current_freq >= TREMOR_LOW_HZ && current_freq <= TREMOR_HIGH_HZ)
        {
            if (fft_output_buffer[k] > max_tremor_mag)
            {
                max_tremor_mag = fft_output_buffer[k];
            }
        }
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

//=============================================================================
// Indicate Results (LED Control)
//=============================================================================
static void IndicateResults(float32_t tremor_magnitude, float32_t dyskinesia_magnitude)
{
    bool tremor_detected = (tremor_magnitude > MAGNITUDE_THRESHOLD_TREMOR);
    bool dyskinesia_detected = (dyskinesia_magnitude > MAGNITUDE_THRESHOLD_DYSKINESIA);

    if (dyskinesia_detected)
    {
        HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_RESET); // LED1 Off
        BSP_LED_On(LED2);                                       // LED2 On
        // TODO: Implement Dyskinesia intensity indication
    }
    else if (tremor_detected)
    {
        HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_SET); // LED1 On
        BSP_LED_Off(LED2);                                    // LED2 Off
        // TODO: Implement Tremor intensity indication
    }
    else
    {
        HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_RESET); // LED1 Off
        BSP_LED_Off(LED2);                                      // LED2 Off
    }
}

//=============================================================================
// Interrupt Service Routines / Callbacks
//=============================================================================
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        int16_t accel_data[3];
        // Read sensor data directly into the circular buffer location
        BSP_ACCELERO_AccGetXYZ(accel_data);

        // Store Z-axis data (converted) in circular buffer
        sensor_raw_buffer[stream_write_idx] = (float32_t)accel_data[2] * LSM6DSL_ACCEL_SENSITIVITY_2G;

        // Increment write index with wrap-around
        stream_write_idx = (stream_write_idx + 1) % SENSOR_BUFFER_SIZE;

        // Increment new sample counter
        new_sample_count++;

        // Check if enough new samples have arrived to trigger FFT
        if (new_sample_count >= FFT_TRIGGER_NEW_SAMPLES)
        {
            // Only trigger if not already processing to avoid backlog issues
            if (!processing_needed)
            {
                processing_needed = true; // Signal main loop
                new_sample_count = 0;     // Reset counter for next trigger
            }
            else
            {
                // Processing is still ongoing, reset counter to avoid immediate re-trigger
                new_sample_count = 0;
            }
        }
    }
}

// --- SysTick Handler Proxy (Needed for HAL) ---
void SysTick_Handler(void)
{
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
}

// --- TIM6 IRQ Handler (Calls HAL Callback) ---
void TIM6_DAC_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim6); // Let HAL handle the interrupt and call the callback
}

//=============================================================================
// Error Handler
//=============================================================================
void Error_Handler(const char *message)
{
    (void)message; // Suppress unused parameter warning
    __disable_irq();
    GPIO_Init_Manual(); // Ensure LEDs are usable
    BSP_LED_Init(LED2);
    while (1)
    {
        HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_SET);
        BSP_LED_Off(LED2);
        HAL_Delay(200);
        HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_RESET);
        BSP_LED_On(LED2);
        HAL_Delay(200);
    }
}

//=============================================================================
// Assert Failed Handler
//=============================================================================
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    Error_Handler("Assert Failed");
}
#endif /* USE_FULL_ASSERT */
