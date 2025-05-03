#include "mbed.h"
#include "arm_math.h"

#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <limits>
#include <cmath>

// ... (Defines and other globals remain the same) ...
typedef enum
{
    STATE_IDLE,
    STATE_PROCESSING
} ApplicationState_t;

#define LSM6DSL_I2C_ADDR_DEFAULT (0x6A << 1)
#define LSM6DSL_WHO_AM_I_REG     0x0F
#define LSM6DSL_WHO_AM_I_VAL     0x6A
#define LSM6DSL_CTRL1_XL_REG     0x10
#define LSM6DSL_OUTX_L_XL_REG    0x28
#define LSM6DSL_ODR_52HZ         0x60
#define LSM6DSL_ACCEL_SENSITIVITY_2G 0.061f

#define FFT_SIZE 64
#define SENSOR_BUFFER_SIZE FFT_SIZE
#define FFT_TRIGGER_OVERLAP_PERCENT 20
#define FFT_TRIGGER_NEW_SAMPLES ((FFT_SIZE * (100 - FFT_TRIGGER_OVERLAP_PERCENT)) / 100)
#if FFT_TRIGGER_NEW_SAMPLES < 1
#error "FFT_TRIGGER_NEW_SAMPLES must be at least 1"
#endif

#define FFT_INPUT_BUFFER_SIZE FFT_SIZE
#define FFT_OUTPUT_BUFFER_SIZE FFT_SIZE

#define TREMOR_LOW_HZ 3.0f
#define TREMOR_HIGH_HZ 5.0f
#define DYSKINESIA_LOW_HZ 5.0f
#define DYSKINESIA_HIGH_HZ 7.0f

#define LED_TREMOR_PIN   LED2
#define LED_DYSKINESIA_PIN LED1

#define MAGNITUDE_THRESHOLD_TREMOR 150.0f
#define MAGNITUDE_THRESHOLD_DYSKINESIA 150.0f

#define MIN_CONSECUTIVE_DETECTIONS 3
#define DYSKINESIA_COOLDOWN_CYCLES 2

#define MIN_BLINK_FREQ_HZ 1.0f
#define MAX_BLINK_FREQ_HZ 10.0f

volatile ApplicationState_t app_state = STATE_IDLE;
volatile uint16_t stream_write_idx = 0;
volatile uint16_t new_sample_count = 0;

uint16_t consecutive_tremor_detections = 0;
uint16_t consecutive_dyskinesia_detections = 0;
uint8_t dyskinesia_cooldown_counter = 0;
bool was_dyskinesia_potentially_active = false;

DigitalOut led_tremor_pin(LED_TREMOR_PIN, 0);
DigitalOut led_dyskinesia_pin(LED_DYSKINESIA_PIN, 0);
Ticker sampler_ticker;
I2C i2c(PB_11, PB_10);
// EventQueue still needed for ISR->Thread communication
EventQueue queue(64 * EVENTS_EVENT_SIZE);

// Use Timeout objects for managing blinking
Timeout tremor_led_flipper;
Timeout dyskinesia_led_flipper;
// Store intervals needed by callbacks
us_timestamp_t current_tremor_toggle_interval_us = 0;
us_timestamp_t current_dyskinesia_toggle_interval_us = 0;
// Flags to indicate if a blinker is *intended* to be running (controlled by IndicateResults)
// These flags prevent the callback from rescheduling if IndicateResults decided to stop the blinker.
volatile bool tremor_blinker_should_be_active = false;
volatile bool dyskinesia_blinker_should_be_active = false;


float32_t sensor_raw_buffer[SENSOR_BUFFER_SIZE];
float32_t fft_input_buffer[FFT_INPUT_BUFFER_SIZE];
float32_t fft_output_buffer[FFT_OUTPUT_BUFFER_SIZE];
float32_t hann_window_buffer[FFT_SIZE];

arm_rfft_fast_instance_f32 fft_instance;

float32_t ACTUAL_SAMPLING_FREQUENCY_HZ = 52.0f;
float32_t FREQUENCY_RESOLUTION = 0.0f;

// --- Function Prototypes ---
// ... (Keep previous prototypes) ...
static void InitializePeripherals(void);
static void PrepareFFTInputFromCircularBuffer(void);
static void ProcessSensorData(void);
static void IndicateResults(bool tremor_currently_detected, bool dyskinesia_currently_detected, float32_t detected_tremor_freq, float32_t detected_dyskinesia_freq);
static void SamplerISRHandler(void);
static void ReadSensorAndMaybeQueueProcessing(void);
static void TriggerProcessing(void);
void toggle_tremor_led();
void toggle_dyskinesia_led();
static us_timestamp_t calculate_toggle_interval_us(float frequency_hz);


// --- Main ---
int main()
{
    printf("--- Mbed Tremor/Dyskinesia Detection Start (Timeout Blinking - Rev2) ---\n");
    InitializePeripherals();
    printf("Starting event queue dispatch...\n");
    // Event queue is still needed for ISR->Thread calls
    queue.dispatch_forever();
    return 0;
}

// --- Initialization ---
static void InitializePeripherals(void)
{
    // ... (I2C/Accel/FFT/Hanning - unchanged) ...
    printf("Initializing I2C and Accelerometer...\n");
    i2c.frequency(400000);
    char cmd[1];
    char data[1];
    cmd[0] = LSM6DSL_WHO_AM_I_REG;
    int status = i2c.write(LSM6DSL_I2C_ADDR_DEFAULT, cmd, 1, true);
    if (status != 0) MBED_ERROR(MBED_ERROR_WRITE_FAILED, "I2C WhoAmI Write Failed");
    status = i2c.read(LSM6DSL_I2C_ADDR_DEFAULT, data, 1, false);
    if (status != 0) MBED_ERROR(MBED_ERROR_READ_FAILED, "I2C WhoAmI Read Failed");
    if (data[0] != LSM6DSL_WHO_AM_I_VAL) {
        printf("LSM6DSL WhoAmI Check Failed. Expected 0x%X, Got 0x%X\n", LSM6DSL_WHO_AM_I_VAL, data[0]);
        MBED_ERROR(MBED_ERROR_INITIALIZATION_FAILED, "Accelerometer WhoAmI Mismatch");
    }
    printf("Accelerometer WhoAmI Check OK (0x%X)\n", data[0]);
    char config[2];
    config[0] = LSM6DSL_CTRL1_XL_REG;
    config[1] = LSM6DSL_ODR_52HZ;
    status = i2c.write(LSM6DSL_I2C_ADDR_DEFAULT, config, 2, false);
    if (status != 0) MBED_ERROR(MBED_ERROR_WRITE_FAILED, "I2C Accel Config Failed");
    printf("Accelerometer configured (ODR=52Hz, FS=2g)\n");
    if (arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE) != ARM_MATH_SUCCESS) {
        MBED_ERROR(MBED_ERROR_INITIALIZATION_FAILED, "FFT Init Failed");
    }
    printf("FFT Initialized for size %d\n", FFT_SIZE);
    arm_hanning_f32(hann_window_buffer, FFT_SIZE);
    printf("Hanning window calculated\n");

    FREQUENCY_RESOLUTION = ACTUAL_SAMPLING_FREQUENCY_HZ / (float)FFT_SIZE;
    // Use chrono duration for attach - Correction Applied Here
    chrono::microseconds interval_chrono = chrono::microseconds((long long)((1.0f / ACTUAL_SAMPLING_FREQUENCY_HZ) * 1e6f));

    printf("Using Sensor ODR: %.2f Hz, Interval: %lld us\n", ACTUAL_SAMPLING_FREQUENCY_HZ, interval_chrono.count());
    printf("Freq Res: %.3f Hz\n", FREQUENCY_RESOLUTION);

    // Fix Deprecation: Use attach with chrono duration
    sampler_ticker.attach(callback(&SamplerISRHandler), interval_chrono);
    printf("Sampler ticker attached\n");

    app_state = STATE_IDLE;
}

// --- Core Logic ---
static void SamplerISRHandler(void) {
    // Queue the read operation - this is safe from ISR
    queue.call(&ReadSensorAndMaybeQueueProcessing);
}

static void ReadSensorAndMaybeQueueProcessing(void) {
    // ... (I2C read and buffer filling - unchanged) ...
    char raw_i2c_data[6];
    char start_reg = LSM6DSL_OUTX_L_XL_REG;
    int write_stat = -1;
    int read_stat = -1;
    write_stat = i2c.write(LSM6DSL_I2C_ADDR_DEFAULT, &start_reg, 1, true);
    if (write_stat == 0) {
        read_stat = i2c.read(LSM6DSL_I2C_ADDR_DEFAULT, raw_i2c_data, 6, false);
    }
    if (write_stat == 0 && read_stat == 0) {
        CriticalSectionLock lock;
        int16_t raw_z = (int16_t)((raw_i2c_data[5] << 8) | raw_i2c_data[4]);
        sensor_raw_buffer[stream_write_idx] = (float32_t)raw_z * LSM6DSL_ACCEL_SENSITIVITY_2G;
        stream_write_idx = (stream_write_idx + 1) % SENSOR_BUFFER_SIZE;
        new_sample_count++;
        if (app_state != STATE_PROCESSING && new_sample_count >= FFT_TRIGGER_NEW_SAMPLES) {
             queue.call(&TriggerProcessing);
             new_sample_count = 0;
        }
    } else {
         printf("ERR: I2C R/W failed in ReadSensor func (%d, %d)\n", write_stat, read_stat);
    }
}

static void TriggerProcessing(void) {
    if (app_state == STATE_IDLE) {
        ProcessSensorData();
    }
}

static void ProcessSensorData(void)
{
    // ... (FFT Processing and Frequency/Magnitude Detection - unchanged) ...
    app_state = STATE_PROCESSING;
    PrepareFFTInputFromCircularBuffer();
    arm_mult_f32(fft_input_buffer, hann_window_buffer, fft_input_buffer, FFT_INPUT_BUFFER_SIZE);
    arm_rfft_fast_f32(&fft_instance, fft_input_buffer, fft_output_buffer, 0);
    arm_cmplx_mag_f32(fft_output_buffer, fft_output_buffer, FFT_SIZE / 2);
    float32_t max_tremor_mag = 0.0f;
    float32_t max_dyskinesia_mag = 0.0f;
    float32_t detected_tremor_freq = 0.0f;
    float32_t detected_dyskinesia_freq = 0.0f;
    for (uint16_t k = 1; k < FFT_SIZE / 2; k++) {
        float32_t current_freq = (float32_t)k * FREQUENCY_RESOLUTION;
        float32_t current_mag = fft_output_buffer[k];
        if (current_freq >= TREMOR_LOW_HZ && current_freq <= TREMOR_HIGH_HZ) {
            if (current_mag > max_tremor_mag) {
                max_tremor_mag = current_mag;
                detected_tremor_freq = current_freq;
            }
        }
        if (current_freq >= DYSKINESIA_LOW_HZ && current_freq <= DYSKINESIA_HIGH_HZ) {
             if (current_mag > max_dyskinesia_mag) {
                max_dyskinesia_mag = current_mag;
                detected_dyskinesia_freq = current_freq;
            }
        }
    }
    bool tremor_currently_detected = (max_tremor_mag > MAGNITUDE_THRESHOLD_TREMOR);
    bool dyskinesia_currently_detected = (max_dyskinesia_mag > MAGNITUDE_THRESHOLD_DYSKINESIA);

    IndicateResults(tremor_currently_detected, dyskinesia_currently_detected, detected_tremor_freq, detected_dyskinesia_freq);
    app_state = STATE_IDLE;
}

static void PrepareFFTInputFromCircularBuffer(void)
{
    // ... (Unchanged - uses lock) ...
     CriticalSectionLock lock;
    uint16_t current_write_idx_copy = stream_write_idx;
    int16_t start_idx = (int16_t)current_write_idx_copy - (int16_t)FFT_SIZE;
    if (start_idx < 0) { start_idx += SENSOR_BUFFER_SIZE; }
    uint16_t read_idx = (uint16_t)start_idx;
    for (uint16_t i = 0; i < FFT_SIZE; i++) {
        fft_input_buffer[i] = sensor_raw_buffer[read_idx];
        read_idx = (read_idx + 1) % SENSOR_BUFFER_SIZE;
    }
}

// --- Indication Logic (Using Timeout for Scheduling) ---
static us_timestamp_t calculate_toggle_interval_us(float frequency_hz) {
    if (frequency_hz < MIN_BLINK_FREQ_HZ) frequency_hz = MIN_BLINK_FREQ_HZ;
    if (frequency_hz > MAX_BLINK_FREQ_HZ) frequency_hz = MAX_BLINK_FREQ_HZ;
    if (frequency_hz < 0.1f) return 0;
    return (us_timestamp_t)((1.0f / (2.0f * frequency_hz)) * 1000000.0f);
}

static void IndicateResults(bool tremor_currently_detected, bool dyskinesia_currently_detected, float32_t detected_tremor_freq, float32_t detected_dyskinesia_freq)
{
    // 1. Update Raw Consecutive Counts
    if (tremor_currently_detected) {
        if (consecutive_tremor_detections < MIN_CONSECUTIVE_DETECTIONS) consecutive_tremor_detections++;
    } else {
        consecutive_tremor_detections = 0;
    }
    if (dyskinesia_currently_detected) {
         if (consecutive_dyskinesia_detections < MIN_CONSECUTIVE_DETECTIONS) consecutive_dyskinesia_detections++;
    } else {
        consecutive_dyskinesia_detections = 0;
    }

    // 2. Determine Potential Active States
    bool dyskinesia_potentially_active = (consecutive_dyskinesia_detections >= MIN_CONSECUTIVE_DETECTIONS);
    bool tremor_potentially_active = (consecutive_tremor_detections >= MIN_CONSECUTIVE_DETECTIONS);

    // 3. Manage Cooldown Timer
    if (was_dyskinesia_potentially_active && !dyskinesia_potentially_active) {
        dyskinesia_cooldown_counter = DYSKINESIA_COOLDOWN_CYCLES;
    }
    was_dyskinesia_potentially_active = dyskinesia_potentially_active;
    bool tremor_output_suppressed = false;
    if (dyskinesia_cooldown_counter > 0) {
        tremor_output_suppressed = true;
        dyskinesia_cooldown_counter--;
    }

    // 4. Determine Final LED Trigger States
    bool trigger_dyskinesia_led = dyskinesia_potentially_active;
    bool trigger_tremor_led = tremor_potentially_active && !trigger_dyskinesia_led && !tremor_output_suppressed;

    // --- Control Dyskinesia Blinker (LED1) ---
    if (trigger_dyskinesia_led) {
        us_timestamp_t new_interval_us = calculate_toggle_interval_us(detected_dyskinesia_freq);
        // Check if we need to start or restart the blinker
        if (new_interval_us > 0 && (!dyskinesia_blinker_should_be_active || new_interval_us != current_dyskinesia_toggle_interval_us)) {
            dyskinesia_led_flipper.detach(); // Stop previous timeout before starting new one
            current_dyskinesia_toggle_interval_us = new_interval_us;
            led_dyskinesia_pin = 1; // Start ON
            // Use the Timeout object to schedule the callback
            dyskinesia_led_flipper.attach(mbed::callback(&toggle_dyskinesia_led), chrono::microseconds(current_dyskinesia_toggle_interval_us));
        }
        // If triggered, mark the blinker as needing to be active
        dyskinesia_blinker_should_be_active = true;

        // Ensure tremor stops if dyskinesia starts (priority)
        if (tremor_blinker_should_be_active) {
             tremor_led_flipper.detach();
             led_tremor_pin = 0;
             tremor_blinker_should_be_active = false;
        }
    } else {
        // Stop dyskinesia blinker if it's not triggered
        if (dyskinesia_blinker_should_be_active) {
            dyskinesia_led_flipper.detach();
            led_dyskinesia_pin = 0;
            dyskinesia_blinker_should_be_active = false;
        }
    }

    // --- Control Tremor Blinker (LED2) ---
    // Only consider if dyskinesia is NOT triggered
    if (!trigger_dyskinesia_led) {
        if (trigger_tremor_led) {
            us_timestamp_t new_interval_us = calculate_toggle_interval_us(detected_tremor_freq);
            if (new_interval_us > 0 && (!tremor_blinker_should_be_active || new_interval_us != current_tremor_toggle_interval_us)) {
                tremor_led_flipper.detach();
                current_tremor_toggle_interval_us = new_interval_us;
                led_tremor_pin = 1; // Start ON
                // Use the Timeout object to schedule the callback
                tremor_led_flipper.attach(mbed::callback(&toggle_tremor_led), chrono::microseconds(current_tremor_toggle_interval_us));
            }
            // If triggered, mark the blinker as needing to be active
            tremor_blinker_should_be_active = true;
        } else {
            // Stop tremor blinker if it's not triggered
            if (tremor_blinker_should_be_active) {
                tremor_led_flipper.detach();
                led_tremor_pin = 0;
                tremor_blinker_should_be_active = false;
            }
        }
    }
    // If dyskinesia IS triggered, tremor blinker is already ensured stopped above.
}


// --- Blinker Callbacks ---
void toggle_tremor_led() {
    // Check if still supposed to be active before toggling and rescheduling
    if (tremor_blinker_should_be_active) {
        led_tremor_pin = !led_tremor_pin;
        // Reschedule the next toggle using the stored interval
        if (current_tremor_toggle_interval_us > 0) {
             // Use the Timeout object to reschedule itself
             tremor_led_flipper.attach(mbed::callback(&toggle_tremor_led), chrono::microseconds(current_tremor_toggle_interval_us));
        } else {
             tremor_blinker_should_be_active = false; // Stop if interval is somehow zero
             led_tremor_pin = 0;
        }
    } else {
        // If flag got turned off externally, ensure LED is off
         led_tremor_pin = 0;
         // Do not reschedule
    }
}

void toggle_dyskinesia_led() {
     if (dyskinesia_blinker_should_be_active) {
        led_dyskinesia_pin = !led_dyskinesia_pin;
         if (current_dyskinesia_toggle_interval_us > 0) {
             // Use the Timeout object to reschedule itself
             dyskinesia_led_flipper.attach(mbed::callback(&toggle_dyskinesia_led), chrono::microseconds(current_dyskinesia_toggle_interval_us));
        } else {
            dyskinesia_blinker_should_be_active = false;
            led_dyskinesia_pin = 0;
        }
     } else {
          // If flag got turned off externally, ensure LED is off
          led_dyskinesia_pin = 0;
          // Do not reschedule
     }
}