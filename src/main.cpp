#include "mbed.h"
#include "arm_math.h"

#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <limits>
#include <cmath>
#include <chrono>

using namespace std::chrono;

typedef enum
{
    STATE_IDLE,
    STATE_PROCESSING
} ApplicationState_t;

typedef enum
{
    SYMPTOM_NONE,
    SYMPTOM_TREMOR,
    SYMPTOM_DYSKINESIA
} SymptomType_t;

typedef enum
{
    INTENSITY_NONE,
    INTENSITY_LOW,
    INTENSITY_MEDIUM,
    INTENSITY_HIGH
} IntensityLevel_t;


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

#define LED_SYMPTOM_TYPE_PIN   LED1
#define LED_INTENSITY_PIN      LED2

#define MAGNITUDE_THRESHOLD_TREMOR_LOW      150.0f
#define MAGNITUDE_THRESHOLD_TREMOR_MEDIUM   400.0f
#define MAGNITUDE_THRESHOLD_TREMOR_HIGH     800.0f

#define MAGNITUDE_THRESHOLD_DYSKINESIA_LOW    150.0f
#define MAGNITUDE_THRESHOLD_DYSKINESIA_MEDIUM 400.0f
#define MAGNITUDE_THRESHOLD_DYSKINESIA_HIGH   800.0f

#define MIN_CONSECUTIVE_DETECTIONS 3
#define DYSKINESIA_COOLDOWN_CYCLES 2

#define LED1_TREMOR_BLINK_HZ        2.0f
#define LED1_DYSKINESIA_BLINK_HZ    5.0f
#define LED2_INTENSITY_BLINK_HZ     3.0f

#define LED1_TREMOR_INTERVAL_US     (us_timestamp_t)((1.0f / (2.0f * LED1_TREMOR_BLINK_HZ)) * 1000000.0f)
#define LED1_DYSKINESIA_INTERVAL_US (us_timestamp_t)((1.0f / (2.0f * LED1_DYSKINESIA_BLINK_HZ)) * 1000000.0f)
#define LED2_INTENSITY_INTERVAL_US  (us_timestamp_t)((1.0f / (2.0f * LED2_INTENSITY_BLINK_HZ)) * 1000000.0f)


volatile ApplicationState_t app_state = STATE_IDLE;
volatile uint16_t stream_write_idx = 0;
volatile uint16_t new_sample_count = 0;

uint16_t consecutive_tremor_detections = 0;
uint16_t consecutive_dyskinesia_detections = 0;
uint8_t dyskinesia_cooldown_counter = 0;
bool was_dyskinesia_potentially_active = false;

DigitalOut led_symptom_type_pin(LED_SYMPTOM_TYPE_PIN, 0);
DigitalOut led_intensity_pin(LED_INTENSITY_PIN, 0);
Ticker sampler_ticker;
I2C i2c(PB_11, PB_10);
EventQueue queue(64 * EVENTS_EVENT_SIZE);

Timeout led1_flipper;
Timeout led2_flipper;

us_timestamp_t current_led1_toggle_interval_us = 0;
us_timestamp_t current_led2_toggle_interval_us = LED2_INTENSITY_INTERVAL_US; // Fixed interval for intensity blink

volatile bool led1_blinker_should_be_active = false;
volatile bool led2_blinker_should_be_active = false;


float32_t sensor_raw_buffer[SENSOR_BUFFER_SIZE];
float32_t fft_input_buffer[FFT_INPUT_BUFFER_SIZE];
float32_t fft_output_buffer[FFT_OUTPUT_BUFFER_SIZE];
float32_t hann_window_buffer[FFT_SIZE];

arm_rfft_fast_instance_f32 fft_instance;

float32_t ACTUAL_SAMPLING_FREQUENCY_HZ = 52.0f;
float32_t FREQUENCY_RESOLUTION = 0.0f;

static void InitializePeripherals(void);
static void PrepareFFTInputFromCircularBuffer(void);
static void ProcessSensorData(void);
static void IndicateResults(bool tremor_detected_raw, bool dyskinesia_detected_raw, float32_t tremor_mag, float32_t dyskinesia_mag);
static void SamplerISRHandler(void);
static void ReadSensorAndMaybeQueueProcessing(void);
static void TriggerProcessing(void);
void toggle_led1();
void toggle_led2();
static IntensityLevel_t get_intensity_level(float32_t magnitude, float32_t low_thresh, float32_t med_thresh, float32_t high_thresh);


int main()
{
    printf("--- Mbed Tremor/Dyskinesia Detection Start (Scheme A - LED1=Type, LED2=Intensity) ---\n");
    InitializePeripherals();
    printf("Starting event queue dispatch...\n");
    queue.dispatch_forever();
    return 0;
}

static void InitializePeripherals(void)
{
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
    microseconds interval_chrono = microseconds((long long)((1.0f / ACTUAL_SAMPLING_FREQUENCY_HZ) * 1e6f));

    printf("Using Sensor ODR: %.2f Hz, Interval: %lld us\n", ACTUAL_SAMPLING_FREQUENCY_HZ, interval_chrono.count());
    printf("Freq Res: %.3f Hz\n", FREQUENCY_RESOLUTION);
    printf("LED1 Tremor Interval: %llu us, Dyskinesia Interval: %llu us\n", LED1_TREMOR_INTERVAL_US, LED1_DYSKINESIA_INTERVAL_US);
    printf("LED2 Intensity Interval: %llu us\n", LED2_INTENSITY_INTERVAL_US);

    sampler_ticker.attach(callback(&SamplerISRHandler), interval_chrono);
    printf("Sampler ticker attached\n");

    app_state = STATE_IDLE;
}

static void SamplerISRHandler(void) {
    queue.call(&ReadSensorAndMaybeQueueProcessing);
}

static void ReadSensorAndMaybeQueueProcessing(void) {
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
    app_state = STATE_PROCESSING;

    PrepareFFTInputFromCircularBuffer();

    arm_mult_f32(fft_input_buffer, hann_window_buffer, fft_input_buffer, FFT_INPUT_BUFFER_SIZE);
    arm_rfft_fast_f32(&fft_instance, fft_input_buffer, fft_output_buffer, 0);
    arm_cmplx_mag_f32(fft_output_buffer, fft_output_buffer, FFT_SIZE / 2);

    float32_t max_tremor_mag = 0.0f;
    float32_t max_dyskinesia_mag = 0.0f;

    for (uint16_t k = 1; k < FFT_SIZE / 2; k++) {
        float32_t current_freq = (float32_t)k * FREQUENCY_RESOLUTION;
        float32_t current_mag = fft_output_buffer[k];

        if (current_freq >= TREMOR_LOW_HZ && current_freq <= TREMOR_HIGH_HZ) {
            if (current_mag > max_tremor_mag) {
                max_tremor_mag = current_mag;
            }
        }

        if (current_freq >= DYSKINESIA_LOW_HZ && current_freq <= DYSKINESIA_HIGH_HZ) {
             if (current_mag > max_dyskinesia_mag) {
                max_dyskinesia_mag = current_mag;
            }
        }
    }

    bool tremor_detected_raw = (max_tremor_mag > MAGNITUDE_THRESHOLD_TREMOR_LOW);
    bool dyskinesia_detected_raw = (max_dyskinesia_mag > MAGNITUDE_THRESHOLD_DYSKINESIA_LOW);

    IndicateResults(tremor_detected_raw, dyskinesia_detected_raw, max_tremor_mag, max_dyskinesia_mag);

    app_state = STATE_IDLE;
}

static void PrepareFFTInputFromCircularBuffer(void)
{
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

static IntensityLevel_t get_intensity_level(float32_t magnitude, float32_t low_thresh, float32_t med_thresh, float32_t high_thresh) {
    if (magnitude >= high_thresh) return INTENSITY_HIGH;
    if (magnitude >= med_thresh) return INTENSITY_MEDIUM;
    if (magnitude >= low_thresh) return INTENSITY_LOW;
    return INTENSITY_NONE;
}


static void IndicateResults(bool tremor_detected_raw, bool dyskinesia_detected_raw, float32_t tremor_mag, float32_t dyskinesia_mag)
{
    if (tremor_detected_raw) {
        if (consecutive_tremor_detections < MIN_CONSECUTIVE_DETECTIONS) consecutive_tremor_detections++;
    } else {
        consecutive_tremor_detections = 0;
    }

    if (dyskinesia_detected_raw) {
         if (consecutive_dyskinesia_detections < MIN_CONSECUTIVE_DETECTIONS) consecutive_dyskinesia_detections++;
    } else {
        consecutive_dyskinesia_detections = 0;
    }

    bool dyskinesia_potentially_active = (consecutive_dyskinesia_detections >= MIN_CONSECUTIVE_DETECTIONS);
    bool tremor_potentially_active = (consecutive_tremor_detections >= MIN_CONSECUTIVE_DETECTIONS);

    if (was_dyskinesia_potentially_active && !dyskinesia_potentially_active) {
        dyskinesia_cooldown_counter = DYSKINESIA_COOLDOWN_CYCLES;
    }
    was_dyskinesia_potentially_active = dyskinesia_potentially_active;

    bool tremor_output_suppressed = false;
    if (dyskinesia_cooldown_counter > 0) {
        tremor_output_suppressed = true;
        dyskinesia_cooldown_counter--;
    }

    SymptomType_t final_symptom = SYMPTOM_NONE;
    IntensityLevel_t final_intensity = INTENSITY_NONE;
    float32_t relevant_magnitude = 0.0f;

    if (dyskinesia_potentially_active) {
        final_symptom = SYMPTOM_DYSKINESIA;
        relevant_magnitude = dyskinesia_mag;
        final_intensity = get_intensity_level(relevant_magnitude,
                                            MAGNITUDE_THRESHOLD_DYSKINESIA_LOW,
                                            MAGNITUDE_THRESHOLD_DYSKINESIA_MEDIUM,
                                            MAGNITUDE_THRESHOLD_DYSKINESIA_HIGH);
    } else if (tremor_potentially_active && !tremor_output_suppressed) {
        final_symptom = SYMPTOM_TREMOR;
        relevant_magnitude = tremor_mag;
        final_intensity = get_intensity_level(relevant_magnitude,
                                            MAGNITUDE_THRESHOLD_TREMOR_LOW,
                                            MAGNITUDE_THRESHOLD_TREMOR_MEDIUM,
                                            MAGNITUDE_THRESHOLD_TREMOR_HIGH);
    } else {
         final_symptom = SYMPTOM_NONE;
         final_intensity = INTENSITY_NONE;
    }


    // --- Control LED1 (Symptom Type) ---
    us_timestamp_t new_led1_interval = 0;
    bool should_led1_blink = false;

    if (final_symptom == SYMPTOM_TREMOR) {
        new_led1_interval = LED1_TREMOR_INTERVAL_US;
        should_led1_blink = true;
    } else if (final_symptom == SYMPTOM_DYSKINESIA) {
        new_led1_interval = LED1_DYSKINESIA_INTERVAL_US;
        should_led1_blink = true;
    }

    if (should_led1_blink) {
        if (!led1_blinker_should_be_active || new_led1_interval != current_led1_toggle_interval_us) {
            led1_flipper.detach();
            current_led1_toggle_interval_us = new_led1_interval;
            led_symptom_type_pin = 1;
            led1_flipper.attach(mbed::callback(&toggle_led1), microseconds(current_led1_toggle_interval_us));
        }
        led1_blinker_should_be_active = true;
    } else {
        if (led1_blinker_should_be_active) {
            led1_flipper.detach();
            led_symptom_type_pin = 0;
            led1_blinker_should_be_active = false;
        }
         led_symptom_type_pin = 0; // Ensure off if not blinking
    }

    // --- Control LED2 (Intensity) ---
    bool should_led2_blink = false;
    bool should_led2_be_solid = false;

    if (final_symptom != SYMPTOM_NONE) {
        if (final_intensity == INTENSITY_MEDIUM) {
            should_led2_blink = true;
        } else if (final_intensity == INTENSITY_HIGH) {
            should_led2_be_solid = true;
        }
    }


    if (should_led2_be_solid) {
         if (led2_blinker_should_be_active) { // If it was blinking, stop it
              led2_flipper.detach();
              led2_blinker_should_be_active = false;
         }
         led_intensity_pin = 1; // Set solid ON
    } else if (should_led2_blink) {
         // Always use the fixed intensity interval
         if (!led2_blinker_should_be_active) { // If it wasn't blinking or was solid
              led2_flipper.detach(); // Detach just in case (e.g., from solid)
              led_intensity_pin = 1; // Start ON
              led2_flipper.attach(mbed::callback(&toggle_led2), microseconds(current_led2_toggle_interval_us));
              led2_blinker_should_be_active = true;
         }
         // If already blinking, the callback will handle rescheduling
    } else { // Intensity is LOW or NONE, turn off LED2
         if (led2_blinker_should_be_active) {
              led2_flipper.detach();
              led2_blinker_should_be_active = false;
         }
         led_intensity_pin = 0; // Ensure off
    }
}


void toggle_led1() {
    if (led1_blinker_should_be_active) {
        led_symptom_type_pin = !led_symptom_type_pin;
        if (current_led1_toggle_interval_us > 0) {
             led1_flipper.attach(mbed::callback(&toggle_led1), microseconds(current_led1_toggle_interval_us));
        } else {
             led1_blinker_should_be_active = false;
             led_symptom_type_pin = 0;
        }
    } else {
         led_symptom_type_pin = 0;
    }
}

void toggle_led2() {
     if (led2_blinker_should_be_active) {
        led_intensity_pin = !led_intensity_pin;
         if (current_led2_toggle_interval_us > 0) {
             led2_flipper.attach(mbed::callback(&toggle_led2), microseconds(current_led2_toggle_interval_us));
        } else {
            led2_blinker_should_be_active = false;
            led_intensity_pin = 0;
        }
     } else {
          led_intensity_pin = 0;
     }
}