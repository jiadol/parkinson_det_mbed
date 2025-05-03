#include "mbed.h"
#include "arm_math.h" // Include CMSIS-DSP library

#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <limits>
#include <cmath>
#include <chrono>

using namespace std::chrono;

// --- Enumeration Definitions ---
typedef enum
{
    STATE_IDLE,         // System is waiting for enough new samples
    STATE_PROCESSING    // System is currently running FFT and analysis
} ApplicationState_t;

typedef enum
{
    SYMPTOM_NONE,       // No symptom detected or below threshold
    SYMPTOM_TREMOR,     // Tremor detected (typically 3-5 Hz)
    SYMPTOM_DYSKINESIA  // Dyskinesia detected (typically 5-7 Hz)
} SymptomType_t;

typedef enum
{
    INTENSITY_NONE,     // No significant intensity or symptom
    INTENSITY_LOW,      // Low intensity / small amplitude movement
    INTENSITY_MEDIUM,   // Medium intensity / medium amplitude movement
    INTENSITY_HIGH      // High intensity / large amplitude movement
} IntensityLevel_t;

// --- I2C and Sensor Constants ---
#define LSM6DSL_I2C_ADDR_DEFAULT (0x6A << 1) // LSM6DSL default I2C address (use 7-bit address shifted left)
#define LSM6DSL_WHO_AM_I_REG     0x0F        // WHO_AM_I register address
#define LSM6DSL_WHO_AM_I_VAL     0x6A        // Expected value for WHO_AM_I register
#define LSM6DSL_CTRL1_XL_REG     0x10        // Accelerometer control register 1
#define LSM6DSL_OUTX_L_XL_REG    0x28        // Start address for accelerometer data registers
#define LSM6DSL_ODR_52HZ         0x60        // ODR = 52 Hz, FS = +/-2g (default)
#define LSM6DSL_ACCEL_SENSITIVITY_2G 0.061f // Sensitivity in mg/LSB at +/-2g Full Scale

// --- FFT and Buffer Constants ---
#define FFT_SIZE 64                        // Number of points for FFT analysis
#define SENSOR_BUFFER_SIZE FFT_SIZE        // Size of the circular buffer for raw sensor data
#define FFT_TRIGGER_OVERLAP_PERCENT 20     // Percentage overlap for consecutive FFT windows
#define FFT_TRIGGER_NEW_SAMPLES ((FFT_SIZE * (100 - FFT_TRIGGER_OVERLAP_PERCENT)) / 100) // Number of new samples needed to trigger an FFT
#if FFT_TRIGGER_NEW_SAMPLES < 1
#error "FFT_TRIGGER_NEW_SAMPLES must be at least 1" // Sanity check for calculation
#endif

#define FFT_INPUT_BUFFER_SIZE FFT_SIZE     // Size of the buffer holding data for FFT input
#define FFT_OUTPUT_BUFFER_SIZE FFT_SIZE    // Size of the buffer holding FFT output (complex or magnitude)

// --- Symptom Frequency Ranges ---
#define TREMOR_LOW_HZ 3.0f                 // Tremor low frequency threshold (Hz)
#define TREMOR_HIGH_HZ 5.0f                // Tremor high frequency threshold (Hz)
#define DYSKINESIA_LOW_HZ 5.0f             // Dyskinesia low frequency threshold (Hz)
#define DYSKINESIA_HIGH_HZ 7.0f            // Dyskinesia high frequency threshold (Hz)
 // Note: Consider adding a small gap (e.g., 4.8Hz / 5.2Hz) if leakage is an issue

// --- LED Pin Definitions ---
#define LED_SYMPTOM_TYPE_PIN   LED1        // LED to indicate symptom type (Tremor vs Dyskinesia)
#define LED_INTENSITY_PIN      LED2        // LED to indicate symptom intensity (Off/Blink/Solid)

// --- Magnitude Threshold Definitions (for Intensity Levels) ---
// These values are critical and NEED TUNING based on real-world testing!
#define MAGNITUDE_THRESHOLD_TREMOR_LOW      150.0f // Tremor low intensity threshold
#define MAGNITUDE_THRESHOLD_TREMOR_MEDIUM   1000.0f // Tremor medium intensity threshold
#define MAGNITUDE_THRESHOLD_TREMOR_HIGH     2000.0f // Tremor high intensity threshold

#define MAGNITUDE_THRESHOLD_DYSKINESIA_LOW    150.0f // Dyskinesia low intensity threshold
#define MAGNITUDE_THRESHOLD_DYSKINESIA_MEDIUM 1000.0f // Dyskinesia medium intensity threshold
#define MAGNITUDE_THRESHOLD_DYSKINESIA_HIGH   2000.0f // Dyskinesia high intensity threshold

// --- State Confirmation and Smoothing Parameters ---
#define MIN_CONSECUTIVE_DETECTIONS 3       // Minimum number of consecutive FFT cycles a symptom must be detected above LOW threshold to be considered active
#define DYSKINESIA_COOLDOWN_CYCLES 2       // Number of cycles to suppress Tremor indication after Dyskinesia stops

// --- LED Blink Frequencies and Intervals ---
#define LED1_TREMOR_BLINK_HZ        2.0f   // Blink frequency for LED1 when indicating Tremor
#define LED1_DYSKINESIA_BLINK_HZ    5.0f   // Blink frequency for LED1 when indicating Dyskinesia
#define LED2_INTENSITY_BLINK_HZ     3.0f   // Blink frequency for LED2 when indicating MEDIUM intensity

// Calculate LED toggle intervals (half-periods) in microseconds
#define LED1_TREMOR_INTERVAL_US     (us_timestamp_t)((1.0f / (2.0f * LED1_TREMOR_BLINK_HZ)) * 1000000.0f)
#define LED1_DYSKINESIA_INTERVAL_US (us_timestamp_t)((1.0f / (2.0f * LED1_DYSKINESIA_BLINK_HZ)) * 1000000.0f)
#define LED2_INTENSITY_INTERVAL_US  (us_timestamp_t)((1.0f / (2.0f * LED2_INTENSITY_BLINK_HZ)) * 1000000.0f)


// --- Global Variables ---
volatile ApplicationState_t app_state = STATE_IDLE; // Current state of the application
volatile uint16_t stream_write_idx = 0;             // Write index for the circular sensor buffer
volatile uint16_t new_sample_count = 0;             // Counter for new samples since last FFT

uint16_t consecutive_tremor_detections = 0;         // Counter for consecutive tremor detections
uint16_t consecutive_dyskinesia_detections = 0;     // Counter for consecutive dyskinesia detections
uint8_t dyskinesia_cooldown_counter = 0;            // Cooldown counter to suppress tremor after dyskinesia
bool was_dyskinesia_potentially_active = false;     // Flag indicating if dyskinesia was active in the previous cycle

// Mbed OS Objects
DigitalOut led_symptom_type_pin(LED_SYMPTOM_TYPE_PIN, 0); // DigitalOut object for LED1
DigitalOut led_intensity_pin(LED_INTENSITY_PIN, 0);       // DigitalOut object for LED2
Ticker sampler_ticker;                                    // Ticker for periodic sampling ISR
I2C i2c(PB_11, PB_10);                                    // I2C object (Adjust pins PB_11, PB_10 for your board)
EventQueue queue(64 * EVENTS_EVENT_SIZE);                // EventQueue to defer processing from ISR

Timeout led1_flipper;                                     // Timeout object to control LED1 blinking
Timeout led2_flipper;                                     // Timeout object to control LED2 blinking

us_timestamp_t current_led1_toggle_interval_us = 0;       // Current toggle interval for LED1 (changes based on symptom)
us_timestamp_t current_led2_toggle_interval_us = LED2_INTENSITY_INTERVAL_US; // Fixed toggle interval for LED2 (medium intensity blink)

volatile bool led1_blinker_should_be_active = false;      // Flag indicating if LED1 blinker should be running
volatile bool led2_blinker_should_be_active = false;      // Flag indicating if LED2 blinker should be running

// --- Buffers ---
float32_t sensor_raw_buffer[SENSOR_BUFFER_SIZE];      // Circular buffer for raw Z-axis accelerometer data (in mg)
float32_t fft_input_buffer[FFT_INPUT_BUFFER_SIZE];    // Buffer for preparing FFT input data
float32_t fft_output_buffer[FFT_OUTPUT_BUFFER_SIZE];  // Buffer for FFT output (magnitudes)
float32_t hann_window_buffer[FFT_SIZE];               // Buffer to store Hann window coefficients

// --- CMSIS-DSP FFT Instance ---
arm_rfft_fast_instance_f32 fft_instance;              // Instance structure for the ARM RFFT function

// --- Runtime Calculated Constants ---
float32_t ACTUAL_SAMPLING_FREQUENCY_HZ = 52.0f;       // Actual sampling frequency used (Hz) - matches LSM6DSL ODR setting
float32_t FREQUENCY_RESOLUTION = 0.0f;                // Frequency resolution of the FFT (Hz per bin)

// --- Function Prototypes ---
static void InitializePeripherals(void);              // Initializes I2C, Accelerometer, FFT, Ticker
static void PrepareFFTInputFromCircularBuffer(void);  // Copies the latest data from circular buffer to FFT input buffer
static void ProcessSensorData(void);                  // Performs FFT, analysis, and calls IndicateResults
static void IndicateResults(bool tremor_detected_raw, bool dyskinesia_detected_raw, float32_t tremor_mag, float32_t dyskinesia_mag); // Updates LEDs based on analysis results
static void SamplerISRHandler(void);                  // ISR called by the Ticker
static void ReadSensorAndMaybeQueueProcessing(void);  // Reads sensor data and queues processing if needed (runs in EventQueue context)
static void TriggerProcessing(void);                  // Triggers the ProcessSensorData function (runs in EventQueue context)
void toggle_led1();                                   // Callback function for LED1 Timeout
void toggle_led2();                                   // Callback function for LED2 Timeout
static IntensityLevel_t get_intensity_level(float32_t magnitude, float32_t low_thresh, float32_t med_thresh, float32_t high_thresh); // Determines intensity level based on magnitude


// --- Main Function ---
int main()
{
    printf("--- Mbed Tremor/Dyskinesia Detection Start (Scheme A - LED1=Type, LED2=Intensity) ---\n");
    InitializePeripherals(); // Initialize hardware and software components
    printf("Starting event queue dispatch...\n");
    queue.dispatch_forever(); // Start processing events in the queue indefinitely
    return 0; // Should theoretically never reach here
}

// --- Peripheral Initialization ---
static void InitializePeripherals(void)
{
    printf("Initializing I2C and Accelerometer...\n");
    i2c.frequency(400000); // Set I2C bus frequency to 400 kHz

    // --- Verify Accelerometer Communication (WHO_AM_I check) ---
    char cmd[1];
    char data[1];
    cmd[0] = LSM6DSL_WHO_AM_I_REG; // Register address to read
    int status = i2c.write(LSM6DSL_I2C_ADDR_DEFAULT, cmd, 1, true); // Send register address, keep bus active (repeated start)
    if (status != 0) MBED_ERROR(MBED_ERROR_WRITE_FAILED, "I2C WhoAmI Write Failed");
    status = i2c.read(LSM6DSL_I2C_ADDR_DEFAULT, data, 1, false); // Read 1 byte from the register
    if (status != 0) MBED_ERROR(MBED_ERROR_READ_FAILED, "I2C WhoAmI Read Failed");

    // Check if the read value matches the expected value
    if (data[0] != LSM6DSL_WHO_AM_I_VAL) {
        printf("LSM6DSL WhoAmI Check Failed. Expected 0x%X, Got 0x%X\n", LSM6DSL_WHO_AM_I_VAL, data[0]);
        MBED_ERROR(MBED_ERROR_INITIALIZATION_FAILED, "Accelerometer WhoAmI Mismatch");
    }
    printf("Accelerometer WhoAmI Check OK (0x%X)\n", data[0]);

    // --- Configure Accelerometer ---
    char config[2];
    config[0] = LSM6DSL_CTRL1_XL_REG; // Address of the control register
    config[1] = LSM6DSL_ODR_52HZ;     // Value to write (sets ODR to 52Hz, FS to +/-2g)
    status = i2c.write(LSM6DSL_I2C_ADDR_DEFAULT, config, 2, false); // Write the configuration
    if (status != 0) MBED_ERROR(MBED_ERROR_WRITE_FAILED, "I2C Accel Config Failed");
    printf("Accelerometer configured (ODR=%.0fHz, FS=2g)\n", ACTUAL_SAMPLING_FREQUENCY_HZ);

    // --- Initialize FFT ---
    if (arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE) != ARM_MATH_SUCCESS) {
        MBED_ERROR(MBED_ERROR_INITIALIZATION_FAILED, "FFT Init Failed");
    }
    printf("FFT Initialized for size %d\n", FFT_SIZE);

    // --- Pre-calculate Hann Window ---
    arm_hanning_f32(hann_window_buffer, FFT_SIZE); // Calculate Hann window coefficients
    printf("Hanning window calculated\n");

    // --- Calculate Runtime Constants ---
    FREQUENCY_RESOLUTION = ACTUAL_SAMPLING_FREQUENCY_HZ / (float)FFT_SIZE; // Calculate frequency resolution
    microseconds interval_chrono = microseconds((long long)((1.0f / ACTUAL_SAMPLING_FREQUENCY_HZ) * 1e6f)); // Calculate sampling interval in microseconds

    printf("Using Sensor ODR: %.2f Hz, Sample Interval: %lld us\n", ACTUAL_SAMPLING_FREQUENCY_HZ, interval_chrono.count());
    printf("FFT Freq Resolution: %.3f Hz/bin\n", FREQUENCY_RESOLUTION);
    printf("LED1 Tremor Interval: %llu us, Dyskinesia Interval: %llu us\n", LED1_TREMOR_INTERVAL_US, LED1_DYSKINESIA_INTERVAL_US);
    printf("LED2 Intensity (Medium) Interval: %llu us\n", LED2_INTENSITY_INTERVAL_US);

    // --- Attach Sampler ISR to Ticker ---
    sampler_ticker.attach(callback(&SamplerISRHandler), interval_chrono); // Call SamplerISRHandler periodically
    printf("Sampler ticker attached\n");

    app_state = STATE_IDLE; // Set initial application state
}

// --- Ticker Interrupt Service Routine ---
// Keep ISR very short: defer actual work to the event queue.
static void SamplerISRHandler(void) {
    // Call the sensor reading function in the context of the event queue
    queue.call(&ReadSensorAndMaybeQueueProcessing);
}

// --- Read Sensor and Potentially Queue Processing ---
// This function runs in the EventQueue context, not the ISR context.
static void ReadSensorAndMaybeQueueProcessing(void) {
    char raw_i2c_data[6]; // Buffer for raw X, Y, Z accelerometer data (2 bytes each)
    char start_reg = LSM6DSL_OUTX_L_XL_REG; // Starting register address for reading accel data
    int write_stat = -1;
    int read_stat = -1;

    // Perform I2C read operation
    write_stat = i2c.write(LSM6DSL_I2C_ADDR_DEFAULT, &start_reg, 1, true); // Send start register address, keep bus active
    if (write_stat == 0) {
        read_stat = i2c.read(LSM6DSL_I2C_ADDR_DEFAULT, raw_i2c_data, 6, false); // Read 6 bytes of data (X, Y, Z)
    }

    if (write_stat == 0 && read_stat == 0) {
        // Successfully read data
        // Enter critical section to safely access shared variables
        CriticalSectionLock lock;

        // Extract Z-axis raw data (assuming little-endian format)
        int16_t raw_z = (int16_t)((raw_i2c_data[5] << 8) | raw_i2c_data[4]);
        // Convert raw data to physical units (mg) and store in circular buffer
        sensor_raw_buffer[stream_write_idx] = (float32_t)raw_z * LSM6DSL_ACCEL_SENSITIVITY_2G;

        // Update circular buffer write index
        stream_write_idx = (stream_write_idx + 1) % SENSOR_BUFFER_SIZE;
        // Increment count of new samples received
        new_sample_count++;

        // Check if enough new samples have arrived and the system is idle
        if (app_state != STATE_PROCESSING && new_sample_count >= FFT_TRIGGER_NEW_SAMPLES) {
             // Queue the data processing task
             queue.call(&TriggerProcessing);
             new_sample_count = 0; // Reset new sample counter
        }
        // Exit critical section automatically when lock goes out of scope
    } else {
         // Handle I2C read/write error
         printf("ERR: I2C R/W failed in ReadSensor func (write=%d, read=%d)\n", write_stat, read_stat);
         // Consider adding more robust error handling here (e.g., retry, reset I2C)
    }
}

// --- Trigger Data Processing ---
// This function runs in the EventQueue context.
static void TriggerProcessing(void) {
    // Only start processing if the application is currently idle to prevent re-entrancy
    if (app_state == STATE_IDLE) {
        ProcessSensorData();
    }
}

// --- Process Sensor Data (FFT and Analysis) ---
// This function runs in the EventQueue context.
static void ProcessSensorData(void)
{
    app_state = STATE_PROCESSING; // Mark state as processing

    // 1. Prepare FFT Input Data
    // Copy the most recent FFT_SIZE samples from the circular buffer
    PrepareFFTInputFromCircularBuffer();

    // 2. Apply Hann Window
    // Multiply input data by the window function in-place to reduce spectral leakage
    arm_mult_f32(fft_input_buffer, hann_window_buffer, fft_input_buffer, FFT_INPUT_BUFFER_SIZE);

    // 3. Perform Real FFT (RFFT)
    // Input: fft_input_buffer (windowed time-domain data)
    // Output: fft_output_buffer (complex frequency-domain data)
    // The input buffer 'fft_input_buffer' is modified (used as scratch space) by the function.
    // The '0' indicates a forward FFT (time to frequency).
    arm_rfft_fast_f32(&fft_instance, fft_input_buffer, fft_output_buffer, 0);

    // 4. Calculate Complex Magnitude
    // Converts the complex output of RFFT into magnitude values.
    // Input: fft_output_buffer (complex data: [Re0, ReN/2, Re1, Im1, Re2, Im2, ...])
    // Output: fft_output_buffer (magnitude data: [Mag0, MagN/2, Mag1, Mag2, ...]) - overwrites input
    // We compute N/2 magnitude points.
    arm_cmplx_mag_f32(fft_output_buffer, fft_output_buffer, FFT_SIZE / 2);

    // 5. Find Maximum Magnitude in Target Frequency Bands
    float32_t max_tremor_mag = 0.0f;
    float32_t max_dyskinesia_mag = 0.0f;

    // Iterate through frequency bins (k=1 to N/2 - 1). Skip DC (k=0) and Nyquist (k=N/2).
    // Note: arm_cmplx_mag_f32 output places Mag[k] at fft_output_buffer[k] for k=1..N/2-1.
    // Mag[0] is at index 0, Mag[N/2] is at index 1. We ignore index 1 for simplicity here.
    for (uint16_t k = 1; k < FFT_SIZE / 2; k++) {
        float32_t current_freq = (float32_t)k * FREQUENCY_RESOLUTION; // Frequency of the current bin
        float32_t current_mag = fft_output_buffer[k];                // Magnitude of the current bin

        // Check if frequency is within the Tremor band
        if (current_freq >= TREMOR_LOW_HZ && current_freq <= TREMOR_HIGH_HZ) {
            if (current_mag > max_tremor_mag) {
                max_tremor_mag = current_mag; // Update max tremor magnitude
            }
        }

        // Check if frequency is within the Dyskinesia band
        if (current_freq >= DYSKINESIA_LOW_HZ && current_freq <= DYSKINESIA_HIGH_HZ) {
             if (current_mag > max_dyskinesia_mag) {
                max_dyskinesia_mag = current_mag; // Update max dyskinesia magnitude
            }
        }
    }

    // Debug Print (Optional - uncomment to see magnitudes)
    /*
    if (max_tremor_mag > 1.0f || max_dyskinesia_mag > 1.0f) { // Print only if significant magnitude exists
        printf("Mags: Trem=%.1f, Dys=%.1f\n", max_tremor_mag, max_dyskinesia_mag);
    }
    */

    // 6. Perform Raw Detection based on Low Thresholds
    bool tremor_detected_raw = (max_tremor_mag > MAGNITUDE_THRESHOLD_TREMOR_LOW);
    bool dyskinesia_detected_raw = (max_dyskinesia_mag > MAGNITUDE_THRESHOLD_DYSKINESIA_LOW);

    // 7. Indicate Results (Update LEDs based on smoothed/prioritized logic)
    IndicateResults(tremor_detected_raw, dyskinesia_detected_raw, max_tremor_mag, max_dyskinesia_mag);

    app_state = STATE_IDLE; // Mark state as idle, ready for next trigger
}

// --- Prepare FFT Input from Circular Buffer ---
static void PrepareFFTInputFromCircularBuffer(void)
{
    // Enter critical section to safely access the circular buffer and write index
    CriticalSectionLock lock;
    uint16_t current_write_idx_copy = stream_write_idx; // Make a local copy of the volatile write index

    // Calculate the starting index in the circular buffer for the oldest sample needed for FFT
    // Go back FFT_SIZE samples from the current write position.
    int16_t start_idx = (int16_t)current_write_idx_copy - (int16_t)FFT_SIZE;
    if (start_idx < 0) {
        // Handle buffer wrap-around
        start_idx += SENSOR_BUFFER_SIZE;
    }

    // Copy FFT_SIZE samples from the circular buffer to the linear FFT input buffer
    uint16_t read_idx = (uint16_t)start_idx;
    for (uint16_t i = 0; i < FFT_SIZE; i++) {
        fft_input_buffer[i] = sensor_raw_buffer[read_idx];
        // Increment read index with wrap-around
        read_idx = (read_idx + 1) % SENSOR_BUFFER_SIZE;
    }
    // Critical section ends automatically when lock goes out of scope
}

// --- Get Intensity Level based on Magnitude and Thresholds ---
static IntensityLevel_t get_intensity_level(float32_t magnitude, float32_t low_thresh, float32_t med_thresh, float32_t high_thresh) {
    if (magnitude >= high_thresh) return INTENSITY_HIGH;   // Above high threshold -> High intensity
    if (magnitude >= med_thresh) return INTENSITY_MEDIUM;  // Above medium threshold -> Medium intensity
    if (magnitude >= low_thresh) return INTENSITY_LOW;    // Above low threshold -> Low intensity
    return INTENSITY_NONE;                                 // Below low threshold -> None
}

// --- Indicate Results (Update LEDs) ---
// Implements smoothing, priority logic, and controls LED states.
static void IndicateResults(bool tremor_detected_raw, bool dyskinesia_detected_raw, float32_t tremor_mag, float32_t dyskinesia_mag)
{
    // --- 1. Update Consecutive Detection Counters ---
    // Increment counter if raw detection is true, reset if false.
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

    // --- 2. Determine Potential Activity (based on consecutive counts) ---
    // A symptom is "potentially active" if it has been detected raw for enough consecutive cycles.
    bool dyskinesia_potentially_active = (consecutive_dyskinesia_detections >= MIN_CONSECUTIVE_DETECTIONS);
    bool tremor_potentially_active = (consecutive_tremor_detections >= MIN_CONSECUTIVE_DETECTIONS);

    // --- 3. Handle Dyskinesia Cooldown Logic ---
    // If dyskinesia was active last cycle but isn't now, start the cooldown timer.
    if (was_dyskinesia_potentially_active && !dyskinesia_potentially_active) {
        dyskinesia_cooldown_counter = DYSKINESIA_COOLDOWN_CYCLES;
    }
    // Update the state for the next cycle's comparison.
    was_dyskinesia_potentially_active = dyskinesia_potentially_active;

    // Check if tremor output should be suppressed due to cooldown.
    bool tremor_output_suppressed = (dyskinesia_cooldown_counter > 0);
    if (tremor_output_suppressed) {
        dyskinesia_cooldown_counter--; // Decrement cooldown timer
    }

    // --- 4. Determine Final Symptom and Intensity (Magnitude Priority Logic) ---
    SymptomType_t final_symptom = SYMPTOM_NONE;
    IntensityLevel_t final_intensity = INTENSITY_NONE;
    float32_t relevant_magnitude = 0.0f; // Magnitude used for intensity calculation

    bool dyskinesia_wins = false; // Flag if Dyskinesia is the final chosen symptom
    bool tremor_wins = false;     // Flag if Tremor is the final chosen symptom

    // Determine candidate status based on potential activity and suppression rules
    bool is_dyskinesia_candidate = dyskinesia_potentially_active;
    bool is_tremor_candidate = tremor_potentially_active && !tremor_output_suppressed;

    // Decide the winner
    if (is_dyskinesia_candidate && is_tremor_candidate) {
        // *** Both potentially active: Decide based on higher magnitude ***
        if (dyskinesia_mag >= tremor_mag) { // Dyskinesia magnitude is higher or equal
            dyskinesia_wins = true;
        } else { // Tremor magnitude is higher
            tremor_wins = true;
        }
    } else if (is_dyskinesia_candidate) {
        // Only Dyskinesia is a candidate
        dyskinesia_wins = true;
    } else if (is_tremor_candidate) {
        // Only Tremor is a candidate
        tremor_wins = true;
    }
    // If neither is a candidate, both flags remain false -> SYMPTOM_NONE

    // Set the final symptom state and calculate intensity based on the winner
    if (dyskinesia_wins) {
        final_symptom = SYMPTOM_DYSKINESIA;
        relevant_magnitude = dyskinesia_mag;
        // Calculate intensity using dyskinesia magnitude and its thresholds
        final_intensity = get_intensity_level(relevant_magnitude,
                                            MAGNITUDE_THRESHOLD_DYSKINESIA_LOW,
                                            MAGNITUDE_THRESHOLD_DYSKINESIA_MEDIUM,
                                            MAGNITUDE_THRESHOLD_DYSKINESIA_HIGH);
    } else if (tremor_wins) {
        final_symptom = SYMPTOM_TREMOR;
        relevant_magnitude = tremor_mag;
        // Calculate intensity using tremor magnitude and its thresholds
        final_intensity = get_intensity_level(relevant_magnitude,
                                            MAGNITUDE_THRESHOLD_TREMOR_LOW,
                                            MAGNITUDE_THRESHOLD_TREMOR_MEDIUM,
                                            MAGNITUDE_THRESHOLD_TREMOR_HIGH);
    } else { // No winner
        final_symptom = SYMPTOM_NONE;
        final_intensity = INTENSITY_NONE;
    }

    // --- 5. Control LEDs based on Final Symptom and Intensity ---

    // --- Control LED1 (Symptom Type) ---
    us_timestamp_t new_led1_interval = 0; // The blink interval needed for LED1
    bool should_led1_blink = false;       // Should LED1 be blinking?

    // Determine LED1 behavior based on the final symptom
    if (final_symptom == SYMPTOM_TREMOR) {
        new_led1_interval = LED1_TREMOR_INTERVAL_US; // Set tremor blink rate
        should_led1_blink = true;
    } else if (final_symptom == SYMPTOM_DYSKINESIA) {
        new_led1_interval = LED1_DYSKINESIA_INTERVAL_US; // Set dyskinesia blink rate
        should_led1_blink = true;
    }

    // Update LED1 state machine
    if (should_led1_blink) {
        // If LED1 should blink:
        // Check if it wasn't blinking before OR if the blink rate needs to change.
        if (!led1_blinker_should_be_active || new_led1_interval != current_led1_toggle_interval_us) {
            led1_flipper.detach(); // Stop any existing timeout
            current_led1_toggle_interval_us = new_led1_interval; // Set the new interval
            led_symptom_type_pin = 1; // Turn LED ON (first step of blink)
            // Schedule the next toggle using the new interval
            led1_flipper.attach(mbed::callback(&toggle_led1), microseconds(current_led1_toggle_interval_us));
        }
        led1_blinker_should_be_active = true; // Mark blinker as active
    } else {
        // If LED1 should NOT blink (SYMPTOM_NONE):
        if (led1_blinker_should_be_active) {
            // If it was blinking, stop it.
            led1_flipper.detach();
            led1_blinker_should_be_active = false; // Mark blinker as inactive
        }
         led_symptom_type_pin = 0; // Ensure LED is OFF
    }

    // --- Control LED2 (Intensity / Movement Amplitude) ---
    bool should_led2_blink = false;   // Corresponds to MEDIUM amplitude (Blinking)
    bool should_led2_be_solid = false; // Corresponds to HIGH amplitude (Solid ON)
                                      // OFF corresponds to LOW amplitude or NONE

    // Determine LED2 behavior only if a symptom is active
    if (final_symptom != SYMPTOM_NONE) {
        if (final_intensity == INTENSITY_MEDIUM) {
            // Medium intensity -> Blinking
            should_led2_blink = true;
        } else if (final_intensity == INTENSITY_HIGH) {
            // High intensity -> Solid ON
            should_led2_be_solid = true;
        }
        // If final_intensity is LOW or NONE, both flags remain false -> LED OFF
    }

    // Update LED2 state machine
    if (should_led2_be_solid) {
         // **High Amplitude: LED2 Solid ON**
         if (led2_blinker_should_be_active) { // If it was blinking, stop it
              led2_flipper.detach();
              led2_blinker_should_be_active = false;
         }
         led_intensity_pin = 1; // Set LED to solid ON
    } else if (should_led2_blink) {
         // **Medium Amplitude: LED2 Blinking**
         // Uses the fixed medium intensity blink interval 'current_led2_toggle_interval_us'
         if (!led2_blinker_should_be_active) { // If it wasn't blinking or was solid before
              led2_flipper.detach(); // Ensure any previous timeout is stopped
              led_intensity_pin = 1; // Turn LED ON (first step of blink)
              // Schedule the next toggle
              led2_flipper.attach(mbed::callback(&toggle_led2), microseconds(current_led2_toggle_interval_us));
              led2_blinker_should_be_active = true; // Mark blinker as active
         }
         // If already blinking (led2_blinker_should_be_active is true), the toggle_led2 callback will reschedule itself.
    } else {
         // **Low Amplitude or No Symptom: LED2 OFF**
         if (led2_blinker_should_be_active) { // If it was blinking, stop it
              led2_flipper.detach();
              led2_blinker_should_be_active = false;
         }
         led_intensity_pin = 0; // Ensure LED is OFF
    }
}


// --- LED1 Toggle Callback ---
// Called by the Timeout 'led1_flipper' to toggle LED1 state during blinking.
void toggle_led1() {
    if (led1_blinker_should_be_active) {
        // If the blinker is supposed to be active:
        led_symptom_type_pin = !led_symptom_type_pin; // Toggle the LED state
        // Reschedule the next toggle if the interval is valid
        if (current_led1_toggle_interval_us > 0) {
             led1_flipper.attach(mbed::callback(&toggle_led1), microseconds(current_led1_toggle_interval_us));
        } else {
             // Safety check: if interval is invalid, stop blinking and turn off LED
             led1_blinker_should_be_active = false;
             led_symptom_type_pin = 0;
        }
    } else {
         // If the blinker should not be active, ensure the LED is off.
         led_symptom_type_pin = 0;
    }
}

// --- LED2 Toggle Callback ---
// Called by the Timeout 'led2_flipper' to toggle LED2 state during blinking (medium intensity).
void toggle_led2() {
     if (led2_blinker_should_be_active) {
        // If the blinker is supposed to be active:
        led_intensity_pin = !led_intensity_pin; // Toggle the LED state
        // Reschedule the next toggle using the fixed medium intensity interval
         if (current_led2_toggle_interval_us > 0) {
             led2_flipper.attach(mbed::callback(&toggle_led2), microseconds(current_led2_toggle_interval_us));
        } else {
            // Safety check: if interval is invalid, stop blinking and turn off LED
            led2_blinker_should_be_active = false;
            led_intensity_pin = 0;
        }
     } else {
          // If the blinker should not be active, ensure the LED is off.
          led_intensity_pin = 0;
     }
}