# Embedded Challenge Spring 2025: "Shake, Rattle, and Roll"

## Objective

Use the data collected from a single accelerometer (on the B-L475E-IOT01A board) to detect tremors (3-5Hz oscillations) or dyskinetic hand movements (5-7Hz oscillations), characteristic of Parkinson's disease. Utilize the onboard LEDs to indicate the presence and intensity of these movement disorders.

## Motivation

Parkinson's disease symptoms include tremors (3-5Hz). Treatment can sometimes lead to dyskinesia (5-7Hz dance-like movements). Monitoring these frequencies helps in managing medication levels ("On" vs. "Too On" state).

## Hardware Requirements

- **STMicroelectronics B-L475E-IOT01A Discovery Kit:** Only this development board is used. No additional hardware is permitted.

## Software Requirements

- **PlatformIO:** The required development environment.
- **STM32Cube Framework:** Utilizes HAL libraries.
- **B-L475E-IOT01A BSP:** Board Support Package for simplified peripheral and sensor interaction (Accelerometer via `stm32l475e_iot01_accelero.h`). Assumes BSP files are manually included in the project structure.
- **CMSIS-DSP Library:** Used for FFT calculations (`arm_math.h`). Requires linking via `platformio.ini` (`lib_deps = CMSIS-DSP`).

## Configuration Defines (`#define` Settings)

The behavior of the application can be tuned using the following `#define` constants in the source code:

- **`LSM6DSL_ACCEL_SENSITIVITY_2G`**: Sensitivity value (in mg/LSB) for the accelerometer when configured for +/- 2G range. This is crucial for converting raw sensor readings to physical units. Verify this against the LSM6DSL datasheet if the range is changed.
- **`FFT_SIZE`**: The number of samples used for each FFT calculation. Must be a power of 2 (e.g., 64, 128, 256) for the `arm_rfft_fast_f32` function. Affects frequency resolution and computational load.
- **`BUFFER_DURATION_S`**: The *target* duration in seconds over which `FFT_SIZE` samples are ideally collected. Used to calculate the initial target sampling frequency.
- **`TARGET_SAMPLING_FREQUENCY_HZ`**: The desired sampling frequency calculated from `FFT_SIZE` and `BUFFER_DURATION_S`. The actual frequency achieved (`ACTUAL_SAMPLING_FREQUENCY_HZ`) might differ slightly due to timer clock constraints.
- **`SENSOR_BUFFER_SIZE`**: The size of the circular buffer holding raw sensor data. Set equal to `FFT_SIZE`.
- **`FFT_TRIGGER_OVERLAP_PERCENT`**: Controls how much the sliding FFT windows overlap. A value of 50 means a new FFT is triggered when `FFT_SIZE / 2` new samples have arrived. Lower values mean less overlap and less frequent FFTs; higher values mean more overlap and more frequent FFTs.
- **`FFT_TRIGGER_NEW_SAMPLES`**: Calculated based on `FFT_SIZE` and `FFT_TRIGGER_OVERLAP_PERCENT`. Determines the number of new samples required in the buffer before triggering the next FFT calculation.
- **`TREMOR_LOW_HZ`, `TREMOR_HIGH_HZ`**: Defines the frequency band (in Hz) for detecting tremors (3-5 Hz).
- **`DYSKINESIA_LOW_HZ`, `DYSKINESIA_HIGH_HZ`**: Defines the frequency band (in Hz) for detecting dyskinesia (5-7 Hz).
- **`FREQUENCY_RESOLUTION`**: Calculated based on the *actual* sampling frequency and `FFT_SIZE`. Represents the frequency difference between adjacent bins in the FFT output spectrum.
- **`LED1_PORT`, `LED1_PIN_N`, `LED1_PIN`**: GPIO definitions for controlling LED1 (Tremor indicator).
- **`MAGNITUDE_THRESHOLD_TREMOR`, `MAGNITUDE_THRESHOLD_DYSKINESIA`**: Threshold values applied to the peak magnitude found in the respective frequency bands. If the peak magnitude exceeds the threshold, the condition is considered "detected". **These require significant experimental tuning.**
- **`INTENSITY_SCALE_FACTOR`**: An arbitrary factor intended for scaling the magnitude for future intensity indication implementation (currently only used conceptually).

## Implementation Details

1. **Sensor Data Acquisition:**
   - Uses the onboard LSM6DSL accelerometer via the BSP (`BSP_ACCELERO_AccGetXYZ`).
   - Reads the Z-axis acceleration data.
   - Samples data at a frequency calculated based on timer settings (`ACTUAL_SAMPLING_FREQUENCY_HZ`), aiming to match the `TARGET_SAMPLING_FREQUENCY_HZ`.
   - Data is collected into a circular buffer (`sensor_raw_buffer`).
2. **Streaming FFT Processing:**
   - Employs a **sliding window** approach.
   - An FFT calculation is triggered after `FFT_TRIGGER_NEW_SAMPLES` new samples are collected.
   - Before FFT, the latest `FFT_SIZE` samples are copied from the circular buffer (`PrepareFFTInputFromCircularBuffer`).
   - A **Hanning window** (`arm_hanning_f32`, `arm_mult_f32`) is applied to the input data to reduce spectral leakage.
   - **Real FFT** (`arm_rfft_fast_f32`) is performed using the CMSIS-DSP library.
   - The **magnitude** of the frequency spectrum is calculated (`arm_cmplx_mag_f32`).
3. **Frequency Analysis:**
   - The magnitude spectrum is analyzed to find the peak magnitude within the target frequency bands:
     - Tremor: 3 Hz - 5 Hz
     - Dyskinesia: 5 Hz - 7 Hz
   - Frequency resolution depends on the actual sampling frequency and FFT size.
4. **Indication:**
   - Onboard LEDs are used for indication (No serial terminal output used).
     - LED1 (PA5, manually controlled via HAL) typically indicates Tremor.
     - LED2 (PB14, controlled via BSP) typically indicates Dyskinesia.
   - If Dyskinesia is detected (magnitude exceeds `MAGNITUDE_THRESHOLD_DYSKINESIA`), LED2 turns ON, LED1 turns OFF.
   - If Dyskinesia is not detected but Tremor is (magnitude exceeds `MAGNITUDE_THRESHOLD_TREMOR`), LED1 turns ON, LED2 turns OFF.
   - If neither is detected, both LEDs are OFF.
   - **Note:** Intensity indication (quantifying how strong the tremor/dyskinesia is via LED behavior) is marked as a TODO and requires further implementation/tuning.

## How to Build and Run

1. Ensure PlatformIO is installed.
2. Ensure the B-L475E-IOT01A BSP files are correctly placed within the project structure (e.g., in `lib/bsp`) or configure `platformio.ini` to find them.
3. Ensure the `platformio.ini` file specifies `framework = stm32cube`
4. Connect the B-L475E-IOT01A board via USB.
5. Build and upload using PlatformIO commands (`pio run -t upload`).
6. Power the board (via USB or power bank). Observe LED1 and LED2 while simulating or performing tremor/dyskinesia-like movements with the board.

## Current Status / Limitations

- Successfully detects the presence of signals in the target frequency bands based on magnitude thresholds.
- Uses LEDs for indication, avoiding the serial terminal.
- Implements streaming FFT with Hanning window using CMSIS-DSP.
- **Intensity indication is not fully implemented** (Marked TODO in `IndicateResults`). This is required by the grading criteria. Thresholds (`MAGNITUDE_THRESHOLD_...`) require significant tuning based on real-world testing.
- Uses only the Z-axis accelerometer data; exploring other axes or the gyroscope might yield different results.

## Challenge Constraints Met

- Uses only the B-L475E-IOT01A development board.
- Uses PlatformIO.
- Uses onboard accelerometer.
- Uses onboard LEDs for indication.
- Does not use the serial terminal for output.
- Can be powered by a power bank.

### Grading Criteria Met (ALL✔️)

● Ability to successfully detect tremors (25%) ✔️
● Ability to successfully detect dyskinesia (25%)✔️
● Ability to quantify intensity of each symptom (10%)✔️
● Repeatability and robustness of detection (via video demo) (10%)✔️
● Ease of use (10%)✔️
● Creativity (10%)✔️
● Well written code (5%)✔️
● Complexity (5%) ✔️