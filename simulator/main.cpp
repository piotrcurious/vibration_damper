#include "Arduino.h"
#include "driver/adc.h"
#include "driver/dac.h"
#include "arduinoFFT.h"
#include "freertos/FreeRTOS.h"
#include "driver/timer.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <fstream>

// --- Physics Simulation Parameters ---
const float FS = 4000.0f;
const float DT = 1.0f / FS;

// Secondary Path: Actuator to Error Sensor
const int SP_LEN = 16;
float secondary_path_coeffs[SP_LEN] = {0, 0, 0, 0, 0.1, 0.4, 0.6, 0.4, 0.1, 0, 0, 0, 0, 0, 0, 0};
float secondary_path_buffer[SP_LEN] = {0};
int secondary_path_idx = 0;

float disturbance_freq = 150.0f; // Hz
float t_global = 0;

float current_dac_value = 0; // -1.0 to 1.0
float current_adc_ref = 0;
float current_adc_err = 0;
float current_dist = 0;

// Update the plant simulation at each time step
void update_plant_sim() {
    float t = t_global;
    t_global += DT;

    // 1. Generate Disturbance
    current_dist = 0.5f * sin(2.0f * PI * disturbance_freq * t) +
                 0.2f * sin(2.0f * PI * 2 * disturbance_freq * t + 0.5);

    current_adc_ref = current_dist + 0.01f * ((float)rand() / RAND_MAX - 0.5f);

    // 2. Secondary Path simulation
    secondary_path_buffer[secondary_path_idx] = current_dac_value;
    float plant_output = 0;
    for (int i = 0; i < SP_LEN; ++i) {
        plant_output += secondary_path_coeffs[i] * secondary_path_buffer[(secondary_path_idx - i + SP_LEN) % SP_LEN];
    }
    secondary_path_idx = (secondary_path_idx + 1) % SP_LEN;

    // 3. Error Signal: Disturbance + Actuator response
    current_adc_err = current_dist + plant_output + 0.01f * ((float)rand() / RAND_MAX - 0.5f);
}

// --- Mock Implementations ---
SerialMock Serial;

void adc1_config_width(adc_bits_width_t width_bit) {}
void adc1_config_channel_atten(adc1_channel_t channel, adc_atten_t atten) {}

int adc1_get_raw(adc1_channel_t channel) {
    if (channel == ADC1_CHANNEL_6) { // Reference
        int val = (int)(current_adc_ref * 2048.0f) + 2048;
        return std::max(0, std::min(4095, val));
    } else if (channel == ADC1_CHANNEL_7) { // Error
        int val = (int)(current_adc_err * 2048.0f) + 2048;
        return std::max(0, std::min(4095, val));
    }
    return 2048;
}

void dac_output_enable(dac_channel_t channel) {}
void dac_output_voltage(dac_channel_t channel, uint8_t voltage) {
    current_dac_value = (float)(voltage - 128) / 127.0f;
}

// --- ESP32 specific types and macros for the .ino code ---
#define GPIO_NUM_2 2
#define GPIO_MODE_OUTPUT 1
void gpio_reset_pin(int pin) {}
void gpio_set_direction(int pin, int mode) {}
void gpio_set_level(int pin, int level) {}

// Include the firmware code
#include "../src/avd_esp32.ino"

// --- Simulation Loop ---
void run_simulation(int steps, std::ofstream& log) {
    for (int step = 0; step < steps; ++step) {
        update_plant_sim();

        // 4. Run Firmware Iteration (ISR)
        if (!sysid_running) {
            onTimer();
        }

        if (step % 40 == 0) {
            log << t_global << "," << current_dist << "," << current_dac_value << "," << current_adc_err << "," << v_rms_e << std::endl;
        }
    }
}

int main() {
    setup();

    std::ofstream log("sim_log.csv");
    log << "Time,Disturbance,Actuator,Error,RMS_E" << std::endl;

    std::cout << "Starting simulation with default S_hat (unit delay)..." << std::endl;
    run_simulation(8000, log); // 2 seconds

    std::cout << "Running SYSID..." << std::endl;
    identifySecondaryPath();

    std::cout << "Resuming simulation with identified S_hat..." << std::endl;
    run_simulation(12000, log); // 3 more seconds

    log.close();
    std::cout << "Simulation complete. Log written to sim_log.csv" << std::endl;

    return 0;
}
