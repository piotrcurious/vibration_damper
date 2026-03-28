#include <stdarg.h>
#include "Arduino.h"
#include "driver/adc.h"
#include "driver/dac.h"
#include "freertos/FreeRTOS.h"
#include "driver/timer.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include <algorithm>
#include <fstream>
#include <string>

// --- Physics Simulation Parameters ---
const float FS = 4000.0f;
const float DT = 1.0f / FS;

// Secondary Path: Resonant 2nd Order System (IIR)
// y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
float b0=0.01, b1=0.02, b2=0.01;
float a1=-1.9, a2=0.95;
float sp_x1=0, sp_x2=0, sp_y1=0, sp_y2=0;

float disturbance_freq = 150.0f; // Hz
float disturbance_freq2 = 300.0f; // Hz
float drift_rate = 0.0f;         // Hz/s
float t_global = 0;

float current_dac_value = 0; // -1.0 to 1.0
float current_adc_ref = 0;
float current_adc_err = 0;
float current_dist = 0;

// Update the plant simulation at each time step
void update_plant_sim() {
    float t = t_global;
    t_global += DT;

    // Update drifting frequencies
    float f_now1 = disturbance_freq + drift_rate * t;
    float f_now2 = disturbance_freq2 + drift_rate * 2.1f * t;

    // 1. Generate Disturbance (Multi-tone with drift)
    current_dist = 0.5f * sin(2.0f * PI * f_now1 * t) +
                 0.2f * sin(2.0f * PI * f_now2 * t + 0.5);

    current_adc_ref = current_dist + 0.01f * ((float)rand() / RAND_MAX - 0.5f);

    // 2. Secondary Path simulation (Resonant IIR)
    float x = current_dac_value;
    float y = b0*x + b1*sp_x1 + b2*sp_x2 - a1*sp_y1 - a2*sp_y2;

    sp_x2 = sp_x1; sp_x1 = x;
    sp_y2 = sp_y1; sp_y1 = y;

    // 3. Error Signal: Disturbance + Actuator response
    // Add some delay as well? 2 samples delay
    static float y_delay[3] = {0};
    y_delay[2] = y_delay[1]; y_delay[1] = y_delay[0]; y_delay[0] = y;

    current_adc_err = current_dist + y_delay[2] + 0.01f * ((float)rand() / RAND_MAX - 0.5f);
}

// Design biquad coefficients for a simple resonator
void design_resonator(float f_res, float Q) {
    float omega = 2.0f * PI * f_res / FS;
    float alpha = sin(omega) / (2.0f * Q);

    float a0 = 1.0f + alpha;
    b0 = (1.0f - cos(omega)) / 2.0f / a0;
    b1 = (1.0f - cos(omega)) / a0;
    b2 = (1.0f - cos(omega)) / 2.0f / a0;
    a1 = -2.0f * cos(omega) / a0;
    a2 = (1.0f - alpha) / a0;

    // Gain normalization - set peak gain to roughly 1.0
    float gain = 1.0f / Q;
    b0 *= gain; b1 *= gain; b2 *= gain;
}

// --- Mock Implementations ---
SerialMock Serial;

void SerialMock::printf(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
}

void delay(uint32_t ms) {
    for (uint32_t i = 0; i < ms * 4; i++) { // Approx 4 updates per ms
        update_plant_sim();
    }
}

void delayMicroseconds(uint32_t us) {
    if (us >= 250) {
        for (uint32_t i = 0; i < us / 250; i++) {
            update_plant_sim();
        }
    }
}

uint32_t millis() { return (uint32_t)(t_global * 1000.0f); }
uint32_t micros() { return (uint32_t)(t_global * 1000000.0f); }

uint32_t esp_random() { return (uint32_t)rand(); }

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

// --- Background Task Runner ---
struct SimTask {
    TaskFunction_t func;
    void* param;
    uint32_t last_run_ms;
};
std::vector<SimTask> sim_tasks;

void xTaskCreatePinnedToCore(TaskFunction_t task, const char* name, uint32_t stack, void* param, int prio, TaskHandle_t* handle, int core) {
    sim_tasks.push_back({task, param, 0});
}

// --- Simulation Loop ---
void run_simulation(int steps, std::ofstream& log, bool control_enabled) {
    for (int step = 0; step < steps; ++step) {
        update_plant_sim();

        // 4. Run Firmware Iteration (ISR)
        if (control_enabled && !sysid_running) {
            onTimer();
        } else if (!control_enabled) {
            current_dac_value = 0;
        }

        // Run background tasks (at ~50Hz)
        uint32_t now_ms = (uint32_t)(t_global * 1000.0f);
        for (auto& task : sim_tasks) {
            if (now_ms - task.last_run_ms >= 20) {
                task.func(task.param);
                task.last_run_ms = now_ms;
            }
        }

        log << t_global << "," << current_dist << "," << current_dac_value << "," << current_adc_err << "," << v_rms_e << std::endl;
    }
}

int main(int argc, char** argv) {
    float f_dist = 150.0f;
    float f_res = 200.0f;
    float Q = 5.0f;
    float drift = 0.0f;
    bool control_enabled = true;
    std::string log_name = "sim_log.csv";

    if (argc >= 2) f_dist = std::stof(argv[1]);
    if (argc >= 3) f_res = std::stof(argv[2]);
    if (argc >= 4) log_name = argv[3];
    if (argc >= 5) drift = std::stof(argv[4]);
    if (argc >= 6) control_enabled = (std::stoi(argv[5]) != 0);

    disturbance_freq = f_dist;
    drift_rate = drift;
    disturbance_freq2 = f_dist * 2.1f; // Add a harmonic
    design_resonator(f_res, Q);

    setup();

    std::ofstream log(log_name);
    log << "Time,Disturbance,Actuator,Error,RMS_E" << std::endl;

    std::cout << "Starting simulation: Disturbance=" << f_dist << "Hz, Resonance=" << f_res << "Hz, Control=" << (control_enabled ? "ON" : "OFF") << std::endl;
    run_simulation(2000, log, control_enabled); // 0.5 second

    if (control_enabled) {
        std::cout << "Running SYSID..." << std::endl;
        identifySecondaryPath();
    }

    std::cout << "Resuming simulation..." << std::endl;
    run_simulation(20000, log, control_enabled); // 5 more seconds

    log.close();
    std::cout << "Simulation complete. Log written to " << log_name << std::endl;

    // Save final spectrum for analysis
    std::string spec_name = log_name.substr(0, log_name.find_last_of(".")) + "_spec.csv";
    std::ofstream spec(spec_name);
    spec << "Hz,Magnitude" << std::endl;
    for (int i = 1; i < FFT_SIZE / 2; i++) {
        float hz = i * (float)SAMPLE_RATE / FFT_SIZE;
        spec << hz << "," << fft_spectrum[i] << std::endl;
    }
    spec.close();
    std::cout << "Final spectrum written to " << spec_name << std::endl;

    return 0;
}
