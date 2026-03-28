/**
 * ╔══════════════════════════════════════════════════════════════════════════╗
 * ║          ESP32  Active Vibration Damping System  v1.0                  ║
 * ║          Algorithm: FxLMS (Filtered-x Least Mean Squares)              ║
 * ╠══════════════════════════════════════════════════════════════════════════╣
 * ║  WHY FxLMS INSTEAD OF FFT-BASED PHASE INVERSION                        ║
 * ║  ─────────────────────────────────────────────────────────────────────  ║
 * ║  The original approach collected 256 samples, ran a batch FFT, then    ║
 * ║  output a counter-phase sine. This has two fatal flaws for active AVC: ║
 * ║   1. 256 ms batch latency → cancellation signal always phase-stale.    ║
 * ║   2. Assumes a single dominant sinusoid; real vibration is broadband.  ║
 * ║                                                                         ║
 * ║  FxLMS is sample-by-sample adaptive: it converges a full FIR filter   ║
 * ║  W[n] that cancels arbitrary broadband disturbances in real-time,      ║
 * ║  including harmonics, transients, and frequency-drifting sources.      ║
 * ║  The "Filtered-x" correction ensures the LMS gradient is unbiased      ║
 * ║  despite the electromechanical delay in the secondary path S           ║
 * ║  (actuator → error sensor), which would otherwise cause divergence.   ║
 * ╠══════════════════════════════════════════════════════════════════════════╣
 * ║  SIGNAL FLOW                                                            ║
 * ║  ─────────────────────────────────────────────────────────────────────  ║
 * ║                                                                         ║
 * ║   disturbance d[n] ──────────────────────────────────────►(+)──► e[n] ║
 * ║                                                            ▲            ║
 * ║   x[n] ──► [W adaptive FIR] ──► y[n] ──► [S actuator] ───┘            ║
 * ║    │                                                                    ║
 * ║    └──► [Ŝ secondary path model] ──► x'[n] ──► LMS ◄── e[n]          ║
 * ║                                                                         ║
 * ║  Weight update:   w[i] += -μ · e[n] · x'[n−i]                         ║
 * ║  NLMS step size:  μ_n  =  μ₀ / (ε + L · E[x'²])                       ║
 * ╠══════════════════════════════════════════════════════════════════════════╣
 * ║  HARDWARE                                                               ║
 * ║  ─────────────────────────────────────────────────────────────────────  ║
 * ║  GPIO34  ADC1_CH6  Reference accelerometer  (e.g. ADXL335 X-out)       ║
 * ║  GPIO35  ADC1_CH7  Error accelerometer      (same type, at target)     ║
 * ║  GPIO25  DAC_CH1   Actuator drive signal    (voice coil / piezo amp)   ║
 * ║  GPIO2             Status LED (built-in on most ESP32 dev boards)      ║
 * ║                                                                         ║
 * ║  Accelerometer wiring (ADXL335 example):                               ║
 * ║    VCC → 3.3V,  GND → GND,  Xout → GPIO34 or GPIO35                   ║
 * ║    Add 100 nF ceramic cap from each Xout to GND near the ESP32 pin.   ║
 * ║                                                                         ║
 * ║  Single-sensor mode: bridge GPIO34 and GPIO35 (same accelerometer).   ║
 * ║  Two-sensor mode is strongly preferred for correct FxLMS operation.    ║
 * ╠══════════════════════════════════════════════════════════════════════════╣
 * ║  REQUIRED LIBRARIES                                                     ║
 * ║    arduinoFFT  v1.x  (Tools → Manage Libraries → "arduinoFFT")         ║
 * ║  BOARD:  ESP32 Dev Module, 240 MHz, PSRAM off                          ║
 * ╚══════════════════════════════════════════════════════════════════════════╝
 */

#include <Arduino.h>
#include <driver/adc.h>
#include <driver/dac.h>
#include <arduinoFFT.h>   // v1.x API
#include <vector>
#include <algorithm>
#include <cmath>


// ════════════════════════════════════════════════════════════════════════════
//  CONFIGURATION — edit these to tune the system
// ════════════════════════════════════════════════════════════════════════════

// ── Sampling ─────────────────────────────────────────────────────────────
static constexpr int   SAMPLE_RATE      = 4000;                      // Hz — covers 0–2 kHz
static constexpr int   SAMPLE_PERIOD_US = 1000000 / SAMPLE_RATE;     // µs per tick

// ── Filter sizes (must be powers of 2 for bitmask-based circular buffers) ─
static constexpr int   W_TAPS  = 64;     // Reduced FIR size (now used for residual broadband)
static constexpr int   S_TAPS  = 64;     // Secondary-path model Ŝ length
static constexpr int   NUM_SOGI = 6;     // Number of parallel SOGI resonators

// ── FxLMS algorithm ───────────────────────────────────────────────────────
static constexpr float MU0             = 0.01f;    // FIR base step size
static constexpr float MU_SOGI         = 0.03f;    // SOGI step size
static constexpr float MU_CEIL         = 0.3f;     // Absolute ceiling on μ_n
static constexpr float LEAKAGE         = 0.9995f;  // Weights leakage to prevent drift
static constexpr float POWER_ALPHA     = 0.999f;   // IIR smoothing for NLMS power estimate
static constexpr float POWER_FLOOR     = 1e-8f;    // Prevents divide-by-zero

// ── Divergence Protection ────────────────────────────────────────────────
static constexpr float DIV_THRESHOLD   = 2.5f;     // Ratio of current RMS to baseline
static constexpr int   DIV_CHECK_MS    = 1000;     // Initial convergence window

// ── DC-blocking first-order HPF: cutoff ≈ Fs·(1−α)/(2π) ─────────────────
static constexpr float DC_ALPHA        = 0.995f;   // ≈ 3 Hz at 4 kHz

// ── ADC / DAC ─────────────────────────────────────────────────────────────
static constexpr int   ADC_MIDPOINT    = 2048;
static constexpr float ADC_NORM        = 1.0f / 2048.0f;  // → ±1.0 normalised
static constexpr int   DAC_MIDPOINT    = 128;              // 8-bit mid-rail = 0 V AC
static constexpr float DAC_SCALE       = 127.0f;           // ±1.0 → ±127 DAC counts

// ── GPIO ──────────────────────────────────────────────────────────────────
static constexpr dac_channel_t DAC_ACT = DAC_CHANNEL_1;   // GPIO25

// ── Telemetry ─────────────────────────────────────────────────────────────
static constexpr int   TELEM_HZ        = 50;    // Serial CSV rate
static constexpr int   PEAK_HZ         = 4;     // Peak-frequency report rate
static constexpr int   FFT_SIZE        = 512;   // Error-signal spectrum window


// ════════════════════════════════════════════════════════════════════════════
//  SOGI-QSG (Second-Order Generalized Integrator - Quadrature Signal Generator)
//  Acts as a bandpass filter that provides in-phase and quadrature components.
// ════════════════════════════════════════════════════════════════════════════
struct SogiQSG {
    float x1, x2;       // Delay line for input
    float us1, us2;     // Delay line for BP output
    float uc1, uc2;     // Delay line for QP output
    float k = 0.1f;     // Gain (damping) - lower is narrower
    float f_center;     // Hz
    float b0, a1, a2;   // Coefficients for BP output (u_s)
    float qb0, qb1, qb2;// Coefficients for QP output (u_c)

    // Secondary path compensation at f_center
    float s_real, s_imag;
    bool  pending_update = false;
    float next_f = 0;

    // Adaptive weights for this resonator
    float w_s, w_c;

    void reset() {
        x1 = x2 = us1 = us2 = uc1 = uc2 = 0;
        w_s = w_c = 0;
        f_center = 0;
        pending_update = false;
    }

    // Initialize coefficients using Tustin transform
    void updateFreq(float freq, float fs, const float* s_hat_coeffs) {
        if (freq < 10.0f) { f_center = 0; return; }
        f_center = freq;
        float wc = 2.0f * PI * freq;
        float T = 1.0f / fs;

        // Tustin: s -> (2/T) * (z-1)/(z+1)
        float theta = wc * T / 2.0f;
        float th2 = theta * theta;
        float k_th = k * theta;
        float den = 1.0f + k_th + th2;

        // Bandpass (D(s) = k*wc*s / (s^2 + k*wc*s + wc^2))
        b0 = k_th / den;
        // b1 = 0, b2 = -b0
        a1 = 2.0f * (th2 - 1.0f) / den;
        a2 = (1.0f - k_th + th2) / den;

        // Quadrature (Q(s) = k*wc^2 / (s^2 + k*wc*s + wc^2))
        qb0 = k * th2 / den; // Q(z) = k*th^2*(1+z^-1)^2 / den(z)
        qb1 = 2.0f * qb0;
        qb2 = qb0;

        // Compute Secondary Path Frequency Response at wc
        // S(wc) = sum( s_hat[k] * exp(-j * wc * k * T) )
        s_real = 0; s_imag = 0;
        for (int i = 0; i < S_TAPS; i++) {
            float angle = wc * i * T;
            s_real += s_hat_coeffs[i] * cosf(angle);
            s_imag -= s_hat_coeffs[i] * sinf(angle);
        }
    }

    // Process one sample and return [u_s, u_c]
    IRAM_ATTR void process(float x, float& u_s, float& u_c) {
        if (f_center < 1.0f) { u_s = u_c = 0; return; }

        u_s = b0*(x - x2) - a1*us1 - a2*us2;
        u_c = qb0*x + qb1*x1 + qb2*x2 - a1*uc1 - a2*uc2;

        x2 = x1; x1 = x;
        us2 = us1; us1 = u_s;
        uc2 = uc1; uc1 = u_c;
    }
};

// ════════════════════════════════════════════════════════════════════════════
//  CIRCULAR DELAY LINE
//  Power-of-2 size → bitmask wrap, zero memmove in ISR
// ════════════════════════════════════════════════════════════════════════════
template<int N>
struct DelayLine {
    static_assert((N & (N-1)) == 0, "DelayLine size must be a power of 2");
    static constexpr int MASK = N - 1;

    float buf[N];
    int   head;

    void reset() { memset(buf, 0, sizeof(buf)); head = 0; }

    // Insert newest sample; head advances modulo N
    IRAM_ATTR inline void push(float x) {
        buf[head] = x;
        head = (head + 1) & MASK;
    }

    // tap(0) = most recent,  tap(k) = k samples ago
    IRAM_ATTR inline float tap(int k) const {
        return buf[(head - 1 - k + N) & MASK];
    }

    // FIR dot product: Σ coeff[i] · tap(i),  i = 0..N-1
    IRAM_ATTR float dot(const float* coeff) const {
        float acc = 0.0f;
        for (int i = 0; i < N; i++) acc += coeff[i] * tap(i);
        return acc;
    }
};


// ════════════════════════════════════════════════════════════════════════════
//  GLOBAL STATE
// ════════════════════════════════════════════════════════════════════════════

// ── Adaptive filter and secondary path model ──────────────────────────────
static float            w[W_TAPS]    = {0};   // Cancellation filter weights W
static float            s_hat[S_TAPS]= {0};   // Secondary path estimate Ŝ

// ── Delay lines ───────────────────────────────────────────────────────────
static DelayLine<W_TAPS> dl_x;    // Reference x[n] → FIR W
static DelayLine<W_TAPS> dl_xf;   // Filtered reference x'[n] → LMS update
static DelayLine<S_TAPS> dl_s;    // Reference x[n] → FIR Ŝ (produces x'[n])

// ── SOGI Bank ─────────────────────────────────────────────────────────────
static SogiQSG sogi_bank[NUM_SOGI];
static float   sogi_freqs[NUM_SOGI] = {0};

// ── DC blocking ───────────────────────────────────────────────────────────
static float dc_ref = 0.0f, dc_err = 0.0f;

// ── NLMS power estimate of filtered reference ─────────────────────────────
static float xf_power = POWER_FLOOR;

// ── Runtime-adjustable step size ─────────────────────────────────────────
static volatile float v_mu = MU0;
static float         baseline_rms = 0.0f;
static uint32_t      start_time   = 0;

// ── RMS metrics  (ISR → telemetry task) ───────────────────────────────────
static constexpr int RMS_BLOCK = SAMPLE_RATE / 10;   // 100 ms block
static float         rms_e_acc = 0.0f, rms_y_acc = 0.0f;
static int           rms_cnt   = 0;
static volatile float v_rms_e  = 0.0f, v_rms_y = 0.0f;
static volatile bool  v_rms_rdy = false;

// ── FFT double-buffer (ISR writes, FFT task reads) ────────────────────────
static float         fft_bufA[FFT_SIZE], fft_bufB[FFT_SIZE];
static float         fft_spectrum[FFT_SIZE / 2];
static volatile int  fft_wr_sel = 0;   // 0 → write to A, 1 → write to B
static volatile int  fft_wr_idx = 0;
static volatile bool fft_ready  = false;

// ── Hardware timer ────────────────────────────────────────────────────────
static hw_timer_t*   htimer = nullptr;
static portMUX_TYPE  isr_mux = portMUX_INITIALIZER_UNLOCKED;

// ── SYSID flag: ISR yields during secondary-path identification ───────────
static volatile bool sysid_running = false;


// ════════════════════════════════════════════════════════════════════════════
//  TIMER ISR  — full FxLMS control loop at SAMPLE_RATE Hz  (Core 1)
// ════════════════════════════════════════════════════════════════════════════
void IRAM_ATTR onTimer() {
    if (sysid_running) return;

    portENTER_CRITICAL_ISR(&isr_mux);

    // ── 1. Acquire & normalise ────────────────────────────────────────────
    float x_n = (adc1_get_raw(ADC1_CHANNEL_6) - ADC_MIDPOINT) * ADC_NORM;
    float e_n = (adc1_get_raw(ADC1_CHANNEL_7) - ADC_MIDPOINT) * ADC_NORM;

    // ── 2. DC-block both channels (first-order HPF) ───────────────────────
    //   y[n] = x[n] - lpf[n]    where  lpf[n] = α·lpf[n-1] + (1-α)·x[n]
    {
        float lp_new  = DC_ALPHA * dc_ref + (1.0f - DC_ALPHA) * x_n;
        x_n          -= lp_new;
        dc_ref        = lp_new;
    }
    {
        float lp_new  = DC_ALPHA * dc_err + (1.0f - DC_ALPHA) * e_n;
        e_n          -= lp_new;
        dc_err        = lp_new;
    }

    // ── 3. Hybrid Output Synthesis (SOGI Bank + FIR) ──────────────────────
    float y_n = 0;

    // a. SOGI Tonal Cancellation
    for (int i = 0; i < NUM_SOGI; i++) {
        // Safe frequency update from background task
        if (sogi_bank[i].pending_update) {
            sogi_bank[i].updateFreq(sogi_bank[i].next_f, SAMPLE_RATE, s_hat);
            sogi_bank[i].pending_update = false;
        }

        if (sogi_bank[i].f_center < 10.0f) continue;

        float u_s, u_c;
        sogi_bank[i].process(x_n, u_s, u_c);

        // Output contribution: y[n] = Σ (w_s*u_s + w_c*u_c)
        y_n += sogi_bank[i].w_s * u_s + sogi_bank[i].w_c * u_c;

        // Adaptive Weight Update (FxLMS)
        // Steady-state Filtered-x signals for a sinusoid at f_center:
        // u_s is cos-like, u_c is sin-like.
        // Filtered-x: u_sf = u_s*R - u_c*I,  u_cf = u_c*R + u_s*I
        float u_sf = sogi_bank[i].s_real * u_s - sogi_bank[i].s_imag * u_c;
        float u_cf = sogi_bank[i].s_real * u_c + sogi_bank[i].s_imag * u_s;

        // Normalization for SOGI (based on input power)
        float sogi_mu = MU_SOGI / (0.1f + u_sf*u_sf + u_cf*u_cf);

        // Update weights
        sogi_bank[i].w_s = LEAKAGE * sogi_bank[i].w_s - sogi_mu * e_n * u_sf;
        sogi_bank[i].w_c = LEAKAGE * sogi_bank[i].w_c - sogi_mu * e_n * u_cf;
    }

    // b. Broadband FIR Contribution
    dl_x.push(x_n);
    y_n += dl_x.dot(w);

    // Clip to ±1.0 (hard limiter; protects actuator)
    if      (y_n >  1.0f) y_n =  1.0f;
    else if (y_n < -1.0f) y_n = -1.0f;

    // ── 4. Drive actuator ─────────────────────────────────────────────────
    const int dac_out = DAC_MIDPOINT + (int)(y_n * DAC_SCALE);
    dac_output_voltage(DAC_ACT, (uint8_t)(dac_out < 0 ? 0 : dac_out > 255 ? 255 : dac_out));

    // ── 5. Weight Update (FxLMS) for FIR ──────────────────────────────────
    dl_s.push(x_n);
    const float xf_n = dl_s.dot(s_hat);
    dl_xf.push(xf_n);

    // FIR Update (Normalized LMS)
    xf_power = POWER_ALPHA * xf_power + (1.0f - POWER_ALPHA) * xf_n * xf_n;
    float mu_n = v_mu / (POWER_FLOOR + xf_power * W_TAPS);
    if (mu_n > MU_CEIL) mu_n = MU_CEIL;

    for (int i = 0; i < W_TAPS; i++) {
        w[i] = LEAKAGE * w[i] - mu_n * e_n * dl_xf.tap(i);
    }

    // ── 7. RMS accumulation (100 ms block) ────────────────────────────────
    rms_e_acc += e_n * e_n;
    rms_y_acc += y_n * y_n;
    if (++rms_cnt >= RMS_BLOCK) {
        v_rms_e   = sqrtf(rms_e_acc / RMS_BLOCK);
        v_rms_y   = sqrtf(rms_y_acc / RMS_BLOCK);
        rms_e_acc = rms_y_acc = 0.0f;
        rms_cnt   = 0;
        v_rms_rdy = true;
    }

    // ── 8. Double-buffered FFT capture (error signal) ─────────────────────
    float* fft_wr = (fft_wr_sel == 0) ? fft_bufA : fft_bufB;
    if (fft_wr_idx < FFT_SIZE) {
        fft_wr[fft_wr_idx++] = e_n;
    } else {
        fft_ready  = true;       // Signal FFT task to process completed buffer
        fft_wr_sel ^= 1;         // Swap buffers
        fft_wr_idx  = 0;
    }

    portEXIT_CRITICAL_ISR(&isr_mux);
}


// ════════════════════════════════════════════════════════════════════════════
//  SECONDARY PATH IDENTIFICATION (SYSID)
//
//  Injects a logarithmic chirp through the actuator, captures the
//  response at the error sensor, and estimates Ŝ via cross-correlation.
//  The chirp provides better SNR across the bandwidth than white noise.
//
//  Call with no external vibration present. Takes ~1.2 s.
//  Resets adaptive weights W after completion (Ŝ has changed).
// ════════════════════════════════════════════════════════════════════════════
static void identifySecondaryPath() {
    constexpr int   LEN = 4096;    // 1 second at 4 kHz
    constexpr float AMP = 0.15f;   // Higher amplitude for chirp
    constexpr float F0  = 20.0f;   // Start freq
    constexpr float F1  = 1800.0f; // End freq

    Serial.println("[SYSID] Starting Chirp Sweep (20-1800 Hz)...");
    sysid_running = true;
    delay(100);

    static float probe[LEN], capture[LEN];

    // Generate Log-Chirp
    const float log_f = logf(F1 / F0);
    for (int n = 0; n < LEN; n++) {
        float t = (float)n / SAMPLE_RATE;
        float phi = 2.0f * PI * F0 * (LEN / (log_f * SAMPLE_RATE)) * (expf(t * log_f * SAMPLE_RATE / LEN) - 1.0f);
        probe[n] = AMP * sinf(phi);
    }

    // Inject and Capture
    for (int i = 0; i < LEN; i++) {
        const int dv = DAC_MIDPOINT + (int)(probe[i] * DAC_SCALE);
        dac_output_voltage(DAC_ACT, (uint8_t)(dv < 0 ? 0 : dv > 255 ? 255 : dv));
        delayMicroseconds(SAMPLE_PERIOD_US);
        capture[i] = (adc1_get_raw(ADC1_CHANNEL_7) - ADC_MIDPOINT) * ADC_NORM;
    }
    dac_output_voltage(DAC_ACT, DAC_MIDPOINT);

    // Cross-correlation
    float peak = 0.0f;
    const int N_CORR = LEN - S_TAPS;
    for (int k = 0; k < S_TAPS; k++) {
        float acc = 0.0f;
        for (int n = 0; n < N_CORR; n++) acc += probe[n] * capture[n + k];
        s_hat[k] = acc / N_CORR;
        if (fabsf(s_hat[k]) > peak) peak = fabsf(s_hat[k]);
    }

    if (peak > 1e-4f) {
        for (int k = 0; k < S_TAPS; k++) s_hat[k] /= peak;
        Serial.print("[SYSID] Ŝ Identified OK. Coefficients: ");
        for (int k = 0; k < S_TAPS; k++) Serial.printf("%.3f ", s_hat[k]);
        Serial.println("");
    } else {
        memset(s_hat, 0, sizeof(s_hat));
        s_hat[S_TAPS / 2] = 1.0f;
        Serial.println("[SYSID] Warning: Low SNR. Using unit delay fallback.");
    }

    // Reset W
    portENTER_CRITICAL(&isr_mux);
    memset(w, 0, sizeof(w));
    dl_x.reset(); dl_xf.reset(); dl_s.reset();
    xf_power = POWER_FLOOR;
    baseline_rms = 0.0f;
    start_time = millis();
    portEXIT_CRITICAL(&isr_mux);

    sysid_running = false;
    Serial.println("[SYSID] Control loop resumed.");
}


// ════════════════════════════════════════════════════════════════════════════
//  FFT TASK  (Core 0)
//  Computes magnitude spectrum of the error signal for diagnostics.
//  Does NOT affect the control loop.
// ════════════════════════════════════════════════════════════════════════════
static double fft_re[FFT_SIZE], fft_im[FFT_SIZE];

static void fftTask(void*) {
    static arduinoFFT fft;
#ifndef SIMULATOR
    while (true) {
#endif
        if (fft_ready) {
            fft_ready = false;
            // Read the buffer NOT currently being written by the ISR
            const float* src = (fft_wr_sel == 0) ? fft_bufB : fft_bufA;
            for (int i = 0; i < FFT_SIZE; i++) { fft_re[i] = src[i]; fft_im[i] = 0.0; }

            fft.Windowing(fft_re, FFT_SIZE, FFT_WIN_TYP_HANN, FFT_FORWARD);
            fft.Compute(fft_re, fft_im, FFT_SIZE, FFT_FORWARD);
            fft.ComplexToMagnitude(fft_re, fft_im, FFT_SIZE);

            const float norm = (float)(FFT_SIZE / 2);
            for (int i = 0; i < FFT_SIZE / 2; i++)
                fft_spectrum[i] = (float)(fft_re[i] / norm);

            // Tonal Inference: Find dominant peaks and update SOGI bank
            // 1. Find local peaks in the spectrum
            struct Peak { int bin; float mag; };
            std::vector<Peak> peaks;
            for (int i = 5; i < FFT_SIZE / 2 - 5; i++) { // Ignore DC and high-freq noise
                if (fft_spectrum[i] > fft_spectrum[i-1] && fft_spectrum[i] > fft_spectrum[i+1] && fft_spectrum[i] > 0.005f) {
                    peaks.push_back({i, fft_spectrum[i]});
                }
            }
            std::sort(peaks.begin(), peaks.end(), [](const Peak& a, const Peak& b) { return a.mag > b.mag; });

            // 2. Assign top 6 peaks to SOGI bank
            for (int i = 0; i < NUM_SOGI; i++) {
                if (i < (int)peaks.size()) {
                    float freq = peaks[i].bin * (float)SAMPLE_RATE / FFT_SIZE;
                    // If frequency has moved significantly (> 4 Hz), schedule update
                    if (fabsf(freq - sogi_bank[i].f_center) > 4.0f && !sogi_bank[i].pending_update) {
                        sogi_bank[i].next_f = freq;
                        sogi_bank[i].pending_update = true;
                    }
                } else if (!sogi_bank[i].pending_update) {
                    sogi_bank[i].next_f = 0;
                    sogi_bank[i].pending_update = true;
                }
            }
        }
#ifndef SIMULATOR
        vTaskDelay(pdMS_TO_TICKS(20));
    }
#endif
}


// ════════════════════════════════════════════════════════════════════════════
//  TELEMETRY TASK  (Core 0)
//  Streams metrics over Serial at TELEM_HZ.
//  Output format is CSV-friendly for live plotting (Python, Serial Plotter).
// ════════════════════════════════════════════════════════════════════════════
static void telemTask(void*) {
    static const int PERIOD_MS   = 1000 / TELEM_HZ;
    static const int PEAK_STRIDE = TELEM_HZ / PEAK_HZ;
    static int       peak_tick   = 0;

#ifndef SIMULATOR
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(PERIOD_MS));
#endif

        if (v_rms_rdy) {
            v_rms_rdy = false;

            // Divergence Detection Logic
            if (baseline_rms < 1e-6f && millis() - start_time > DIV_CHECK_MS) {
                baseline_rms = v_rms_e;
                Serial.printf("[PROT] Baseline RMS established: %.5f\n", baseline_rms);
            } else if (baseline_rms > 0.0f && v_rms_e > baseline_rms * DIV_THRESHOLD) {
                Serial.printf("[PROT] Divergence detected! RMS: %.5f > %.5f. Reducing MU.\n", v_rms_e, baseline_rms * DIV_THRESHOLD);
                v_mu *= 0.5f;
                // Reset weights
                portENTER_CRITICAL(&isr_mux);
                memset(w, 0, sizeof(w));
                dl_x.reset(); dl_xf.reset(); dl_s.reset();
                xf_power = POWER_FLOOR;
                portEXIT_CRITICAL(&isr_mux);
                baseline_rms = 0.0f; // Re-establish baseline
                start_time = millis();
            }

            Serial.printf("RMS_E:%.5f RMS_Y:%.5f MU:%.2e\n",
                          v_rms_e, v_rms_y, v_mu);
        }

        if (++peak_tick >= PEAK_STRIDE) {
            peak_tick = 0;
            // Report dominant frequency bin in the error spectrum
            float peak_mag = 0.0f; int peak_bin = 1;
            for (int i = 1; i < FFT_SIZE / 2; i++) {
                if (fft_spectrum[i] > peak_mag) { peak_mag = fft_spectrum[i]; peak_bin = i; }
            }
            const float peak_hz = peak_bin * (float)SAMPLE_RATE / FFT_SIZE;
            Serial.printf("PEAK: %.1f Hz  mag=%.5f\n", peak_hz, peak_mag);
        }
#ifndef SIMULATOR
    }
#endif
}


// ════════════════════════════════════════════════════════════════════════════
//  SERIAL COMMAND PARSER
//  All commands are case-insensitive.  Send via Serial Monitor at 115200 baud.
// ════════════════════════════════════════════════════════════════════════════
static void parseCmd(const String& raw) {
    String cmd = raw; cmd.trim(); cmd.toUpperCase();

    if (cmd.startsWith("MU ")) {
        // MU <value>  — set base step size.  Typical range: 1e-5 to 1e-2.
        // Larger μ → faster convergence but potential instability.
        // Smaller μ → slower but more robust.
        const float val = cmd.substring(3).toFloat();
        if (val > 0.0f && val <= 1.0f) {
            v_mu = val;
            Serial.printf("[CMD] μ₀ = %.2e\n", v_mu);
        } else {
            Serial.println("[CMD] Range: 0 < mu ≤ 1.0   example: MU 0.001");
        }

    } else if (cmd == "RESET") {
        // Clear adaptive weights and delay lines; restart from zero.
        // Use after changing the mechanical setup or after SYSID.
        portENTER_CRITICAL(&isr_mux);
        memset(w, 0, sizeof(w));
        dl_x.reset(); dl_xf.reset(); dl_s.reset();
        xf_power = POWER_FLOOR;
        for (int i = 0; i < NUM_SOGI; i++) sogi_bank[i].reset();
        portEXIT_CRITICAL(&isr_mux);
        Serial.println("[CMD] Filter weights and SOGI bank reset.");

    } else if (cmd == "SYSID") {
        // Estimate secondary path Ŝ (actuator → error sensor impulse response).
        // Run once after first power-on, or when the mechanical setup changes.
        identifySecondaryPath();

    } else if (cmd == "WEIGHTS") {
        // Dump all W coefficients — useful for offline analysis.
        Serial.println("[CMD] W (adaptive filter):");
        for (int i = 0; i < W_TAPS; i++)
            Serial.printf("  w[%02d] = %+.6f\n", i, w[i]);

    } else if (cmd == "SPATH") {
        // Dump secondary path model Ŝ.
        Serial.println("[CMD] Ŝ (secondary path model):");
        for (int i = 0; i < S_TAPS; i++)
            Serial.printf("  s[%02d] = %+.6f\n", i, s_hat[i]);

    } else if (cmd == "STATUS") {
        Serial.printf("[STATUS] RMS_E=%.5f  RMS_Y=%.5f  μ₀=%.2e  xf_pwr=%.2e\n",
                      v_rms_e, v_rms_y, v_mu, xf_power);

    } else if (cmd == "SPEC") {
        // Dump full error spectrum (frequency : magnitude CSV)
        Serial.println("[SPEC] Hz,Magnitude");
        for (int i = 1; i < FFT_SIZE / 2; i++) {
            float hz = i * (float)SAMPLE_RATE / FFT_SIZE;
            Serial.printf("%.1f,%.5f\n", hz, fft_spectrum[i]);
        }

    } else if (cmd == "HELP") {
        Serial.println(
            "╔─────────────────────────────────────────────╗\n"
            "║  Commands                                    ║\n"
            "║  MU <val>   Set step size  (e.g. MU 0.001)  ║\n"
            "║  RESET      Zero adaptive filter weights     ║\n"
            "║  SYSID      Identify secondary path Ŝ        ║\n"
            "║  WEIGHTS    Dump W coefficients              ║\n"
            "║  SPATH      Dump Ŝ coefficients              ║\n"
            "║  STATUS     Print RMS and μ                  ║\n"
            "║  SPEC       Dump error spectrum (CSV)        ║\n"
            "║  HELP       This message                     ║\n"
            "╚─────────────────────────────────────────────╝"
        );
    } else if (cmd.length() > 0) {
        Serial.println("[CMD] Unknown. Type HELP.");
    }
}


// ════════════════════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(400);
    Serial.println(
        "\n╔══════════════════════════════════════════════╗\n"
        "║   ESP32 Active Vibration Damping System      ║\n"
        "║   Hybrid SOGI-FxLMS | 4 kHz                  ║\n"
        "╚══════════════════════════════════════════════╝"
    );

    // ── GPIO: built-in LED ────────────────────────────────────────────────
    gpio_reset_pin(GPIO_NUM_2);
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_2, 0);

    // ── ADC: 12-bit resolution, 11 dB attenuation (0–3.3 V full range) ───
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);  // GPIO34
    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11);  // GPIO35

    // ── DAC: initialise to mid-rail (AC zero) ─────────────────────────────
    dac_output_enable(DAC_ACT);
    dac_output_voltage(DAC_ACT, DAC_MIDPOINT);

    // ── Default secondary path: pure delay of S_TAPS/2 samples ───────────
    memset(s_hat, 0, sizeof(s_hat));
    s_hat[S_TAPS / 2] = 1.0f;
    Serial.println("[INIT] Ŝ initialised to unit delay (run SYSID for accuracy).");

    // ── Delay lines & Baseline ───────────────────────────────────────────
    dl_x.reset(); dl_xf.reset(); dl_s.reset();
    start_time = millis();

    // ── Background tasks pinned to Core 0 (leaves Core 1 for control) ────
    xTaskCreatePinnedToCore(fftTask,  "FFT",   4096, nullptr, 1, nullptr, 0);
    xTaskCreatePinnedToCore(telemTask,"TELEM", 2048, nullptr, 1, nullptr, 0);

    // ── Hardware timer: 80 MHz base clock / prescaler 80 = 1 µs tick ─────
    htimer = timerBegin(0, 80, true);                        // Timer 0, div 80
    timerAttachInterrupt(htimer, &onTimer, true);             // Edge-triggered
    timerAlarmWrite(htimer, SAMPLE_PERIOD_US, true);          // Auto-reload
    timerAlarmEnable(htimer);

    gpio_set_level(GPIO_NUM_2, 1);   // LED on = running
    Serial.printf("[INIT] Timer running at %d Hz.  Type HELP for commands.\n", SAMPLE_RATE);
}


// ════════════════════════════════════════════════════════════════════════════
//  LOOP — Serial command handler (Core 1, low priority)
// ════════════════════════════════════════════════════════════════════════════
void loop() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        parseCmd(cmd);
    }
    delay(5);
}
