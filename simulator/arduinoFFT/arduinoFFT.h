#include <vector>
#include <cmath>
#include <complex>
#include <algorithm>

#define FFT_WIN_TYP_HANN 0
#define FFT_FORWARD 0
#define FFT_WIN_TYP_HAMMING 1

class arduinoFFT {
public:
    void Windowing(double* vReal, uint16_t samples, uint8_t windowType, uint8_t direction) {
        if (windowType == FFT_WIN_TYP_HANN) {
            for (uint16_t i = 0; i < samples; i++) {
                vReal[i] *= 0.5 * (1.0 - cos(2.0 * M_PI * i / (samples - 1)));
            }
        }
    }

    void Compute(double* vReal, double* vImag, uint16_t samples, uint8_t direction) {
        if (direction != FFT_FORWARD) return;

        // Very naive DFT for simulation purposes (slow but works for small N like 512)
        // Since this runs in a separate thread/task and is not part of the time-critical loop, it's okay.
        // Or we could use a better FFT if needed. Let's use a simple Cooley-Tukey if we want speed.

        int n = samples;
        for (int i = 1, j = 0; i < n; i++) {
            int bit = n >> 1;
            for (; j & bit; bit >>= 1) j ^= bit;
            j ^= bit;
            if (i < j) {
                std::swap(vReal[i], vReal[j]);
                std::swap(vImag[i], vImag[j]);
            }
        }

        for (int len = 2; len <= n; len <<= 1) {
            double ang = 2.0 * M_PI / len * (direction == FFT_FORWARD ? -1 : 1);
            std::complex<double> wlen(cos(ang), sin(ang));
            for (int i = 0; i < n; i += len) {
                std::complex<double> w(1);
                for (int j = 0; j < len / 2; j++) {
                    std::complex<double> u(vReal[i+j], vImag[i+j]);
                    std::complex<double> v = std::complex<double>(vReal[i+j+len/2], vImag[i+j+len/2]) * w;
                    vReal[i+j] = (u + v).real();
                    vImag[i+j] = (u + v).imag();
                    vReal[i+j+len/2] = (u - v).real();
                    vImag[i+j+len/2] = (u - v).imag();
                    w *= wlen;
                }
            }
        }
    }

    void ComplexToMagnitude(double* vReal, double* vImag, uint16_t samples) {
        for (uint16_t i = 0; i < samples; i++) {
            vReal[i] = sqrt(vReal[i] * vReal[i] + vImag[i] * vImag[i]);
        }
    }
};
