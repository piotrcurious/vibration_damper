import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os

def plot_spectra(csv_files, title, output_name):
    plt.figure(figsize=(12, 6))

    for csv_file in csv_files:
        if not os.path.exists(csv_file):
            print(f"File not found: {csv_file}")
            continue

        # Read full time series (Error signal)
        df = pd.read_csv(csv_file)

        # Use last 0.5 seconds for spectrum
        fs = 4000
        n_samples = int(0.5 * fs)
        err = df['Error'].values[-n_samples:]

        # Compute FFT
        window = np.hanning(len(err))
        fft_res = np.fft.rfft(err * window)
        freqs = np.fft.rfftfreq(len(err), 1/fs)
        mags = 2.0 * np.abs(fft_res) / len(err)

        label = os.path.basename(csv_file).replace(".csv", "")
        plt.plot(freqs, mags, label=label, alpha=0.8)

    plt.title(title)
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Magnitude (RMS)')
    plt.grid(True, which='both', linestyle='--', alpha=0.5)
    plt.legend()
    plt.yscale('log')
    plt.xlim(0, 2000)
    plt.ylim(1e-4, 1.0)
    plt.savefig(output_name)
    print(f"Spectral plot saved to {output_name}")

if __name__ == "__main__":
    plot_spectra(["sim_150_200_off.csv", "sim_150_200_on.csv"],
                 "Spectral Purity Comparison (150 Hz Disturbance)",
                 "simulator/spectrum_comparison_150_v2.png")

    plot_spectra(["sim_drift_on.csv"],
                 "Spectrum under Frequency Drift (25 Hz/s)",
                 "simulator/spectrum_drift_v2.png")
