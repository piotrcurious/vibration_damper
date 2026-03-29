import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os

def plot_advanced(log_file):
    if not os.path.exists(log_file):
        print(f"Log {log_file} not found.")
        return

    df = pd.read_csv(log_file)
    fs = 4000.0  # Sample rate in Hz

    error_sig = df['Error'].values
    time = df['Time'].values

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), gridspec_kw={'height_ratios': [1, 2]})

    # Time domain RMS
    window = 400
    rms = np.sqrt(np.convolve(error_sig**2, np.ones(window)/window, mode='same'))
    ax1.plot(time, error_sig, color='gray', alpha=0.2, label='Raw Error')
    ax1.plot(time, rms, color='red', label='RMS Error', linewidth=1.5)
    ax1.set_title(f"Performance Analysis: {log_file}", fontsize=14)
    ax1.set_ylabel("Amplitude")
    ax1.set_xlabel("Time (s)")
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)
    ax1.set_ylim(-1.1, 1.1)

    # Manual Spectrogram using STFT with NumPy
    nperseg = 512
    noverlap = 480
    step = nperseg - noverlap

    # Pre-allocate spectrogram matrix
    num_steps = (len(error_sig) - nperseg) // step
    spec = np.zeros((nperseg // 2 + 1, num_steps))
    t_spec = np.zeros(num_steps)

    win = np.hanning(nperseg)
    for i in range(num_steps):
        start = i * step
        segment = error_sig[start : start + nperseg] * win
        spectrum = np.fft.rfft(segment)
        spec[:, i] = np.abs(spectrum)
        t_spec[i] = time[start + nperseg // 2]

    freqs = np.fft.rfftfreq(nperseg, 1/fs)

    # Use log scale for intensity
    pcm = ax2.pcolormesh(t_spec, freqs, 20 * np.log10(spec + 1e-6), shading='gouraud', cmap='magma')
    ax2.set_ylabel('Frequency [Hz]')
    ax2.set_xlabel('Time [sec]')
    ax2.set_title('Error Signal Spectrogram (dB)', fontsize=14)
    ax2.set_ylim(0, 1000)
    fig.colorbar(pcm, ax=ax2, label='Magnitude [dB]')

    ax2.axvline(x=0.5, color='white', linestyle='--', alpha=0.5)
    ax2.axvline(x=1.5, color='white', linestyle='--', alpha=0.5)

    plt.tight_layout()
    out_name = log_file.replace(".csv", "_advanced.png")
    plt.savefig(out_name, dpi=150)
    plt.close()
    print(f"Saved advanced plot: {out_name}")

if __name__ == "__main__":
    plot_advanced("verify_150_on.csv")
    plot_advanced("verify_drift_on.csv")
    plot_advanced("verify_plant_drift.csv")
