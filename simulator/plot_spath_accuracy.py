import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import subprocess
import os

def run_spath_dump():
    # 1. Run simulation to identify path
    log = "spath_test.csv"
    cmd = ["./simulator/sim", "150", "300", log, "0", "1", "0"]
    subprocess.run(cmd)

    # 2. Extract s_hat from simulator output (captured from stdout)
    # Since we can't easily capture stdout from here, let's modify main.cpp to save s_hat
    pass

def plot_spath_comparison(f_res, Q=5.0):
    fs = 4000.0
    s_hat = []

    # Read the s_hat coefficients from the last simulation run
    # For now, let's assume we've extracted them or use verify_150_on logs if we added them there
    # Instead, let's just use the SPATH command in the simulator to dump to a file

    if not os.path.exists("sim_spath.csv"):
        print("Secondary path model file not found.")
        return

    df_hat = pd.read_csv("sim_spath.csv")
    h_hat = df_hat['Coeff'].values

    # Theoretical Model (Biquad response)
    # H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)
    omega = 2.0 * np.pi * f_res / fs
    alpha = np.sin(omega) / (2.0 * Q)
    a0 = 1.0 + alpha
    b = np.array([(1.0 - np.cos(omega)) / 2.0, (1.0 - np.cos(omega)), (1.0 - np.cos(omega)) / 2.0]) / a0
    a = np.array([1.0, -2.0 * np.cos(omega) / a0, (1.0 - alpha) / a0])
    # Normalize gain
    gain = 1.0 / Q
    b *= gain

    freqs = np.linspace(20, 1800, 500)
    w = 2 * np.pi * freqs / fs

    # Frequency Response of Identified FIR (s_hat)
    z = np.exp(-1j * np.outer(w, np.arange(len(h_hat))))
    H_fir = np.sum(h_hat * z, axis=1)

    # Frequency Response of Theoretical IIR
    z_iir = np.exp(-1j * w)
    H_theory = (b[0] + b[1]*z_iir + b[2]*z_iir**2) / (a[0] + a[1]*z_iir + a[2]*z_iir**2)
    H_theory *= np.exp(-1j * w * 2) # Add the 2-sample delay from simulation

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))

    # Magnitude
    ax1.plot(freqs, 20*np.log10(np.abs(H_theory)), label='Theoretical (Physics Engine)', color='gray', alpha=0.6)
    ax1.plot(freqs, 20*np.log10(np.abs(H_fir)), label='Identified (FxLMS Ŝ)', color='#d62728', linewidth=2)
    ax1.set_title(f"Secondary Path Identification Accuracy (Resonance: {f_res} Hz)", fontsize=14)
    ax1.set_ylabel("Magnitude (dB)")
    ax1.grid(True, alpha=0.3)
    ax1.legend()

    # Phase
    ax2.plot(freqs, np.unwrap(np.angle(H_theory)), color='gray', alpha=0.6)
    ax2.plot(freqs, np.unwrap(np.angle(H_fir)), color='#d62728', linewidth=2)
    ax2.set_ylabel("Phase (rad)")
    ax2.set_xlabel("Frequency (Hz)")
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig("spath_accuracy.png", dpi=150)
    plt.close()
    print("Saved: spath_accuracy.png")

if __name__ == "__main__":
    # Note: Requires running sim and dumping SPATH first
    # This is a stub for logic, actual execution needs the CSV
    plot_spath_comparison(200.0)
