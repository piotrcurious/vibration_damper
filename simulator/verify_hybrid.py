import subprocess
import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Scenarios to test:
# f_dist, f_res, log_name, drift, control, plant_drift
scenarios = [
    (150, 200, "verify_150_off.csv", 0, 0, 0),
    (150, 200, "verify_150_on.csv", 0, 1, 0),
    (300, 200, "verify_300_on.csv", 0, 1, 0),
    (250, 200, "verify_drift_on.csv", 15, 1, 0),
    (150, 200, "verify_plant_drift.csv", 0, 1, 25),
    (150, 200, "verify_plant_step.csv", 0, 1, -1000),
    (150, 200, "verify_noisy.csv", 0, 1, 0, 0.2), # High broadband noise
    (150, 200, "verify_shocks.csv", 0, 1, 0, 0.15), # Shocks (triggered by noise_level > 0.1)
]

def run_verify():
    if not os.path.exists("simulator/sim"):
        print("Compiling simulator...")
        subprocess.run("g++ -O3 -DSIMULATOR -I simulator/mock_arduino -I simulator/mock_esp32 -I simulator/arduinoFFT simulator/main.cpp -o simulator/sim", shell=True)

    for s in scenarios:
        f_dist, f_res, log, drift, control = s[0:5]
        plant_drift = s[5] if len(s) > 5 else 0
        noise = s[6] if len(s) > 6 else 0.01
        cmd = ["./simulator/sim", str(f_dist), str(f_res), log, str(drift), str(control), str(plant_drift), str(noise)]
        print(f"Running: {' '.join(cmd)}")
        subprocess.run(cmd)

def calculate_metrics(log_file):
    if not os.path.exists(log_file):
        return None
    df = pd.read_csv(log_file)

    # Calculate RMS of 'Error' column manually
    def get_rms(window):
        if window.empty: return 1.0
        return np.sqrt((window['Error']**2).mean())

    # Initial state (Baseline phase: 0.1s to 0.4s to avoid RMS estimator startup)
    initial = df[(df['Time'] > 0.1) & (df['Time'] < 0.4)]
    # Steady state (last 1.5 seconds of simulation)
    steady_state = df[df['Time'] > (df['Time'].max() - 1.5)]

    rms_initial = get_rms(initial)
    rms_final = get_rms(steady_state)

    # Avoid log of zero or negative
    reduction_db = 20 * np.log10(max(1e-6, rms_initial) / max(1e-6, rms_final))
    return {
        "Initial RMS": rms_initial,
        "Final RMS": rms_final,
        "Reduction (dB)": reduction_db
    }

def print_summary():
    print("\n" + "="*50)
    print(" VERIFICATION SUMMARY")
    print("="*50)
    print(f"{'Scenario':<25} | {'Reduction':<10}")
    print("-" * 50)
    for s in scenarios:
        if s[4] == 0: continue # Skip 'OFF'
        metrics = calculate_metrics(s[2])
        if metrics:
            print(f"{s[2]:<25} | {metrics['Reduction (dB)']:>6.1f} dB")
    print("="*50 + "\n")

def plot_verify():
    plt.style.use('seaborn-v0_8-muted')

    # 1. Performance Overview
    fig, ax = plt.subplots(figsize=(12, 6))
    if os.path.exists("verify_150_off.csv"):
        df_off = pd.read_csv("verify_150_off.csv")
        ax.plot(df_off['Time'], df_off['Error'], label='Control OFF', color='gray', alpha=0.3, linewidth=0.5)

    if os.path.exists("verify_150_on.csv"):
        df_on = pd.read_csv("verify_150_on.csv")
        # Plot smoothed error for clarity
        ax.plot(df_on['Time'], df_on['Error'], label='Error Signal (Control ON)', color='#1f77b4', alpha=0.6, linewidth=0.5)

        # Calculate a rolling RMS for plotting
        window = 100
        rolling_rms = np.sqrt((df_on['Error']**2).rolling(window=window).mean())
        ax.plot(df_on['Time'], rolling_rms, label='RMS Error', color='#d62728', linewidth=2)

        ax.axvline(x=0.5, color='black', alpha=0.5, linestyle='--', label='SYSID Point')
        ax.axvline(x=1.5, color='green', alpha=0.5, linestyle='--', label='Convergence')

    ax.set_title("Hybrid SOGI-FxLMS Convergence & Damping Performance", fontsize=14)
    ax.set_xlabel("Time (s)", fontsize=12)
    ax.set_ylabel("Amplitude", fontsize=12)
    ax.legend(frameon=True, loc='upper right')
    ax.grid(True, alpha=0.3)
    ax.set_ylim(-1.0, 1.0)
    plt.tight_layout()
    plt.savefig("verification_performance.png", dpi=150)
    plt.close()

    # 2. Spectral Suppression
    fig, ax = plt.subplots(figsize=(12, 6))
    if os.path.exists("verify_150_off_spec.csv"):
        df_off_spec = pd.read_csv("verify_150_off_spec.csv")
        ax.semilogy(df_off_spec['Hz'], df_off_spec['Magnitude'], label='Control OFF', color='gray', alpha=0.4)

    if os.path.exists("verify_150_on_spec.csv"):
        df_on_spec = pd.read_csv("verify_150_on_spec.csv")
        ax.semilogy(df_on_spec['Hz'], df_on_spec['Magnitude'], label='Control ON', color='#1f77b4', linewidth=1.5)

    ax.set_title("Steady-State Power Spectrum (Residual Noise vs Tonal Cancellation)", fontsize=14)
    ax.set_xlabel("Frequency (Hz)", fontsize=12)
    ax.set_ylabel("Magnitude (log)", fontsize=12)
    ax.set_xlim(0, 1200)
    ax.legend()
    ax.grid(True, which='both', alpha=0.2)
    plt.tight_layout()
    plt.savefig("verification_spectrum.png", dpi=150)
    plt.close()

    # 3. Dynamic Adaptation (Drift and Plant)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))

    def plot_rms_log(ax, file, color, label):
        df = pd.read_csv(file)
        rms = np.sqrt((df['Error']**2).rolling(window=200).mean())
        ax.plot(df['Time'], rms, color=color, label=label, linewidth=2)
        ax.axvline(x=1.5, color='green', alpha=0.3, linestyle='--')
        ax.grid(True, alpha=0.3)
        ax.set_ylabel("RMS Error")
        ax.legend()

    if os.path.exists("verify_drift_on.csv"):
        plot_rms_log(ax1, "verify_drift_on.csv", "#ff7f0e", "Frequency Tracking: 15 Hz/s Drift")
        ax1.set_title("Disturbance Frequency Tracking Performance", fontsize=14)

    if os.path.exists("verify_plant_drift.csv"):
        plot_rms_log(ax2, "verify_plant_drift.csv", "#9467bd", "ASPM Tracking: 25 Hz/s Resonance Drift")

        if os.path.exists("verify_plant_step.csv"):
            df_step = pd.read_csv("verify_plant_step.csv")
            rms_step = np.sqrt((df_step['Error']**2).rolling(window=200).mean())
            ax2.plot(df_step['Time'], rms_step, color='#2ca02c', label='Plant Step Change (200->350 Hz)', alpha=0.7)
            ax2.axvline(x=3.5, color='red', alpha=0.3, linestyle=':')

        ax2.set_title("Runtime Mechanical Plant Adaptation (ASPM)", fontsize=14)
        ax2.set_xlabel("Time (s)")

    plt.tight_layout()
    plt.savefig("verification_adaptation.png", dpi=150)
    plt.close()

if __name__ == "__main__":
    run_verify()
    print_summary()
    plot_verify()
