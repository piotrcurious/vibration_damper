import subprocess
import os
import pandas as pd
import matplotlib.pyplot as plt
import glob

# Scenarios to test:
# 1. 150Hz Disturbance, 200Hz Resonance (Clean tone)
# 2. 300Hz Disturbance, 200Hz Resonance (Harmonic interference)
# 3. 250Hz Disturbance, 200Hz Resonance, 15Hz/s Drift (Tracking)

scenarios = [
    # f_dist, f_res, log_name, drift, control, plant_drift
    (150, 200, "verify_150_off.csv", 0, 0, 0),
    (150, 200, "verify_150_on.csv", 0, 1, 0),
    (300, 200, "verify_300_on.csv", 0, 1, 0),
    (250, 200, "verify_drift_on.csv", 15, 1, 0),
    (150, 200, "verify_plant_drift.csv", 0, 1, 20), # 20 Hz/s plant resonance drift
]

def run_verify():
    if not os.path.exists("simulator/sim"):
        print("Compiling simulator...")
        subprocess.run("g++ -DSIMULATOR -I simulator/mock_arduino -I simulator/mock_esp32 -I simulator/arduinoFFT simulator/main.cpp -o simulator/sim", shell=True)

    for s in scenarios:
        f_dist, f_res, log, drift, control = s[0:5]
        plant_drift = s[5] if len(s) > 5 else 0
        cmd = ["./simulator/sim", str(f_dist), str(f_res), log, str(drift), str(control), str(plant_drift)]
        print(f"Running: {' '.join(cmd)}")
        subprocess.run(cmd)

def plot_verify():
    # 1. Compare 150Hz On vs Off
    plt.figure(figsize=(12, 6))
    if os.path.exists("verify_150_off.csv"):
        df_off = pd.read_csv("verify_150_off.csv")
        plt.plot(df_off['Time'], df_off['RMS_E'], label='Control OFF', color='gray', alpha=0.5)

    if os.path.exists("verify_150_on.csv"):
        df_on = pd.read_csv("verify_150_on.csv")
        plt.plot(df_on['Time'], df_on['RMS_E'], label='Control ON (Hybrid SOGI-FxLMS)', color='blue')
        plt.axvline(x=0.5, color='red', linestyle='--', label='SYSID Start')
        plt.axvline(x=1.5, color='green', linestyle='--', label='Control Start')

    plt.title("Cancellation Performance: 150 Hz Tone + Harmonics")
    plt.xlabel("Time (s)")
    plt.ylabel("RMS Error")
    plt.legend()
    plt.grid(True)
    plt.savefig("verification_convergence.png")
    print("Saved: verification_convergence.png")

    # 2. Spectral Analysis
    plt.figure(figsize=(12, 6))
    if os.path.exists("verify_150_off_spec.csv"):
        df_off_spec = pd.read_csv("verify_150_off_spec.csv")
        plt.semilogy(df_off_spec['Hz'], df_off_spec['Magnitude'], label='Control OFF', color='gray', alpha=0.5)

    if os.path.exists("verify_150_on_spec.csv"):
        df_on_spec = pd.read_csv("verify_150_on_spec.csv")
        plt.semilogy(df_on_spec['Hz'], df_on_spec['Magnitude'], label='Control ON', color='blue')

    plt.title("Error Spectrum: Control ON vs OFF (150 Hz Disturbance)")
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Magnitude (log)")
    plt.xlim(0, 1000)
    plt.legend()
    plt.grid(True)
    plt.savefig("verification_spectrum.png")
    print("Saved: verification_spectrum.png")

    # 3. Drift Tracking
    if os.path.exists("verify_drift_on.csv"):
        plt.figure(figsize=(12, 6))
        df_drift = pd.read_csv("verify_drift_on.csv")
        plt.plot(df_drift['Time'], df_drift['RMS_E'], color='orange')
        plt.axvline(x=1.5, color='green', linestyle='--', label='Control Start')
        plt.title("Frequency Drift Tracking (15 Hz/s)")
        plt.xlabel("Time (s)")
        plt.ylabel("RMS Error")
        plt.grid(True)
        plt.savefig("verification_drift.png")
        print("Saved: verification_drift.png")

    # 4. Plant Drift Adaptation
    if os.path.exists("verify_plant_drift.csv"):
        plt.figure(figsize=(12, 6))
        df_pd = pd.read_csv("verify_plant_drift.csv")
        plt.plot(df_pd['Time'], df_pd['RMS_E'], color='purple')
        plt.axvline(x=1.5, color='green', linestyle='--', label='Control Start')
        plt.title("Plant Resonance Drift Adaptation (20 Hz/s)")
        plt.xlabel("Time (s)")
        plt.ylabel("RMS Error")
        plt.grid(True)
        plt.savefig("verification_plant_drift.png")
        print("Saved: verification_plant_drift.png")

if __name__ == "__main__":
    run_verify()
    plot_verify()
