import pandas as pd
import matplotlib.pyplot as plt
import glob
import os

def plot_scenarios():
    files = sorted(glob.glob("scenario_*.csv"))
    if not files:
        print("No scenario logs found.")
        return

    fig, axes = plt.subplots(len(files), 1, figsize=(12, 4 * len(files)), sharex=True)
    if len(files) == 1:
        axes = [axes]

    for i, file in enumerate(files):
        df = pd.read_csv(file)
        # Parse params from filename
        # scenario_100_200.csv -> f_dist=100, f_res=200
        parts = os.path.basename(file).split("_")
        f_dist = parts[1]
        f_res = parts[2].split(".")[0]

        ax = axes[i]
        ax.plot(df['Time'], df['RMS_E'], label=f"RMS Error", color='green')
        ax.axvline(x=2.0, color='red', linestyle='--', label='SYSID Point')
        ax.set_title(f"Disturbance: {f_dist} Hz | Plant Resonance: {f_res} Hz")
        ax.set_ylabel("RMS Error")
        ax.legend()
        ax.grid(True)

    plt.xlabel("Time (s)")
    plt.tight_layout()
    plt.savefig("simulator/scenarios_results.png")
    print("Scenario results saved to simulator/scenarios_results.png")

if __name__ == "__main__":
    plot_scenarios()
