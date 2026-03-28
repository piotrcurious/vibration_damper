import pandas as pd
import matplotlib.pyplot as plt
import glob
import os

def plot_individual_scenarios():
    files = sorted(glob.glob("scenario_*.csv"))
    if not files:
        print("No scenario logs found.")
        return

    for file in files:
        df = pd.read_csv(file)
        # scenario_100_200_0.csv -> f_dist=100, f_res=200, drift=0
        parts = os.path.basename(file).split("_")
        f_dist = parts[1]
        f_res = parts[2]
        drift = parts[3].split(".")[0]

        plt.figure(figsize=(10, 6))
        plt.plot(df['Time'], df['RMS_E'], label='Error Signal (RMS)', color='#2ca02c', linewidth=1.5)
        plt.axvline(x=1.0, color='#d62728', linestyle='--', label='SYSID Point')

        title = f"Disturbance: {f_dist} Hz | Resonance: {f_res} Hz"
        if int(drift) > 0:
            title += f" | Drift: {drift} Hz/s"

        plt.title(title)
        plt.xlabel("Time (s)")
        plt.ylabel("RMS Error")
        plt.grid(True, which='both', linestyle='--', alpha=0.5)
        plt.legend()

        out_name = f"simulator/result_{f_dist}_{f_res}_{drift}.png"
        plt.savefig(out_name, dpi=120)
        plt.close()
        print(f"Saved: {out_name}")

if __name__ == "__main__":
    plot_individual_scenarios()
