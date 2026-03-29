import subprocess
import os

scenarios = [
    # f_dist, f_res, log_name, drift, control
    (150, 200, "sim_150_200_off.csv", 0, 0),
    (150, 200, "sim_150_200_on.csv", 0, 1),
    (250, 200, "sim_250_200_on.csv", 0, 1),
    (150, 200, "sim_drift_on.csv", 25, 1),
]

def run():
    if not os.path.exists("simulator/sim"):
        print("Simulator not found. Please compile first.")
        return

    for f_dist, f_res, log, drift, control in scenarios:
        cmd = ["./simulator/sim", str(f_dist), str(f_res), log, str(drift), str(control)]
        print(f"Running: {' '.join(cmd)}")
        subprocess.run(cmd)

if __name__ == "__main__":
    run()
