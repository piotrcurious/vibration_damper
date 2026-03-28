import subprocess
import os
import pandas as pd

def run_scenario(f_dist, f_res, drift=0):
    log_name = f"scenario_{int(f_dist)}_{int(f_res)}_{int(drift)}.csv"
    cmd = f"./simulator/sim {f_dist} {f_res} {log_name} {drift}"
    print(f"Running scenario: F_dist={f_dist}, F_res={f_res}, Drift={drift}")
    subprocess.run(cmd, shell=True, check=True, capture_output=True)
    return log_name

def main():
    # Define scenarios (f_dist, f_res, drift)
    scenarios = [
        (100, 200, 0),
        (250, 300, 0),
        (400, 100, 0), # Resonance below frequency
        (600, 600, 0), # Frequency at resonance
        (200, 250, 25), # Frequency drift 25 Hz/s
    ]

    # Compile
    cmd = "g++ -O3 -I simulator/mock_arduino -I simulator/mock_esp32 -I simulator/arduinoFFT simulator/main.cpp -o simulator/sim"
    subprocess.run(cmd, shell=True, check=True)

    results = []
    for f_dist, f_res, drift in scenarios:
        log_name = run_scenario(f_dist, f_res, drift)
        results.append((f_dist, f_res, drift, log_name))

    print("All scenarios completed.")

if __name__ == "__main__":
    main()
