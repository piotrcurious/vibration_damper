import subprocess
import os
import pandas as pd

def run_scenario(f_dist, f_res):
    log_name = f"scenario_{int(f_dist)}_{int(f_res)}.csv"
    cmd = f"./simulator/sim {f_dist} {f_res} {log_name}"
    print(f"Running scenario: F_dist={f_dist}, F_res={f_res}")
    subprocess.run(cmd, shell=True, check=True, capture_output=True)
    return log_name

def main():
    # Define scenarios
    scenarios = [
        (100, 200),
        (250, 300),
        (400, 100), # Resonance below frequency
        (600, 600), # Frequency at resonance
    ]

    # Compile
    cmd = "g++ -O3 -I simulator/mock_arduino -I simulator/mock_esp32 -I simulator/arduinoFFT simulator/main.cpp -o simulator/sim"
    subprocess.run(cmd, shell=True, check=True)

    results = []
    for f_dist, f_res in scenarios:
        log_name = run_scenario(f_dist, f_res)
        results.append((f_dist, f_res, log_name))

    print("All scenarios completed.")

if __name__ == "__main__":
    main()
