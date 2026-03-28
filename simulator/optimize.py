import subprocess
import os
import pandas as pd
import matplotlib.pyplot as plt

def run_simulation(mu0, leakage):
    # Update avd_esp32.ino with new parameters
    with open('src/avd_esp32.ino', 'r') as f:
        lines = f.readlines()

    new_lines = []
    for line in lines:
        if 'static constexpr float MU0' in line:
            new_lines.append(f'static constexpr float MU0             = {mu0}f;\n')
        elif 'static constexpr float LEAKAGE' in line:
            new_lines.append(f'static constexpr float LEAKAGE         = {leakage}f;\n')
        else:
            new_lines.append(line)

    with open('src/avd_esp32.ino', 'w') as f:
        f.writelines(new_lines)

    # Compile and run
    cmd = "g++ -O3 -I simulator/mock_arduino -I simulator/mock_esp32 -I simulator/arduinoFFT simulator/main.cpp -o simulator/sim && ./simulator/sim"
    subprocess.run(cmd, shell=True, check=True, capture_output=True)

    # Read results
    df = pd.read_csv('sim_log.csv')
    return df

def main():
    params = [
        (2e-2, 0.9999),
        (5e-2, 0.9999),
        (1e-1, 0.9999),
    ]

    plt.figure(figsize=(12, 8))
    for mu0, leakage in params:
        print(f"Testing MU0={mu0}, LEAKAGE={leakage}")
        try:
            df = run_simulation(mu0, leakage)
            plt.plot(df['Time'], df['RMS_E'], label=f'MU0={mu0}, L={leakage}')
        except Exception as e:
            print(f"Failed MU0={mu0}: {e}")

    plt.title('RMS Error for different MU0/LEAKAGE')
    plt.xlabel('Time (s)')
    plt.ylabel('RMS Error')
    plt.legend()
    plt.grid(True)
    plt.savefig('optimization_results.png')
    print("Optimization results saved to optimization_results.png")

if __name__ == "__main__":
    main()
