import matplotlib.pyplot as plt
import pandas as pd

def main():
    try:
        df = pd.read_csv('sim_log.csv')
    except Exception as e:
        print(f"Error reading CSV: {e}")
        return

    plt.figure(figsize=(12, 10))

    plt.subplot(4, 1, 1)
    plt.plot(df['Time'], df['Disturbance'], label='Disturbance')
    plt.title('Disturbance')
    plt.legend()
    plt.grid(True)

    plt.subplot(4, 1, 2)
    plt.plot(df['Time'], df['Actuator'], label='Actuator', color='orange')
    plt.title('Actuator Output')
    plt.legend()
    plt.grid(True)

    plt.subplot(4, 1, 3)
    plt.plot(df['Time'], df['Error'], label='Error Signal', color='red')
    plt.title('Error Signal (Disturbance + Plant(Actuator))')
    plt.legend()
    plt.grid(True)

    plt.subplot(4, 1, 4)
    plt.plot(df['Time'], df['RMS_E'], label='RMS Error', color='green')
    plt.title('RMS Error over Time')
    plt.xlabel('Time (s)')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.savefig('sim_results.png')
    print("Plot saved to sim_results.png")

if __name__ == "__main__":
    main()
