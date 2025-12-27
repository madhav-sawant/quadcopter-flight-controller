import matplotlib.pyplot as plt
import re

def parse_data(filename):
    gyro_data = {'roll': [], 'pitch': []}
    accel_data = {'roll': [], 'pitch': []}
    fused_data = {'roll': [], 'pitch': []}

    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            
            # Regex to match the line format: LABEL | Roll: <val> | Pitch: <val>
            match = re.search(r'(GYRO_ONLY|ACCEL_ONLY|FUSED)\s*\|\s*Roll:\s*([-\d.]+)\s*\|\s*Pitch:\s*([-\d.]+)', line)
            if match:
                label = match.group(1)
                roll = float(match.group(2))
                pitch = float(match.group(3))

                if label == 'GYRO_ONLY':
                    gyro_data['roll'].append(roll)
                    gyro_data['pitch'].append(pitch)
                elif label == 'ACCEL_ONLY':
                    accel_data['roll'].append(roll)
                    accel_data['pitch'].append(pitch)
                elif label == 'FUSED':
                    fused_data['roll'].append(roll)
                    fused_data['pitch'].append(pitch)
    
    return gyro_data, accel_data, fused_data

def plot_data(gyro, accel, fused):
    # Determine minimum length to use the same number of samples
    min_len = min(len(gyro['roll']), len(accel['roll']), len(fused['roll']))
    
    print(f"Samples found - Gyro: {len(gyro['roll'])}, Accel: {len(accel['roll'])}, Fused: {len(fused['roll'])}")
    print(f"Truncating to minimum length: {min_len}")

    # Truncate data
    g_roll = gyro['roll'][:min_len]
    g_pitch = gyro['pitch'][:min_len]
    a_roll = accel['roll'][:min_len]
    a_pitch = accel['pitch'][:min_len]
    f_roll = fused['roll'][:min_len]
    f_pitch = fused['pitch'][:min_len]

    samples = range(min_len)

    fig, axs = plt.subplots(3, 1, figsize=(10, 15), sharex=True)
    
    # Plot 1: Gyro Only
    axs[0].plot(samples, g_roll, label='Roll', color='red', alpha=0.7)
    axs[0].plot(samples, g_pitch, label='Pitch', color='blue', alpha=0.7)
    axs[0].set_title('Test 1: Gyroscope Only (Drift Demonstration)')
    axs[0].set_ylabel('Angle (degrees)')
    axs[0].legend()
    axs[0].grid(True, which='both', linestyle='--', linewidth=0.5)

    # Plot 2: Accel Only
    axs[1].plot(samples, a_roll, label='Roll', color='red', alpha=0.7)
    axs[1].plot(samples, a_pitch, label='Pitch', color='blue', alpha=0.7)
    axs[1].set_title('Test 2: Accelerometer Only (Noise/Vibration)')
    axs[1].set_ylabel('Angle (degrees)')
    axs[1].legend()
    axs[1].grid(True, which='both', linestyle='--', linewidth=0.5)

    # Plot 3: Fused
    axs[2].plot(samples, f_roll, label='Roll', color='red', alpha=0.7)
    axs[2].plot(samples, f_pitch, label='Pitch', color='blue', alpha=0.7)
    axs[2].set_title('Test 3: Fused / Complementary Filter (Stable)')
    axs[2].set_ylabel('Angle (degrees)')
    axs[2].set_xlabel('Sample Number')
    axs[2].legend()
    axs[2].grid(True, which='both', linestyle='--', linewidth=0.5)

    plt.tight_layout()
    plt.savefig('docs/sensor_comparison_graph.png')
    print("Graph saved to docs/sensor_comparison_graph.png")

if __name__ == "__main__":
    gyro, accel, fused = parse_data('docs/graph.txt')
    plot_data(gyro, accel, fused)
