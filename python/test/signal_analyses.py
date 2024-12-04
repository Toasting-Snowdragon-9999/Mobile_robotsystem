import numpy as np
import matplotlib.pyplot as plt

# Function to load data from a file
def load_data(file_path):
    try:
        # Read all lines from the file and convert to a NumPy array
        with open(file_path, 'r') as file:
            data = np.array([float(line.strip()) for line in file if line.strip()])
        return data
    except Exception as e:
        print(f"Error loading data from {file_path}: {e}")
        return np.array([])

# Compute FFT
def compute_fft(data, sample_rate):
    N = len(data)  # Number of data points
    yf = np.fft.fft(data)  # Compute the FFT
    xf = np.fft.fftfreq(N, d=1 / sample_rate)  # Compute the frequency bins
    xf = xf[:N // 2]  # Only take the positive frequencies
    magnitude = 2.0 / N * np.abs(yf[:N // 2])  # Magnitude of FFT (normalized)
    return xf, magnitude

def main():
    # Paths to signal and noise files
    signal_file = "test_test.txt"  # Replace with the actual file path
    noise_file = "noise.txt"    # Replace with the actual file path

    # Load signal and noise data
    signal_data = load_data(signal_file)
    noise_data = load_data(noise_file)

    if signal_data.size == 0 or noise_data.size == 0:
        print("Error: Unable to load signal or noise data.")
        return

    # Plot the FFT results
    plt.figure(figsize=(12, 6))
    plt.plot(signal_data, label="Signal", alpha=0.8)
    plt.plot(noise_data, label="Noise", alpha=0.8)
    plt.title("Signal vs Noise")
    plt.xlabel("Samples")
    plt.ylabel("Amplitude")
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    main()
