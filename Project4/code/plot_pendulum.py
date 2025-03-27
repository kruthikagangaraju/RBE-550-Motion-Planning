import numpy as np
import matplotlib.pyplot as plt
import sys

def plot_pendulum(file_path):
    # Read data from file
    theta = []
    omega = []
    with open(file_path, "r") as file:
        for line in file:
            if line.strip() and not line.startswith("Pendulum"):
                t, w = map(float, line.split())
                theta.append(t)
                omega.append(w)

    # Create plot of theta vs omega
    plt.figure(figsize=(10, 5))
    plt.plot(theta, omega, label="Theta vs Omega", color='b')
    plt.plot(-np.pi/2, 0, 'bo', label='Start')
    plt.plot(np.pi/2, 0, 'ro', label='Goal')
    plt.xlabel("Theta (radians)")
    plt.ylabel("Omega (rad/s)")
    plt.title("Pendulum: Theta vs Omega")
    plt.legend()
    plt.grid(True)

    # Save figure
    output_filename = file_path.replace(".txt", "_theta_vs_omega.png")
    plt.savefig(output_filename, dpi=300)
    print(f"Plot saved as {output_filename}")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 plot_pendulum.py <data_file.txt>")
        sys.exit(1)

    file_path = sys.argv[1]
    plot_pendulum(file_path)
