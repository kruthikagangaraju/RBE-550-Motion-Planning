import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import sys

def plot_car(car_file, obstacle_file):
    # Read car trajectory data
    car_data = np.loadtxt(car_file, skiprows=1)  # Skip header
    car_x, car_y = car_data[:, 0], car_data[:, 1]

    # Read obstacle data
    obstacle_data = np.loadtxt(obstacle_file)
    obs_x, obs_y, obs_w, obs_h = obstacle_data[:, 0], obstacle_data[:, 1], obstacle_data[:, 2], obstacle_data[:, 3]

    # Create plot
    plt.figure(figsize=(10, 5))
    plt.plot(car_x, car_y, 'b-', label="Car Path")  # Car path
    
    # Plot obstacles as ellipses
    for x, y, w, h in zip(obs_x, obs_y, obs_w, obs_h):
        rect = patches.Rectangle((x, y), w, h, linewidth=1.5, edgecolor='r', facecolor='r', alpha=0.5)
        plt.gca().add_patch(rect)

    # Labels and display
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.title("Car Trajectory and Obstacles")
    plt.legend()
    plt.axis("equal")
    plt.grid(True)

    # Save figure
    output_filename = "car_trajectory_plot.png"
    plt.savefig(output_filename, dpi=300)
    print(f"Plot saved as {output_filename}")
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python3 plot_car.py <car_1.txt> <obstacles.txt>")
        sys.exit(1)

    car_file = sys.argv[1]
    obstacle_file = sys.argv[2]
    plot_car(car_file, obstacle_file)
