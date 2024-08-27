import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

# Step 1: Load the CSV data
data = pd.read_csv(
    "/media/muayman17/New/Muhammad/Work/MPC_Controller_Task/PathFollowingMPC/Model/trajectory.csv"
)

# Step 2: Extract the states and convert them to numpy arrays
x = data["x"].to_numpy()
y = data["y"].to_numpy()
yaw = data["yaw"].to_numpy()
steering_angle = data["steeringAngle"].to_numpy()

# Step 3: Create the figure for the interactive plot
fig, ax = plt.subplots(figsize=(8, 8))

# Set up the plot limits and labels
ax.set_xlim(min(x) - 1, max(x) + 1)
ax.set_ylim(min(y) - 1, max(y) + 1)
ax.set_xlabel("x (meters)")
ax.set_ylabel("y (meters)")
ax.set_title("Vehicle Trajectory")

# Initialize the line and the quiver (arrows) that will represent the vehicle's trajectory, orientation, and steering angle
(trajectory_line,) = ax.plot([], [], label="Trajectory")
vehicle_orientation = ax.quiver([], [], [], [], scale=10, color="red", label="Yaw")
steering_orientation = ax.quiver(
    [], [], [], [], scale=10, color="blue", label="Steering Angle"
)


# Function to initialize the plot elements
def init():
    trajectory_line.set_data([], [])
    vehicle_orientation.set_UVC([], [])
    steering_orientation.set_UVC([], [])
    return trajectory_line, vehicle_orientation, steering_orientation


# Function to update the plot with new data
def update(frame):
    # Update the trajectory line
    trajectory_line.set_data(x[:frame], y[:frame])

    # Update the vehicle's yaw orientation using quiver
    vehicle_orientation.set_offsets([x[frame], y[frame]])
    vehicle_orientation.set_UVC(np.cos(yaw[frame]), np.sin(yaw[frame]))

    # Update the vehicle's steering angle orientation using quiver
    # The steering angle is relative to the yaw, so we add it to the yaw to get the direction
    steering_orientation.set_offsets([x[frame], y[frame]])
    steering_orientation.set_UVC(
        np.cos(yaw[frame] + steering_angle[frame]),
        np.sin(yaw[frame] + steering_angle[frame]),
    )

    return trajectory_line, vehicle_orientation, steering_orientation


# Create the animation object
ani = FuncAnimation(fig, update, frames=len(x), init_func=init, blit=True, interval=50)

# Save the animation as a GIF file
ani.save("trajectory_animation_with_steering.gif", writer="pillow", fps=10)

# Optionally, show the plot
plt.legend()
plt.grid(True)
plt.show()
