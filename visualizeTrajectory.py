import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Step 1: Load the CSV data
data = pd.read_csv("trajectory.csv")

# Step 2: Extract the states and convert them to numpy arrays
x = data["x"].to_numpy()
y = data["y"].to_numpy()
yaw = data["yaw"].to_numpy()
steering_angle = data["steeringAngle"].to_numpy()

# Step 3: Create the first set of plots
plt.figure(figsize=(12, 10))

# Plot x vs y (trajectory)
plt.subplot(2, 2, 1)
plt.plot(x, y, label="Trajectory")
plt.xlabel("x (meters)")
plt.ylabel("y (meters)")
plt.title("Vehicle Trajectory")
plt.legend()
plt.grid(True)

# Plot yaw over time
plt.subplot(2, 2, 2)
plt.plot(yaw, label="Yaw")
plt.xlabel("Time Step")
plt.ylabel("Yaw (radians)")
plt.title("Yaw vs Time Step")
plt.legend()
plt.grid(True)

# Plot steering angle over time
plt.subplot(2, 2, 3)
plt.plot(steering_angle, label="Steering Angle")
plt.xlabel("Time Step")
plt.ylabel("Steering Angle (radians)")
plt.title("Steering Angle vs Time Step")
plt.legend()
plt.grid(True)

# Plot x and y positions over time in a combined plot
time_steps = np.arange(len(x))

plt.subplot(2, 2, 4)
plt.plot(time_steps, x, label="x position")
plt.plot(time_steps, y, label="y position")
plt.xlabel("Time Step")
plt.ylabel("Position (meters)")
plt.title("x and y Positions vs Time Step")
plt.legend()
plt.grid(True)

# # Show the first set of plots
plt.tight_layout()

# Step 4: Create a second figure for 2D motion with orientation
plt.figure(figsize=(8, 8))
plt.plot(x, y, label="Trajectory")

# Add quivers to show the orientation of the vehicle
for i in range(0, len(x), max(1, len(x) // 20)):  # Reduce number of arrows for clarity
    plt.arrow(
        x[i],
        y[i],
        0.5 * np.cos(yaw[i]),
        0.5 * np.sin(yaw[i]),
        head_width=0.2,
        head_length=0.3,
        fc="red",
        ec="red",
    )

plt.xlabel("x (meters)")
plt.ylabel("y (meters)")
plt.title("2D Vehicle Motion with Orientation")
plt.legend()
plt.grid(True)
plt.axis("equal")  # Ensure equal scaling on both axes
plt.tight_layout()
plt.show()
