import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import math

# Step 1: Load the CSV data
data = pd.read_csv("trajectory.csv")

# Step 2: Extract the states and convert them to numpy arrays
x = data["x"].to_numpy()
y = data["y"].to_numpy()
yaw = data["velocity"].to_numpy()
steering_angle = data["steeringAngle"].to_numpy()

# Reference trajectory points (given. as x_ref, y_ref, theta_ref)
x_ref = -150.0
y_ref = 100.0
theta_ref = math.atan(y_ref / x_ref)
vel_upper_limit = 20
vel_lower_limit = -20

# Step 3: Create the first set of plots
plt.figure(figsize=(12, 10))

# Plot x vs y (trajectory)
plt.subplot(2, 2, 1)
plt.plot(x, y, label="Vehicle Trajectory")
plt.scatter([x_ref], [y_ref], color="red", label="Reference Point", marker="x")
plt.xlabel("x (meters)")
plt.ylabel("y (meters)")
plt.title("Vehicle Trajectory")
plt.legend()
plt.grid(True)

# Plot x and y positions over time in a combined plot
time_steps = np.arange(len(x))

plt.subplot(2, 2, 2)
plt.plot(time_steps, x, label="x position")
plt.plot(time_steps, y, label="y position")
plt.axhline(y=x_ref, color="red", linestyle="--", label="x Reference")
plt.axhline(y=y_ref, color="orange", linestyle="--", label="y Reference")
plt.xlabel("Time Step")
plt.ylabel("Position (meters)")
plt.title("x and y Positions vs Time Step")
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


# Plot velocity over time
plt.subplot(2, 2, 4)
plt.plot(yaw, label="Vehicle Yaw")
plt.axhline(y=vel_upper_limit, color="red", linestyle="--", label="Upper Limit")
plt.axhline(y=vel_lower_limit, color="red", linestyle="--", label="Lower Limit")
plt.xlabel("Time Step")
plt.ylabel("Velocity (m/s)")
plt.title("Velocity vs Time Step")
plt.legend()
plt.grid(True)

# Show the first set of plots
plt.tight_layout()
plt.show()
