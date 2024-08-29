## README.md
# Nonlinear Model Predictive Control (NLMPC) Implementation

This repository contains a C++ implementation of a Nonlinear Model Predictive Control (NLMPC) system for controlling a vehicle's motion. The system optimizes the vehicle's trajectory to follow a reference path while considering constraints on velocity and steering rate.

## Files

- **main.cpp**: The main file containing the NLMPC implementation.
- **model.hpp**: Header file defining the vehicle model and associated structures.
- **TrajectoryPlot.py**: Python script for plotting the vehicle's trajectory.
- **TrajectoryAnimate.py**: Python script for animating the vehicle's trajectory.

## Dependencies

- **nlopt**: A library for nonlinear optimization. You can install it using:
  ```bash
  sudo apt-get install libnlopt-dev

### Overview

This project implements a Nonlinear Model Predictive Control (NLMPC) system for a vehicle model using the NLopt library. The objective is to control the vehicle's position, yaw, and steering angle by minimizing a quadratic cost function over a prediction horizon. The Sequential Quadratic Programming (SQP) algorithm (`NLOPT_LD_SLSQP`) is used for optimization, handling the nonlinearity in the vehicle's dynamics and constraints.

### Project Structure

- **main.cpp**: The main C++ file containing the implementation of the NLMPC and the vehicle model.
- **model.hpp**: A header file defining the `Vehicle` class and the `States` and `ControlInputs` structures for the vehicle model.
- **trajectory.csv**: Output file generated during execution, storing the vehicle's states and control inputs over time.

### Prerequisites

- **C++ Compiler**: Make sure you have a C++11-compatible compiler installed (e.g., g++, clang++).
- **NLopt Library**: Install the NLopt library for nonlinear optimization.
  - On Ubuntu:
    ```bash
    sudo apt-get install libnlopt-dev
    ```
- **CMake**: To build the project using CMake.

### Setting Up and Running the Code

1. **Clone the Repository**
   ```bash
   git clone <repository-url>
   cd <repository-directory>
   ```

2. **Build the Project**
   - Create a `build` directory:
     ```bash
     mkdir build && cd build
     ```
   - Use CMake to generate the makefile:
     ```bash
     cmake ..
     ```
   - Compile the project:
     ```bash
     make
     ```

3. **Run the Program**
   - Execute the compiled program:
     ```bash
     ./NLMPC
     ```
   - This will generate the `trajectory.csv` file containing the trajectory of the vehicle over the simulation period.

### Code Explanation

1. **NLMPC Class**:
   - The `NLMPC` class is responsible for setting up and solving the nonlinear optimization problem. It uses the NLopt library's SQP algorithm (`NLOPT_LD_SLSQP`) to minimize the objective function over the prediction horizon.
   - **Key Methods**:
     - `solve()`: Solves the optimization problem and returns the optimal control inputs.
     - `objFunc()`: The objective function to be minimized, which includes the cost associated with the vehicle's state deviation from the reference trajectory and the control effort.
     - `setBounds()`: Defines the bounds on the velocity and steering rate inputs.
     - `initOpt()`: Initializes the optimization problem by setting the objective function, tolerance values, and maximum allowed computation time per iteration.

2. **Objective Function**:
   - The objective function includes the cost terms for the x and y positions, yaw, and the combined Euclidean distance, penalizing deviations from the reference trajectory. The control effort (velocity and steering rate) is also penalized to avoid excessive control actions.

3. **Input Bounds**:
   - The constraints for the velocity and steering rate are defined to ensure the vehicle operates within safe and realistic limits.

4. **Output**:
   - The program outputs the vehicle's state and control inputs at each time step to a CSV file (`trajectory.csv`) for further analysis and visualization.

### Visualization

You can visualize the trajectory stored in `trajectory.csv` using Python with libraries like `Matplotlib` or any other data visualization tool.

### Example Python Script for Visualization

```python
import pandas as pd
import matplotlib.pyplot as plt

# Load the data
data = pd.read_csv('trajectory.csv')

# Plot the trajectory
plt.plot(data['x'], data['y'], label='Trajectory')
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('Vehicle Trajectory')
plt.legend()
plt.show()
```

### License

This project is licensed under the MIT License. See the LICENSE file for details.

### Contributing

If you find any bugs or have suggestions for improvements, feel free to open an issue or submit a pull request. Contributions are welcome!
