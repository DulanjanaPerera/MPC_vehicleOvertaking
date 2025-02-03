# Nonlinear Model Predictive Control for Autonomous Vehicle Overtaking

This repository contains a MATLAB implementation of a Nonlinear Model Predictive Control (NMPC) system designed to manage autonomous vehicle overtaking maneuvers. The system is capable of handling unforeseen obstacles and dense traffic conditions by adhering to predefined safety protocols.

## Table of Contents

- [Introduction](#introduction)
- [Features](#features)
- [System Requirements](#system-requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Project Structure](#project-structure)
- [Simulation Scenarios](#simulation-scenarios)
- [Results](#results)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgments](#acknowledgments)

## Introduction

Autonomous driving necessitates advanced control systems to ensure safety and efficiency, especially during complex maneuvers like overtaking. This project leverages NMPC to predict and optimize vehicle trajectories, ensuring safe overtaking by considering vehicle dynamics and environmental constraints.

## Features

- **Dynamic Obstacle Avoidance:** Adjusts vehicle trajectory in real-time to avoid collisions.
- **Traffic Handling:** Manages overtaking in both single-lane and two-lane traffic scenarios.
- **Parking Maneuver:** Demonstrates the controller's capability to park the vehicle at a specified location.
- **Customizable Parameters:** Allows users to modify vehicle and simulation parameters to test various scenarios.

## System Requirements

- MATLAB R2023a or later
- Optimization Toolbox
- Control System Toolbox

## Installation

1. **Clone the Repository:**

   ```bash
   git clone https://github.com/DulanjanaPerera/MPC_vehicleOvertaking.git
   cd MPC_vehicleOvertaking
   ```

2. **Set Up MATLAB Environment:**

   - Add the cloned directory to the MATLAB path:

     ```matlab
     addpath(genpath('path_to_cloned_repository'));
     ```

## Usage

1. **Configure Simulation Parameters:**

   - Open `nlmpc_main.m`.
   - Set the initial state of the vehicle:

     ```matlab
     x = [0; -2.5; 0; 0]; % [x_position; y_position; heading_angle; velocity]
     y = x;
     ```

   - Define obstacle states (other vehicles):

     ```matlab
     obsState = [10, -2.5, 0, 0;
                 30, -2.5, 0, 0;
                 30, 2.5, pi, 0];
     ```

   - Set the reference point:

     ```matlab
     yref = [60; -2.5; 0; 0];
     ```

   - Specify obstacle velocities:

     ```matlab
     obsV = 10; % Velocity of other vehicles
     ```

2. **Run the Simulation:**

   - Execute `nlmpc_main.m` to start the simulation.

3. **Visualize Results:**

   - Use the `plotting` function to visualize the vehicle's trajectory:

     ```matlab
     plotting(yref, obsHistory, xHistory, time_obs, lane_length, nlobj);
     ```

## Project Structure

- `nlmpc_main.m`: Main script to initialize parameters and run the NMPC simulation.
- `ObstacleConstraint.m`: Defines constraints related to obstacle avoidance.
- `discreteStateEq.m`: Contains the discrete state equations for the vehicle model.
- `discretestateJacobian.m`: Computes the Jacobian of the discrete state equations.
- `outputFunc.m`: Defines the output function for the NMPC.
- `outputJacobian.m`: Computes the Jacobian of the output function.
- `plotting.m`: Functions for visualizing simulation results.
- `vehicle_drawing.m`: Utility for rendering the vehicle in plots.

## Simulation Scenarios

The project includes simulations for various scenarios:

1. **Heavy Traffic in One Direction:** The vehicle navigates through dense traffic moving in the same direction.
2. **Two-Way Traffic:** The vehicle overtakes in the presence of oncoming traffic.
3. **Parking:** The vehicle maneuvers to park at a designated spot.

## Results

The NMPC controller demonstrates effective performance across different scenarios, ensuring safety and adherence to constraints. Detailed results and discussions are available in the [Final Project Report](ENTC%20689%20-%20Final%20Project%20Report%20-%20Dulanjana%20Perera.pdf).

## Contributing

Contributions are welcome! Please fork the repository and submit a pull request with your proposed changes. Ensure that your code adheres to the existing style and includes appropriate comments.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Acknowledgments

This project was developed as part of the ENTC 689 – Applied Modeling, Controlling, and Optimization Techniques in Automation Industries course at Texas A&M University. Special thanks to the course instructors and peers for their support and feedback.

---

For a visual demonstration of the system's capabilities, watch the [project video](https://youtu.be/wrtBWJXG60M).

