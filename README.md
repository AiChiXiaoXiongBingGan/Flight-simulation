# 🚀 Flight Control Simulation Based on PID Algorithm

[![Java](https://img.shields.io/badge/language-Python-blue.svg)](https://www.oracle.com/java/)

## 📝 Description

This project simulates flight control using the PID(Proportional Integral derivative) algorithm. The purpose of the simulation is to demonstrate the principle of PID control and how to apply PID control to maintain stable flight under various conditions. The project is based on the current open source library, as well as existing data for development.
## 🎯 Features

- **PID Control**: Implemented PID algorithm for flight stabilization.
- **Simulation Environment**: Visual and interactive simulation environment.
- **Parameter Tuning**: Adjustable PID parameters for experimentation.
- **Real-time Feedback**: Real-time visualization of control response and error correction.


## 🚀 Usage

You can choose to run different simulations, but we recommend trying the two main ones:

1. **Autopilot Controller with Airspeed, Heading, and Altitude Command**: Allows manual control of the aircraft.
2. **Path Following Autopilot**: Autopilot follows a provided path.

## 🎮 Controller

To run this simulation, execute the `_autopilot_fail-safe_` file located in the `control` folder.

### PS4 Controller Integration

- **Analog Controls**:
  - Left Analog: Controls both the ailerons and the elevator.
  - Right Analog: Controls the rudder.
- **Buttons**:
  - R2: Controls the throttle.
  - Circle (⚪): Turns the autopilot off.
  - Triangle (🔺): Turns the autopilot on.

When the autopilot is on, you can define the input in the window that will open along with the simulation.

## 🗺️ Path Following

To run this simulation, execute the `_path_following_simulation.py_` file located in the `path_following` folder.

### Path Options

- **Straight Line Path**
- **Orbital Path**

You can define the path to follow inside the `_path_following_simulation.py_` file.

## 🧠 PID Algorithm

The PID algorithm is a control loop mechanism employing feedback that is widely used in industrial control systems.

- **Proportional (P)**: Determines the reaction to the current error.
- **Integral (I)**: Determines the reaction based on the sum of recent errors.
- **Derivative (D)**: Determines the reaction based on the rate at which the error has been changing.

## 📸 Screenshots

![Simulation Running](https://github.com/yourusername/yourrepository/raw/main/screenshots/simulation_running.png)
*Figure 1: Flight control simulation in action.*


