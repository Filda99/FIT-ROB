# Mobile Robotics Project

This repository contains the source code for a project developed for the Robotics lecture at FIT VUT in Brno. The project implements a Monte Carlo Localization (MCL) algorithm to localize a robot in a given environment.

## Monte Carlo Localization

Monte Carlo Localization is a probabilistic algorithm used to determine the position of a robot based on sensor measurements and a map of the environment. The algorithm uses a set of particles to represent possible positions of the robot and updates these particles based on sensor data and movement commands.

## How to Run the Program

Install the dependencies listed in `requirements.txt` into your environment.

To run the main program, use the following command:
```sh
python3 main_mcl.py
```

## How to Control the Robot
The robot can be controlled using the following keyboard keys:

- Left Arrow: Turn the robot left
- Right Arrow: Turn the robot right
- Up Arrow: Move the robot forward
- Escape: Close the simulator
- Additionally, you can "kidnap" the robot by clicking on the canvas with the left mouse button. This will move the robot to the clicked position.

## How It Works
- Initialization: The simulator initializes the user interface, the robot, particles, and landmarks.
- Sensor Model: The robot takes measurements from its sensors to detect landmarks in the environment.
- Particle Filter: The particles are updated based on the robot's movements and sensor measurements.
- Resampling: Particles are resampled based on their weights to focus on the more likely positions.
- Drawing: The robot, particles, and landmarks are drawn on the canvas to visualize the localization process.
- Kidnap Robot: Clicking on the canvas moves the robot to the clicked position, simulating a "kidnap" scenario.
