# 6-DOF Robot Arm Sphere Tracing Simulation

A MATLAB App simulating a 6-DOF robotic arm tracing a user-defined circle on the surface of a sphere.

## Overview

For the final project of EEL4660 - Robotic Systems (Fall 2024), this project explores the kinematics and simulation of the [GS Automatic 6DOF Industrial Robotic Arm 10KG Payload Cobot Robot](https://www.google.com/search?q=https://www.gsautomatic.com/automatic-6dof-industrial-robotic-arm-10kg-payload-cobot-robot-collaborative-robot/). The project focuses on modeling the robot, implementing forward and inverse kinematics, and animating its end-effector as it traces a 3D path on a sphere's surface using a custom MATLAB App Designer GUI.

## Key Features

- Interactive GUI: Easily set the path parameters (circle radius, latitude, longitude) and start the simulation.
- 3D Visualization: Renders the 6-DOF robot arm, the target sphere, and the calculated circular path in a 3D plot.
- Inverse Kinematics (IK) Calculation: Solves the inverse kinematics for each point along the path to determine the necessary joint angles. The end-effector is oriented to be normal to the sphere's surface at every point.
- Path Animation: Animates the robot model moving its end-effector along the traced circle.
- Real-time Validation: Provides immediate feedback if the specified path is unreachable or if the inverse kinematics solution fails to converge.

## Approach & Implementation

The simulation is built entirely within MATLAB App Designer, leveraging the powerful robotics and visualization tools available.

### Robot Modeling

The 6-DOF robot arm is modeled using Denavit-Hartenberg (DH) parameters. Each of the six links is defined using the `Link` object from the Peter Corke Robotics Toolbox. These links are then combined into a `SerialLink robot` object, which serves as the basis for all kinematic calculations and plotting.

```MATLAB
% Define DH parameters for each joint
L1 = Link('d',      0, 'a',     0, 'alpha',  pi/2, 'qlim', deg2rad([-175, 175]));
L2 = Link('d',    197, 'a',   647, 'alpha',     0, 'qlim', deg2rad([-175, 175]));
L3 = Link('d', -123.5, 'a', 600.5, 'alpha',     0, 'qlim', deg2rad([-175, 175]));
L4 = Link('d',  127.8, 'a',     0, 'alpha',  pi/2, 'qlim', deg2rad([-175, 175]));
L5 = Link('d',  102.5, 'a',     0, 'alpha', -pi/2, 'qlim', deg2rad([-175, 175]));
L6 = Link('d',     94, 'a',     0, 'alpha',     0, 'qlim', deg2rad([-175, 175]));

% Combine links into a SerialLink robot
app.Robot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', '6DOF Robot');
```

### Path Generation

The user specifies a circle using latitude, longitude, and radius. The app calculates 100 equidistant points along this circle on the sphere's surface. This is achieved by:

1. Calculating the normal vector to the sphere at the given latitude and longitude.
2. Finding the center of the circle along this normal vector.
3. Defining a plane for the circle using tangent vectors.
4. Generating points along the circle's circumference in this plane.

### Kinematics and Animation

For each point on the generated path, the application performs the following steps:

1. Define Target Pose: A target transformation matrix (Ttarget​) is created. The position is the point on the circle, and the orientation is calculated so the end-effector's z-axis is normal to the sphere's surface.
2. Solve Inverse Kinematics: The `ikine` function from the Robotics Toolbox is used to find the joint angles (q) required to achieve the target pose. This calculation is wrapped in a `try-catch` block to handle cases where a solution cannot be found.
3. Animate: If valid IK solutions are found for all points, the app clears the axes and plots the sphere and the path. It then iterates through the sequence of joint angles, calling the `robot.plot()` method at each step to create a smooth animation.

## Getting Started
### Prerequisites

- MATLAB (R2020a or newer recommended)
- MATLAB App Designer (included with MATLAB)
- Robotics Toolbox for MATLAB by Peter Corke. You can install it from the Add-On Explorer or download it from his website.

### Installation & Usage

1. Clone this repository or download the source code.
2. Open MATLAB.
3. Navigate to the project directory.
4. Open the `6DOF_Sim.mlapp` file. This will launch the MATLAB App Designer.
5. Click the Run button in the App Designer's toolbar.
6. In the app window, enter the desired Radius, Latitude, and Longitude.
7. Click the Animate button to start the simulation.

## Code Structure

The app's logic is contained within the `6DOF_Sim.mlapp` classdef file and is organized into several key private methods:

- `startupFcn()`: Executes on app launch; calls `initializeRobot()`.
- `initializeRobot()`: Defines the robot's DH parameters and creates the SerialLink object.
- `findEquidistantPoints()`: Calculates the 3D coordinates of the circle to be traced.
- `calculations()`: The core function that iterates through path points, solves inverse kinematics, and validates reachability.
- `plot()`: Handles all plotting, including rendering the sphere, the path, and animating the robot's movement.
- `validRange()`: Checks user inputs to ensure they are within logical bounds (e.g., latitude is between -90 and 90).
- Callback Functions (`...ValueChanged`, `AnimateButtonPushed`): These functions are triggered by user interactions with the GUI components.

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.
