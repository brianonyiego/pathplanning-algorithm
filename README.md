# MATLAB Script for Forward Kinematics Simulation and 3D Visualization

## Overview
This MATLAB script performs a forward kinematics simulation for a robotic leg with three joints. It visualizes the trajectory of the leg's end-effector in 3D space, displaying the movement over specified joint angles.

## Prerequisites
- MATLAB (version R2020b or later recommended)
- Basic understanding of MATLAB plotting functions
- Knowledge of forward kinematics and robotic leg configurations

## Data Points
The script defines the lengths of the leg segments and the range of motion for each joint. It then iteratively computes the position of the end-effector based on these joint angles.

### Leg Parameters
```matlab
len          = [27.0, 48.0, 153.0];  % Lengths of the leg segments
offset_angle = deg2rad(0);           % Offset angle (radians)
offset_coord = [0, 0, 0];            % Offset coordinates (mm)
axis_limits  = [0, len(3)+len(2)+len(1), -len(3)-len(2)-len(1), len(3)+len(2)+len(1), -len(3)-len(2), len(3)+len(2)];
```

## Script Description
The script performs the following steps:
1. Initializes the leg with specified segment lengths and offset parameters.
2. Creates a 3D plot with appropriate axis limits.
3. Iteratively computes the end-effector position using forward kinematics over a range of joint angles.
4. Plots the end-effector trajectory in 3D space and in two orthographic views.

### Forward Kinematics Simulation and Plotting
```matlab
clear
cla

len          = [27.0, 48.0, 153.0];
offset_angle = deg2rad(0);
offset_coord = [0, 0, 0];
axis_limits  = [0, len(3)+len(2)+len(1), -len(3)-len(2)-len(1), len(3)+len(2)+len(1), -len(3)-len(2), len(3)+len(2)];            
Leg_1        = Leg(len, offset_coord, offset_angle);
Leg_1.createPlot(axis_limits);

% FK Simulation
x = [];
y = [];
z = [];
for theta1 = -pi/2 : pi/12 : pi/2
    for theta2 = -pi/3 : 0.25 : pi/2
        for theta3 = 0 : 0.25 : pi*(5/6)
            Leg_1.forward(theta1, theta2, theta3);
            Leg_1.updateBody()
            cla
            plot3(Leg_1.body(1, :), Leg_1.body(2, :), Leg_1.body(3, :),...
                    '-bo', ...
                    'LineWidth', 2,...
                    'MarkerSize', 4,...
                    'MarkerFaceColor', 'blue');
            pause(0.0001)
            x = [x, Leg_1.body(1, :)];
            y = [y, Leg_1.body(2, :)];
            z = [z, Leg_1.body(3, :)];
        end
    end
end

% 3D Scatter Plot
figure(2);
subplot(2,2,[3 4]);
scatter3(x, z, y, 'k.');
axis equal
grid on
xlabel('X-axis (mm)');
ylabel('Z-axis (mm)');
zlabel('Y-axis (mm)');
title('Orthographic');

subplot(2,2,1);
scatter3(x, z, y, 'k.');
view(0,0);
axis equal
grid on
xlabel('X-axis (mm)');
ylabel('Z-axis (mm)');
zlabel('Y-axis (mm)');
title('XY plane');

subplot(2,2,2);
scatter3(x, y, z, 'k.');
view(0,90);
axis equal
grid on
xlabel('X-axis (mm)');
ylabel('Z-axis (mm)');
zlabel('Y-axis (mm)');
title('XZ plane');
```

## Explanation
- **Leg Initialization**: The leg is initialized with specified segment lengths and offset parameters.
- **Forward Kinematics Calculation**: The script iteratively calculates the end-effector position for different joint angles.
- **3D Plotting**: The end-effector positions are plotted in 3D space and in two orthographic views (XY and XZ planes).
- **Visualization**: The scatter plots provide a visual representation of the leg's movement in 3D space.

## Usage
1. Ensure the `Leg` class is defined with methods for `forward` and `updateBody`.
2. Copy the script into a new MATLAB script file.
3. Run the script in MATLAB.
4. The figure window will display the 3D trajectory of the leg's end-effector and orthographic views of the movement.

## Conclusion
This script provides a comprehensive simulation of forward kinematics for a robotic leg, visualizing the end-effector's trajectory in 3D space. This can be useful for analyzing and optimizing robotic leg movements.
