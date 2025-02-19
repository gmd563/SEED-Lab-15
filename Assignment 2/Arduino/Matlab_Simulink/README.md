# MatLab Simulink
This has all of the files that were used to create controllers for the motors.

# ReadFromArdunio.mlx
This takes in tab seperated value data from the ardunio and makes it into data in matlab.

# motor_control.slx
This is our simulink model with the proportional controller for our motors.

# motorsim.slx
This is a simulink model that models our motors.

# readposition.m
This uses position data from the ardunio to graph a path of the movement of the robot.

# runmotorcontrol.m
This plotted a graph of the controlled motor in order to tune the proportional values.

# runmotorsimleft.m and runmotorsimright.m
These plotted the response of the motors from actual testing compared to the simulation in motorsim.slx in order to determine the step response of the motors.
