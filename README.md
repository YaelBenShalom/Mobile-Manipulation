# Mobile-Manipulation
GitHub repository - `https://github.com/YaelBenShalom/Mobile-Manipulation`


## Overview

This project is the final project in the course [ME-449 - Robotic Manipulation](http://hades.mech.northwestern.edu/index.php/ME_449_Robotic_Manipulation). In this project I wrote a software that plans a trajectory for the end-effector of the youBot mobile manipulator, performs odometry as the chassis moves, and performs feedback control to drive the youBot to pick up a block at a specified location, carry it to a desired location, and put it down in the V-REP simulation softwarw. 

The projects description can be found [here](http://hades.mech.northwestern.edu/index.php/Mobile_Manipulation_Capstone#Milestone_1:_youBot_Kinematics_Simulator_and_csv_Output).

The projects covers the following topics:<br>
1. Generates the kinematics simulator of the youBot - an omnidirectional mobile robot with a 5-DOF robotic arm.<br>
2. Plans the end-effector's trajectory between waypoints.<br>
3. Apply feedback control to drive the robot in the desired trajectory.<br>
4. Simulate the planned trajectory in CoppeliaSim.<br>


## Package Description

This project contains 3 milestones, and was implemented in the code `Full_Program.py`. The main code calls the 3 milestone sub-code:<br>
1. `Next_State.py` - Milestone 1 code. This code uses a function `NextState()`, that compute the configuration of the robot in the next time step.<br>
2. `Trajectory_Generator.py` - Milestone 2 code. This code uses a function `TrajectoryGenerator()`, that generates the reference trajectory for the end-effector frame {e}.<br>
3. `Feedback_Control.py` - Milestone 3 code. This code uses a function `FeedbackControl()`, that calculates the kinematic task-space feedforward plus feedback control law.<br>

The code also uses functions from the `Modern Robotics` library. This library can be downloaded from this git repository - `https://github.com/NxRLab/ModernRobotics`.



## Usage and Configuration instructions

1. To run the program, download the `modern robotics` library (the python version) from `https://github.com/NxRLab/ModernRobotics/tree/master/packages/Python`, by running:
```
pip install modern_robotics
``` 

2. Run the `Full_Program.py` code.


## Results

The results for this projects can be split into 3 categories:

1. `Best` results - Planning and executing a motion without overshoot or steady-state error.<br>
2. `Overshoot` results - Planning and executing a motion with overshoot but without steady-state error.<br>
3. `NewTast` results - Planning and executing the trajectory with different start and finish configuration.<br>


### Motion planning with different initial and final cube location:

For this part, I simulated the the controlled motion with different initial and final cube location, and different initial robot's configuration.

1. For this part, I used the following initial and final conditions:

    Initial robot's configuration:<br>
    initial_config = np.array([0.1, -0.2, 0, 0, 0, 0.2, -1.6, 0, 0, 0, 0, 0, 0])

    Initial cube location:<br>
    Tsc_initial =<br>
    [1, 0, 0,     1],<br>
    [0, 1, 0,     0],<br>
    [0, 0, 1, 0.025],<br>
    [0, 0, 0,     1]])

    Final cube location:<br>
    Tsc_initial =<br>
    [ 0, 1, 0,     0],<br>
    [-1, 0, 0,    -1],<br>
    [ 0, 0, 1, 0.025],<br>
    [ 0, 0, 0,     1]])<br>

    The simulated controlled motion with the following condition is:

    ![best simulation](https://github.com/YaelBenShalom/Mobile-Manipulation/blob/main/results/best/best.gif)

2. For this part, I used the following initial and final conditions:

    Initial robot's configuration:<br>
    initial_config = np.array([pi/6, -0.5, 0, 0, 0, 0.2, -1.6, 0, 0, 0, 0, 0, 0])

    Initial cube location:<br>
    Tsc_initial =<br>
    [1, 0, 0,     1],<br>
    [0, 1, 0,     1],<br>
    [0, 0, 1, 0.025],<br>
    [0, 0, 0,     1]])

    Final cube location:<br>
    Tsc_initial =<br>
    [ 0, 1, 0,     1],<br>
    [-1, 0, 0,    -1],<br>
    [ 0, 0, 1, 0.025],<br>
    [ 0, 0, 0,     1]])<br>

    The simulated controlled motion with the following condition is:

    ![best simulation](https://github.com/YaelBenShalom/Mobile-Manipulation/blob/main/results/newTask/newTask.gif)


### Motion planning with different control gains:

For this part, I plotted the end-effector's twist error as a function of time.

1. For PI controller with feedback gains of Kp = 20 and Ki = 5, The error plot is:

    ![Xerr ,kp=20, ki=5](https://github.com/YaelBenShalom/Mobile-Manipulation/blob/main/results/best/Xerr%2Ckp%3D20%2Cki%3D5.png)

    We can see that there is no overshoot, no steady-state error, and fast settling time.


2. For PI controller with feedback gains of Kp = 2 and Ki = 40, The error plot is:

    ![Xerr, kp=2, ki=40.png](https://github.com/YaelBenShalom/Mobile-Manipulation/blob/main/results/overshoot/Xerr%2Ckp%3D2%2Cki%3D40.png)

    We can see that there is an overshoot at the beginning of the motion, no steady-state error, and fast settling time.