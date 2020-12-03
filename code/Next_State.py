from modern_robotics import core
import numpy as np
from numpy import cos, sin
from math import pi
import csv


"""
Code for Milestone 1: Next State.
In this code I wrote a function NextState to simulate the kinematics of the youBot.
This function compute the configuration of the robot in the next time step.
The next step is saved in an csv file 'next_state.csv'.
"""

def NextState(current_config, speeds, delta_t, max_ang_speed):
	""" This function compute the configuration of the robot in the next time step.

	The function NextState is based on a simple first-order Euler step, i.e.:
	- new arm joint angles = (old arm joint angles) + (joint speeds) * Δt		(Equation 1)
	- new wheel angles = (old wheel angles) + (wheel speeds) * Δt				(Equation 2)
	- new chassis configuration is obtained from odometry (as described in Chapter 13.4)

	Input:
	  current_config - A 12-vector representing the current configuration of the robot (3 variables for
	  				the chassis configuration, 5 variables for the arm configuration, and 4 variables
					for the wheel angles).
	  speeds - A 9-vector of controls indicating the arm joint speeds theta_dot (5 variables) and the
	  				wheel speeds u (4 variables).
	  delta_t - The time step Δt.
	  max_ang_speed - A positive real value indicating the maximum angular speed of the arm joints and
	  				the wheels.

	Return: 
	  new_config - A 12-vector representing the configuration of the robot time Δt later.
	"""
	# Initialize variables:
	l = 0.47/2			# The forward-backward distance between the wheels to frame {b} [m]
	w = 0.3/2			# The side-to-side distance between the wheels to frame {b} [m]
	r = 0.0475			# The radius of each wheel [m]

	# Get current chassis configuration, arm configuration (joints angles) and wheel angles:
	current_q = current_config[:3]
	current_joint_ang = current_config[3:8]
	current_wheel_ang = current_config[8:12]

	# Restrict the speeds executed by the wheels and joints to the maximum speed:
	for i in range(len(speeds)):
		abs_speed = abs(speeds[i])
		if abs_speed > max_ang_speed:
			speeds[i] = speeds[i]/abs_speed * max_ang_speed
	
	# Get current arm joint speeds and wheel speeds (with restricted speeds):
	theta_dot = speeds[:5]
	u = speeds[5:]

	# Calculate new arm joint angles and wheel angles (according to equtions 1,2):
	new_joint_ang = current_joint_ang + theta_dot * delta_t
	new_wheel_ang = current_wheel_ang + u * delta_t

	# Calculate new chasis configuration (according to Chapter 13.4):
	F = r/4 * np.array([[-1/(l + w), 1/(l + w), 1/(l + w), -1/(l + w)],
						[ 1,  1,  1,  1],
						[-1,  1, -1,  1]])
	Vb = F.dot((u * delta_t).T).T
	w_bz, v_bx, v_by = Vb

	if w_bz == 0.:
		delta_qb = np.array([0, v_bx, v_by]).T
	else:
		delta_qb = np.array([w_bz, (v_bx * sin(w_bz) + v_by * (cos(w_bz) - 1))/w_bz,
							(v_by * sin(w_bz) + v_bx * (1 - cos(w_bz)))/w_bz])

	# Transforming the ∆q b in {b} to ∆q in the fixed frame {s} using the chassis angle:
	chassis_angle = current_config[0]
	Tsb = np.array([[1, 0, 0],
					[0, cos(chassis_angle), -sin(chassis_angle)],
					[0, sin(chassis_angle), cos(chassis_angle)]])
	delta_q = Tsb.dot(delta_qb.T)

	# Calculating new chasis configuration:
	new_q = current_q + delta_q

	# Combining the three vectors to the new configuration vector:
	new_config = np.concatenate((new_q, new_joint_ang, new_wheel_ang), axis=None)
	
	return new_config


########## Testing the NextState function ##########

# Initialize variables:
# The initial configuration of the youBot(3 chassis configuration variables, 5 arm joint angles, 4 wheel angles, and "0" for "gripper open"):
current_config = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

# The speeds vector (arm joint speeds = (0, 0, 0, 0, 0), wheel speeds = (-10, 10, -10, 10)):
speeds = np.array([0, 0, 0, 0, 0, -10, 10, -10, 10]) # The robot chassis should slide sideways in the y_b direction

# Restrictions on the speeds vector:
max_ang_speed = 5

# Time variables:
delta_t = 0.01						# Time step [sec]
t_total = 1							# Simulation run time [sec]
iteration = int(t_total/delta_t)	# Number of iterations

# Initialize configuration list (with current_config as the first raw):
config_list = np.zeros((iteration, 13))
config_list[0] = current_config

# Calculate the new configuration for every iteration:
for i in range(1, iteration):
	current_config = NextState(current_config, speeds, delta_t, max_ang_speed)
	config_list[i][:12] = current_config

# Save the 13-segment of new configurations as a csv file:
with open("next_state.csv","w+") as my_csv:
	csvWriter = csv.writer(my_csv, delimiter=',')
	csvWriter.writerows(config_list)