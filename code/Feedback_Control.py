from modern_robotics import core
import numpy as np
from numpy import cos, sin
from math import pi
import csv


"""
Code for Milestone 3: Feedback Control.
In this code I wrote a function FeedbackControl to calculate the kinematic task-space feedforward plus feedback control law.
"""

def FeedbackControl(X, Xd, Xd_next, Kp, Ki, delta_t, current_config):
	""" This function calculates the kinematic task-space feedforward plus feedback control law
	(written both as Equation (11.16) and (13.37) in the textbook).

	Input:
	  X - The current actual end-effector configuration (Tse).
	  Xd - The current end-effector reference configuration (Tse_d).
	  Xd_next - The end-effector reference configuration at the next timestep in the reference trajectory (Tse_d_next).
	  Kp - the P gain matrix.
	  Ki - the I gain matrix.
	  delta_t - The time step Δt between reference trajectory configurations.
	  current_config - The current cunfiguration.

	Return: 
	  V - The commanded end-effector twist, expressed in the end-effector frame {e}.
	  controls - The commanded wheel and arm joint controls.
	  Xerr - The error.

	"""

	# Initialize variables:
	l = 0.47/2			# The forward-backward distance between the wheels to frame {b} [m]
	w = 0.3/2			# The side-to-side distance between the wheels to frame {b} [m]
	r = 0.0475			# The radius of each wheel [m]

	# The fixed offset from the chassis frame {b} to the base frame of the arm {0}:
	Tb0 = np.array([[ 1, 0, 0, 0.1662],
				    [ 0, 1, 0,      0],
				    [ 0, 0, 1, 0.0026],
				    [ 0, 0, 0,      1]])

	# The end-effector frame {e} relative to the arm base frame {0}, when the arm is at its home configuration:
	M0e = np.array([[ 1, 0, 0,  0.033],
				    [ 0, 1, 0,      0],
				    [ 0, 0, 1, 0.6546],
				    [ 0, 0, 0,      1]])
	
	# The screw axes B for the 5 joints expressed in the end-effector frame {e}, when the arm is at its home configuration:
	Blist = np.array([[0,  0, 1,       0, 0.0330, 0],
                      [0, -1, 0, -0.5076,      0, 0],
                      [0, -1, 0, -0.3526,      0, 0],
                      [0, -1, 0, -0.2176,      0, 0],
                      [0,  0, 1,       0,      0, 0]]).T
	
	# Get current arm joint angles:
	arm_joints = current_config[3:8]

	# Calculate the chasis configuration (according to Chapter 13.4):
	F = r/4 * np.array([[         0,         0,         0,          0],
						[         0,         0,         0,          0],
						[-1/(l + w), 1/(l + w), 1/(l + w), -1/(l + w)],
						[         1,         1,          1,         1],
						[        -1,         1,         -1,         1],
						[         0,         0,          0,        0]])
	
	# The fixed offset from the base frame of the arm {0} to the end-effector frame {e}:
	T0e = core.FKinBody(M0e, Blist, arm_joints)

	# The the transformation between end-effector frame {e} to the chassis frame {b}:
	Teb = (core.TransInv(T0e)).dot(core.TransInv(Tb0))

	# Calculate the feedforward reference twist:
	Vd = core.se3ToVec(core.MatrixLog6((core.TransInv(Xd)).dot(Xd_next))/delta_t)
	print("Vd: ", Vd)
	
	# Calculate the Adx-1xd matrix:
	ADx1xd = core.Adjoint((core.TransInv(X)).dot(Xd))
	ADx1xdVd = ADx1xd.dot(Vd)
	print("ADx1xdVd: ", ADx1xdVd)

	# Calculate the error:
	Xerr = core.se3ToVec(core.MatrixLog6((core.TransInv(X)).dot(Xd)))
	print("Xerr: ", Xerr)

	# Calculate command end effector twist (when the numerical integral of the error is Xerr * delta_t):
	V = ADx1xdVd + Kp.dot(Xerr) + Ki.dot(Xerr * delta_t)
	print("V: ", V)

	# Calculate the arm, body and end-effector Jacobian matrices:
	Ja = core.JacobianBody(Blist, arm_joints)
	Jb = (core.Adjoint(Teb)).dot(F)
	Je = np.concatenate((Jb, Ja), axis=1)
	print("Je: ", Je)

	# Calculate the wheel and arm joint controls:
	Je_inv = np.linalg.pinv(Je)
	controls = Je_inv.dot(V)
	print("controls: ", controls)

	return V, controls, Xerr


########## Testing the FeedbackControl Function ##########

# Initialize variables:
# The current configuration:
current_config = np.array([0, 0, 0, 0, 0, 0.2, -1.6, 0, 0, 0, 0, 0, 0])

# The current actual end-effector configuration:
X = np.array([[ 0.170, 0, 0.985, 0.387],
			  [     0, 1,     0,     0],
			  [-0.985, 0, 0.170, 0.570],
			  [     0, 0,     0,     1]])

# The current end-effector reference configuration:
Xd = np.array([[  0, 0, 1, 0.5],
			   [  0, 1, 0,   0],
			   [ -1, 0, 0, 0.5],
			   [  0, 0, 0,   1]])

# The end-effector reference configuration at the next timestep in the reference trajectory:
Xd_next = np.array([[ 0, 0, 1, 0.6],
					[ 0, 1, 0,   0],
					[-1, 0, 1, 0.3],
					[ 0, 0, 0,   1]])

kp_gain = 1						  # The kp gain
ki_gain = 0						  # The ki gain
Kp = np.identity(6)	* kp_gain		  # The P gain matrix
Ki = np.identity(6)	* ki_gain		  # The I gain matrix
delta_t = 0.01	  # Time step [sec]

# Calculate the commanded end-effector twist
V, speeds, Xerr = FeedbackControl(X, Xd, Xd_next, Kp, Ki, delta_t, current_config)