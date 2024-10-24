#!/usr/bin/env python3

'''
This python file runs a ROS 2-node of name pico_control which holds the position of Swift Pico Drone on the given dummy.
This node publishes and subscribes to the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/pid_error			/throttle_pid
						/pitch_pid
						/roll_pid
					
Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

# Importing the required libraries

from swift_msgs.msg import SwiftMsgs
from geometry_msgs.msg import PoseArray
from pid_msg.msg import PIDTune, PIDError
import rclpy
from rclpy.node import Node


class Swift_Pico(Node):
    def __init__(self):
        super().__init__('pico_controller')  # initializing ros node with name pico_controller

        # Current position of drone. This value must be updated each time in your whycon callback
        # [x, y, z]
        self.drone_position = [0.0, 0.0, 0.0]

        # Setpoints for [x_setpoint, y_setpoint, z_setpoint]
        self.setpoint = [2, 2, 19]  # target positions for the drone

        # Declaring a cmd of message type swift_msgs and initializing values
        self.cmd = SwiftMsgs()
        self.cmd.rc_roll = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_throttle = 1500

        # Initial PID settings for [roll, pitch, throttle]
        self.Kp = [0.0, 0.0, 0.0]
        self.Ki = [0.0, 0.0, 0.0]
        self.Kd = [0.0, 0.0, 0.0]

        # Error variables
        self.error = [0.0, 0.0, 0.0]
        self.sum_error = [0.0, 0.0, 0.0]
        self.diff_error = [0.0, 0.0, 0.0]
        self.prev_error = [0.0, 0.0, 0.0]

        # Command limits
        self.max_values = [2000, 2000, 2000]  # limits for roll, pitch, throttle
        self.min_values = [1000, 1000, 1000]  # limits for roll, pitch, throttle

        # Sample time for PID control
        self.sample_time = 0.060  # in seconds

        # Publishing /drone_command and /pid_error
        self.command_pub = self.create_publisher(SwiftMsgs, '/drone_command', 10)
        self.pid_error_pub = self.create_publisher(PIDError, '/pid_error', 10)
        self.pid_error = PIDError()

        # Subscribing to required topics
        self.create_subscription(PoseArray, '/whycon/poses', self.whycon_callback, 1)
        self.create_subscription(PIDTune, "/throttle_pid", self.altitude_Set_pid, 1)
        self.create_subscription(PIDTune, "/roll_pid", self.roll_Set_pid, 1)
        self.create_subscription(PIDTune, "/pitch_pid", self.pitch_Set_pid, 1)

        self.arm()  # ARMING THE DRONE
        self.timer = self.create_timer(self.sample_time, self.pid)  # Timer to run the PID function periodically

    def disarm(self):
        self.cmd.rc_roll = 1000
        self.cmd.rc_yaw = 1000
        self.cmd.rc_pitch = 1000
        self.cmd.rc_throttle = 1000
        self.command_pub.publish(self.cmd)

    def arm(self):
        self.disarm()
        self.cmd.rc_roll = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_throttle = 1500
        self.cmd.rc_aux4 = 2000
        self.command_pub.publish(self.cmd)  # Publishing /drone_command

    # Whycon callback function
    def whycon_callback(self, msg):
        self.drone_position[0] = msg.poses[0].position.x 
        self.drone_position[1] = msg.poses[0].position.y
        self.drone_position[2] = msg.poses[0].position.z

    # Callback function for /throttle_pid
    def altitude_Set_pid(self, alt):
        self.Kp[2] = alt.kp * 0.03
        self.Ki[2] = alt.ki * 0.008
        self.Kd[2] = alt.kd * 0.5

    # Define callback functions to tune pitch and roll
    def roll_Set_pid(self, roll):
        self.Kp[0] = roll.kp * 0.01 * (-1)
        self.Ki[0] = roll.ki * 0.008
        self.Kd[0] = roll.kd * 0.2

    def pitch_Set_pid(self, pitch):
        self.Kp[1] = pitch.kp * 0.03
        self.Ki[1] = pitch.ki * 0.008
        self.Kd[1] = pitch.kd * 0.6

    def pid(self):
        # Compute errors for each axis
        self.error[0] = self.drone_position[0] - self.setpoint[0]  # Error in x
        self.error[1] = self.drone_position[1] - self.setpoint[1]  # Error in y
        self.error[2] = self.drone_position[2] - self.setpoint[2]  # Error in z 

        # Implement PID control for each axis
        for i in range(3):  # Loop through roll, pitch, throttle
            self.sum_error[i] += self.error[i]
            self.diff_error[i] = self.error[i] - self.prev_error[i]
            self.prev_error[i] = self.error[i]

            # Calculate PID output
            output = self.Kp[i] * self.error[i] + self.Ki[i] * self.sum_error[i] + self.Kd[i] * self.diff_error[i]

            # Adjust command values for roll, pitch, and throttle
            if i == 0:  # Roll
                self.cmd.rc_roll = int(1500 + output)
                # Limit roll output
                self.cmd.rc_roll = max(self.min_values[0], min(self.max_values[0], self.cmd.rc_roll))
            elif i == 1:  # Pitch
                self.cmd.rc_pitch = int(1500 + output)
                # Limit pitch output
                self.cmd.rc_pitch = max(self.min_values[1], min(self.max_values[1], self.cmd.rc_pitch))
            elif i == 2:  # Throttle
                self.cmd.rc_throttle = int(1500 + output)
                # Limit throttle output
                self.cmd.rc_throttle = max(self.min_values[2], min(self.max_values[2], self.cmd.rc_throttle))

        # Publish the command and errors
        self.command_pub.publish(self.cmd)
        self.pid_error.roll_error = self.error[0]
        self.pid_error.pitch_error = self.error[1]
        self.pid_error.throttle_error = self.error[2]
        self.pid_error_pub.publish(self.pid_error)


def main(args=None):
    rclpy.init(args=args)
    swift_pico = Swift_Pico()
    rclpy.spin(swift_pico)
    swift_pico.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
