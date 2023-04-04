#!/usr/bin/env python3
import rclpy
import numpy as np
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import Parameter, ParameterType, ParameterDescriptor
from actuator_msgs.msg import Actuators
from sensor_msgs.msg import Joy

class JoyTestMixerActuators(Node):
    def __init__(self):
        super().__init__('joy_test_mixer_actuators_node')


        joy_input_topic_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='joy input topic name.')

        actuators_output_topic_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='actuators output topic name.')

        joy_scale_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Scale value for joy input.')

        thrust_axes_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER,
            description='Thrust axes on joystick.')

        yaw_axes_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER,
            description='Yaw axes on joystick.')

        arm_button_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER,
            description='Arm button on joystick.')

        disarm_button_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER,
            description='Disarm button on joystick.')

        mix_thrust_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE_ARRAY,
            description='Mixer for thrust to actuators.')

        mix_yaw_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE_ARRAY,
            description='Mixer for yaw to actuators.')

        self.declare_parameter("joy_input_topic", "/joy", 
            joy_input_topic_descriptor)

        self.declare_parameter("actuators_output_topic", "/actuators",
            actuators_output_topic_descriptor)

        self.declare_parameter("joy_scale", 500, 
            joy_scale_descriptor)

        self.declare_parameter("thrust_axes", 1, 
            thrust_axes_descriptor)

        self.declare_parameter("yaw_axes", 3, 
            yaw_axes_descriptor)

        self.declare_parameter("arm_button", 7, 
            arm_button_descriptor)

        self.declare_parameter("disarm_button", 6, 
            disarm_button_descriptor)

        self.declare_parameter("mix_thrust", [1.0, 1.0, 1.0, 1.0], 
            mix_thrust_descriptor)

        self.declare_parameter("mix_yaw", [-1.0, 1.0, 1.0, -1.0], 
            mix_yaw_descriptor)

        self.MixYaw = self.get_parameter("mix_yaw").value
        self.MixThrust = self.get_parameter("mix_thrust").value

        self.ThrustAxes = self.get_parameter("thrust_axes").value
        self.YawAxes = self.get_parameter("yaw_axes").value

        self.ArmButton = self.get_parameter("arm_button").value
        self.DisarmButton = self.get_parameter("disarm_button").value

        self.JoyScale = self.get_parameter("joy_scale").value
        self.ActuatorsOutputTopic = self.get_parameter("actuators_output_topic").value
        self.JoyInputTopic = self.get_parameter("joy_input_topic").value

        self.Armed = False

        self.JoySub = self.create_subscription(Joy, '{:s}'.format(self.JoyInputTopic), self.JoyCallback, 1)
        self.ActuatorsPub = self.create_publisher(Actuators, '{:s}'.format(self.ActuatorsOutputTopic), 0)

    def JoyCallback(self, msgJoy):
        if not self.Armed and msgJoy.buttons[self.ArmButton] > 0:
            self.Armed = True
        if self.Armed and msgJoy.buttons[self.DisarmButton] > 0:
            self.Armed = False

        if self.Armed:
            msg = Actuators()
            vel=[]
            for i in range(len(self.MixThrust)):
                mix = float(self.MixThrust[i]*msgJoy.axes[self.ThrustAxes] + self.MixYaw[i]*msgJoy.axes[self.YawAxes])
                if mix > 1.0:
                    mix = 1.0
                vel.append(float(self.JoyScale)*mix)
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "can0"
            msg.velocity = vel
            self.ActuatorsPub.publish(msg)
        return

def main(args=None):
    rclpy.init()
    JTMA = JoyTestMixerActuators()
    rclpy.spin(JTMA)
    JTMA.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()