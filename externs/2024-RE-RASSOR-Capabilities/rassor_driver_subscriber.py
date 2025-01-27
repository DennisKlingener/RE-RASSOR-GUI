# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Modifications Copyright (c) 2022 Florida Space Institute
#     - Renamed the simple subscriber to RassorDriverSubscruber to subscribe
#       to 'ezrassor/wheel_instructions' instead of 'topic'
#     - Created the PySerial code for serial variable 'usb', and properly
#       opened and terminated it in the init function and main function
#     - Modified the callback function to support the change
#     - Created functions "bin_command_format" and "bin_command_send"
# Modified code is under the MIT License
#
#
#
# MIT License
#
# Copyright (c) 2022 Florida Space Institute

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist

import serial

class RassorDriverSubscriber(Node):
    usb = serial.Serial()
    usb.baudrate = 115200
    usb.port = "/dev/arduino"
    usb.write_timeout = 0.1

    def __init__(self):
        RassorDriverSubscriber.usb.open() # opens the serial connection
        super().__init__('rassor_driver_subscriber')
        self.get_logger().info("Starting the RE-RASSOR Serial forwarding service.\nFor more information, or to report an issue, please visit the FSI Github located at https://github.com/FlaSpaceInstitute\n\n")
        self.subscription = self.create_subscription(
            Twist,
            'ezrassor/wheel_instructions',
            self.listener_callback,
            100)
        self.subscription  # prevent unused variable warning
        self.subscription2 = self.create_subscription(
            Twist,
            'ezrassor/shoulder_instructions',
            self.listener_callback2,
            100)
        self.subscription2  # prevent unused variable warning
    

    def listener_callback(self, msg):
        self.get_logger().info('linear.x: %f' % msg.linear.x)
        self.get_logger().info('angular.z: %f' % msg.angular.z)
        lin_x = msg.linear.x
        ang_z = msg.angular.z
        self.bin_command_format (lin_x, ang_z)
    def listener_callback2(self, msg):
        self.get_logger().info('linear.y: %f' % msg.linear.y)
        self.get_logger().info('angular.y: %f' % msg.angular.y)
        lin_y = msg.linear.y
        ang_y = msg.angular.y
        self.bin_command_format_shoulder(lin_y, ang_y)
    def info_listener_callback(self,msg):
        self.get_logger().info(f"Received rover info: {msg.data}")
        try:
            rover_info = json.loads(msg.data)
            self.get_logger().info(f"Rover IP: {rover_info.get('ip')}")
            self.get_logger().info(f"Rover Speed: {rover_info.get('speed')}")
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse rover info: {e}")

    def bin_command_format(self, lin_x, ang_z):
        # makes our binary command
        packet = bytearray()
        if lin_x == 0.0 and ang_z == 0.0:
            packet.append(0x00) # coast
        elif lin_x == 99:
            packet.append(0x13) # Servo snap left
        elif lin_x == -99:
            packet.append(0x14) # Servo Snap Right
        elif lin_x == 50:
            packet.append(0x15) # Servo center
        elif lin_x > 0:
            packet.append(0x01) # forwards
        elif lin_x < 0:
            packet.append(0x02) # backwards
        elif ang_z > 0:
            packet.append(0x03) # left
        else:
            packet.append(0x04) # right
        self.bin_command_send(packet)
    def bin_command_format_shoulder(self, lin_y, ang_y):
        # makes our binary command
        packet = bytearray()
        if lin_y == 0.0 and ang_y == 0.0:
            packet.append(0x00) # Do nothing
        elif lin_y > 0:
            packet.append(0x05) # Raise Front
        elif lin_y < 0:
            packet.append(0x06) # Lower Front
        elif ang_y > 0:
            packet.append(0x07) # Raise Back
        else:
            packet.append(0x08) # Lower Back
        self.bin_command_send(packet)
    def bin_command_send (self, packet):
        try:
            self.get_logger().info("Sending packet: " + str(packet))
            RassorDriverSubscriber.usb.write(packet)
        except Exception as e:
            self.get_logger().info("Write timeout from input flooding. Flushing buffers.")
            RassorDriverSubscriber.usb.flushInput()
            RassorDriverSubscriber.usb.flushOutput()
            pass

def main(args=None):
    rclpy.init(args=args)

    rassor_driver_subscriber = RassorDriverSubscriber()

    rclpy.spin(rassor_driver_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    RassorDriverSubscriber.usb.close()
    rassor_driver_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
