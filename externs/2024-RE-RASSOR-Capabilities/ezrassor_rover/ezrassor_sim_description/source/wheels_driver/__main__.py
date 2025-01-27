"""Execute a ROS node using the wheels_driver module.

This node reads messages from a custom topic (external), parses them into
the value that Gazebo expects, and then publishes these messages
to Gazebo (internal). For the wheels driver, these messages are Twist values
that represent the linear and angular velocity that should be applied to wheels.

The abstraction done by this node is necessary to be able to switch
between hardware and software; a similarly written hardware node could
listen to the same instructions topic and command actuators, instead of the sim.
"""
import geometry_msgs.msg
import rclpy
from rclpy.node import Node
import socket

def get_ip_address():
    try:
        # Create a dummy socket connection to get the IP address
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Connect to a public IP address (Google's DNS), but we don't actually send any data
        s.connect(("8.8.8.8", 80))
        ip_address = s.getsockname()[0]
        s.close()
        formatted_ip =  "ip_" + ip_address.replace(".", "_")
        return formatted_ip
    except Exception as e:
        self.get_logger().error(f"Error finding IP for potato: {e}")
        return None  # Return None or a default value
ROVER_NAME = get_ip_address()  # Unique rover name for each instance
WHEELS_EXTERNAL_TOPIC = f"/{ROVER_NAME}/wheel_instructions" # move other topics later
WHEELS_INTERNAL_TOPIC = "diff_drive_controller/cmd_vel_unstamped"
MAX_LINEAR_VELOCITY = 6
LINEAR_VELOCITY_INCREMENT = MAX_LINEAR_VELOCITY / 3
MAX_ANGULAR_VELOCITY = 6
ANGULAR_VELOCITY_INCREMENT = MAX_ANGULAR_VELOCITY / 3
QUEUE_SIZE = 10
PUBLISH_INTERVAL = 0.1  # Time in seconds between publish 


class WheelsSubscriber(Node):
    """Create a WHEELS_EXTERNAL_TOPIC subscriber node"""

    def __init__(self):
        """
        Class constructor for this subscriber node
        """
        super().__init__("wheels_driver")

        self.last_twist = geometry_msgs.msg.Twist()

        self.subscription = self.create_subscription(
            geometry_msgs.msg.Twist,
            WHEELS_EXTERNAL_TOPIC,
            self.handle_wheel_movements,
            QUEUE_SIZE,
        )

        self.simulation_publisher = self.create_publisher(
            geometry_msgs.msg.Twist, WHEELS_INTERNAL_TOPIC, QUEUE_SIZE
        )

        self.timer = self.create_timer(PUBLISH_INTERVAL, self.publish_last_twist)
        
        self.get_logger().info("wheels_driver node created successfully")

    def handle_wheel_movements(self, twist):
        """
        Callback function which places limits on the speed passed in before publishing
        to the simulation diff_drive topic to move the robot
        """
        
        if(twist.linear.x == 0 and twist.angular.z == 0):
            self.last_twist.linear.x = 0.0
            self.last_twist.angular.z = 0.0

        # Make sure we can increment and decrement the speed without going over the maximum
        if(abs(self.last_twist.linear.x + (twist.linear.x * LINEAR_VELOCITY_INCREMENT)) <= MAX_LINEAR_VELOCITY and self.last_twist.linear.x != twist.linear.x):
            self.last_twist.linear.x += twist.linear.x * LINEAR_VELOCITY_INCREMENT
        
        if(abs(self.last_twist.angular.z + (twist.angular.z * ANGULAR_VELOCITY_INCREMENT)) <= MAX_ANGULAR_VELOCITY and self.last_twist.angular.z != twist.angular.z):
            self.last_twist.angular.z += twist.angular.z * ANGULAR_VELOCITY_INCREMENT
            
        # self.last_twist.linear.y = twist.linear.y * MAX_LINEAR_VELOCITY
        # self.last_twist.linear.z = twist.linear.z * MAX_LINEAR_VELOCITY
        # self.last_twist.angular.x = twist.angular.x * MAX_ANGULAR_VELOCITY
        # self.last_twist.angular.y = twist.angular.y * MAX_ANGULAR_VELOCITY
        # self.last_twist.angular.z = twist.angular.z * MAX_ANGULAR_VELOCITY

        # if twist.linear.x == 0.0 and twist.angular.z == 0.0:
        #     self.last_twist.linear.x = 0.0
        #     self.last_twist.angular.z = 0.0

        # # Update the linear and angular velocities if they are not 0.
        # if twist.linear.x > 0:
        #     self.last_twist.linear.x = twist.linear.x * MAX_LINEAR_VELOCITY
        # elif twist.linear.x < 0:
        #     self.last_twist.linear.x = twist.linear.x * MAX_LINEAR_VELOCITY

        # if twist.angular.z > 0:
        #     self.last_twist.angular.z = twist.angular.z * MAX_ANGULAR_VELOCITY
        # elif twist.angular.z < 0:
        #     self.last_twist.angular.z = twist.angular.z * MAX_ANGULAR_VELOCITY

        # self.get_logger().info("CURRENT VELOCITY")
        # self.get_logger().info('linear.x: %f' % self.last_twist.linear.x)

        # self.get_logger().info("CURRENT ANGULAR")
        # self.get_logger().info('angular.z: %f' % self.last_twist.angular.z)

    def publish_last_twist(self):
        """Continuously publish the last command to the simulation topic"""
        self.simulation_publisher.publish(self.last_twist)


def main(passed_args=None):
    """Main entry point for the ROS node"""
    try:
        rclpy.init(args=passed_args)
        wheels_subscriber = WheelsSubscriber()
        rclpy.spin(wheels_subscriber)

    except KeyboardInterrupt:
        pass
