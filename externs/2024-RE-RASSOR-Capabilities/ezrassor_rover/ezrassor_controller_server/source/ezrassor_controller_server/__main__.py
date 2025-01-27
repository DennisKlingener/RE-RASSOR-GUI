"""Execute a ROS node using the ezrassor_controller_server module.

This node listens for HTTP messages over the wire that can be translated into
commands for the EZRASSOR. The translated commands are published to ROS topics
that the EZRASSOR understands.
"""
import ezrassor_controller_server as server
import geometry_msgs.msg
import rclpy
import std_msgs.msg
import sys
from flask import Flask, request, Response, render_template
from flask_cors import CORS # Import CORS
import cv2
from ezcamera import cameraController
import time
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import socket
from std_msgs.msg import String
import threading
import json 

#the topics have to have /{rover_name}/ and match instructions in sim_description
NODE = "controller_server"
AUTONOMY_TOPIC = "/ezrassor/autonomy_instructions"
FRONT_ARM_ACTIONS_TOPIC = "/ezrassor/front_arm_instructions"
BACK_ARM_ACTIONS_TOPIC = "/ezrassor/back_arm_instructions"
FRONT_DRUM_ACTIONS_TOPIC = "/ezrassor/front_drum_instructions"
BACK_DRUM_ACTIONS_TOPIC = "/ezrassor/back_drum_instructions"
SHOULDER_ACTIONS_TOPIC = "/ezrassor/shoulder_instructions"
ROUTINE_ACTIONS_TOPIC = "/ezrassor/routine_actions"
IMAGE_RAW_TOPIC = "/usb_cam/image_raw"
SIDEKICK_ROVER_INFO_TOPIC = "/sidekick/sidekick_rover_info"
ARM_ROVER_INFO_TOPIC = "/arm/arm_rover_info"
QUEUE_SIZE = 11

FPS = 30.0
DISPLAY_FPS = 10.0
DISPLAY_IMG = True

latest_arm_info = {
    "name": None,
    "ip_address": None,
}

def get_unaltered_ip_address():
    try:
        # Create a dummy socket connection to get the IP address
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Connect to a public IP address (Google's DNS), but we don't actually send any data
        s.connect(("8.8.8.8", 80))
        ip_address = s.getsockname()[0]
        s.close()
        return ip_address
    except Exception as e:
        self.get_logger().error(f"Error finding IP for potato: {e}")
        return None  # Return None or a default value

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
WHEEL_ACTIONS_TOPIC = f"/{ROVER_NAME}/wheel_instructions" # move other topics later
SERVO_ACTIONS_TOPIC = f"/{ROVER_NAME}/servo_instructions"

def main(passed_args=None):
    print(sys.path)
    """Main entry point for the ROS node."""
    try:
        cameraController.initialize()
        rclpy.init(args=passed_args)
        node = rclpy.create_node(NODE)
        # Only wheel, autonomy and shoulder publishers are working atm


        """
        Communication between rovers starts here.
        """
        # Define a callback function for the subscriber
        def subscribe_arm_rover_info(msg):
            """Callback to process incoming arm rover info."""
            # Deserialize the JSON string back to a dictionary
            try:
                global latest_arm_info
                
                rover_info = json.loads(msg.data)  # Deserialize JSON string to a Python dictionary
                name = rover_info.get("name")
                ip_address = rover_info.get("ip_address")

                latest_arm_info = {
                    "name": name,
                    "ip_address": ip_address,
                }
                node.get_logger().info(f'Received Arm Rover Info: {latest_arm_info}')
                        
                # Process the received data (log it, use it, etc.)
                # node.get_logger().info(f'Received Arm Rover Info: Name: {name}, IP Address: {ip_address}')
            except json.JSONDecodeError as e:
                node.get_logger().error(f'Failed to decode JSON: {e}')

        # Create a subscriber for the sidekick_rover_info_topic
        arm_info_subscriber = node.create_subscription(
            String,  # Use the String message type
            ARM_ROVER_INFO_TOPIC,
            subscribe_arm_rover_info,
            QUEUE_SIZE,
        )

        # Define a function to keep the subscriber active in a thread
        def subscribe_arm_info(node):
            """Keep the subscriber active."""
            rclpy.spin(node)  # Spin the node to keep processing incoming messages

        # Start the subscriber in a separate thread
        subscriber_thread = threading.Thread(target=subscribe_arm_info, args=(node,))
        subscriber_thread.daemon = True  # Allow the thread to exit when the main program exits
        subscriber_thread.start()

        def publish_sidekick_rover_info(node, publisher):
            """Continuously publish sidekick rover info."""
            msg = String()
            while rclpy.ok():
                ip_address = get_unaltered_ip_address()  # Get the IP address
                rover_info = {
                    "name": "Sidekick",
                    "ip_address": ip_address + ':5000',
                    "paver_count": 3,
                }
                
                # Convert the dictionary to a JSON string
                json_msg = json.dumps(rover_info)

                # Create a String message and set its data
                msg = String()
                msg.data = json_msg

                # Publish the message
                publisher.publish(msg)
                node.get_logger().info(f'Publishing: "{msg.data}"')
                
                time.sleep(1)  # Publish every second, adjust as needed

         # Create a publisher for string messages
        sidekick_rover_info_publisher = node.create_publisher(
            String,  # Use the String message type
            SIDEKICK_ROVER_INFO_TOPIC,
            QUEUE_SIZE,
        )
        
        servo_actions_publisher = node.create_publisher(
             geometry_msgs.msg.Twist,
             SERVO_ACTIONS_TOPIC,
             QUEUE_SIZE
		)

        # Start the continuous publishing in a separate thread
        thread = threading.Thread(target=publish_sidekick_rover_info, args=(node, sidekick_rover_info_publisher))
        thread.daemon = True  # Allow the thread to exit when the main program exits
        thread.start()
        """
        Communication between rovers ends here.
        """

        # Local publishers for this rover
        # Create publishers for wheel actions -- sends twist commands
        wheel_actions_publisher = node.create_publisher(
            geometry_msgs.msg.Twist,
            WHEEL_ACTIONS_TOPIC,
            QUEUE_SIZE,
        )
        
        # MADI fix this
        # servo_actions_publisher = node.create_publisher(
        #     std_msgs.msg.String,
        #     SERVO_ACTIONS_TOPIC,
        #     QUEUE_SIZE,
        # )
        # Create a publisher for the autonomy functions -- sends twist commands -- should be changed to only sending x and y 
        autonomy_publisher = node.create_publisher(
            geometry_msgs.msg.Twist,
            AUTONOMY_TOPIC,
            QUEUE_SIZE,
        )
        # Shoulder publisher -- sends twist commands 
        shoulder_actions_publisher = node.create_publisher(
            geometry_msgs.msg.Twist,
            SHOULDER_ACTIONS_TOPIC,
            QUEUE_SIZE,
        )
        # From here below the logic is not implemented yet 
        front_arm_actions_publisher = node.create_publisher(
            std_msgs.msg.Float64,
            FRONT_ARM_ACTIONS_TOPIC,
            QUEUE_SIZE,
        )
        back_arm_actions_publisher = node.create_publisher(
            std_msgs.msg.Float64,
            BACK_ARM_ACTIONS_TOPIC,
            QUEUE_SIZE,
        )
        front_drum_actions_publisher = node.create_publisher(
            std_msgs.msg.Float64,
            FRONT_DRUM_ACTIONS_TOPIC,
            QUEUE_SIZE,
        )
        back_drum_actions_publisher = node.create_publisher(
            std_msgs.msg.Float64,
            BACK_DRUM_ACTIONS_TOPIC,
            QUEUE_SIZE,
        )
        routine_actions_publisher = node.create_publisher(
            std_msgs.msg.Int8,
            ROUTINE_ACTIONS_TOPIC,
            QUEUE_SIZE,
        )
        img_publisher = node.create_publisher(
            Image,
            IMAGE_RAW_TOPIC,
            QUEUE_SIZE,
        )
        # End of unimplemented logic 

        def process_request(request):
            """Callback to create and publish a command from a request."""
            server.verify(request)
            command = server.create_command(request)
            if command.wheel_action is not None:
                wheel_action = geometry_msgs.msg.Twist() 
                # Converting linear x and angular z to floats since i am recieving as a string from controller code             
                wheel_action.linear.x = float(command.wheel_action.linear_x)
                wheel_action.angular.z = float(command.wheel_action.angular_z)
                wheel_actions_publisher.publish(wheel_action)
            # autonomy code to recieve from controller and send to forwarder 
            if command.target_coordinate is not None:
                target_coordinate = geometry_msgs.msg.Twist()
                # sending the autonomy coordinates as floats 
                target_coordinate.linear.x = float(command.target_coordinate.x)
                target_coordinate.linear.y = float(command.target_coordinate.y)
                autonomy_publisher.publish(target_coordinate)
            # Shoulder code to recieve and send actions to the shoulders
            if command.shoulder_action is not None:
                shoulder_action = geometry_msgs.msg.Twist() 
                # converting linear and angular y  to floats since I am sending it as a string            
                shoulder_action.linear.x = float(command.shoulder_action.linear_x)
                shoulder_action.angular.z = float(command.shoulder_action.angular_z)
                shoulder_action.linear.y = float(command.shoulder_action.linear_y)
                shoulder_action.angular.y = float(command.shoulder_action.angular_y)
                shoulder_actions_publisher.publish(shoulder_action)
            
            if command.servo_action is not None:
                    servo_action = geometry_msgs.msg.Twist()
                    servo_action.linear.x = float(command.servo_action.linear_x)
                    servo_actions_publisher.publish(servo_action)

            # Not fully implemented 
            if command.front_arm_action is not None:
                front_arm_action = std_msgs.msg.Float64()
                front_arm_action.data = command.front_arm_action.value
                front_arm_actions_publisher.publish(front_arm_action)
            # Not fully implemented 
            if command.back_arm_action is not None:
                back_arm_action = std_msgs.msg.Float64()
                back_arm_action.data = command.back_arm_action.value
                back_arm_actions_publisher.publish(back_arm_action)
            # Not fully implemented 
            if command.front_drum_action is not None:
                front_drum_action = std_msgs.msg.Float64()
                front_drum_action.data = command.front_drum_action.value
                front_drum_actions_publisher.publish(front_drum_action)
            # Not fully implemented
            if command.back_drum_action is not None:
                back_drum_action = std_msgs.msg.Float64()
                back_drum_action.data = command.back_drum_action.value
                back_drum_actions_publisher.publish(back_drum_action)
            # Not fully implemented
            if command.routine_action is not None:
                routine_action = std_msgs.msg.Int8()
                routine_action.data = command.routine_action.value
                routine_actions_publisher.publish(routine_action)
        
        def process_detection_request(request):
            detection_status = None
            if "detectionStatus" in request:
                detection_status = request["detectionStatus"]
                cameraController.toggle_detection(detection_status)

        # Create a Flask app to serve HTTP requests.
        app = Flask(__name__)
        CORS(app)

        @app.route("/", methods=["OPTIONS"])
        def handle_options():
            return {'status': 200}

        # Default get request
        @app.route("/", methods=["GET"])
        def default_get():
            return {"status": 200}

        @app.route("/", methods=["POST"])
        def handle_request():
            """Handle HTTP requests and log any errors.

            This function is the glue between Flask/HTTP and ROS business logic
            in this package.
            """
            try:
                process_request(request.get_json())

                return {"status": 200}
            except server.VerificationError as error:
                node.get_logger().error(str(error))

                return {"status": 400}
                
        @app.after_request
        def apply_cors_headers(response):
            response.headers["Access-Control-Allow-Origin"]="*"
            response.headers["Access-Control-Allow-Methods"]="GET,POST,OPTIONS"
            response.headers["Access-Control-Allow-Headers"]="Content-Type,Authorization"
            return response
            
        @app.route('/video_feed')
        def video_feed():
            """Stream video to webpage on local network"""
            return Response(cameraController.generate_stream(), mimetype="multipart/x-mixed-replace; boundary=frame")
        
        @app.route('/get_target_status')
        def get_target_status():
            """Check if the target ArUco marker is within the green zone for alignment."""
            return Response(cameraController.get_target_marker_detection_status(), mimetype="text/plain")

        @app.route('/is_detection')
        def is_detection():
            """Updates if a paver has been detected and if so where it is"""
            return Response(cameraController.getDetectionReport(), mimetype="text/plain")
        
        # Add a Flask route to serve the latest sidekick rover info
        @app.route('/arm_info')
        def get_arm_info():
            """Serve the latest arm rover info to the frontend."""
            return latest_arm_info  # Automatically converts to JSON format

        # Run the app! Note that we don't spin the ROS node here. Only nodes
        # containing subscribers must be spun.
        app.run(host='0.0.0.0', port = 5000, debug = False, threaded = True)
    except KeyboardInterrupt:
        pass
