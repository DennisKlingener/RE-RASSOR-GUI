# Class for controlling video feed and computer vision software on EZ-RASSOR
# 2023 & 2024 Noah Gregory, Adam Whitlock, Riya Signh for Florida Space Institute
# 2024 Ritam Saha

from PIL import Image
import cv2
from ezcamera import arucoGenerator
import numpy as np
from datetime import datetime, timedelta
import json
import os  # Added to handle dynamic path

# Debugging tag for this class
DEBUG_TAG = "Debug: cameraController"

# Constants for offset correction
X_OFFSET_CORRECTION = 0.09  # Correction to shift the x-axis translation
X_OFFSET_TOLERANCE_BEHIND = 0.12  # Tolerance value to determine green zone
X_OFFSET_TOLERANCE_AHEAD = 0.05  # Tolerance value to determine green zone
ORIENTATION_TOLERANCE = 0.4  # Tolerance for orientation (in radians)
MIN_DISTANCE = 0.5  # Minimum allowable distance in meters
MAX_DISTANCE = 0.9  # Maximum allowable distance in meters

# Target ArUco marker ID
TARGET_MARKER_ID = 42  # Change this value to the desired marker ID

# Dictionaries of ArUco markers
ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

MARKER_DICTIONARY = "DICT_5X5_100"  # Default marker dictionary, can be changed
MARKER_DIR = 'tags/'

# Instance of Video Capture
port = 2  # Adjust as needed for your camera
frame_width = 400
frame_height = 280
capture = cv2.VideoCapture(port)
capture.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
capture.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)

current_dir = os.path.dirname(os.path.realpath(__file__))  # Get the current directory where the script resides
# Load the camera calibration parameters using dynamic paths
camera_matrix = np.load(os.path.join(current_dir, "camera_matrix.npy"))
dist_coeffs = np.load(os.path.join(current_dir, "dist_coeffs.npy"))

# Initialize the ArUco detector with the selected dictionary
arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[MARKER_DICTIONARY])
arucoParams = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)
detected_markerIds = []
detectionReport = {}
target_marker_detection_status = "No target marker detected yet." # Global variable to store detection status of target aruco marker

# Function to convert rotation vector to rotation matrix and extract yaw (rotation around y-axis)
def get_yaw_from_rvec(rvec):
    # Convert the rotation vector to a rotation matrix
    rotation_matrix, _ = cv2.Rodrigues(rvec)

    # From the rotation matrix, extract Euler angles (roll, pitch, yaw)
    # Yaw (rotation around y-axis) is of interest for determining left/right rotation
    yaw = np.arctan2(rotation_matrix[2, 0], rotation_matrix[0, 0])
    
    return yaw

# Draws bounding boxes around aruco markers
def aruco_display(corners, ids, rejected, image):
    if len(corners) > 0:
        ids = ids.flatten()
        
        for(markerCorner, markerID) in zip(corners, ids):
            
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
            
            cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
            
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
            
            cv2.putText(image, str(markerID), (topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 0), 2)
            print("[Inference] ArUco marker ID: {}".format(markerID))
    return image

# Current Time in format: YYYY-MM-DD HH:MM:SS.SSS
def getCurrentTime():
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

# Returns true if it has been more than 7.5 seconds since last marker detected
def isItTimeToDefaultResponse(lastDetectionTime):
    currTime = datetime.now()
    lastTime = datetime.strptime(lastDetectionTime, "%Y-%m-%d %H:%M:%S.%f")
    timeDif = currTime - lastTime
    return timeDif >= timedelta(seconds=7.5)

# Response when a marker hasn't been detected
def defaultResponse():
    return {
        "detection": "false",
        "time": str(getCurrentTime())
    }

# When it is called for, turns python dictionary into a json object
def getDetectionReport():
    global detectionReport
    return json.dumps(detectionReport)

# Updates detection report with ID, time, and x, y, z coordinates
def setDetectionReport(markerId, time, x, y, z, rvec, tvec):
    return {
        "detection": str(markerId), 
        "time": str(time), 
        "x": x,
        "y": y,
        "z": z,
        "rvec": rvec.tolist(),  # Rotation vector (orientation)
        "tvec": tvec.tolist()   # Translation vector (position)
    }

# Checks if the target marker is aligned within the green zone
def in_green_zone(tvec, yaw):
    global target_marker_detection_status

    retval = ""

    # Distance check (translation along z-axis)
    distance = tvec[2]
    if distance < MIN_DISTANCE:
        retval = "Not in green zone, too close."
    elif distance > MAX_DISTANCE:
        retval = "Not in green zone, too far."
    else:
        # Correct the x translation to account for offset
        corrected_x_translation = tvec[0] + X_OFFSET_CORRECTION

        # Position check (translation)
        if corrected_x_translation < -X_OFFSET_TOLERANCE_BEHIND:
            retval = "Not in green zone, Sidekick too far behind."
        elif corrected_x_translation > X_OFFSET_TOLERANCE_AHEAD:
            retval = "Not in green zone, Sidekick too far ahead."
        else:
            # Rotation check (orientation) using the yaw value
            if abs(yaw) > ORIENTATION_TOLERANCE:
                if yaw > (ORIENTATION_TOLERANCE - 0.25):
                    retval = "Not in green zone, Sidekick too far rotated to the right."
                elif yaw < (ORIENTATION_TOLERANCE + 0.5):
                    retval = "Not in green zone, Sidekick too far rotated to the left."
            else:
                retval = "In green zone."

    target_marker_detection_status = retval  # Update the global status with the result from green zone check


# Captures video feed from attached camera, detects aruco markers, and renders it in a webpage
def generate_stream():
    print(DEBUG_TAG + "Generating video stream")

    global detectionReport
    global target_marker_detection_status

    if not capture.isOpened():
        while True:
            # Get video paused image to display in web page
            img = Image.open(r'../../resources/cameraerror.jpg')
            frame = img.tobytes()

            # Yield image so it can be displayed in webpage template
            yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

    while True:
        ret, frame = capture.read()

        if not ret:
            print("Error: Unable to retrieve frame.")
            # Get video paused image to display in web page
            img = Image.open(r'../../resources/cameraerror.jpg')
            frame = img.tobytes()

            # Yield image so it can be displayed in webpage template
            yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

        # The markers that are actively being detected
        markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(frame)

        # If there are detected markers, process only the target marker (e.g., ID 42)
        if markerIds is not None and TARGET_MARKER_ID in markerIds:
            targetIndex = np.where(markerIds == TARGET_MARKER_ID)[0][0]
            targetCorners = markerCorners[targetIndex]

            # Estimate the pose of the detected target marker
            marker_size = 0.079375  # Marker size in meters (adjust as needed)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                [targetCorners], marker_size, camera_matrix, dist_coeffs
            )

            rvec = rvecs[0][0]
            tvec = tvecs[0][0]
            detectionReport = setDetectionReport(
                TARGET_MARKER_ID, getCurrentTime(), tvec[0], tvec[1], tvec[2], rvec, tvec
            )

            # Extract the yaw from the rotation vector using Rodrigues
            yaw = get_yaw_from_rvec(rvec)
            # Account for yaw offset
            yaw_offset = -0.25
            yaw += yaw_offset

            # Check if we are in the green zone
            in_green_zone(tvec, yaw)

            # Draw the axis for the target marker
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.1)

        # If 7500ms (7.5 seconds) have passed and nothing is detected, give default response
        elif isItTimeToDefaultResponse(detectionReport["time"]):
            detectionReport = defaultResponse()
            target_marker_detection_status = "No target marker detected yet."  # Update the global status

        # Displays markers
        detected_markers = aruco_display(markerCorners, markerIds, rejectedCandidates, frame)

        ret, jpeg = cv2.imencode('.jpg', detected_markers)
        frame = jpeg.tobytes()
        yield (b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n') 

def get_target_marker_detection_status():
    global target_marker_detection_status
    return target_marker_detection_status

# Initialize variables before ArUCo detection polling starts
def initialize():
    global detectionReport
    detectionReport = defaultResponse()
