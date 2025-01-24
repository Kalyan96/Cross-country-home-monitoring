import time
import cv2
import numpy as np
import requests
from datetime import datetime
import serial
import logging


# Replace with your ESP32's IP address and port
ESP32_URL = "http://10.0.0.209:81/stream"
ESP32_serialport = 'COM8'

# Parameters for motion detection
motion_detection_threshold = 10  # Adjust as needed
motion_detection_frame_interval = 5  # Number of frames to skip before processing again

# Configure logging
logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s [%(levelname)s] %(message)s')
logger = logging.getLogger(__name__)

# # Create a StreamHandler to display logs on the console
# console_handler = logging.StreamHandler()
# console_handler.setLevel(logging.INFO)
# formatter = logging.Formatter('%(asctime)s [%(levelname)s] %(message)s')
# console_handler.setFormatter(formatter)
# logger.addHandler(console_handler)



def reset_serial(serial_port_identifier):
    try:
        # Define the serial port and baud rate
        serial_port = serial.Serial(serial_port_identifier, baudrate=115200)

        # Send the reset command
        reset_command = "RESET\n"  # Replace with the actual reset command your device expects
        serial_port.write(reset_command.encode())  # Encode the string as bytes before sending
        logger.info("Reset the ESP module via serial")

        # Wait for a response (if needed)
        # response = serial_port.readline()
        # logger.info("Response from device: %s", response.decode().strip())  # Decode and log the response

        # Close the serial port
        serial_port.close()
        logger.info("Closed the serial port")

        # Wait for it to reset and reconnect to Wi-Fi
        time.sleep(3)
        # logger.info("Waited for 3 seconds after reset")
    except Exception as e:
        logger.error("Error during serial initialization: %s", str(e))
        return None, None, 0

def reset_video_capture(esp32_url):
    global cap, prev_frame, frame_count
    try:
        # Initialize video capture
        cap = cv2.VideoCapture(esp32_url)
        logger.info("Initialized video capture")

        # Initialize variables for motion detection
        prev_frame = None
        frame_count = 0
        logger.info("Initialized motion detection variables")


    except Exception as e:
        logger.error("Error during video capture initialization: %s", str(e))


reset_serial(ESP32_serialport)
reset_video_capture(ESP32_URL)

# Initialize video capture
cap = cv2.VideoCapture(ESP32_URL)
logger.info("Initialized video capture")

# Initialize variables for motion detection
prev_frame = None
frame_count = 0
logger.info("Initialized motion detection variables")

while True:
    # Capture a frame from the video stream
    ret, frame = cap.read()
    if not ret:
        logger.error("No frame recieved !!!")
        reset_serial(ESP32_serialport)
        reset_video_capture(ESP32_URL)
        continue
        # break
    # logger.info("Captured a frame from the video stream")

    # Convert the frame to grayscale for motion detection
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray_frame = cv2.GaussianBlur(gray_frame, (21, 21), 0)
    # logger.info("Converted frame to grayscale and applied Gaussian blur")

    if prev_frame is None:
        prev_frame = gray_frame
        continue

    # Compute the absolute difference between the current and previous frame
    frame_delta = cv2.absdiff(prev_frame, gray_frame)
    thresh = cv2.threshold(frame_delta, 25, 255, cv2.THRESH_BINARY)[1]
    thresh = cv2.dilate(thresh, None, iterations=2)
    # logger.info("Performed motion detection processing")

    # Find contours of motion regions
    contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # logger.info("Found contours of motion regions")

    motion_detected = False

    for contour in contours:
        if cv2.contourArea(contour) > motion_detection_threshold:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            motion_detected = True
    # logger.info("Checked for motion and drew rectangles if detected")

    # Display the frame with motion detection
    cv2.imshow("Motion Detection", frame)
    # logger.info("Displayed frame with motion detection")

    # Save frame if motion is detected
    if motion_detected:
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        filename = f"motion_{timestamp}.jpg"
        cv2.imwrite(filename, frame)
        logger.info("Motion detected. Saved frame as %s", filename)

    # Update the previous frame and frame count
    prev_frame = gray_frame.copy()
    frame_count += 1
    # logger.info("Updated previous frame and frame count")

    # Exit the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break



# Release the video capture and close OpenCV windows
cap.release()
cv2.destroyAllWindows()
logger.info("Released video capture and closed OpenCV windows")