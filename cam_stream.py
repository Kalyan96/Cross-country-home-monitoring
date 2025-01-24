import threading
import cv2
from onvif import ONVIFCamera
import tkinter as tk
from collections import deque
import logging
import time
from threading import Lock


# Configure the logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

# ONVIF camera credentials
camera_ip = '192.168.0.172'  # Replace with your camera's IP address
CAMERA_PORT = 80  # Default ONVIF port
CAMERA_USERNAME = 'your_username'  # Replace with your camera's username
CAMERA_PASSWORD = 'your_password'  # Replace with your camera's password

# Maximum number of frames to buffer
MAX_FRAME_BUFFER_SIZE = 100

# Global queue to buffer video frames
frames_queue = deque(maxlen=MAX_FRAME_BUFFER_SIZE)

# Maximum allowed frame delay (in seconds)
MAX_FRAME_DELAY = 1.0  # Adjust this threshold as needed

# Maximum allowed missed frames before resetting
MAX_MISSED_FRAMES = 2

# Variable to hold frame delay
frame_delay = 0.0

cap_lock = Lock()  # NEW: Lock for synchronizing access to the cap object


# Global variable for camera and video capture
global cam, cap

# Function to fetch the RTSP URL using ONVIF
def get_rtsp_url(cam):
    media_service = cam.create_media_service()
    profiles = media_service.GetProfiles()
    if not profiles:
        raise ValueError("No media profiles found")
    stream_uri = media_service.GetStreamUri({
        'StreamSetup': {'Stream': 'RTP-Unicast', 'Transport': {'Protocol': 'RTSP'}},
        'ProfileToken': profiles[0].token
    })
    return stream_uri.Uri

# Function to reset the video capture and re-fetch RTSP URL
def reset_video_capture(cam, cap):
    logging.warning("reset_video_capture: Resetting video capture and recovering...")
    cap.release()
    # Fetch the new RTSP stream URL
    rtsp_url = get_rtsp_url(cam)
    cap = cv2.VideoCapture(rtsp_url)
    return cap


missed_frames =None
previous_frame_time =None
frame_delayed =None
current_frame_time =None
frame_freq =None
# Function to display the RTSP stream in a thread and buffer frames
def display_rtsp_stream():
    global frame_delay, frames_queue,cam, cap, missed_frames, previous_frame_time, frame_delayed, current_frame_time, frame_freq, cap_lock
    # thread start indication
    logging.info("display_rtsp_stream: Started thread...")
    # Fetch the RTSP stream URL
    logging.info("Fetching RTSP URL...")
    rtsp_url = get_rtsp_url(cam)
    logging.info(f"RTSP URL fetched: {rtsp_url}")

    # with cap_lock:
    cap = cv2.VideoCapture(rtsp_url)
    logging.info("display_rtsp_stream: created video capture object")

    # Create an OpenCV window
    # cv2.namedWindow('RTSP Stream', cv2.WINDOW_NORMAL)

    previous_frame_time = None
    frame_delayed = False
    missed_frames = 0

    while True:
        # with cap_lock:
        ret, frame = cap.read()
        if ret:
            # frame successfully captured
            # Buffer the frame
            frames_queue.append(frame)
            logging.info("display_rtsp_stream: frame rx")
            # Display the frame
            cv2.imshow('RTSP Stream', frame)


        else:
            # frame missed
            logging.warning("display_rtsp_stream: Failed to retrieve a frame. Stream may be interrupted.")
            # time.sleep(1)  # Sleep to avoid busy-wait
            missed_frames += 1

        # current_frame_time = cv2.getTickCount()
        # frame_freq = cv2.getTickFrequency()

# Function to monitor frame delay
def monitor_frame_delay(cam):
    global frame_delay, cap,  previous_frame_time, frame_delayed, current_frame_time, frame_freq, missed_frames, cap_lock
    # thread start indication
    logging.info("monitor_frame_delay: Started thread...")
    missed_frames = 0

    while True:
        #checking for missed frames
        if missed_frames > MAX_MISSED_FRAMES:
            logging.info("monitor_frame_delay: too many missed frames. Resetting video capture and recovering...")
            with cap_lock:
                cap = reset_video_capture(cam, cap)
            missed_frames = 0

        # checking for delayed frames
        elif previous_frame_time is not None:
            logging.warning(f"display_rtsp_stream: Frame delay detected")
            # Calculate frame time in seconds
            frame_time = (current_frame_time - previous_frame_time) / frame_freq
            previous_frame_time = current_frame_time  # Move this line inside the if condition

            # Update frame_delay
            frame_delay = frame_time

            # Check for frame delay
            if frame_delay > MAX_FRAME_DELAY:
                if not frame_delayed:
                    frame_delayed = True
                    logging.warning(f"display_rtsp_stream: Frame delay detected: {frame_time:.2f} seconds")
                    # Implement frame delay recovery here
                    with cap_lock:
                        cap = reset_video_capture(cam, cap)
                    frame_delayed = False  # Reset frame_delayed after recovery

        time.sleep(1)  # Adjust the monitoring interval as needed

# Function to control the camera using arrow keys
def control_camera(event):
    if event.keysym == 'Up':
        # Send command to move the camera up
        # Replace this with your camera control logic
        pass
    elif event.keysym == 'Down':
        # Send command to move the camera down
        # Replace this with your camera control logic
        pass
    elif event.keysym == 'Left':
        # Send command to move the camera left
        # Replace this with your camera control logic
        pass
    elif event.keysym == 'Right':
        # Send command to move the camera right
        # Replace this with your camera control logic
        pass

def main():
    global camera_ip, MAX_FRAME_BUFFER_SIZE, frames_queue, MAX_FRAME_DELAY, MAX_MISSED_FRAMES, cam, cap

    # Create an ONVIF camera instance
    logging.info("Initializing ONVIF camera...")
    cam = ONVIFCamera(camera_ip, 8899, 'admin', '', 'C:/Users/Kalyan/PycharmProjects/homecam/venv/Lib/site-packages/wsdl/')
    logging.info("Camera initialized.")



    # Start the thread to display the RTSP stream and buffer frames
    rtsp_thread = threading.Thread(target=display_rtsp_stream, args=())
    rtsp_thread.start()

    # Start the monitoring thread for frame delay
    # monitor_thread = threading.Thread(target=monitor_frame_delay, args=(cam,))
    # monitor_thread.start()

    # Create a tkinter window for capturing arrow key events
    logging.info("Starting the tkinter window for arrow key events...")
    root = tk.Tk()
    root.bind("<Key>", control_camera)
    root.mainloop()

if __name__ == "__main__":
    main()
