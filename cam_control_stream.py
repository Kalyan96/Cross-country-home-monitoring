import tkinter as tk
from onvif import ONVIFCamera
import cv2
from threading import Thread
from PIL import Image, ImageTk
import os
import platform
import logging
import time


# Set up logging configurations
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

logging.info("Starting application...")


def ping_camera(ip_address):
    logging.info(f"Pinging camera at {ip_address}...")
    param = "-n" if platform.system().lower() == "windows" else "-c"
    response = os.popen(f'ping {param} 1 {ip_address}').read()

    if 'time=' in response:
        latency = response.split('time=')[1].split(' ')[0]
        logging.info(f"Camera at {ip_address} is reachable with a latency of {latency} ms.")
    else:
        logging.warning(f"Camera at {ip_address} is not reachable.")




def get_rtsp_url(cam):
    # Create media service client
    media_service = cam.create_media_service()
    # Get media profiles
    profiles = media_service.GetProfiles()
    if not profiles:
        raise ValueError("No media profiles found")
    # Get the RTSP stream URL
    stream_uri = media_service.GetStreamUri({'StreamSetup': {'Stream': 'RTP-Unicast', 'Transport': {'Protocol': 'UDP'}}, 'ProfileToken': profiles[
        0].token})

    return stream_uri.Uri


camera_ip = '192.168.0.172'
last_frame_time = time.time()

ping_camera(camera_ip)

logging.info("Initializing ONVIF camera...")

# Initialize the camera with the provided details
cam = ONVIFCamera(camera_ip, 8899, 'admin', '', 'C:/Users/Kalyan/PycharmProjects/homecam/venv/Lib/site-packages/wsdl/')
logging.info("Camera initialized.")

logging.info("Fetching RTSP URL...")
rtsp_url = get_rtsp_url(cam)
logging.info(f"RTSP URL fetched: {rtsp_url}")

logging.info("Setting up PTZ service...")
ptz = cam.create_ptz_service()
ptz_configurations = ptz.GetConfigurations()
ptz_token = ptz_configurations[0].token
logging.info("PTZ service initialized.")

cap = cv2.VideoCapture(rtsp_url)
logging.info("Video capture initialized.")

# Initialize current position variables with middle values
current_pan = 0
current_tilt = 0
current_zoom = 0

step_value = 0.1  # Adjust this as per your requirement

def move_camera_absolute(direction):
    global current_pan, current_tilt, current_zoom

    # Adjust position based on direction
    if direction == "UP":
        current_tilt = min(current_tilt + step_value, 1)
    elif direction == "DOWN":
        current_tilt = max(current_tilt - step_value, 0)
    elif direction == "LEFT":
        current_pan = max(current_pan - step_value, 0)
    elif direction == "RIGHT":
        current_pan = min(current_pan + step_value, 1)

    # Create the AbsoluteMove request
    request = ptz.create_type('AbsoluteMove')
    request.ProfileToken = ptz_token
    request.Position = {'PanTilt': {'x': current_pan, 'y': current_tilt}, 'Zoom': {'x': current_zoom}}
    print("pan = "+current_pan+" tilt = "+current_tilt+" zoom = "+current_zoom)
    # Send the request
    ptz.AbsoluteMove(request)

def move_camera(direction):
    request = ptz.create_type('ContinuousMove')
    request.ProfileToken = ptz_token

    if direction == "UP":
        request.Velocity = {'PanTilt': {'x': 0, 'y': -1}, 'Zoom': {'x': 0}}
    elif direction == "DOWN":
        request.Velocity = {'PanTilt': {'x': 0, 'y': 1}, 'Zoom': {'x': 0}}
    elif direction == "LEFT":
        request.Velocity = {'PanTilt': {'x': -1, 'y': 0}, 'Zoom': {'x': 0}}
    elif direction == "RIGHT":
        request.Velocity = {'PanTilt': {'x': 1, 'y': 0}, 'Zoom': {'x': 0}}

    ptz.ContinuousMove(request)

def stop_camera(event):
    ptz.Stop({'ProfileToken': ptz_token})


def update_image():
    global last_frame_time, cap  # Declare cap as global

    logging.info("update_image function called.")

    reconnect_attempts = 0
    MAX_RETRIES = 5  # Maximum number of reconnection attempts

    while True:
        ret, frame = cap.read()

        if ret and frame is not None and frame.size > 0:
            reconnect_attempts = 0  # Reset the counter if a valid frame is received
            # Update the timestamp for the last received frame
            last_frame_time = time.time()

            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image = Image.fromarray(frame)
            photo = ImageTk.PhotoImage(image=image)
            label.config(image=photo)
            label.image = photo
        else:
            # If no valid frame for more than 5 seconds or after several retries
            if time.time() - last_frame_time > 5 or reconnect_attempts > 0:
                if reconnect_attempts < MAX_RETRIES:
                    logging.warning("Stream seems stuck. Attempting to reconnect...")
                    cap.release()
                    time.sleep(2)
                    cap = cv2.VideoCapture(rtsp_url)
                    reconnect_attempts += 1
                else:
                    logging.error("Exceeded maximum reconnection attempts. Please check the camera or network connection.")
                    break


root = tk.Tk()
frame = tk.Frame(root, width=800, height=600)
frame.bind("<KeyPress-Up>", lambda e: move_camera("UP"))
frame.bind("<KeyPress-Down>", lambda e: move_camera("DOWN"))
frame.bind("<KeyPress-Left>", lambda e: move_camera("LEFT"))
frame.bind("<KeyPress-Right>", lambda e: move_camera("RIGHT"))
frame.bind("<KeyRelease>", stop_camera)
frame.focus_set()
frame.pack()

label = tk.Label(frame)
label.pack(fill=tk.BOTH, expand=True)

# Create a separate thread to update the video feed to avoid freezing the GUI
thread = Thread(target=update_image)
thread.daemon = 1
thread.start()

root.mainloop()



