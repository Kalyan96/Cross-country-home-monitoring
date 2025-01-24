import tkinter as tk
from onvif import ONVIFCamera
import cv2
from threading import Thread
from PIL import Image, ImageTk
import os
import platform
import logging
import time
import numpy as np

# Set up logging configurations
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logging.info("Starting application...")

def move_to_center(x, y):
    frame_center_x = cap.get(cv2.CAP_PROP_FRAME_WIDTH) // 2
    frame_center_y = cap.get(cv2.CAP_PROP_FRAME_HEIGHT) // 2

    if x < frame_center_x - 50:  # 50 is a threshold
        move_camera("LEFT")
    elif x > frame_center_x + 50:
        move_camera("RIGHT")

    if y < frame_center_y - 50:
        move_camera("UP")
    elif y > frame_center_y + 50:
        move_camera("DOWN")

    # Stop the camera movement after a brief delay
    time.sleep(0.5)  # Adjust the delay for your requirements
    stop_camera(None)

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
    stream_uri = media_service.GetStreamUri({'StreamSetup': {'Stream': 'RTP-Unicast', 'Transport': {'Protocol': 'UDP'}}, 'ProfileToken': profiles[0].token})

    return stream_uri.Uri

camera_ip = '192.168.0.172'
last_frame_time = time.time()
ping_camera(camera_ip)

logging.info("Initializing ONVIF camera...")
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
update_interval = 5  # Adjust as needed
last_update_time = 0

# Initialize the background subtractor and morphological kernel
fgbg = cv2.createBackgroundSubtractorMOG2(history=500, varThreshold=16, detectShadows=True)
kernel = np.ones((5,5),np.uint8)

def move_to_center(cx, cy):
    """
    Given the centroid of motion, this function will decide the direction
    the camera should move in using ContinuousMove.
    """
    width = 800  # Set according to the camera resolution or stream resolution
    height = 600

    if cx < width / 2:
        move_camera("LEFT")
    elif cx > width / 2:
        move_camera("RIGHT")

    if cy < height / 2:
        move_camera("UP")
    elif cy > height / 2:
        move_camera("DOWN")

def move_camera_absolute(x, y):
    global current_pan, current_tilt, current_zoom

    current_pan = x
    current_tilt = y

    # Create the AbsoluteMove request
    request = ptz.create_type('AbsoluteMove')
    request.ProfileToken = ptz_token
    request.Position = {'PanTilt': {'x': current_pan, 'y': current_tilt}, 'Zoom': {'x': current_zoom}}
    ptz.AbsoluteMove(request)

def get_ptz_status():
    global current_pan, current_tilt
    status = ptz.GetStatus({'ProfileToken': ptz_token})
    if status and status.Position:
        current_pan = status.Position.PanTilt.x
        current_tilt = status.Position.PanTilt.y

def update_position_labels():
    pan_label.config(text=f"Pan: {current_pan:.2f}")
    tilt_label.config(text=f"Tilt: {current_tilt:.2f}")
def move_camera_to_position(target_pan, target_tilt):
    global current_pan, current_tilt, step_value

    pan_direction = 1 if target_pan > current_pan else -1
    tilt_direction = 1 if target_tilt > current_tilt else -1

    request = ptz.create_type('ContinuousMove')
    request.ProfileToken = ptz_token

    while abs(current_pan - target_pan) > step_value or abs(current_tilt - target_tilt) > step_value:
        if abs(current_pan - target_pan) > step_value:
            current_pan += step_value * pan_direction
            request.Velocity = {'PanTilt': {'x': pan_direction, 'y': 0}, 'Zoom': {'x': 0}}

        if abs(current_tilt - target_tilt) > step_value:
            current_tilt += step_value * tilt_direction
            request.Velocity = {'PanTilt': {'x': 0, 'y': tilt_direction}, 'Zoom': {'x': 0}}

        ptz.ContinuousMove(request)
        update_position_labels()
        time.sleep(0.1)

    # ptz.Stop({'ProfileToken': ptz_token})

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
    global last_frame_time, cap, last_update_time, pan_value, tilt_value  # Declare cap as global
    logging.info("update_image function called.")
    motion_centroid = None  # Initialize here


    reconnect_attempts = 0
    MAX_RETRIES = 5  # Maximum number of reconnection attempts


    while True:
        try:
            ret, frame = cap.read()
        except Exception as e:
            logging.error(f"Error reading frame: {e}")
            ret = False

        if ret and frame is not None and frame.size > 0:
            reconnect_attempts = 0  # Reset the counter if a valid frame is received
            last_frame_time = time.time()

            fgmask = fgbg.apply(frame)
            fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, kernel)
            fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_CLOSE, kernel)
            contours, _ = cv2.findContours(fgmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            largest_area = 0
            largest_centroid = (0, 0)
            for contour in contours:
                if cv2.contourArea(contour) > 50:  # you can set a different value for your needs
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        if cv2.contourArea(contour) > largest_area:
                            largest_area = cv2.contourArea(contour)
                            largest_centroid = (cX, cY)

            if largest_area > 500:  # Only move the camera if the motion is significant
                move_to_center(*largest_centroid)

            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image = Image.fromarray(frame)
            photo = ImageTk.PhotoImage(image=image)
            label.config(image=photo)
            label.image = photo
        else:
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
frame.bind("<KeyPress-space>", lambda e: move_camera_to_position(0,0))  # Bind space keypress
# frame.bind("<KeyRelease>", stop_camera)
frame.focus_set()
frame.pack()

# Labels to display current pan and tilt
pan_label = tk.Label(frame, text=f"Pan: {current_pan:.2f}")
pan_label.pack(pady=10)

tilt_label = tk.Label(frame, text=f"Tilt: {current_tilt:.2f}")
tilt_label.pack(pady=10)

label = tk.Label(frame)
label.pack(fill=tk.BOTH, expand=True)

thread = Thread(target=update_image)
thread.daemon = 1
thread.start()

root.mainloop()







