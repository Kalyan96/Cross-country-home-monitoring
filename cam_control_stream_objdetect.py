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
from queue import Queue
from threading import Thread



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

frame_queue = Queue()
recording = False
recording_thread_active = False
last_motion_time = None
record_duration = 10
frames_queue = Queue()


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
last_check_time = time.time()  # Store the last check time outside the loop


# Initialize the background subtractor and morphological kernel
fgbg = cv2.createBackgroundSubtractorMOG2(history=1000, varThreshold=25, detectShadows=True)
kernel = np.ones((7,7),np.uint8)  # Increase the size

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


def start_recording():
    global recording_thread_active,recording
    print("Recording started.")
    # Create a filename based on the current timestamp
    filename = time.strftime("%Y%m%d-%H%M%S") + '.mp4'

    # Get the frame size from the first frame in the queue
    first_frame = frames_queue.queue[0]
    frame_height, frame_width, _ = first_frame.shape

    # Define the codec and create a VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'MP4V')
    out = cv2.VideoWriter(filename, fourcc, 20.0, (frame_width, frame_height))

    print(f"Size of the frames queue: {frames_queue.qsize()}")

    try:
        while not frames_queue.empty():
            frame = frames_queue.get()
            out.write(frame)
            # cv2.imshow('Frame from Queue', frame)
            # cv2.waitKey(1)
    except Exception as e:
        print(f"Error while writing frame: {e}")


    out.release()
    recording_thread_active = False
    print("Recording finished.")


def update_image():
    global last_frame_time, cap, recording, recording_thread_active, last_motion_time, last_check_time  # Store the last check time outside the loop
  # Declare cap as global
    frames_to_record = None
    record_thread = None
    motion_detected = False

    logging.info("update_image function called.")

    reconnect_attempts = 0
    MAX_RETRIES = 5  # Maximum number of reconnection attempts

    while True:
        try:
            ret, frame = cap.read()
            # print(cap.get(cv2.CAP_PROP_POS_MSEC))
        except Exception as e:
            logging.error(f"Error reading frame: {e}")
            ret = False

        if ret and frame is not None and frame.size > 0:
            reconnect_attempts = 0  # Reset the counter if a valid frame is received
            # Update the timestamp for the last received frame
            last_frame_time = time.time()

            # Apply background subtraction
            fgmask = fgbg.apply(frame)

            # Morphological operations to get rid of the noise
            fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, kernel)
            fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_CLOSE, kernel)

            # Find contours in the mask
            contours, _ = cv2.findContours(fgmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            motion_detected = False

            # Draw green rectangles around the moving objects
            for contour in contours:
                if cv2.contourArea(contour) > 10000:  # you can set a different value for your needs
                    (x, y, w, h) = cv2.boundingRect(contour)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

                    motion_detected = True
                    logging.info("Motion detected !!")
                    last_motion_time = time.time()

            if motion_detected:
                recording = True
                last_motion_time = time.time()

                # If recording is active, add frames to the queue
            if recording:
                frames_queue.put(frame)
                # print("Frame added to the queue.")

                # If more than record_duration seconds passed since the last motion
                if time.time() - last_motion_time > record_duration:
                    recording = False
                    if not recording_thread_active:
                        recording_thread_active = True
                        thread = Thread(target=start_recording)
                        thread.start()


            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image = Image.fromarray(frame)
            photo = ImageTk.PhotoImage(image=image)
            label.config(image=photo)
            label.image = photo
        else:
            # If no valid frame for more than 5 seconds or after several retries
            logging.warning("no frame rx")
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



