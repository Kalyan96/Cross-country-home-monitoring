import tkinter as tk
from onvif import ONVIFCamera
import cv2
from threading import Thread
from PIL import Image, ImageTk

# Initialize the camera with the provided details
cam = ONVIFCamera('192.168.0.172', 8899, 'admin', '', 'C:/Users/Kalyan/PycharmProjects/homecam/venv/Lib/site-packages/wsdl/')


cap = cv2.VideoCapture(rtsp_url)  # Use the RTSP URL we fetched

# ... [Rest of the script remains unchanged]
