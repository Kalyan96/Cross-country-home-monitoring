import time

import serial

# Define the serial port and baud rate
# serial_port = serial.Serial('COM8', baudrate=115200)  # Replace 'COM1' with your serial port identifier
serial_port = serial.Serial('/dev/ttyUSB0', baudrate=115200)  # Replace 'COM1' with your serial port identifier


try:
    # Send the reset command
    reset_command = "RESET\n"  # Replace with the actual reset command your device expects
    serial_port.write(reset_command.encode())  # Encode the string as bytes before sending
    # # Wait for a response (if needed)
    # response = serial_port.readline()
    # print("Response from device:", response.decode().strip())  # Decode and print the response
    # time.sleep(600000)
except Exception as e:
    print("Error:", str(e))
finally:
    # Close the serial port
    serial_port.close()
