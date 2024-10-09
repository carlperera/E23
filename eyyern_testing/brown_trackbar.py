import cv2
import numpy as np
import tkinter as tk
from tkinter import simpledialog

# Initialize the webcam
cap = cv2.VideoCapture(2)

# Define initial HSV range values
low_hue = 0
high_hue = 179
low_saturation = 0
high_saturation = 255
low_value = 0
high_value = 255

# Create a window to display the results
cv2.namedWindow('HSV Filtered Image')

def get_hsv_ranges():
    global low_hue, high_hue, low_saturation, high_saturation, low_value, high_value
    low_hue = simpledialog.askinteger("Input", "Enter Low Hue (0-179):", minvalue=0, maxvalue=179)
    high_hue = simpledialog.askinteger("Input", "Enter High Hue (0-179):", minvalue=0, maxvalue=179)
    low_saturation = simpledialog.askinteger("Input", "Enter Low Saturation (0-255):", minvalue=0, maxvalue=255)
    high_saturation = simpledialog.askinteger("Input", "Enter High Saturation (0-255):", minvalue=0, maxvalue=255)
    low_value = simpledialog.askinteger("Input", "Enter Low Value (0-255):", minvalue=0, maxvalue=255)
    high_value = simpledialog.askinteger("Input", "Enter High Value (0-255):", minvalue=0, maxvalue=255)

# Create a Tkinter root window
root = tk.Tk()
root.withdraw()  # Hide the main window

# Get HSV ranges from the user
get_hsv_ranges()

while True:
    # Capture a frame from the webcam
    ret, frame = cap.read()
    
    if not ret:
        break
    
    # Convert the frame to HSV color space
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Create the lower and upper bounds for the HSV filter
    lower_bound = np.array([low_hue, low_saturation, low_value])
    upper_bound = np.array([high_hue, high_saturation, high_value])
    
    # Apply the HSV filter to the frame
    mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)
    filtered_frame = cv2.bitwise_and(frame, frame, mask=mask)
    
    # Show the filtered frame
    cv2.imshow('HSV Filtered Image', filtered_frame)
    
    # Wait for a key press; if 'q' is pressed, exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
