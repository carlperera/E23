import cv2
import numpy as np

# Function to display HSV value on mouse hover
def get_hsv_value(event, x, y, flags, param):
    if event == cv2.EVENT_MOUSEMOVE:
        # Extract the pixel from the frame at (x, y)
        pixel = frame[y, x]
        
        # Convert the BGR pixel to HSV
        hsv_pixel = cv2.cvtColor(np.uint8([[pixel]]), cv2.COLOR_BGR2HSV)[0][0]
        
        # Get the HSV values
        h, s, v = hsv_pixel
        
        # Show the HSV value on the image
        hsv_text = f'H: {h} S: {s} V: {v}'
        
        # Print the HSV value to the console
        print(f"{hsv_text}")

# Function to combine contours
def combine_contours(contours, threshold=100, min_area = 300):
    combined = []
    
    for cnt in contours:
        merged = False
        for combined_cnt in combined:
            if cv2.contourArea(cnt) < min_area:
                # Calculate the bounding boxes
                x1, y1, w1, h1 = cv2.boundingRect(combined_cnt)
                x2, y2, w2, h2 = cv2.boundingRect(cnt)

                # Check if the bounding boxes overlap
                if (abs(x1 - x2) < threshold and abs(y1 - y2) < threshold):
                    # Merge the contours by taking the convex hull
                    combined_cnt = cv2.convexHull(np.vstack((combined_cnt, cnt)))
                    merged = True
                    break
        
        if not merged:
            combined.append(cnt)
    
    return combined

# Function to get the largest contour
def get_largest_contour(contours):
    if not contours:
        return None
    return max(contours, key=cv2.contourArea)

# Start capturing video from the webcam
cap = cv2.VideoCapture(2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)  # Set the video width
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)  # Set the video height

# Create a named window
cv2.namedWindow("Webcam Feed")

# Set the mouse callback function for displaying HSV values
cv2.setMouseCallback("Webcam Feed", get_hsv_value)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    # Make a copy of the frame to overlay the contour and HSV features
    frame_copy = frame.copy()

    # Convert to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define the range for brown color
    lower_brown = np.array([4, 30, 75])  # Adjust as needed
    upper_brown = np.array([24, 150, 190])
    mask = cv2.inRange(hsv, lower_brown, upper_brown)

    # Morphological operations to clean up the mask
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Find contours in the mask
    contours, _ = cv2.findContours(closing, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Combine contours if necessary
    combined_contours = combine_contours(contours)

    # Get the largest contour
    largest_contour = get_largest_contour(combined_contours)

    # Draw the largest contour if it exists
    if largest_contour is not None:
        cv2.drawContours(frame, [largest_contour], -1, (0, 255, 0), 2)
        
        # Calculate the center of the largest contour
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            # Draw a dot at the center
            cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)

    # Display the resulting frame with contours
    cv2.imshow("Webcam Feed", frame)

    # Display the masked frame showing brown areas
    cv2.imshow("Masked Frame", mask)

    # Wait for key press; exit on 'q'
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

# Release the webcam and close OpenCV windows
cap.release()
cv2.destroyAllWindows()
