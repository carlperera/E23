import cv2

# Capture video from the default camera (usually the webcam)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open video source.")
else:
    while True:
        # Read a frame from the video source
        ret, frame = cap.read()
        
        # Check if the frame was read correctly
        if not ret:
            print("Error: Could not read frame.")
            break
        
        # Display the frame in a window named 'Live Feed'
        cv2.imshow('Live Feed', frame)
        
        # Wait for 1 ms and check if the 'q' key was pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the video capture object
    cap.release()
    # Close all OpenCV windows
    cv2.destroyAllWindows()
