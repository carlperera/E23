# E23

## GPIO Pin layout (as of 24th Aug)
PIN_MOTOR1_IN1 = 17 # LOW - good
PIN_MOTOR1_IN2 = 27 # LOW -good 
PIN_MOTOR1_PWM_ENABLE = 18 # LOW - good 
PIN_MOTOR1_A_OUT = 21# LOW - good 
PIN_MOTOR1_B_OUT = 20 # LOW - good

# motor 1 outputs for feedback (yellow and white) - signal outputted by encoder into the pi pwm 
PIN_MOTOR2_IN1 = 23 # LOW - good
PIN_MOTOR2_IN2 = 24 # LOW -good 
PIN_MOTOR2_PWM_ENABLE = 9 # LOW - good 
PIN_MOTOR2_A_OUT = 14# LOW - good 
PIN_MOTOR2_B_OUT = 15 # LOW - good

PIN_SERVO = 12

# Parameters that can be changed in green_ball_tracker
capWidth and capHeight = The Width and Height of the frame respectively. CHANGE THE LINE DETECTION SUBTRACTION TO HALF OF WHATEVER THE HEIGHT IS IF THE HEIGHT IS CHANGED. 
i.e if min(y1, y2) > y_below - 960/2
Gaussian Blur kernel size must be an odd number
v1_min, v2_min, v3_min, v1_max, v2_max, v3_max = for the tennis ball green filter, HSV goes 0 to 179 for RYGBP for every 30, S and V are 0-255
radius = the size set by openCV to recognise circles.
maxarea and minare = maximum and minimum area (in terms of pixels) that tennis balls will be recognised within.
cv2.VideoCapture(INSERT_CAMERA_NUMBER_HERE)

min_line_length = minimum pixel length for lines to be detected using Hough Transform

# PLEASE NOTE CHANGING THE CAMERA FRAME RESOLUTION WILL REQUIRE RECALIBRATING ALL OF THE MINAREA AND MAXAREA AND RADIUS VALS.

# Milestone 2
## Delivery to collection box
1. How to find the collection box?
    1.1 vision to detect brown? 
    1.2 hard-coded with odometry?
2. How to navigate to the collection box?
    1.1 just go towards it? Do we want it to position the robot not on the edges of the collection box 
3. how to avoid going over our quadrant borders? 
    3.1 odometry would ensure that we never step outside of the borders, but;
    3.2 if we detect a ball on another quadrant, there's no way to tell the robot to not navigate towards it 
        3.2.1 how to make robot know that a detected ball is also within the quadrant bounds?
