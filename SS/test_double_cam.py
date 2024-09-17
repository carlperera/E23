import cv2
import numpy as np 

class Vision:
    class Tennis_ball:
        pixels = 0
        x = 0
        y = 0
        ball_index = 0
        def __init__(self,pixels,x,y,ball_index):
            self.pixels = pixels
            self.x = x
            self.y = y
            self.ball_index = ball_index

        def __str__(self):
            return "Tennis ball #"+str(self.ball_index)+"\n Pixels"+str(self.pixels)+"\n X:"+str(self.x)+" Y:"+str(self.y)+"\n"


    def __init__(self):
        # ------------------- PRIMARY  -------------------
        self.capWidth_primary = 1280
        self.capHeight_primary = 960
        self.camera_primary = cv2.VideoCapture(0) # REMOVE cv2.CAP_DSHOW ON THE RPI
        self.camera_primary.set(cv2.CAP_PROP_FRAME_WIDTH, self.capWidth_primary) #1280
        self.camera_primary.set(cv2.CAP_PROP_FRAME_HEIGHT, self.capHeight_primary) #550

        # ------------------- SECONDARY  -------------------
        self.capWidth_secondary = 640
        self.capHeight_secondary = 480
        self.camera_secondary = cv2.VideoCapture(2) 
        self.camera_secondary.set(cv2.CAP_PROP_FRAME_WIDTH, self.capWidth_secondary) #1280
        self.camera_secondary.set(cv2.CAP_PROP_FRAME_HEIGHT, self.capHeight_secondary) #550
        # cv2.namedWindow("Masked frame", cv2.WINDOW_NORMAL)
        # cv2.namedWindow("Webcam", cv2.WINDOW_NORMAL)

        self.__regions = []

        self.__ball_list = []
        self.windowHeight = 0
        self.windowWidth = 0
        self.max_ball_x = None
        self.max_ball_y = None

        self.max_ball = None 
        self.averaged_out_bounds = None
        self.inCentre = 0 

    def track_ball(self, frame, camNum):

        self.windowHeight = frame.shape[1]
        self.windowWidth = frame.shape[0]

        # Apply Gaussian blur to the image to make the tennis balls lines smudge
        blurred_image = cv2.GaussianBlur(frame, (31, 31), 0)

        frame_to_thresh = cv2.cvtColor(blurred_image, cv2.COLOR_BGR2HSV) # HSV goes 0 to 179 for RYGBP for every 30, S and V are 0-255
        #v1_min, v2_min, v3_min, v1_max, v2_max, v3_max = [40, 70, 70, 80, 200, 200]
        v1_min, v2_min, v3_min, v1_max, v2_max, v3_max = [25, 45, 70, 80, 255, 255]
        thresh = cv2.inRange(frame_to_thresh, (v1_min, v2_min, v3_min), (v1_max, v2_max, v3_max)) # The cv2.inRange function checks each pixel in the frame_to_thresh image to see if its HSV values fall within the specified range. If a pixel's HSV values are within the range, the corresponding pixel in the thresh image is set to 255 (white). Otherwise, it is set to 0 (black).

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cntss = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

        center = None
        if camNum == 1: # For the primary camera
            maxarea = 170000.0
            minare = 800.0
        elif camNum == 2: # For the secondary camera
            maxarea = 17000
            minare = 300

        ball_number = 0
        ball_list = []

        for cnts in cntss:
            confirm = False
            try:
                pixels = cv2.contourArea(cnts)

                if maxarea >= cv2.contourArea(cnts) >= minare:
                    confirm = True
                    ball_number += 1

            except Exception as e:
                print(e)

            # only proceed if at least one contour was found
            if confirm == True:
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and
                # centroidq

                c = cnts
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                # only proceed if the radius meets a minimum size
                if radius > 7: # CHANGE THIS VALUE FOR CALIBRATION
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    theball = self.Tennis_ball(pixels,int(x),int(y),ball_number)
                    ball_list .append(theball)
                    self.set_ball_list(ball_list)

                    cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    cv2.circle(frame, center, 3, (0, 0, 255), -1)
                    cv2.putText(frame, "Tennis Ball #" + str(ball_number), (center[0] + 10, center[1]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255),
                                1)
                    cv2.putText(frame, "(" + str(center[0]) + "," + str(center[1]) + ")",
                                (center[0] + 10, center[1] + 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
                    cv2.putText(frame, "Pixel area: " + str(pixels), (center[0] + 10, center[1] + 25),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
                    cv2.putText(frame, "Radius: " + str(radius), (center[0] + 10, center[1] + 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
                    # Optional: Draw vertical lines for center region boundaries
                    cv2.line(frame , (int(self.capWidth_primary * 0.4), 0), (int(self.capWidth_primary * 0.4), self.capHeight_primary), (0, 255, 255), 2)  # Top center boundary
                    cv2.line(frame, (int(self.capWidth_primary * 0.6), 0), (int(self.capWidth_primary * 0.6), self.capHeight_primary), (0, 255, 255), 2)  # Bottom center boundary
        
        
        vision_x = -1 
        vision_y = -1

        if ball_list:
            self.max_ball = max(ball_list, key=lambda ball: ball.pixels)

            if self.max_ball:
                cv2.circle(frame, (int(self.max_ball.x), int(self.max_ball.y)), int(radius), (0, 0, 255), 2)
                # print("Max ball is ball: ",max_ball.ball_index)

                left_band = self.capWidth_primary *0.3
                right_band = self.capWidth_primary * 0.7

                if self.max_ball.x < left_band:
                    self.inCentre = 2  # Left third
                elif left_band <= self.max_ball.x <= right_band:
                    self.inCentre = 1  # Middle third
                else:
                    self.inCentre = 3  # Right third
            
                vision_x = self.inCentre
                # print(f"inCentre: {inCentre}")

                top_band = self.capHeight_primary*0.7
                if self.max_ball.y < top_band:
                    vision_y = 0  # not close 
                else:   
                    vision_y = 1  # close 

        # cv2.imshow("Masked frame", mask)
        # cv2.imshow("Webcam", frame)
            # cv2.imshow('Line detected Image', line_detected_img)

        # if cv2.waitKey(1) & 0xFF is ord('q'):
        #     break
        if camNum == 1:
            line_detection = self.line_detection(frame)

            # no balls detected if ball outside of the lines
            if line_detection:
                print("line detected!")
                vision_x = -1
                vision_y = -1

        return (vision_x, vision_y)

    def line_detection(self, frame):
        height = frame.shape[0]
        frame = frame[height // 2:, :]
        
        bounds_averager_counter = 0
        
        mask = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Apply a threshold to keep only close-to-white values
        # Values close to 255 (white) will be kept, others will be set to black
        _, mask = cv2.threshold(mask, 200, 255, cv2.THRESH_BINARY) # Can adjust threshold

        # Apply Gaussian blur to the masked image
        kernel_size = 5
        blur_gray = cv2.GaussianBlur(mask, (kernel_size, kernel_size), 0)

        # Perform Canny edge detection on the blurred image
        low_threshold = 50
        high_threshold = 150
        edges = cv2.Canny(blur_gray, low_threshold, high_threshold)

        # Define the Hough transform parameters
        rho = 1  # Distance resolution in pixels of the Hough grid
        theta = np.pi / 180  # Angular resolution in radians of the Hough grid
        threshold = 15  # Minimum number of votes (intersections in Hough grid cell)
        min_line_length = 150  # Minimum number of pixels making up a line
        max_line_gap = 20  # Maximum gap in pixels between connectable line segments

        # Create an empty image to draw lines on
        line_image = np.zeros_like(frame)

        # Perform Hough Line Transform to detect lines
        lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]),
                                min_line_length, max_line_gap)

        # Draw the detected lines on the line_image
        if lines is not None:
            valid_line_list = []
            
            for line in lines:
                for x1, y1, x2, y2 in line:
                    if (x2-x1 > 125) or (y2-y1 > 125): # valid lines > 100 pixels wide checking that the line isn't on the tennis ball itself, filtering out short lines (court lines are long)
                        valid_line_list.append(line)

                        cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 0), 5)
                        cv2.line(line_image, (x1, y1), (x2, y2), (255, 0, 0), 5)
          
            bounds_averager = [None] * len(valid_line_list)  
        
        output = False # True if out of bounds 
        
        # Check the y coordinate directly above the max ball
        if self.max_ball is not None:
            y_below = self.max_ball.y -1  # Slightly above the ball
            x_pos = self.max_ball.x

            # Check if there is a line detected directly above the ball
            out_bounds = False
            if lines is not None:
                for line in valid_line_list:
                    for x1, y1, x2, y2 in line:
                        
                        # print(min(y1,y2), line)
                        if min(y1, y2) > y_below - 960/2 : # we are checking the line in vertical direction at where the ball is
                            # # Check if the x position is within the line's segment. Coordinate system is the bigger the y, the lower it is
                            if min(x1, x2) <= x_pos <= max(x1, x2): # making sure that the ball is between this line's x coords
                                out_bounds = True
                                # cv2.line(frame, (x1,y1), (x2,y2), (0,0,255), 5)
                                break
                            else: # ball is not actually out of bounds
                                out_bounds = False 
                    
                    if out_bounds:
                        bounds_averager[bounds_averager_counter] = 0
                        out_bounds = True
                        bounds_averager_counter+=1 
                        # print("ball out of bounds")
                        # break
                        # output = False
                        output = True
                        break
                    else:
                        bounds_averager[bounds_averager_counter] = 1
                        bounds_averager_counter+=1 
                        output = False
                        # print("Ball within bounds!")
                    # if bounds_averager_counter == len(bounds_averager)-1: # resetting counter
                    #     bounds_averager_counter = 0
                    #     average_val = sum(x for x in bounds_averager if x is not None)
                    #     # print(average_val)
                    #     if average_val >= 0.8*len(bounds_averager):
                    #         self.averaged_out_bounds = True
                    #         output = False
                    #         print("ball within bounds!")
                    #     else:
                    #         self.averaged_out_bounds = False
                    #         print("ball out of bounds")
                    #         output = True
                    #         break

        return output      


        # # Combine the original frame with the line image
        # if len(frame.shape) == 3:
        #     lines_edges = cv2.addWeighted(cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), 0.8, line_image, 1, 0)
        # else:
        #     lines_edges = cv2.addWeighted(mask, 0.8, line_image, 1, 0)
        # # cv2.imshow("Line Detection", line_image)
        # return lines_edges
    

    def close(self):
        cv2.waitKey(1)
        cv2.destroyAllWindows()

    def get_ball_list(self):
        return self.__ball_list
    

    def set_ball_list(self,ball_list):
        self.__ball_list = ball_list

    def get_balls_number(self):
        return len(self.get_ball_list())
    
    def get_ball_coordinates(self):
        coordinate_list = []
        for ball in self.get_ball_list():
            coordinate_list.append((ball.x,ball.y))
        if(len(coordinate_list) != 0):
            return coordinate_list
        else:
            return None
        
    def get_regions(self):
        return self.__regions
    
    
    def set_regions(self,r_list):
        self.__regions = r_list

    def centroid(self):
        h  = self.windowHeight
        w =self.windowWidth
        r_list =[]
        for i in range(5):
            r_list.append(((w/10)+(+w/5)*i,h/2))
        self.set_regions(r_list)
        return (r_list)
    

    def get_balls_number(self):
        return len(self.get_ball_list())



