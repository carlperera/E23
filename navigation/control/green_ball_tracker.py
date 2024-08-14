from abc import ABC, abstractmethod
import cv2
import numpy as np

class Tennis_ball_detect(ABC):
    __ball_list = []
    windowWidth = 0
    windowHeight = 0
    def __init__(self):
        __ball_list = self.__ball_list
        windowHeight = self.windowHeight
        windowWidth = self.windowWidth

    # Tennis ball class for define every ball as object
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

    def get_ball_list(self):
        return self.__ball_list

    def set_ball_list(self,ball_list):
        self.__ball_list = ball_list

    @abstractmethod
    def get_balls_number(self):
        pass
    @abstractmethod
    def get_ball_coordinates(self):
        pass
    # Tennis ball tracking function
    def track_balls(self):
        camera = cv2.VideoCapture(0)
        global close_signal
        close_signal = False
        while close_signal==False:

            ret, image = camera.read()
            self.windowHeight = image.shape[1]
            self.windowWidth = image.shape[0]

            if not ret:
                break

            frame_to_thresh = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            #v1_min, v2_min, v3_min, v1_max, v2_max, v3_max = [40, 70, 70, 80, 200, 200]
            v1_min, v2_min, v3_min, v1_max, v2_max, v3_max = [35,27,86, 80, 200, 255]
            thresh = cv2.inRange(frame_to_thresh, (v1_min, v2_min, v3_min), (v1_max, v2_max, v3_max))

            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # find contours in the mask and initialize the current
            # (x, y) center of the ball
            cntss = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

            center = None
            maxarea = 170000.0
            minare = 3000.0
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
                    if radius > 10:
                        # draw the circle and centroid on the frame,
                        # then update the list of tracked points
                        theball = self.Tennis_ball(pixels,int(x),int(y),ball_number)
                        ball_list .append(theball)
                        self.set_ball_list(ball_list)

                        cv2.circle(image, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                        cv2.circle(image, center, 3, (0, 0, 255), -1)
                        cv2.putText(image, "Tennis Ball #" + str(ball_number), (center[0] + 10, center[1]),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255),
                                    1)
                        cv2.putText(image, "(" + str(center[0]) + "," + str(center[1]) + ")",
                                    (center[0] + 10, center[1] + 15),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
                        cv2.putText(image, "Pixel area" + str(pixels), (center[0] + 10, center[1] + 25),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)


            # show the frame to our screen

            cv2.imshow("Webcam", image)

            if cv2.waitKey(1) & 0xFF is ord('q'):
                break

    def close(self):
        global close_signal
        close_signal = True
        cv2.waitKey(1)
        cv2.destroyAllWindows()



class Region_number(Tennis_ball_detect):

    __regions = []
    def __init__(self):
        __regions = self.__regions
        super().__init__()

    # Function for gettion number of all tennis balls
    def get_balls_number(self):
        return len(self.get_ball_list())

    # Return all tennis ball coordinates
    def get_ball_coordinates(self):
        coordinate_list = []
        for i in self.get_ball_list():
            coordinate_list.append((i.x,i.y))
        if(len(coordinate_list) != 0):
            return coordinate_list
        else:
            return None

    def get_regions(self):
        return self.__regions

    def set_regions(self,r_list):
        self.__regions = r_list

    #The function that calculates the center coordinates of 5 different regions
    def centroid(self):
        h  = self.windowHeight
        w =self.windowWidth
        r_list =[]
        for i in range(5):
            r_list.append(((w/10)+(+w/5)*i,h/2))
        self.set_regions(r_list)
        return (r_list)

class General_control(Tennis_ball_detect):
    __regions = []

    def __init__(self):
        __regions = self.__regions
        super().__init__()

    # Function for gettion number of all tennis balls
    def get_balls_number(self):
        return len(self.get_ball_list())

    # Return all tennis ball coordinates
    def get_ball_coordinates(self):
        coordinate_list = []
        for i in self.get_ball_list():
            coordinate_list.append((i.x, i.y))
        if (len(coordinate_list) != 0):
            return coordinate_list
        else:
            return None

    # The function that calculates the center coordinates of overall green balls
    def centroid(self):
        x = 0
        y = 0
        for i in self.get_ball_list():
            x +=i.x
            y += i.y
        return x/len(self.get_ball_list()),y/len(self.get_ball_list())