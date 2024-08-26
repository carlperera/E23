import cv2

class Vision:

    def __init__(self):
        capWidth = 1280
        capHeight = 960
        self.camera = cv2.VideoCapture(0) # REMOVE cv2.CAP_DSHOW ON THE RPI
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, capWidth) #1280
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, capHeight) #550
        # cv2.namedWindow("Masked frame", cv2.WINDOW_NORMAL)
        # cv2.namedWindow("Webcam", cv2.WINDOW_NORMAL)
        global close_signal
        close_signal = False

    def detect():
        print("nothing")
         



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
