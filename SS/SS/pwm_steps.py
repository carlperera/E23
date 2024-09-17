import sys
import time

from PyQt5 import QtWidgets, QtGui, QtCore

from green_ball_tracker import  General_control,Region_number
ball_number = 0
coordinates = ""
centroid_text = ""
ball_list = ""
t = General_control()
def Window():
    global ball_number
    global coordinates
    global centroid_text
    global ball_list
    global t

    app = QtWidgets.QApplication(sys.argv)
    v_box = QtWidgets.QVBoxLayout()
    label1 = QtWidgets.QLabel("Python green ball tracking app with opencv")
    v_box.addWidget(label1,alignment=QtCore.Qt.AlignCenter)
    label2 = QtWidgets.QLabel()
    pixmap = QtGui.QPixmap("balls.jpeg")
    pixmap_resized = pixmap.scaled(480, 360, QtCore.Qt.KeepAspectRatio)
    label2.setPixmap(pixmap_resized)
    v_box.addWidget(label2,alignment=QtCore.Qt.AlignCenter)

    # Select Class
    label0 = QtWidgets.QLabel("Select Class")
    Region_button = QtWidgets.QRadioButton("Region Number")
    General_button = QtWidgets.QRadioButton("General Control")
    radio_button = QtWidgets.QPushButton("Select")
    def click0():
        global  t
        if(Region_button.isChecked()):
            t = Region_number()
        if(General_button.isChecked()):
            t = General_control()
        t.close()

    radio_button.clicked.connect(click0)

    v_box0 = QtWidgets.QVBoxLayout()
    v_box0.addWidget(label0)
    v_box0.addWidget(Region_button)
    v_box0.addWidget(General_button)
    v_box0.addWidget(radio_button)
    v_box.addLayout(v_box0)



    # Webcam start
    button1 = QtWidgets.QPushButton("Start green ball tracking (press q for stop) ")
    def click1():
        global t
        t.track_balls()
    button1.clicked.connect(click1)
    v_box.addWidget(button1)


    # Find total green  ball numbers

    button2 = QtWidgets.QPushButton("Get number of all balls")
    label3 = QtWidgets.QLabel("Total tennis ball number is:" + str(ball_number))
    def click2():
        global t
        global ball_number
        ball_number = t.get_balls_number()
        if ball_number > 0 :
            label3.setText("Total tennis ball number is:" + str(ball_number))
        else:
            label3.setText("There are no tennis balls in view")

    button2.clicked.connect(click2)

    h_box = QtWidgets.QHBoxLayout()
    h_box.addWidget(button2)
    h_box.addWidget(label3)
    v_box.addLayout(h_box)

    # Ball coordinates

    label4 = QtWidgets.QLabel(str(coordinates))
    def click3():
        global t
        global coordinates
        coordinates = t.get_ball_coordinates()
        label4.setText(str(coordinates))

    button3 = QtWidgets.QPushButton("Find all coordinates")

    button3.clicked.connect(click3)
    h_box2 = QtWidgets.QHBoxLayout()
    h_box2.addWidget(button3)
    h_box2.addWidget(label4)
    v_box.addLayout(h_box2)
    # centroid
    label5 = QtWidgets.QLabel(str(centroid_text))
    def click4():
        global t
        global centroid_text
        centroid_text = t.centroid()
        label5.setText(str(centroid_text))
    button4 = QtWidgets.QPushButton("Centroid")
    button4.clicked.connect(click4)
    h_box3 = QtWidgets.QHBoxLayout()
    h_box3.addWidget(button4)
    h_box3.addWidget(label5)
    v_box.addLayout(h_box3)

    # ball list

    label6 = QtWidgets.QLabel(str(ball_list))

    def click5():
        global t
        global ball_list
        ball_list = ""

        for i in  t.get_ball_list():
            ball_list +=str(i)
        label6.setText(str(ball_list))

    button5 = QtWidgets.QPushButton("Ball List")
    button5.clicked.connect(click5)
    h_box4 = QtWidgets.QHBoxLayout()
    h_box4.addWidget(button5)
    h_box4.addWidget(label6)
    v_box.addLayout(h_box4)

    v_box.addStretch()
    window = QtWidgets.QWidget()
    window.setWindowTitle("Green Ball Tracking")
    window.setLayout(v_box)

    window.setGeometry(100,100,640,480)
    window.show()
    time.sleep(1)
    sys.exit(app.exec())




def main():
    Window()

if __name__ == '__main__':
    main()