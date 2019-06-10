import sys
import rospy
import matplotlib
import time
import os

matplotlib.use('Qt5Agg')
import PyQt5
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot, QThread
from PyQt5.QtGui import QFont

from interface_elements import *
import time

title_style = "\
                    QLabel {    \
                        font-family: Helvetica Neue;    \
                        font-size: 25pt;    \
                        font-weight: 100; \
                        text-align: center;    \
                        color: white;  \
                    }"

logo_style = "\
                    QLabel {    \
                        padding: 0px;   \
                        margin: 0px;    \
                    }"

# Using material design colors: https://material.io/guidelines/style/color.html#color-color-palette
main_widget_style = "\
                        QWidget {   \
                            background-color: #263238;     \
                        }"
# Used to be lightgray

quit_btn_style = "\
                    QPushButton {   \
                        color: white;   \
                        background-color: #E57373;  \
                    }"

human_push_style = "\
                    QWidget {   \
                        background-color: #B0BEC5  \
                    }"

class Thread(QThread):
    changePixmap = pyqtSignal(QImage)

    def __init__(self):
        QThread.__init__(self)
        self.drone_name = 'Drone 1'
        self.name = "Drone Video"
        self.service_name = '/get_camera_view'
        self.size = (500,350)
        self.img = 'placeholder.png'
        self.format = QImage.Format_RGB888

        rospy.init_node('camera_view_client')



    def run(self):
        while True:
            rospy.wait_for_service(self.service_name)
            camera_image = rospy.ServiceProxy(self.service_name, GetCameraImage)
            self.new_image = camera_image(0, 'lit')

            msg = self.new_image.image
            image_data = msg.data
            image_height = msg.height
            image_width = msg.width
            bytes_per_line = msg.step

            # convert image from little endian BGR to big endian RGB
            length = int(len(image_data)/2)
            # # unpack data into array
            unpacked_data = array.array('H',image_data)
            # # swap bytes (to swap B and R)
            unpacked_data.byteswap() # causes strange vertical line artifacts
            unpacked_data.reverse() #<>NOTE: reversing the entire list of bytes causes the image to be displayed upside down, but also removes artifacts for some reason
            # # repack with opposite endian format
            image_data = struct.pack('<'+str(length)+'H',*unpacked_data)

            self.image = QImage(image_data,image_width,image_height,bytes_per_line,self.format)

            self.changePixmap.emit(self.image)


class ObservationInterface(QMainWindow): 

    def __init__(self):

        self.app_name = 'Drone Video'

        super(QMainWindow,self).__init__()
        self.main_widget = QWidget()
        self.setCentralWidget(self.main_widget)
        self.initUI()

        print('Observation Interface ready.')


    @pyqtSlot(QImage)
    def setImage(self, image):
        self.label.setPixmap(QPixmap.fromImage(image))


    def initUI(self):
        # create the main layout
        self.main_layout = QGridLayout()
        self.main_widget.setLayout(self.main_layout)
        self.main_widget.setStyleSheet(main_widget_style)

        # create title
        self.title = QLabel(self.app_name)
        self.title.setAlignment(Qt.AlignCenter)
        self.title.setStyleSheet(title_style)
        self.main_layout.addWidget(self.title,0,0,1,13)

        # create quit button and add at top left corner
        self.quit_btn = QPushButton('QUIT')
        self.quit_btn.clicked.connect(self.close)
        self.quit_btn.setStyleSheet(quit_btn_style)
        self.main_layout.addWidget(self.quit_btn,0,0)

        self.drone_video = QLabel()
        # self.drone_video_image = 

        th = Thread()
        th.changePixmap.connect(self.setImage)
        th.start()

        # drone_name = "Drone1"
        # self.drone_video = DroneVideo(drone_name)

        # self.main_layout.addWidget(self.drone_video,1,11,4,1,Qt.AlignCenter)




if __name__ == "__main__":
    app = QApplication(sys.argv)
    obs_app = ObservationInterface()
    print("Before exec")
    obs_app.display_video()
    sys.exit(app.exec_())
    print("End hello")