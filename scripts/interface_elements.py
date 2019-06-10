#!/usr/bin/env python

from __future__ import division

"""
This file contains the class definitions of various elements used in the human
interface for the Cops and Robots 2.0 experiment.
"""

__author__ = "Ian Loefgren"
__copyright__ = "Copyright 2017, Cohrint"
__credits__ = ["Ian Loefgren"]
__license__ = "GPL"
__version__ = "1.1" # Image topic subscription over ros
__maintainer__ = "Ian Loefgren"
__email__ = "ian.loefgren@colorado.edu"
__status__ = "Development"

import sys
import yaml
import rospy
import struct
import array
import time
import os

import PyQt5
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot, QSize, QByteArray, QRect, QTimer
from PyQt5.QtGui import QFont, QPixmap, QImage, QPainter, QColor

from std_msgs.msg import String
from airsim_bridge.srv import *
# from observation_interface.msg import *

# For viewing the image topic
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# various style sheets for widgets

PullQuestion_style = "\
                        .QWidget {   \
                            min-width: 300px;   \
                            max-width: 300px;   \
                            background-color: #546E7A; \
                        }"

groupbox_style = "\
                    QGroupBox {  \
                        font-size: 12pt;    \
                        font-weight: 10;    \
                        text-align: center; \
                        background-color: #B0BEC5; \
                    }"

tab_style = "\
                    .QWidget {  \
                        background-color: white; \
                    }"
tab_widget_style = "\
                    .QWidget {  \
                        background-color: white; \
                        color: #B0BEC5; \
                    }"


yes_btn_style = "\
                QPushButton GetCameraImage{   \
                    color: white;   \
                    background-color: green;    \
                    font-size: 8pt; \
                    margin: 5px;    \
                    padding: 5px;   \
                    min-width: 50px; \
                }"

no_btn_style = "\
                QPushButton {   \
                    color: white;   \
                    background-color: darkred;  \
                    font-size: 8pt; \
                    margin: 5px;    \
                    padding: 5px;   \
                    min-width: 50px; \
                }"

null_btn_style = "\
                    QPushButton {   \
                        color: black;   \
                        background-color: skyblue;  \
                        font-size: 8pt; \
                        margin: 5px;    \
                        padding: 5px;   \
                        min-GetCameraImagewidth: 50px; \
                    }"

question_text_style = "\
                        QLabel {    \
                            font: bold; \
                            font-size: 9pt;   \
                            background-color: #B0BEC5  \
                        }"
past_question_text_style = "\
                        QLabel {    \
                            font: bold; \
                            font-size: 9pt;   \
                            color: white \
                        }"

send_btn_style = "\
                    QPushButton {   \
                        color: white;   \
                        background-color: green;    \
                        font-size: 12pt;    \
                        min-height: 35px;   \
                    }"

clear_btn_style = "\
                    QPushButton {   \
                        color: black;   \
                        background-color: #B0BEC5;    \
                        font-size: 11pt;    \
                        min-height: 25px;   \
                    }"

widget_title_style = "\
                QLabel {    \
                    font-size: 12pt;    \
                    font-weight: 10;    \
                    text-align: center; \
                    color: white;  \
                    background-color: #263238;  \
                }"

i_know_robber_style = "\
                QLabel {    \
                    font-size: 10pt;    \
                    font-weight: 10;    \
                    text-align: center; \
                    color: white;  \
                    background-color: #263238;  \
                }"

answer_indicator_style = "\
                            .QLabel {   \
                                max-width: 10px;    \
                                max-height: 20px;   \
                                color: white;  \
                            }"


tab_widget_style = "\
                QTabWidget {   \
                    color: white;  \
                    background-color: #B0BEC5; \
                }"


class RobotPull(QWidget):
    """
    The robot pull questions widget. Displays top <num_questions> questions, and
    presents user with 'yes','no' and 'I don't know' options. Yes and no options
    publish to the specified ROS topic. The RobotPull widget contains
    <num_questions> number of PullQuestion widgets.

    The questions are received via subscription to specified ROS topic, and
    the text of the questions are set as the text of the PullQuestion widgets.
    """

    def __init__(self):
        super(QWidget,self).__init__()

        # parameters for display
        self.num_questions = 3
        self.name = "Robot Questions"
        # self.yes_btn_style

        self.initUI()

        rospy.Subscriber("robot_questions", Question, self.question_update)
        self.pub = rospy.Publisher("answers",Answer,queue_size=10)

    def initUI(self):
        # create visual container for PullQuestion widgets
        self.container = QGroupBox('Robot Questions')
        size_policy = self.container.sizePolicy()
        size_policy.setVerticalPolicy(QSizePolicy.Expanding)
        size_policy.setHorizontalPolicy(QSizePolicy.Expanding)
        self.container.setSizePolicy(size_policy)
        self.container.setStyleSheet(groupbox_style)
        # self.container.setStyleSheet(PullQuestion_style)
        self.container_layout = QVBoxLayout()
        self.container_layout.addWidget(self.container)


        self.previous_q_container = QGroupBox()
        self.prev_q_layout = QVBoxLayout()
        self.container_layout.addWidget(self.previous_q_container)
        self.previous_q_container.setLayout(self.prev_q_layout)

        # create last question and last answer labels
        self.last_question = QLabel("Last question was: ")
        self.last_question.setStyleSheet(past_question_text_style)
        self.prev_q_layout.addWidget(self.last_question)

        self.last_answer = QLabel("Last answer was: ")
        self.last_answer.setStyleSheet(past_question_text_style)
        self.prev_q_layout.addWidget(self.last_answer)

        # main widget layout
        self.main_layout = QVBoxLayout()

        # create PullQuestion widgets and add them to layout
        self.make_question_fields()

        self.container.setLayout(self.main_layout)
        self.setLayout(self.container_layout)

    def make_question_fields(self):
        """
        Creates the question widgets to be added to the question list.
        The question widget contains the text of the question, a yes button,
        a no button, and a I don't know button. These buttons are connected to
        an answered question function described in PullQuestion
        """
        self.question_fields = []

        for i in range(0,self.num_questions):
            question_field = PullQuestion()
            self.question_fields.append(question_field)
            self.main_layout.addWidget(question_field)
            # question_field.hide()

    def get_questions(self,qids):
        """
        For transmitted integer question IDs, get the corresponding questions
        from the question list, and return a dictionary. (NOT IN USE)
        """

        question_list = []
        for qid in qids:
            question_list.append(questions[qid])

        return question_list

    def scale_VOI_magnitude(self,weights):
        """
        Scale the magintudes of the VOI wrt the maximum weight. (NOT IN USE)
        """
        max_value = 100
        scale_factor = max_value / max(weights)
        weights = [weight*scale_factor for weight in weights]
        return weights

    def get_new_question(self):
        """
        Get new next most valuable question to display if a question is answered
        'I don't know'.
        """
        if self.count < len(self.questions):
            new_question = self.questions[self.count]
            new_question_weight = self.question_weights[self.count]
            new_qid = self.qids[self.count]
            self.count += 1
            return (new_qid,new_question,new_question_weight)
        else:
            return None

    def question_update(self,msg):
        """
        Update the displayed questions by getting the new questions from ROS
        topic, creating the associated question objects, and updating the display.
        """
        print('msg received')
        self.count = 0
        self.qids = msg.qids
        self.question_weights = msg.weights
        self.questions = msg.questions
        # weights = self.scale_VOI_magnitude(weights)

        for i in range(0,self.num_questions):
            self.question_fields[i].set_question(self.qids[i],self.questions[i],
                            self.question_weights[i])
            self.count += 1
            # self.question_fields[i].show()

class PullQuestion(QWidget):
    """
    Widget to display a robot pull question. Each instance contains a text field
    a yes button, a no button, and an I don't know button. NOT IN USE: also a
    VOI bar
    """

    def __init__(self,qid=0,question_text='hello'):

        super(QWidget,self).__init__()

        self.pub = rospy.Publisher("answered",Answer,queue_size=10)

        self.layout = QHBoxLayout()

        self.qid = qid

        # make sure widget doesn't resize when hidden
        size_policy = self.sizePolicy()
        size_policy.setRetainSizeWhenHidden(True)
        self.setSizePolicy(size_policy)

        self.text = QLabel(question_text)
        self.text.setWordWrap(True)
        self.text.setStyleSheet(question_text_style)

        self.yes_btn = QPushButton('YES')
        self.yes_btn.setSizePolicy(QSizePolicy())
        self.yes_btn.setStyleSheet(yes_btn_style)
        self.yes_btn.clicked.connect(self.answered)

        self.no_btn = QPushButton('NO')
        self.no_btn.setSizePolicy(QSizePolicy())
        self.no_btn.setStyleSheet(no_btn_style)
        self.no_btn.clicked.connect(self.answered)

        self.null_btn = QPushButton('?')
        self.null_btn.setSizePolicy(QSizePolicy())
        self.null_btn.setStyleSheet(null_btn_style)
        self.null_btn.clicked.connect(self.answered)

        self.buttons = [self.yes_btn,self.no_btn,self.null_btn]

        # Make bar to indicate VOI of question
        # self.voi_weight = QProgressBar()
        # self.voi_weight.setSizePolicy(QSizePolicy())
        # self.voi_weight.setTextVisible(False)

        self.layout.addWidget(self.text)
        self.layout.addWidget(self.yes_btn)
        self.layout.addWidget(self.no_btn)
        self.layout.addWidget(self.null_btn)
        # self.layout.addWidget(self.voi_weight)

        self.layout.setSpacing(0)
        self.setLayout(self.layout)

        self.hide()

    def set_question(self,qid,question_text,weight):
        self.qid = qid
        self.weight = weight
        # self.voi_weight.setValue(int(weight))
        self.text.setText(question_text)
        self.show()

    def answer_color(self):
        """
        Flash color when question is answered. NOT IN USE
        """
        color = ''
        if self.sender() is self.yes_btn:
            # self.setStyleSheet("background-color: green;")
            color = 'green'
            self.answer_rect.fill(QColor(color))
            self.answer_rect_container.setPixmap(self.answer_rect)

        elif self.sender() is self.no_btn:
            # self.setStyleSheet("background-color: darkred;")
            color = 'darkred'
            self.answer_rect.fill(QColor(color))
            self.answer_rect_container.setPixmap(self.answer_rect)

        elif self.sender() is self.null_btn:
            # self.setStyleSheet("background-color: skyblue;")
            color = 'skyblue'
            self.answer_rect.fill(QColor(color))
            self.answer_rect_container.setPixmap(self.answer_rect)

        # w = QWidget()
        # self.show()
        # time.sleep(1)
        # self.setStyleSheet("background-color: lightgray")

    def answered(self):
        """
        When a question is answered, publish that answer to ROS topic and hide
        the question field until next update
        """
        # hide the question
        self.hide()
        # determine answer based on which button was clicked
        ans = None
        ans_text = None
        if self.sender() is self.yes_btn:
            ans = 1
            ans_text = 'Yes'
        elif self.sender() is self.no_btn:
            ans = 0
            ans_text = 'No'
        elif self.sender() is self.null_btn:
            ans_text = 'I don\'t know'
        prev_q = self.sender().parentWidget().text.text()
        self.parentWidget().parentWidget().last_question.setText('Last question was: ' + self.sender().parentWidget().text.text())
        self.parentWidget().parentWidget().last_answer.setText('Last answer was: ' + ans_text)
        question = self.parentWidget().parentWidget().get_new_question()
        if question is not None:
            self.set_question(question[0],question[1],question[2])

        # create answer ROS message
        if ans is not None:
            msg = Answer()
            msg.qid = self.qid
            msg.question = prev_q
            msg.ans = ans
            # publish answer
            self.pub.publish(msg)




robots = ["Deckard","Roy","Pris","Zhora"]

targets = ["nothing","a robber","Roy","Pris","Zhora"]

certainties = ["I know"]

positivities = ["is", "is not"]

object_relations = ["behind","in front of","left of","right of"] # removed 'near'

objects = ["the bookcase","the cassini poster","the chair","the checkers table",
            "the desk","the dining table","the fern","the filing cabinet",
            "the fridge","the mars poster","the cop"]
            # removed Deckard from objects as no voi questions exist for Deckard.
            # See voi.py in policy_translator package
            # added 'the cop'

area_relations = ["inside"] # removed 'near', 'outside'

areas = ["the study","the billiard room","the hallway","the dining room",
            "the kitchen","the conservatory"]

movement_types = ["moving","stopped"]

movement_qualities = ["slowly","moderately","quickly"]

# class HumanPush(QWidget):
#     """
#     The human push questions widget. At any time the user can construct a
#     sentence from a codebook of words and push that observation to the robot via
#     ROS topic.
#     """

#     def __init__(self):
#         super(QWidget,self).__init__()

#         self.name = "Human Observations"

#         self.pub = rospy.Publisher("human_push",String,queue_size=10)

#         self.initUI()

#     def initUI(self):
#         self.main_and_title = QVBoxLayout()
#         self.main_layout = QHBoxLayout()

#         # add name as a label
#         self.name_label = QLabel(self.name)
#         self.name_label.setStyleSheet(widget_title_style)
#         self.main_and_title.addWidget(self.name_label)

#         self.main_and_title.addLayout(self.main_layout)

#         # make tab container
#         self.tabs = QTabWidget()
#         self.position_objects_tab = QWidget()
#         self.position_objects_tab.layout = QHBoxLayout()
#         self.position_objects_tab.setStyleSheet(tab_widget_style)
#         self.position_area_tab = QWidget()
#         self.position_area_tab.layout = QHBoxLayout()
#         self.position_area_tab.setStyleSheet(tab_widget_style)
#         self.movement_tab = QWidget()
#         self.movement_tab.layout = QHBoxLayout()
#         self.tabs.addTab(self.position_objects_tab,'Position (Objects)')
#         self.tabs.addTab(self.position_area_tab,'Position (Area)')
#         # self.tabs.addTab(self.movement_tab,'Movement')

#         # add statement 'I know a robber is...' next to tabs
#         self.statment_preface = QLabel('I know a robber...')
#         self.statment_preface.setStyleSheet(i_know_robber_style)
#         self.main_layout.addWidget(self.statment_preface)

#         # add tabs to main layout
#         self.tabs.setStyleSheet(tab_widget_style)
#         self.main_layout.addWidget(self.tabs)

#         self.widget_list = []
#         # make Position Objects codebook
#         # object_boxes = [certainties,targets,positivities,object_relations,
#         #                     objects]
#         object_boxes = [positivities,object_relations,objects]
#         object_layout, object_widget_list = self.make_codebook(object_boxes,
#                                                 self.position_objects_tab.layout)
#         self.position_objects_tab.setLayout(object_layout)
#         self.widget_list.append(object_widget_list)

#         # make Position Area codebook
#         # area_boxes = [certainties,targets,positivities,area_relations,areas]
#         area_boxes = [positivities,area_relations,areas]
#         area_layout, area_widget_list = self.make_codebook(area_boxes,
#                                             self.position_area_tab.layout)
#         self.position_area_tab.setLayout(area_layout)
#         self.widget_list.append(area_widget_list)

#         # make Movement codebook
#         # movement_boxes = [certainties,targets,positivities,movement_types,
#         #                     movement_qualities]
#         # movement_layout, movement_widget_list = self.make_codebook(movement_boxes,
#         #                                             self.movement_tab.layout)
#         # self.movement_tab.setLayout(movement_layout)
#         # self.widget_list.append(movement_widget_list)

#         # add the question parts to the codebooks
#         # self.add_list_items()

#         # make the 'send' and 'clear' buttons
#         self.send_btn = QPushButton('Send')
#         self.send_btn.clicked.connect(self.publish_msg)
#         self.send_btn.setStyleSheet(send_btn_style)
#         self.clear_btn = QPushButton('Clear')
#         self.clear_btn.clicked.connect(self.clear_selection)
#         self.clear_btn.setStyleSheet(clear_btn_style)

#         # make layout for 'send' and 'clear' buttons
#         self.btn_column = QVBoxLayout()
#         self.btn_column.addWidget(self.clear_btn)
#         self.btn_column.addWidget(self.send_btn)
#         self.main_layout.addLayout(self.btn_column)

#         # self.setSizePolicy(QSizePolicy())
#         self.setAutoFillBackground(True);
#         palette = self.palette()
#         role = self.backgroundRole()
#         palette.setColor(role, QColor('green'))
#         self.setPalette(palette)
#         self.setLayout(self.main_and_title)

#     def make_codebook(self,boxes,tab_widget_layout):
#         """
#         Make a codebook for a category of observations given the chosen items
#         """
#         widget_list = []
#         for box in boxes:
#             codebook_box = QListWidget()
#             count = 1
#             for item in box:
#                 list_item = QListWidgetItem(item)
#                 codebook_box.insertItem(count,list_item)
#                 count += 1
#             tab_widget_layout.addWidget(codebook_box)
#             widget_list.append(codebook_box)

#         return tab_widget_layout, widget_list

#     def get_answer(self):
#         """
#         Get the answer the user has created with the selections in the three
#         codebook boxes.
#         """
#         # get index of selected tab
#         idx = self.tabs.currentIndex()
#         answer = 'I know Roy'

#         # get selected text from all boxes in selected tab
#         for codebook in self.widget_list[idx]:
#             ans = ''
#             try:
#                 ans = codebook.selectedItems()[0].text()
#             except IndexError:
#                 error_dialog = QErrorMessage(self)
#                 error_dialog.showMessage('You must make a selection in all boxes before \
#                                             attempting to send a message.')
#                 return None
#             answer = answer + " " + ans
#         answer.lstrip(' ')
#         print(answer)
#         return answer

#     def clear_selection(self):
#         """
#         Clear the selected components of an answer when the 'CLEAR' button is
#         is pressed, or when an answer is sent.
#         """
#         for widget in self.widget_list:
#             for codebook in widget:
#                 codebook.clearSelection()

#     def publish_msg(self):
#         """
#         Get the answer, clear selections and publish answer to question to
#         topic over ROS.
#         """
#         answer = self.get_answer()
#         if answer is not None:
#             msg = String(answer)
#             self.pub.publish(msg)

# #        self.clear_selection()


# MapDisplay_style = "\
#                         MapDisplay {   \
#                             border-style: solid;   \
#                             border-width: 10 px;  \
#                             border-color: black; \
#                         }"

# class MapDisplay(QWidget):
#     """
#     The widget to display the cop's belief and position overlayed onto a map
#     of the environment. Loaded from directory policy_translator/tmp/ when
#     callback is triggered. Callback is triggered by ROS topic, but recevies
#     empty message.
#     """

#     def __init__(self):
#         super(QWidget,self).__init__()
#         print("Initializing map")

#         self.name = "Belief Map"

#         rospy.Subscriber("/interface_map", Image, self.map_update)
#         self.bridge = CvBridge()
#         self.format = QImage.Format_RGB888

#         self.initUI()


#     def initUI(self):
#         self.main_layout = QVBoxLayout()

#         # set style and add title
#         self.name_label = QLabel(self.name)
#         self.name_label.setStyleSheet(widget_title_style)
#         self.main_layout.addWidget(self.name_label)

#         # create label to hold image
#         self.image_view = QPixmap()
#         self.pic_label = QLabel(self)
#         self.main_layout.addWidget(self.pic_label)

#         self.setLayout(self.main_layout)

#         self.setAutoFillBackground(True);
#         palette = self.palette()
#         role = self.backgroundRole()
#         palette.setColor(role, QColor('green'))
#         self.setPalette(palette)
#         self.show()

#     def map_update(self, map_msg): # callback for the policy translator publisher
#         # load image from the topic
#         image = self.bridge.imgmsg_to_cv2(map_msg, "rgb8")
#         height, width, channel = image.shape
#         bytesPerLine = 3 * width
#         qImg = QImage(image.data, width, height, bytesPerLine, QImage.Format_RGB888)
#         pixMap = QPixmap.fromImage(qImg)
#         self.pic_label.setPixmap(pixMap)

class VideoContainer(QWidget):
    """
    A general class for widgets to display video feeds by receiving video data
    from ROS topic
    """

    def __init__(self):
        super(QWidget,self).__init__()
        self.initUI()

    def initUI(self):
        self.main_layout = QVBoxLayout()

        self.name_label = QLabel(self.name)
        self.name_label.setStyleSheet(widget_title_style)
        self.main_layout.addWidget(self.name_label)

        self.counter = 0

        # create canvas to display image
        self.canvas = VideoCanvas(self.size)
        self.canvas.setMinimumSize(QSize(*self.size))

        self.image = QImage()

        self.main_layout.addWidget(self.canvas)

        self.setLayout(self.main_layout)

class VideoCanvas(QWidget):
    """
    Widget on which video frames can be painted by overiding paintEvent
    """
    def __init__(self,size):
        super(VideoCanvas,self).__init__()
        self.size = size
        self.image = None

    def paintEvent(self,event):
        painter = QPainter(self)
        if (self.image is not None) and (not self.image.isNull()):
            painter.drawImage(self.rect(),self.image)

class DroneVideo(VideoContainer):
    """
    Subclasses VideoDisplay to display the cop's camera feed. Performs some
    intermediate processing to convert from BRG to RGB.
    """
    def __init__(self,drone_name='pris'):
        super(VideoContainer,self).__init__()
        self.name = "Drone Video"
        # self.topic_name = '/' + cop_name + '/camera/rgb/image_color'
        # self.service_name = '/get_camera_view "camera_id: 0\nview_mode: '+"''"+'" '
        self.service_name = '/get_camera_view'
        self.size = (500,350)
        self.img = 'placeholder.png'
        self.format = QImage.Format_RGB888
        self.initUI()
        print(self.service_name)
        # rospy.ServiceProxy(self.service_name, GetCameraImage, self.ros_update)
        # try:
        rospy.init_node('camera_view_client')
        # while not rospy.is_shutdown():
        rospy.wait_for_service(self.service_name)
        camera_image = rospy.ServiceProxy(self.service_name, GetCameraImage)
        self.new_image = camera_image(0, 'lit')
        self.ros_update()
        # # print(new_image)
        # height = new_image.image.height
        # data = new_image.image.data
        # width = new_image.image.width
        # bytes_per_line = new_image.image.step
        # self.image = QImage(data, width, height, bytes_per_line, QImage.Format_RGB888)
        # # print(self.img)
        # self.canvas.image = self.image.mirrored(True,True)
        # self.canvas.update()
        # print("updating canvas")
        # # except:in
        # #     print("service proxy error")
        
    def display_video(self):
        # rospy.init_node('camera_view_client')
        # while not rospy.is_shutdown():
        self.initUI()
        rospy.wait_for_service(self.service_name)
        camera_image = rospy.ServiceProxy(self.service_name, GetCameraImage)
        self.new_image = camera_image(0, 'lit')
        self.ros_update()

    def ros_update(self):
        """
        Callback function to display a ROS image topic with streaming video data.
        """
        print("callback")
        # process received message
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

        # create QImage with received image data and metadata
        self.image = QImage(image_data,image_width,image_height,bytes_per_line,self.format)
        if not self.image.isNull():
            print("NULL")
            self.canvas.image = self.image.mirrored(True,True) #undo previous reversal
        self.canvas.update()
        print("Updating canvas")
