#!/usr/bin/env python3

from cgitb import text
from glob import glob
from turtle import color
from kivy.core.window import Window
import numpy as np
import cv2
import sys
import pyrealsense2 as rs
from kivy.lang import Builder
from kivy.clock import Clock
from kivy.graphics.texture import Texture
from kivy.uix.image import Image 
from kivy.uix.video import Video 
from kivy.config import Config
from kivymd.app import MDApp
from kivymd.uix.boxlayout import MDBoxLayout 
from kivymd.uix.menu import MDDropdownMenu 
import rospy
from std_msgs.msg import Empty, Bool, Int64, String
import rospkg
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseActionResult
from gui_interface.msg import patient_assistance, face_detection
import random

Config.set('graphics', 'width', '1920')
Config.set('graphics', 'height', '1080')
Config.write()

Config.set('graphics', 'fullscreen', 1)
Config.set('graphics', 'window_state', 'maximized')
Config.write()

class DOT_PAQUITOP_GUI(MDApp):

    def __init__(self, **kwargs):
        rospy.init_node('paquitop_gui')
        super().__init__(**kwargs)

        # Init interface
        self.layout = Builder.load_file('dot_paquitop_GUI.kv')

        # Init camera
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        profile = self.pipeline.start(config)
        align_to = rs.stream.color
        self.align = rs.align(align_to)
        
        # Init topic
        self.patient_data = patient_assistance()
        self.patient_data.temperature = -1
        self.patient_data.need_help = False
        self.id = rospy.Publisher("/id", Int64, queue_size=1)
        self.face_publisher = rospy.Publisher("/faces", face_detection, queue_size=1)
        self.patient_publisher = rospy.Publisher("/patient_data", patient_assistance, queue_size=1)
        rospy.Subscriber("/patient_name", String, self.NameReceiver)

         # Create the haar cascade
        self.rospack = rospkg.RosPack()
        self.directoryPath = self.rospack.get_path('gui_interface')
        self.subdirectory = "scripts"
        self.cascName = "haarcascade_frontalface_default.xml"
        self.faceCascade = cv2.CascadeClassifier(self.directoryPath+"/"+self.subdirectory+"/"+self.cascName)
        test = self.faceCascade.load(self.directoryPath+"/"+self.subdirectory+"/"+self.cascName)

    def build(self):
        
        self.image = Image(pos_hint={"center_x": .775, "center_y":0.45},size_hint=(.4,.5),keep_ratio=True)
        self.layout.add_widget(self.image)
        Clock.schedule_interval(self.load_video,1.0/10.0)
                 
        return self.layout

    def load_video(self, *args):
        
        default = cv2.aruco.DICT_5X5_100
        arucoDict = cv2.aruco.Dictionary_get(default)
        arucoParams = cv2.aruco.DetectorParameters_create()

        # Get frameset of color and depth
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())

        (corners, ids, rejected) = cv2.aruco.detectMarkers(color_image, arucoDict, parameters=arucoParams)
        
        
        # Face detect:
        frame = cv2.flip(color_image,1)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.faceCascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=10, minSize=(40, 40), flags = cv2.CASCADE_SCALE_IMAGE)
        face_data = face_detection()
        face_data.num_faces = len(faces)
        face_data.face = np.zeros(4)
        if face_data.num_faces != 0:
            face_data.face = faces[0]
        self.face_publisher.publish(face_data)

        if len(corners) > 0:
            # flatten the ArUco IDs list         
            ids = ids.flatten()             

            for (markerCorner, markerID) in zip(corners, ids):
                # extract the marker corners (which are always returned in
                # top-left, top-right, bottom-right, and bottom-left order)
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                cv2.line(color_image, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(color_image, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(color_image, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(color_image, bottomLeft, topLeft, (0, 255, 0), 2)
                # compute and draw the center (x, y)-coordinates of the ArUco
                # marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(color_image, (cX, cY), 4, (0, 0, 255), -1)
                # draw the ArUco marker ID on the image
                cv2.putText(color_image, str(markerID),(topLeft[0] - 15, topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                id = Int64()
                id.data = markerID
                self.id.publish(id)
                
                        
        frame = cv2.resize(color_image, None, fx=1.0, fy=1.0, interpolation=cv2.INTER_AREA)
        frame = cv2.flip(frame, 1)
        buffer = cv2.flip(frame, 0).tobytes()
        texture = Texture.create(size=(frame.shape[1], frame.shape[0]), colorfmt='bgr')
        texture.blit_buffer(buffer, colorfmt = 'bgr', bufferfmt = 'ubyte')
        self.image.texture = texture

    def identificationOK(self, name):
        self.layout.ids.identification.md_bg_color = (20/255,180/255,10/255,.6)
        self.layout.ids.identification.text = "Hi " + name + "!"

        self.layout.ids.help_text.text_color = (0,0,0,1)
        self.layout.ids.needHelp.md_bg_color = (20/255,180/255,10/255,.6)
        self.layout.ids.noNeedHelp.md_bg_color = (220/255,20/255,60/255,.6)
    
    def helpPlease(self, *args):
        self.layout.ids.bodyTemp_text.text_color = (0,0,0,1)
        self.layout.ids.acquireTemp.md_bg_color = (52/255,168/255,235/255,.6)
        self.patient_data.need_help = True

    def noHelpThanks(self, *args):
        self.layout.ids.bodyTemp_text.text_color = (0,0,0,1)
        self.layout.ids.acquireTemp.md_bg_color = (52/255,168/255,235/255,.6)
        self.patient_data.need_help = False

    def acqTemp(self, *args):
        self.layout.ids.goOn_text.text_color = (0,0,0,1)
        self.layout.ids.moveOn.md_bg_color = (20/255,180/255,10/255,.6)
        delta = random.randint(-10,5)
        delta = delta/5
        self.patient_data.temperature = 38 + delta


    def goON(self, *args):
        self.layout.ids.identification.md_bg_color = (200/255,200/255,200/255,1)
        self.layout.ids.identification.text = "Waiting for identifier"
        
        self.layout.ids.help_text.text_color = (.8, .8, .8, 1)
        self.layout.ids.needHelp.md_bg_color = (20/255,180/255,10/255,.1)
        self.layout.ids.noNeedHelp.md_bg_color = (220/255,20/255,60/255,.1)
        self.layout.ids.bodyTemp_text.text_color = (.8, .8, .8, 1)
        self.layout.ids.acquireTemp.md_bg_color = (52/255,168/255,235/255,.1)
        self.layout.ids.goOn_text.text_color = (.8, .8, .8, 1)
        self.layout.ids.moveOn.md_bg_color = (20/255,180/255,10/255,.1)

        
        self.patient_publisher.publish(self.patient_data)
        self.patient_data.temperature = -1
        self.patient_data.need_help = False
        
        # Tablet store
        # count = 0
        # while count < 3:
        #     count = count +1
        #     retrain = rospy.Publisher("/retrain_tablet", Bool, queue_size=1)
        #     retrain_msg = Bool()
        #     retrain_msg.data = True
        #     retrain.publish(retrain_msg)

    def NameReceiver(self, data):
        name = data.data
        self.identificationOK(name)

if __name__ == '__main__':
    
    DOT_PAQUITOP_GUI().run()

    # initialize Publisher topic extract/retrain table    
    # gui.tab_ext = rospy.Publisher("/extract_tablet", Bool, queue_size=1)
    # gui.tab_ret = rospy.Publisher("/retrain_tablet", Bool, queue_size=1)
    # gui.id = rospy.Publisher("/id", Int64, queue_size=1)
    # tab_ext_msg = Bool()
    # tab_ext_msg.data = False
    # gui.tab_ext.publish(tab_ext_msg)
    # gui.tab_ret.publish(tab_ext_msg)

    # initialize Subscriber topic
    
    # gui.run()
    
