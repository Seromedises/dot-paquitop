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

Config.set('graphics', 'width', '1920')
Config.set('graphics', 'height', '1080')
Config.write()

Config.set('graphics', 'fullscreen', 1)
Config.set('graphics', 'window_state', 'maximized')
Config.write()

def NameReceiver(data):
    name = data.data
    DOT_PAQUITOP_GUI.identificationOK(name)


class DOT_PAQUITOP_GUI(MDApp):

    def __init__(self, **kwargs):
        rospy.init_node('paquitop_gui')
        super().__init__(**kwargs)
        self.layout = Builder.load_file('dot_paquitop_GUI.kv')
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        profile = self.pipeline.start(config)
        align_to = rs.stream.color
        self.align = rs.align(align_to)

        # initialize Publisher topic extract/retrain table    
        self.tab_ext = rospy.Publisher("/extract_tablet", Bool, queue_size=1)
        self.tab_ret = rospy.Publisher("/retrain_tablet", Bool, queue_size=1)
        self.id = rospy.Publisher("/id", Int64, queue_size=1)
        tab_ext_msg = Bool()
        tab_ext_msg.data = False
        self.tab_ext.publish(tab_ext_msg)
        self.tab_ret.publish(tab_ext_msg)

        # initialize Subscriber topic
        rospy.Subscriber("/patient_name", String, NameReceiver)
        
        
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

        images = color_image

        (corners, ids, rejected) = cv2.aruco.detectMarkers(images, arucoDict, parameters=arucoParams)
        
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

                cv2.line(images, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(images, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(images, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(images, bottomLeft, topLeft, (0, 255, 0), 2)
                # compute and draw the center (x, y)-coordinates of the ArUco
                # marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(images, (cX, cY), 4, (0, 0, 255), -1)
                # draw the ArUco marker ID on the image
                cv2.putText(images, str(markerID),(topLeft[0] - 15, topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                id = Int64()
                id.data = markerID
                self.id.publish(id)
                
                        
        frame = cv2.resize(images, None, fx=1.0, fy=1.0, interpolation=cv2.INTER_AREA)
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
        # Aggiungere codice per salvataggio richiesta
        self.layout.ids.bodyTemp_text.text_color = (0,0,0,1)
        self.layout.ids.acquireTemp.md_bg_color = (52/255,168/255,235/255,.6)

    def noHelpThanks(self, *args):
        # Aggiungere codice per salvataggio richiesta
        self.layout.ids.bodyTemp_text.text_color = (0,0,0,1)
        self.layout.ids.acquireTemp.md_bg_color = (52/255,168/255,235/255,.6)

    def acqTemp(self, *args):
        self.layout.ids.goOn_text.text_color = (0,0,0,1)
        self.layout.ids.moveOn.md_bg_color = (20/255,180/255,10/255,.6)


    def goON(self, *args):
        self.layout.ids.identification.md_bg_color = (200/255,200/255,200/255,1)
        self.layout.ids.identification.text = "Waiting for identifier"
        
        self.layout.ids.help_text.text_color = (0,0,0,.1)
        self.layout.ids.needHelp.md_bg_color = (20/255,180/255,10/255,.1)
        self.layout.ids.noNeedHelp.md_bg_color = (220/255,20/255,60/255,.1)
        self.layout.ids.bodyTemp_text.text_color = (0,0,0,.1)
        self.layout.ids.acquireTemp.md_bg_color = (52/255,168/255,235/255,.1)
        self.layout.ids.goOn_text.text_color = (0,0,0,.1)
        self.layout.ids.moveOn.md_bg_color = (20/255,180/255,10/255,.1)

        # Tablet store
        count = 0
        while count < 3:
            count = count +1
            retrain = rospy.Publisher("/retrain_tablet", Bool, queue_size=1)
            retrain_msg = Bool()
            retrain_msg.data = True
            retrain.publish(retrain_msg)

if __name__ == '__main__':
    
    DOT_PAQUITOP_GUI().run()
    
