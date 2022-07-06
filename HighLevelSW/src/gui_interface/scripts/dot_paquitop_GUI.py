#!/usr/bin/env python3

# Per evitare problemi di risoluzione con diversi schermi:
from cgitb import text
from glob import glob
#from ctypes import windll, c_int64
from turtle import color
#windll.user32.SetProcessDpiAwarenessContext(c_int64(-4))

# Per aprire l'interfaccia a schermo intero:
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
from std_msgs.msg import Empty, Bool
import rospkg
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist

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
        self.arm_up = False
        self.layout = Builder.load_file('dot_paquitop_GUI.kv')
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        profile = self.pipeline.start(config)
        align_to = rs.stream.color
        self.align = rs.align(align_to)
        #initialize topic extract table    
        tab_ext = rospy.Publisher("/extract_tablet", Bool, queue_size=1)
        tab_ret = rospy.Publisher("/retrain_tablet", Bool, queue_size=1)
        tab_ext_msg = Bool()
        tab_ext_msg.data = False
        tab_ext.publish(tab_ext_msg)
        tab_ret.publish(tab_ext_msg)
        self.path_counter = 0
        self.paziente = -1
        self.sacca = -1
        self.last = -1
        self.seat1 = False
        self.seat2 = False
        global PAQUITOP_STOP
        PAQUITOP_STOP = False
        
        

        
    def build(self):
        
        self.image = Image(pos_hint={"center_x": .775, "center_y":0.45},size_hint=(.4,.5),keep_ratio=True)
        self.layout.add_widget(self.image)
        Clock.schedule_interval(self.load_video,1.0/10.0)
                 
        return self.layout

    def load_video(self, *args):
        # Paquitop Movement controll
        rospy.Subscriber("/cmd_vel", Twist, is_in_movement)
        # Load the aruco dict
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

                self.markerID = markerID
                self.identificationOK()
            
            global PAQUITOP_STOP    
            
            if  PAQUITOP_STOP:
                
                if (self.markerID == 0 or self.markerID == 1) and not self.seat1 and not self.arm_up:
                    self.seat1 = True
                    self.goUP()

                if (self.markerID == 2 or self.markerID == 3) and not self.seat2 and not self.arm_up:
                    self.seat2 = True
                    self.goUP()
            
        frame = cv2.resize(images, None, fx=1.0, fy=1.0, interpolation=cv2.INTER_AREA)

        buffer = cv2.flip(frame, 0).tobytes()
        texture = Texture.create(size=(frame.shape[1], frame.shape[0]), colorfmt='bgr')
        texture.blit_buffer(buffer, colorfmt = 'bgr', bufferfmt = 'ubyte')
        self.image.texture = texture

    def identificationOK(self, *args):
        if self.arm_up:
            self.paziente = self.markerID
        elif not self.arm_up:
            self.sacca = self.markerID

        if self.sacca == 0 and self.paziente == 1:
            nome = "Lorenzo"
            self.layout.ids.identification.md_bg_color = (20/255,180/255,10/255,.6)
            self.layout.ids.identification.text = "Hi " + nome + "!"

        elif self.sacca == 2 and self.paziente == 3:
            nome = "Luigi"
            self.layout.ids.identification.md_bg_color = (20/255,180/255,10/255,.6)
            self.layout.ids.identification.text = "Hi " + nome + "!"
        elif self.sacca == 4 and self.paziente == 5:
            nome = "Giovanni"
            self.layout.ids.identification.md_bg_color = (20/255,180/255,10/255,.6)
            self.layout.ids.identification.text = "Hi " + nome + "!"
        elif self.sacca == 6 and self.paziente == 7:
            nome = "Giulia"
            self.layout.ids.identification.md_bg_color = (20/255,180/255,10/255,.6)
            self.layout.ids.identification.text = "Hi " + nome + "!"
        

    def goON(self, *args):
        self.layout.ids.identification.md_bg_color = (200/255,200/255,200/255,1)
        self.layout.ids.identification.text = "Waiting for identifier"
        self.paziente = -1
        self.sacca = -1
        # Tablet store
        count = 0
        while count < 3:
            count = count +1
            retrain = rospy.Publisher("/retrain_tablet", Bool, queue_size=1)
            retrain_msg = Bool()
            retrain_msg.data = True
            retrain.publish(retrain_msg)
        # Update status
        self.arm_up = False
    
    def goUP(self, *args):
        
        count = 0
        while count < 3:
            count = count +1
            tab_ext = rospy.Publisher("/extract_tablet", Bool, queue_size=1)
            tab_ext_msg = Bool()
            tab_ext_msg.data = True
            tab_ext.publish(tab_ext_msg)
        # Update status
        self.arm_up = True    
        self.last = self.markerID
    
def is_in_movement(movePAQUITOP):
    global PAQUITOP_STOP
    
    if movePAQUITOP.linear.x < 0.05 and movePAQUITOP.linear.y < 0.05 and movePAQUITOP.angular.z <0.1:
        PAQUITOP_STOP = True
    else:
        PAQUITOP_STOP = False

if __name__ == '__main__':
    
    DOT_PAQUITOP_GUI().run()
    
