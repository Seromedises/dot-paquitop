#!/usr/bin/env python3

# Per evitare problemi di risoluzione con diversi schermi:
from cgitb import text
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

Config.set('graphics', 'width', '1920')
Config.set('graphics', 'height', '1080')
Config.write()

Config.set('graphics', 'fullscreen', 1)
Config.set('graphics', 'window_state', 'maximized')
Config.write()

from kivymd.app import MDApp
from kivymd.uix.boxlayout import MDBoxLayout 
from kivymd.uix.menu import MDDropdownMenu 

import rospy

class DOT_PAQUITOP_GUI(MDApp):

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.layout = Builder.load_file('dot_paquitop_GUI.kv')

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        profile = self.pipeline.start(config)
        align_to = rs.stream.color
        self.align = rs.align(align_to)

    def build(self):
    
        self.image = Image(pos_hint={"center_x": .775, "center_y":0.45},size_hint=(.4,.5),keep_ratio=True)
        self.layout.add_widget(self.image)
        Clock.schedule_interval(self.load_video,1.0/30.0)
         
        return self.layout

    def load_video(self, *args):
        # Get frameset of color and depth
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())

        frame = color_image
        frame = cv2.resize(frame, None, fx=1.0, fy=1.0, interpolation=cv2.INTER_AREA)

        buffer = cv2.flip(frame, 0).tobytes()
        texture = Texture.create(size=(frame.shape[1], frame.shape[0]), colorfmt='bgr')
        texture.blit_buffer(buffer, colorfmt = 'bgr', bufferfmt = 'ubyte')
        self.image.texture = texture

    def identificationOK(self, *args):
        self.layout.ids.identification.md_bg_color = (20/255,180/255,10/255,.6)
        nome = "Lorenzo"
        self.layout.ids.identification.text = "Hi " + nome + "!"

    def goON(self, *args):
        self.layout.ids.identification.md_bg_color = (200/255,200/255,200/255,1)
        self.layout.ids.identification.text = "Waiting for identifier"

if __name__ == '__main__':
    rospy.init_node('paquitop_gui')
    DOT_PAQUITOP_GUI().run()
    
