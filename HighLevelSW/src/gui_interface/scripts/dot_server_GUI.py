#!/usr/bin/env python3

import rospy
import time
import numpy
import actionlib
from nav_msgs.msg import Path
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalStatusArray, GoalID
from std_msgs.msg import Empty, Bool, String
import time
import rospkg
from kivy.lang import Builder
from kivymd.app import MDApp
from kivymd.uix.menu import MDDropdownMenu 
from kivy.uix.image import Image
import csv
from imutils.video import videostream

#from PIL import Image as PILImage
#import requests
#from io import BytesIO
from skimage import io  
from kivy.clock import Clock
import cv2
from kivy.graphics.texture import Texture
import os

def quaternion_multiply(Q0,Q1):
    """
    Multiplies two quaternions.
 
    Input
    :param Q0: A 4 element array containing the first quaternion (q01,q11,q21,q31) 
    :param Q1: A 4 element array containing the second quaternion (q02,q12,q22,q32) 
 
    Output
    :return: A 4 element array containing the final quaternion (q03,q13,q23,q33) 
    """
    # Extract the values from Q0
    w0 = Q0[0]
    x0 = Q0[1]
    y0 = Q0[2]
    z0 = Q0[3]
     
    # Extract the values from Q1
    w1 = Q1[0]
    x1 = Q1[1]
    y1 = Q1[2]
    z1 = Q1[3]
     
    # Computer the product of the two quaternions, term by term
    Q0Q1_w = -x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0
    Q0Q1_x = x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0
    Q0Q1_y = -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0
    Q0Q1_z = x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0
     
    # Create a 4 element array containing the final quaternion
    final_quaternion = numpy.array([Q0Q1_w, Q0Q1_x, Q0Q1_y, Q0Q1_z])
     
    # Return a 4 element array containing the final quaternion (q02,q12,q22,q32) 
    return final_quaternion

def save_pose(goal):
    rospack = rospkg.RosPack()
    # global folder_save
    folder= rospack.get_path('navstack_pub')
    folder = folder + "/trajectory_point/" + goal + ".txt"
 
    # rospy.init_node('check_pose')
    msg = rospy.wait_for_message('/trajectory', Path)
    value = len(msg.poses) -1
    a = msg.poses[value].pose.position.x
    b = msg.poses[value].pose.position.y
    c = msg.poses[value].pose.position.z
    d = msg.poses[value].pose.orientation.x
    e = msg.poses[value].pose.orientation.y
    f1 = msg.poses[value].pose.orientation.z
    g = msg.poses[value].pose.orientation.w
    
    f = open(folder, "a")
    
    l= str(a) + "\n" + str(b)+ "\n" + str(c) + "\n" +str(d) + "\n" + str(e) + "\n" + str(f1) + "\n" + str(g)
    l = l + "\n\n"

    f.write(str(l))
    f.close()

def clean_pose(goal):
    rospack = rospkg.RosPack()
    folder = rospack.get_path('navstack_pub')
    folder = folder + "/trajectory_point/" + goal + ".txt"

    f = open(folder,'w')
    f.close()

def home():
    input_file_path = rospkg.RosPack().get_path('follow_waypoints')+"/saved_path/pose.csv"
    output_file_path = rospkg.RosPack().get_path('follow_waypoints')+"/saved_path/pose_back.csv"
    quaternion180 = numpy.array([0, 0, 0, -1])
    data = []
    with open(input_file_path, 'r') as file:
        reader = csv.reader(file, delimiter = ',')
            
        for row in reader:
            x = float(row[3])
            y = float(row[4])
            z = float(row[5])
            w = float(row[6])
        
            row_quaternion = numpy.array([w,x,y,z])
            rotated_quat = quaternion_multiply(quaternion180,row_quaternion)

            row[3] = str(rotated_quat[1])
            row[4] = str(rotated_quat[2])
            row[5] = str(rotated_quat[3])
            row[6] = str(rotated_quat[0])

            data.append(row)

            
    with open(output_file_path, 'w') as file:
        writer = csv.writer(file, delimiter = ',')
        value = len(data)
        while value > 0:
            value = value -1
            writer.writerow(data[value])
        row = ['0','0','0','0','0','0','1']
        writer.writerow(row)

    return True

class DOT_PAQUITOP_GUI(MDApp):

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        # ROS INIT
        rospy.init_node('gui_interface')
        self.pose_pub = rospy.Publisher("/pub_pose",String, queue_size=1)
        self.extract = rospy.Publisher("/extract_tablet", Bool, queue_size=1)
        self.retrain = rospy.Publisher("/retrain_tablet", Bool, queue_size=1)
        self.reset = rospy.Publisher('/path_reset', Empty, queue_size=1)
        self.home_topic = rospy.Publisher('/start_journey', Empty, queue_size=1)
        self.cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        rospy.Subscriber("/patientHelp",Bool, self.PatientHelp)
        rospy.Subscriber("/current_bed", String, self.paquitop_bed)

        # Interface init
        ip = os.environ["ROS_IP"]
        self.url = "http://" + ip + ":8080/stream?topic=/rviz_stream/camera_rviz/Image"# "http://172.21.15.100:8080/stream?topic=/rviz_stream/camera_rviz/Image"
        self.cap = cv2.VideoCapture(self.url)
        self.screen = Builder.load_file('dot_server_GUI.kv')

        # Variable Init
        self.StartReady = False
        self.bed = ""

        places_items = [
            {
                "text": f"Letto 1",
                "viewclass": "OneLineListItem",
                "on_press": lambda place=f"Letto 1": self.screen.ids.drop_item.set_item(place),
            },
            {
                "text": f"Letto 2",
                "viewclass": "OneLineListItem",
                "on_press": lambda place=f"Letto 2": self.screen.ids.drop_item.set_item(place),
            },
            {
                "text": f"Sala Prelievi",
                "viewclass": "OneLineListItem",
                "on_press": lambda place=f"Sala Prelievi": self.screen.ids.drop_item.set_item(place),
            },
            {
                "text": f"Laboratorio",
                "viewclass": "OneLineListItem",
                "on_press": lambda place=f"Laboratorio": self.screen.ids.drop_item.set_item(place),
            }
        ]

        self.drop_item = MDDropdownMenu(
            caller=self.screen.ids.drop_item,
            items=places_items,
            width_mult=4,
        )

    def build(self):
        self.image = Image(pos_hint={"center_x": .25, "center_y":0.375},size_hint=(.4,.5),keep_ratio=True)
        self.screen.add_widget(self.image)
        Clock.schedule_interval(self.rviz_stream,1.0/20.0)
        return self.screen
    
    def rviz_stream(self, *args):
        ret, frame = self.cap.read()
        frame = cv2.resize(frame, None, fx=1.0, fy=1.0, interpolation=cv2.INTER_AREA)
        buffer = cv2.flip(frame, 0).tobytes()
        texture = Texture.create(size=(frame.shape[1], frame.shape[0]), colorfmt='bgr')
        texture.blit_buffer(buffer, colorfmt = 'bgr', bufferfmt = 'ubyte')
        self.image.texture = texture
        
    def startPAQUITOP(self,*args):
        print("Avvio PAQUITOP")
        self.screen.ids.statusFB._set_text("DEMO avviata")

        if self.StartReady:
            self.pose_pub.publish(self.goal_pose)
            input_file_path = rospkg.RosPack().get_path('follow_waypoints')+"/saved_path/pose.csv"
            f = open(input_file_path, 'w')
            f.close()
        # count = 0
        # while count < 2:
        #     count = count +1
        #     Start = Empty()
        #     publisher = rospy.Publisher('/path_ready', Empty, queue_size=1)
        #     publisher.publish(Start)
        
        
        """
        stop_pub = rospy.Publisher('/stop_arm', Bool, queque_size=1)
        message_stop = Bool()
        message_stop.data = False
        stop_pub.publish(message_stop)
        """
            
    def stopPAQUITOP(self,*args):
        print("Fermo PAQUITOP")
        self.screen.ids.statusFB._set_text("DEMO fermata")
        cancel_msg = GoalID()
        self.cancel_pub.publish(cancel_msg)

        # stop_kinova = rospy.Publisher("/stop_arm", Bool, queque_size=1)
        # message_stop = Bool()
        # message_stop.data = True
        # print(message_stop)
        # stop_pub.publish(message_stop)        

    def go2(self,goal):
        print("Naviga fino a " + goal)
        self.goal_pose = String()
        self.goal_pose.data = goal
        self.StartReady = True
        # self.pose_pub.publish(goal_pose)                

    def clear(self,goal):
        print("Pulisco tutte le pose in " + goal)
        self.screen.ids.statusFB._set_text("Pulisco pose  in " + goal)
        clean_pose(goal)
        self.screen.ids.statusFB._set_text("Pose  in " + goal + " pulite")

    def save(self,goal):
        print("Aggiungo posa a " + goal)
        self.screen.ids.statusFB._set_text("Aggiungo posa " + goal)
        save_pose(goal)
        self.screen.ids.statusFB._set_text("Posa a " + goal + " aggiunta")
        
    def VSMeasure(self,*args):
        print("Misura de parametri vitali")
        self.screen.ids.statusFB._set_text("Parametri vitali acquisiti")

    def TabletLift(self,*args):
        print("Alzo il tablet")
        self.screen.ids.statusFB._set_text("Alzo il tablet")
        extract_msg = Bool()
        extract_msg.data = True
        self.extract.publish(extract_msg)

    def TabletStore(self,*args):
        print("Ripongo il tablet")
        self.screen.ids.statusFB._set_text("Ripongo il tablet")
        retrain_msg = Bool()
        retrain_msg.data = True
        self.retrain.publish(retrain_msg)

    def ResetPath(self,*args):
        print("Reset path")
        self.screen.ids.statusFB._set_text("Reset path")
        Reset = Empty()
        self.reset.publish(Reset)
    
    def home(self, *args):
        print("Return Home")
        self.screen.ids.statusFB._set_text("Return Home")
        wait = False
        wait = home()
        if wait:
            Start = Empty()
            self.home_topic.publish(Start)
    
    def PatientHelp(self, data):
        if data.data:
            self.screen.ids.patientFB.text = "Assistenza al " + self.bed
            self.screen.ids.patientFB.md_bg_color = (220/255,20/255,60/255,.6)
    
    def RestoreFB(self, *args):
        self.screen.ids.patientFB.text = "Patient Feedback"
        self.screen.ids.patientFB.md_bg_color = (20/255,180/255,10/255,.6)

    def paquitop_bed(self, data):
        self.bed = data.data


if __name__ == '__main__':
    
    DOT_PAQUITOP_GUI().run()

