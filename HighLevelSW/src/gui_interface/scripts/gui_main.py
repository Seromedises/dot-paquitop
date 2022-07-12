#!/usr/bin/env python

import re
import rospy
import numpy as np
from std_msgs.msg import Empty, Bool, Int64, String
import rospkg
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
from move_base_msgs.msg import MoveBaseActionResult
from gui_interface.msg import patient_assistance

def pub_pose(data):
    global ALL_POINT_PUBLISHED
    global published_pose
    goal = data.data
    
    NOT_YET_PUBLISHED = True
    for element in published_pose:
        if element == goal:
            NOT_YET_PUBLISHED = False
        
    if NOT_YET_PUBLISHED and not ALL_POINT_PUBLISHED:
        published_pose.append(goal)
        rospack = rospkg.RosPack()
        folder = rospack.get_path('navstack_pub')
        folder = folder + "/trajectory_point/" + goal + ".txt"
        f = open(folder,'r')

        publisher = rospy.Publisher("/addpose", PoseWithCovarianceStamped, queue_size=10)
        rate = rospy.Rate(0.5) 
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.pose.covariance = np.zeros(36)

        values = np.zeros(7)
        end = False
        while not rospy.is_shutdown() and not end: 		
            count = 0
            while count < 7 and not end:
                line = f.readline()
                if line:
                    values[count] = round(float(line.strip()),5)
                    count = count+1
                else:
                    end = True
            
            line = f.readline()

            pose.pose.pose.position.x = values[0]
            pose.pose.pose.position.y = values[1]
            pose.pose.pose.position.z = values[2]
            pose.pose.pose.orientation.x = values[3]
            pose.pose.pose.orientation.y = values[4]
            pose.pose.pose.orientation.z = values[5]
            pose.pose.pose.orientation.w = values[6]  
            rate.sleep()
            publisher.publish(pose)

        ALL_POINT_PUBLISHED = True
        

def start(data):
    global ALL_POINT_PUBLISHED
    global ARM_UP
    if data.data and ALL_POINT_PUBLISHED and not ARM_UP:
        ALL_POINT_PUBLISHED = False
        count = 0
        while count < 3:
            cout = count +1
            Start = Empty()
            publisher = rospy.Publisher('/path_ready', Empty, queue_size=1)
            publisher.publish(Start)

def move_base_goal_reached(data):
    global GOAL_REACHED
    
    if data.status.status == 3:
        GOAL_REACHED = True
    else:
        GOAL_REACHED = False

def goUP():
        global ARM_UP
        count = 0
        while count < 3:
            count = count +1
            tab_ext = rospy.Publisher("/extract_tablet", Bool, queue_size=1)
            tab_ext_msg = Bool()
            tab_ext_msg.data = True
            tab_ext.publish(tab_ext_msg)
        # Update status
        ARM_UP = True    
        
def goON():
    global ARM_UP
    # Tablet store
    count = 0
    while count < 3:
        count = count +1
        retrain = rospy.Publisher("/retrain_tablet", Bool, queue_size=1)
        retrain_msg = Bool()
        retrain_msg.data = True
        retrain.publish(retrain_msg)
    # Update status
    ARM_UP = False

def patient_data(data):
    global assistance_patient
    global temp_patient
    
    assistance_patient.append(data.need_help)  
    temp_patient.append(data.temperature)
    goON()


if __name__ == '__main__':
    rospy.init_node("gui_main")
    # intialazing the patient list
    patient_list = ["Letto 1", "Letto 2"]
    # initialazing the path from the last bed to home
    last_bed_to_home = "Laboratorio"
    # number of patients
    num_el = len(patient_list)

    # database variables
    blood_bag = []
    person_id = []
    name = []
    match_id = []
    global temp_patient
    temp_patient = []
    global assistance_patient
    assistance_patient = []
    
    DataName = ["Lorenzo", "Luigi", "Giovanni", "Giulia"]

    # definition of global variables
    
    global GOAL_REACHED
    GOAL_REACHED = False
    global ARM_UP
    ARM_UP = False
    global ALL_POINT_PUBLISHED
    ALL_POINT_PUBLISHED = False   

    # nodes to subscribe to
    rospy.Subscriber("/move_base/result", MoveBaseActionResult, move_base_goal_reached)
    rospy.Subscriber("/patient_data", patient_assistance, patient_data )
    rospy.Subscriber("/tablet_stored", Bool, start)
    rospy.Subscriber("/pub_pose",String,pub_pose)
    patient_name = rospy.Publisher("/patient_name", String, queue_size=1)
    orient_gui = rospy.Publisher("")
    patient_name_msg = String()
    
    

    while not rospy.is_shutdown():
        count = 0
        global published_pose
        published_pose = []
        while count < num_el and not rospy.is_shutdown():
            next_goal = String()
            if count != num_el:
                next_goal.data = patient_list[count+1]
            else:
                next_goal.data = last_bed_to_home
            
            
            while not ARM_UP and not rospy.is_shutdown():
                print("Waiting for Goal Reached")
                if GOAL_REACHED:
                    pub_pose(next_goal)
                    print("Waiting for blood id bag")
                    id_bag = rospy.wait_for_message("/id", Int64 )
                    blood_bag.append(float(id_bag.data))
                        
                    if blood_bag[count] == 0 or float(blood_bag[count])%2 == 0:
                        print("Going Up")
                        goUP()
                else:
                    time.sleep(0.5)
            
            wait = rospy.wait_for_message("/tablet_extracted", Bool)
            # From this time the tablet orienting procedure has started
            print("Tablet extracted, waiting for person id")
            id_patient = rospy.wait_for_message("/id", Int64 )
            person_id.append(id_patient.data)

            if person_id[count] == 1:
                name.append(DataName[0])
                patient_name_msg.data = DataName[0]
                patient_name.publish(patient_name_msg)
            elif person_id[count] == 3:
                name.append(DataName[1])
                patient_name_msg.data = DataName[1]
                patient_name.publish(patient_name_msg)
            elif person_id[count] == 5:
                name.append(DataName[2])
                patient_name_msg.data = DataName[2]
                patient_name.publish(patient_name_msg)
            elif person_id[count] == 7:
                name.append(DataName[3])
                patient_name_msg.data = DataName[3]
                patient_name.publish(patient_name_msg)
            else: 
                name.append("None")

            if person_id[count] == blood_bag[count]+1:
                match_id.append(True)
            else:
                match_id.append(False)

            
                
            
            
            

