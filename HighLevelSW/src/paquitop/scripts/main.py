#!/usr/bin/env python

import csv
import os
import time
import numpy as np

import rospy
import rospkg
from std_msgs.msg import Empty, Bool, Int64, String
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseActionResult
from gui_interface.msg import patient_assistance

class PAQUITOP_MAIN:
    def __init__(self):
        self.patient_list = ["Letto 1", "Letto 2"]
        # initialazing the path from the last bed to home
        self.last_bed_to_home = "Laboratorio"
        # number of patients
        self.num_el = len(self.patient_list)

        # database variables
        self.blood_bag = []
        self.person_id = []
        self.name = []
        self.match_id = []
        self.temp_patient = []
        self.assistance_patient = []
        self.published_pose = []
        self.DataName = ["Lorenzo", "Luigi", "Giovanni", "Giulia"]

        # definition of global variables
        self.GOAL_REACHED = False
        self.ARM_UP = False
        self.ALL_POINT_PUBLISHED = False 
        

        # topics to subscribe to
        rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.move_base_goal_reached)
        # rospy.Subscriber("/patient_data", patient_assistance, self.patient_data )
        #rospy.Subscriber("/tablet_stored", Bool, self.start)
        rospy.Subscriber("/pub_pose",String, self.pub_pose)

        # output topics
        self.patient_name = rospy.Publisher("/patient_name", String, queue_size=1)
        self.orient_gui = rospy.Publisher("/orient_gui", Bool,queue_size=10)
        self.retrain = rospy.Publisher("/retrain_tablet", Bool, queue_size=1)
        self.pose_publisher = rospy.Publisher("/addpose", PoseWithCovarianceStamped, queue_size=10)
        self.path_ready_publisher = rospy.Publisher('/path_ready', Empty, queue_size=1)
        self.tab_ext = rospy.Publisher("/extract_tablet", Bool, queue_size=1)

        
        
        
    def pub_pose(self, data):
        
        goal = data.data
        
        self.NOT_YET_PUBLISHED = True
        for element in self.published_pose:
            if element == goal:
                self.NOT_YET_PUBLISHED = False
            
        if self.NOT_YET_PUBLISHED and not self.ALL_POINT_PUBLISHED:
            self.published_pose.append(goal)
            rospack = rospkg.RosPack()
            folder = rospack.get_path('navstack_pub')
            folder = folder + "/trajectory_point/" + goal + ".txt"
            f = open(folder,'r')

            
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
                self.pose_publisher.publish(pose)

            self.ALL_POINT_PUBLISHED = True
            
    def start(self):#, data):
        self.GOAL_REACHED = False
        if self.ALL_POINT_PUBLISHED and not self.ARM_UP:
            self.ALL_POINT_PUBLISHED = False
            count = 0
            while count < 3:
                count = count +1
                Start = Empty()
                self.path_ready_publisher.publish(Start)

    def move_base_goal_reached(self, data):
        
        if data.status.status == 3:
            self.GOAL_REACHED = True
        else:
            self.GOAL_REACHED = False

    def goUP(self):
            
        count = 0
        while count < 3:
            count = count +1
            tab_ext_msg = Bool()
            tab_ext_msg.data = True
            self.tab_ext.publish(tab_ext_msg)
        # Update status
        self.ARM_UP = True    

    def patient_data(self, data):
              
        self.assistance_patient.append(data.need_help)  
        self.temp_patient.append(data.temperature)

        # orient_gui_msg = Bool()
        # orient_gui_msg.data = False
        # self.orient_gui.publish(orient_gui_msg)
        

    def goON(self):
        # wait = rospy.wait_for_message("/orient_gui", Bool)
        time.sleep(0.5)
        # Tablet store
        retrain_msg = Bool()
        retrain_msg.data = True
        self.retrain.publish(retrain_msg)
        # Update status
        self.ARM_UP = False

    def main(self):
        count = 0
        
        while not rospy.is_shutdown():

            self.pose_goal = rospy.wait_for_message("/pub_pose",String)
            self.pub_pose(self.pose_goal)

            while count < self.num_el and not rospy.is_shutdown():
                next_goal = String()
                if count != self.num_el-1:
                    next_goal.data = self.patient_list[count+1]
                else:
                    next_goal.data = self.last_bed_to_home
                
                
                while not self.ARM_UP and not rospy.is_shutdown():
                    
                    if self.GOAL_REACHED:
                        self.pub_pose(next_goal)
                        print("Waiting for blood id bag")
                                            
                        id_bag = rospy.wait_for_message("/id", Int64 )

                        if float(id_bag.data) % 2 == 0:
                            ALREADY_RECIVED_BAG = False
                            for recived_bag in self.blood_bag:
                                if recived_bag == id_bag.data:
                                    ALREADY_RECIVED_BAG == True
                            
                            if not ALREADY_RECIVED_BAG:
                                self.blood_bag.append(id_bag.data)
                                print("Going Up")
                                self.goUP()
                        
                    else:
                        print("Waiting for Goal Reached")
                        time.sleep(0.5)
                
                wait = rospy.wait_for_message("/tablet_extracted", Bool)

                # Start gui orienting 
                orient_gui_msg = Bool()
                orient_gui_msg.data = True
                self.orient_gui.publish(orient_gui_msg)
                # From this time the tablet orienting procedure has started
                print("Tablet extracted, waiting for person id")

                person_id_recieved = False

                while not person_id_recieved and not rospy.is_shutdown():
                    id_patient = rospy.wait_for_message("/id", Int64 )

                    already_person_id_recived = False
                    for patient in self.person_id:
                        if patient == id_patient:
                            already_person_id_recived = True

                    if not already_person_id_recived:
                        self.person_id.append(id_patient.data)
                        person_id_recieved = True
                        patient_name_msg = String()
                        print("new person added")

                        if self.person_id[count] == 1:
                            self.name.append(self.DataName[0])
                            patient_name_msg.data = self.DataName[0]
                            self.patient_name.publish(patient_name_msg)
                        elif self.person_id[count] == 3:
                            self.name.append(self.DataName[1])
                            patient_name_msg.data = self.DataName[1]
                            self.patient_name.publish(patient_name_msg)
                        elif self.person_id[count] == 5:
                            self.name.append(self.DataName[2])
                            patient_name_msg.data = self.DataName[2]
                            self.patient_name.publish(patient_name_msg)
                        elif self.person_id[count] == 7:
                            self.name.append(self.DataName[3])
                            patient_name_msg.data = self.DataName[3]
                            self.patient_name.publish(patient_name_msg)
                        else: 
                            self.name.append("None")

                        if self.person_id[count] == self.blood_bag[count]+1:
                            self.match_id.append(True)
                            print("bag and person matched")
                        else:
                            self.match_id.append(False)

                p_data = rospy.wait_for_message("/patient_data", patient_assistance)
                self.patient_data(p_data)
                self.goON()
                rospy.wait_for_message("/tablet_stored", Bool)
                self.start()                   
                
                count = count +1
                print("end of cycle")
                
                
            
            first_line = ["Letto", "Nome", "Id Paziente", "Id Sacca", "Match", "Temp Paziente", "Assistenza"]
            # folder = rospkg.RosPack().get_path('paquitop')
            # folder = folder + '/../../../result/patient_result.csv'
            folder = os.path.normpath(os.path.expanduser("~/Desktop"))
            folder = folder + "/result/patient_result.csv"
            f = open(folder,'w')
            writer = csv.writer(f)
            writer.writerow(first_line)

            i = 0
            while i<count:
                line = [self.patient_list[i], self.name[i], self.person_id[i], self.blood_bag[i], self.match_id[i], self.temp_patient[i], self.assistance_patient[i] ]
                writer.writerow(line)
                i = i+1
            f.close()

if __name__ == '__main__':
    
    rospy.init_node("paquitop_main")

    paquitop = PAQUITOP_MAIN()
    paquitop.main()
    rospy.spin()