#!/usr/bin/env python

#from distutils.command.build_scripts import first_line_re
from pickle import FALSE, TRUE
import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
import time
import numpy as np
from math import pi
from moveit_commander.conversions import pose_to_list
import pyrealsense2 as rs
import cv2

global first_joint_pos #at 30 cm from object
first_joint_pos = [-0.4190,-0.5174, 1.5178, 1.0017, -0.8191, 0.5066-pi/2]
global second_joint_pos #at 40 cm from object
second_joint_pos = [-0.5676, -0.2391, 1.9228, 0.9022, -0.9966, 0.5462-pi/2]
global third_joint_pos #at 50 cm from object
third_joint_pos = [-0.8342, 0.0492, 2.2100, 0.6677, -1.1528, 0.7247-pi/2]

def TransformMatrix(joint_number,joint_values):
	# Denavit-Hartenberg Modified parameters
	alpha = [0, pi/2, pi, pi/2, pi/2, -pi/2] # rad
	a = [0, 0, 0.280, 0, 0, 0] # meter
	d = [0.2433, 0, -0.01, 0.245, 0.057, 0.235] # meter
	theta = [joint_values[0], pi/2+joint_values[1], pi/2+joint_values[2], pi/2+joint_values[3], joint_values[4], joint_values[5]] #rad
	
	# matrix construction
	index = 0
	T = np.identity(4)

	while index < joint_number:

		# Submatrix that compose the Denavit-Hartenberg modified matrix
		Rot_X_alpha = [[1, 0, 0, 0],[0, math.cos(alpha[index]), -math.sin(alpha[index]), 0],[0, math.sin(alpha[index]), math.cos(alpha[index]),0],[0,0,0,1]]
		Trans_X_a = [[1,0,0,a[index]],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
		Trans_Z_d = [[1,0,0,0],[0,1,0,0],[0,0,1,d[index]],[0,0,0,1]]
		Rot_Z_theta = [[math.cos(theta[index]), -math.sin(theta[index]),0,0],[math.sin(theta[index]), math.cos(theta[index]),0,0],[0,0,1,0],[0,0,0,1]]
		
		# multiplcation between al the matrix
		B = np.matmul(Rot_X_alpha,Trans_X_a)
		B = np.matmul(B,Trans_Z_d)
		
		# Denavit-Hartenberg matrix betwenn the i-th link and the (i-1)-th link
		Tnew = np.matmul(B,Rot_Z_theta)
		
		# Upgrading matrix to build the total matrix between zero and the selected joint (defined by joint_number)
		T = np.matmul(T,Tnew)
		# print("\n\nMatrice Denavit numero "+str(index+1)+" fra base ed terminale:\n"+str(Tnew)+"\n\n")
		index = index+1

	return T

class MoveGroupPythonIntefaceTutorial(object):

    def __init__(self):

        # Initialize the node
        super(MoveGroupPythonIntefaceTutorial, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('robot_camera_interaction', )

        try:
            self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
            if self.is_gripper_present:
                gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
                self.gripper_joint_name = gripper_joint_names[0]
            else:
                gripper_joint_name = ""
            self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 6)

            # Create the MoveItInterface necessary objects
            arm_group_name = "arm"
            self.robot = moveit_commander.RobotCommander("robot_description")
            self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
            self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
            self.arm_group.set_max_velocity_scaling_factor(1)
            self.arm_group.set_max_acceleration_scaling_factor(1)
            self.arm_group.set_num_planning_attempts(50)
            self.arm_group.set_planning_time(20)
            self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=50)

            if self.is_gripper_present:
                gripper_group_name = "gripper"
                self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

            rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
        except Exception as e:
            print (e)
            self.is_init_success = False
        else:
            self.is_init_success = True
          
    def reach_cartesian_pose(self, pose, tolerance, constraints):
        arm_group = self.arm_group
        
        # Set the tolerance
        arm_group.set_goal_position_tolerance(tolerance)

        # Set the trajectory constraint if one is specified
        if constraints is not None:
            arm_group.set_path_constraints(constraints)

        # Get the current Cartesian Position
        arm_group.set_pose_target(pose)

        # Plan and execute
        rospy.loginfo("Planning and going to the Cartesian Pose")
        return arm_group.go(wait=True)

    def reach_gripper_position(self, relative_position):
        gripper_group = self.gripper_group
        
        # We only have to move this joint because all others are mimic!
        gripper_joint = self.robot.get_joint(self.gripper_joint_name)
        gripper_max_absolute_pos = gripper_joint.max_bound()
        gripper_min_absolute_pos = gripper_joint.min_bound()
        try:
            val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)
            return val
        except:
            return False
            
    def get_cartesian_pose(self):
        arm_group = self.arm_group

        # Get the current pose and display it
        pose = arm_group.get_current_pose()
        
        return pose.pose

    def get_joint(self):
        
        joint = self.arm_group.get_current_joint_values()
        joint = np.array(joint)

        return joint

    def ValidateMatrix(self,number_joint):

        # get joint
        joint = self.arm_group.get_current_joint_values()
        joint = np.array(joint)
        # get trasform matrix
        T = TransformMatrix(number_joint,joint)
        T = np.asanyarray(T)

        return T

    def rest_pos(self):
        self.arm_group.set_goal_joint_tolerance(0.01)
        joint_goal = self.arm_group.get_current_joint_values()
        joint_goal[0] = -3.0*pi/180
        joint_goal[1] = 21*pi/180
        joint_goal[2] = 145*pi/180
        joint_goal[3] = 92*pi/180 #-88+180
        joint_goal[4] = -33*pi/180
        joint_goal[5] = -90*pi/180

        self.arm_group.set_joint_value_target(joint_goal)
        return self.arm_group.go(wait=True)
    
    def reach_joint_pos(self,joint_pos):
        self.arm_group.set_goal_joint_tolerance(0.01)
        joint_goal = self.arm_group.get_current_joint_values()
        i = 0
        while i<len(joint_pos):
            joint_goal[i]=joint_pos[i]
            i = i+1 
        self.arm_group.set_joint_value_target(joint_goal)
        return self.arm_group.go(wait=True)
    
def convert_depth_pixel_to_metric_coordinate(depth, pixel_x, pixel_y, camera_intrinsics):
    """
    Convert the depth and image point information to metric coordinates
    Parameters:
    -----------
    depth 	 	 	 : double
                            The depth value of the image point
    pixel_x 	  	 	 : double
                            The x value of the image coordinate
    pixel_y 	  	 	 : double
                            The y value of the image coordinate
    camera_intrinsics : The intrinsic values of the imager in whose coordinate system the depth_frame is computed
    Return:
    ----------
    X : double
        The x value in meters
    Y : double
        The y value in meters
    Z : double
        The z value in meters
    """
    X = (pixel_x - camera_intrinsics.ppx)/camera_intrinsics.fx *depth
    Y = (pixel_y - camera_intrinsics.ppy)/camera_intrinsics.fy *depth

    return X, Y, depth

def main():
    # Data for 
    
    Cal_matr = np.loadtxt("../cw_kinova/src/kortex_tools/calibration_tools/Calib_Matrix.txt", delimiter=",")

    # Declaring variable of interest for robot
    example = MoveGroupPythonIntefaceTutorial()
	
    rospy.loginfo(example.is_init_success)
    try:
        rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
    except:
        pass

    # Declaring Variable for aruko marker
    default = cv2.aruco.DICT_5X5_100
    default_string = "DICT_5X5_100"

    # load the ArUCo dictionary and grab the ArUCo parameters
    print("[INFO] detecting "+default_string+" tags...")
    arucoDict = cv2.aruco.Dictionary_get(default)
    arucoParams = cv2.aruco.DetectorParameters_create()

    ## L515 Video Building
    # Create a pipeline
    pipeline = rs.pipeline()

    # Create a config and configure the pipeline to stream
    #  different resolutions of color and depth streams
    config = rs.config()

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    profile = pipeline.start(config)
    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_sensor = profile.get_device().first_depth_sensor()
    # inserita da me per avere il sensore in SHORT RANGE
    depth_sensor.set_option(rs.option.visual_preset, 5) # 5 = short range commentare istruzione per il long range
    depth_scale = depth_sensor.get_depth_scale()

    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align_to = rs.stream.color #con color si vede meglio, con depth si vede quello che non riesce a misurare
    align = rs.align(align_to)
    # Intrinsic caracteristic of the camera, could be rs.stream.color or rs.stream.depth but the first seems to work better
    intrd = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    
    #declare save variable
    save = FALSE
    acquisition=0
    # print variable
    X30 = np.zeros((9,10))
    X40 = np.zeros((9,10))
    X50 = np.zeros((9,10))
    Y30 = np.zeros((9,10))
    Y40 = np.zeros((9,10))
    Y50 = np.zeros((9,10))
    Z30 = np.zeros((9,10))
    Z40 = np.zeros((9,10))
    Z50 = np.zeros((9,10))

    while acquisition<10:
        #FIRST Moving Robot at 30 cm
        example.reach_joint_pos(first_joint_pos)
        feedback_robot = example.ValidateMatrix(6)
        # f = open("../cw_kinova/src/kortex_tools/calibration_tools/30cm/Mat_0-c.txt", "w")
        mat_0c = np.matmul(feedback_robot,Cal_matr)
        # f.write(mat_0c)
        np.savetxt('../cw_kinova/src/kortex_tools/calibration_tools/30cm/Mat_0-c.txt', mat_0c, delimiter=" ") 

        print('stop the camera acquisition with "Q" or "esc" when all aruco marker are detected')
        print_data = [(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0)]
        while not rospy.is_shutdown():
            
            # Get frameset of color and depth
            frames = pipeline.wait_for_frames()
            # Align the depth frame to color frame
            aligned_frames = align.process(frames)

            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
            color_frame = aligned_frames.get_color_frame()

            # Validate that both frames are valid
            if not aligned_depth_frame or not color_frame:
                continue

            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            depth = depth_image * depth_scale
            images = color_image 
            
            ## ISTRUZIONI PER RICONOSCERE ARUCO MARKERS
            # detect ArUco markers in the input frame
            (corners, ids, rejected) = cv2.aruco.detectMarkers(images, arucoDict, parameters=arucoParams)

            # images = cv2.aruco.drawDetectedMarkers(images, corners)
            
            # verify *at least* one ArUco marker was detected
            if len(corners) > 0:
            # flatten the ArUco IDs list
                ids = ids.flatten()

                # loop over the detected ArUCo corners
                # for (markerCorner, markerID) in zip(corners, ids):# (corners, ids): 
                if len(ids) == 9:
                    # control to save the center of Aruko markers
                    save = TRUE
                    # f = open("../cw_kinova/src/kortex_tools/calibration_tools/Validation_experiment30cm.txt", "w")
                    # f.write("X\t\tY\t\tZ\n")

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
                    cv2.putText(images, str(markerID),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    if save:
                        dist_c = depth[cY,cX]
                        Xprint, Yprint, Zprint = convert_depth_pixel_to_metric_coordinate(dist_c,cX,cY,intrd)

                        X30[markerID][acquisition] = Xprint
                        Y30[markerID][acquisition] = Yprint
                        Z30[markerID][acquisition] = Zprint
                        
                        print_data[markerID] = (Xprint, Yprint, Zprint) 
                
                if save:
                    print(str(print_data)+"\n\n")
                    save = 0  
                """    
                i=0
                while i<len(print_data) and len(ids) == 9:
                    
                    Xprint, Yprint, Zprint = print_data[i]
                    line =str(Xprint)+" "+str(Yprint)+" "+str(Zprint)+"\n"
                    f = open("../cw_kinova/src/kortex_tools/calibration_tools/Validation_experiment30cm.txt", "a")
                    f.write(line)
                    save = FALSE
                    i=i+1
                """  
                
            cv2.namedWindow('L515 Video')#, cv2.WINDOW_AUTOSIZE)
            cv2.imshow('L515 Video', images)
                    
            key = cv2.waitKey(1)
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break

        #SECOND Moving Robot at 40 cm
        example.reach_joint_pos(second_joint_pos)
        feedback_robot = example.ValidateMatrix(6)
        # f = open("../cw_kinova/src/kortex_tools/calibration_tools/40cm/Mat_0-c.txt", "w")
        mat_0c = np.matmul(feedback_robot,Cal_matr)
        # f.write(mat_0c)
        np.savetxt('../cw_kinova/src/kortex_tools/calibration_tools/40cm/Mat_0-c.txt', mat_0c, delimiter=" ") 
        
        print('stop the camera acquisition with "Q" or "esc" when all aruco marker are detected')
        print_data = [(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0)]
        while not rospy.is_shutdown():
            
            # Get frameset of color and depth
            frames = pipeline.wait_for_frames()
            # Align the depth frame to color frame
            aligned_frames = align.process(frames)

            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
            color_frame = aligned_frames.get_color_frame()

            # Validate that both frames are valid
            if not aligned_depth_frame or not color_frame:
                continue

            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            depth = depth_image * depth_scale
            images = color_image 
            
            ## ISTRUZIONI PER RICONOSCERE ARUCO MARKERS
            # detect ArUco markers in the input frame
            (corners, ids, rejected) = cv2.aruco.detectMarkers(images, arucoDict, parameters=arucoParams)

            # images = cv2.aruco.drawDetectedMarkers(images, corners)
            
            # verify *at least* one ArUco marker was detected
            if len(corners) > 0:
            # flatten the ArUco IDs list
                ids = ids.flatten()

                # loop over the detected ArUCo corners
                # for (markerCorner, markerID) in zip(corners, ids):# (corners, ids): 
                if len(ids) == 9:
                    # control to save the center of Aruko markers
                    save = TRUE
                    # f = open("../cw_kinova/src/kortex_tools/calibration_tools/Validation_experiment40cm.txt", "w")
                    # f.write("X\t\tY\t\tZ\n")

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
                    cv2.putText(images, str(markerID),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    if save:
                        dist_c = depth[cY,cX]
                        Xprint, Yprint, Zprint = convert_depth_pixel_to_metric_coordinate(dist_c,cX,cY,intrd)

                        X40[markerID][acquisition] = Xprint
                        Y40[markerID][acquisition] = Yprint
                        Z40[markerID][acquisition] = Zprint

                        print_data[markerID] = (Xprint, Yprint, Zprint) 
                
                if save:
                    print(str(print_data)+"\n\n")
                    save = 0  
                """  
                i=0
                while i<len(print_data) and len(ids) == 9:
                    
                    Xprint, Yprint, Zprint = print_data[i]
                    line =str(Xprint)+" "+str(Yprint)+" "+str(Zprint)+"\n"
                    f = open("../cw_kinova/src/kortex_tools/calibration_tools/Validation_experiment40cm.txt", "a")
                    f.write(line)
                    save = FALSE
                    i=i+1
                """       
            
            cv2.namedWindow('L515 Video')#, cv2.WINDOW_AUTOSIZE)
            cv2.imshow('L515 Video', images)
                    
            key = cv2.waitKey(1)
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break

        #THIRD Moving Robot at 50 cm
        example.reach_joint_pos(third_joint_pos)
        feedback_robot = example.ValidateMatrix(6)
        # f = open("../cw_kinova/src/kortex_tools/calibration_tools/50cm/Mat_0-c.txt", "w")
        mat_0c = np.matmul(feedback_robot,Cal_matr)
        # f.write(mat_0c)
        np.savetxt('../cw_kinova/src/kortex_tools/calibration_tools/50cm/Mat_0-c.txt', mat_0c, delimiter=" ") 
        
        print('stop the camera acquisition with "Q" or "esc" when all aruco marker are detected')
        print_data = [(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0),(0,0,0)]
        while not rospy.is_shutdown():
            
            # Get frameset of color and depth
            frames = pipeline.wait_for_frames()
            # Align the depth frame to color frame
            aligned_frames = align.process(frames)

            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
            color_frame = aligned_frames.get_color_frame()

            # Validate that both frames are valid
            if not aligned_depth_frame or not color_frame:
                continue

            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            depth = depth_image * depth_scale
            images = color_image 
            
            ## ISTRUZIONI PER RICONOSCERE ARUCO MARKERS
            # detect ArUco markers in the input frame
            (corners, ids, rejected) = cv2.aruco.detectMarkers(images, arucoDict, parameters=arucoParams)

            # images = cv2.aruco.drawDetectedMarkers(images, corners)
            
            # verify *at least* one ArUco marker was detected
            if len(corners) > 0:
            # flatten the ArUco IDs list
                ids = ids.flatten()

                # loop over the detected ArUCo corners
                # for (markerCorner, markerID) in zip(corners, ids):# (corners, ids): 
                if len(ids) == 9:
                    # control to save the center of Aruko markers
                    save = TRUE
                    # f = open("../cw_kinova/src/kortex_tools/calibration_tools/Validation_experiment50cm.txt", "w")
                    # f.write("X\t\tY\t\tZ\n")

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
                    cv2.putText(images, str(markerID),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    if save:
                        dist_c = depth[cY,cX]
                        Xprint, Yprint, Zprint = convert_depth_pixel_to_metric_coordinate(dist_c,cX,cY,intrd)

                        X50[markerID][acquisition] = Xprint
                        Y50[markerID][acquisition] = Yprint
                        Z50[markerID][acquisition] = Zprint

                        print_data[markerID] = (Xprint, Yprint, Zprint) 
                
                if save:
                    print(str(print_data)+"\n\n")
                    save = 0        
                """    
                i=0
                while i<len(print_data) and len(ids) == 9:
                    
                    Xprint, Yprint, Zprint = print_data[i]
                    line =str(Xprint)+" "+str(Yprint)+" "+str(Zprint)+"\n"
                    f = open("../cw_kinova/src/kortex_tools/calibration_tools/Validation_experiment50cm.txt", "a")
                    f.write(line)
                    save = FALSE
                    i=i+1
                """        
            
            cv2.namedWindow('L515 Video')#, cv2.WINDOW_AUTOSIZE)
            cv2.imshow('L515 Video', images)
                    
            key = cv2.waitKey(1)
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break

        acquisition = acquisition+1
    
    # processing the savatage

    np.savetxt('../cw_kinova/src/kortex_tools/calibration_tools/30cm/X30.txt', X30, delimiter=" ")
    np.savetxt('../cw_kinova/src/kortex_tools/calibration_tools/30cm/Y30.txt', Y30, delimiter=" ")
    np.savetxt('../cw_kinova/src/kortex_tools/calibration_tools/30cm/Z30.txt', Z30, delimiter=" ")

    np.savetxt('../cw_kinova/src/kortex_tools/calibration_tools/40cm/X40.txt', X40, delimiter=" ")
    np.savetxt('../cw_kinova/src/kortex_tools/calibration_tools/40cm/Y40.txt', Y40, delimiter=" ")
    np.savetxt('../cw_kinova/src/kortex_tools/calibration_tools/40cm/Z40.txt', Z40, delimiter=" ")

    np.savetxt('../cw_kinova/src/kortex_tools/calibration_tools/50cm/X50.txt', X50, delimiter=" ")
    np.savetxt('../cw_kinova/src/kortex_tools/calibration_tools/50cm/Y50.txt', Y50, delimiter=" ")
    np.savetxt('../cw_kinova/src/kortex_tools/calibration_tools/50cm/Z50.txt', Z50, delimiter=" ")


if __name__ == '__main__':
    main()
