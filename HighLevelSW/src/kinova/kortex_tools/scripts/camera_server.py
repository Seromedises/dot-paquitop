#!/usr/bin/env python

import cv2
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
import math
from kortex_tools.srv import SaveData, SaveDataResponse
import rospy
import rospkg

# from imutils.video import VideoStream
# import argparse
# import imutils
# import time
# import sys
# from numpy.core.defchararray import center
# from numpy.core.fromnumeric import shape
# from numpy.lib.type_check import imag

print('Press ESC or Q to stop video streaming\n')

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

def coordinate_and_normal(origin,X,Y,intrd,depth):
    # function that transform three pixel in point and calculate the orientation
    # evaluate depth of center
    pix_x, pix_y = origin
    dist_c, _, _, _ = cv2.mean(depth[pix_y-1:pix_y+1,pix_x-1:pix_x+1].astype(float))
    Centvec = convert_depth_pixel_to_metric_coordinate(dist_c,origin[0],origin[1],intrd)

    # evaluate depth of x axis
    pix_x, pix_y = X
    dist_x, _, _, _ = cv2.mean(depth[pix_y-1:pix_y+1,pix_x-1:pix_x+1].astype(float))
    Xvec = convert_depth_pixel_to_metric_coordinate(dist_x,X[0],X[1],intrd)
    Xvec = np.array(Xvec) - np.array(Centvec)
    # evaluate depth of y axis
    pix_x, pix_y = Y
    dist_y, _, _, _ = cv2.mean(depth[pix_y-1:pix_y+1,pix_x-1:pix_x+1].astype(float))
    Yvec = convert_depth_pixel_to_metric_coordinate(dist_y,Y[0],Y[1],intrd)
    Yvec = np.array(Yvec) - np.array(Centvec)

    np.seterr(divide='ignore', invalid='ignore')
    Zvec = np.cross(Xvec,Yvec)
    magnitude = np.linalg.norm(Zvec) 
    Znorm = Zvec/magnitude

    theta = [0,0,0]
    theta[0] = math.acos(np.dot(Znorm,[1,0,0]))*180/math.pi
    theta[1] = math.acos(np.dot(Znorm,[0,1,0]))*180/math.pi
    theta[2] = math.acos(np.dot(Znorm,[0,0,1]))*180/math.pi

    return Xvec,Yvec,Zvec,Centvec

def CameraToPoint(Xvec,Yvec,Zvec,OrigVec):

    magnitude = np.linalg.norm(Xvec) 
    Xvec = Xvec/magnitude
    magnitude = np.linalg.norm(Yvec) 
    Yvec = Yvec/magnitude
    magnitude = np.linalg.norm(Zvec) 
    Zvec = Zvec/magnitude

    X = np.array([1,0,0])
    Y = np.array([0,1,0])
    Z = np.array([0,0,1])

    r11 = np.dot(Xvec,X)
    r12 = np.dot(Yvec,X)
    r13 = np.dot(Zvec,X)
    r14 = OrigVec[0]

    r21 = np.dot(Xvec,Y)
    r22 = np.dot(Yvec,Y)
    r23 = np.dot(Zvec,Y)
    r24 = OrigVec[1]

    r31 = np.dot(Xvec,Z)
    r32 = np.dot(Yvec,Z)
    r33 = np.dot(Zvec,Z)
    r34 = OrigVec[2]

    T = [[r11,r12,r13,r14],[r21,r22,r23,r24],[r31,r32,r33,r34],[0,0,0,1]]
    T = np.asanyarray(T)
    return T

def save_data(req):
    if req.Save:
        print("\nPose Matrix of point respect the camera")
        print(str(T))
        save_file(T)
        return SaveDataResponse(1)
    else:
        return SaveDataResponse(0)

def save_file(Matrix):
    Row = Matrix.shape[0]
    Col = Matrix.shape[1]

    i = 0
    j = 0

    while i<Row:
        line = ""
        while j<Col:
            line = line+str(Matrix[i][j])+" "
            j = j+1
        file_path = rospkg.RosPack().get_path('kortex_tools')+"/calibration_tools/Camera_fb.txt"
        f = open(file_path, "a")
        f.write(line)
        f.write("\n")
        i = i+1
        j = 0


def main():
    global f
    file_path = rospkg.RosPack().get_path('kortex_tools')+"/calibration_tools/Camera_fb.txt"
    f = open(file_path, "w")
    rospy.init_node('camera_data_save')
    rospy.Service("/my_gen3_lite/camera_server", SaveData, save_data)
    default = cv2.aruco.DICT_5X5_100
    default_string = "DICT_5X5_100"
    
    # define names of each possible ArUco tag OpenCV supports
    ARUCO_DICT = {
            "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
            "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
            "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
            "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
            "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
            "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
            "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
            "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
            "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
            "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
            "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
            "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
            "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
            "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
            "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
            "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
            "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
        #	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
        #	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
        #	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
        #	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
    }

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

    try:
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
            
            for markerCorner in corners:
                # extract the marker corners (which are always returned
                # in top-left, top-right, bottom-right, and bottom-left
                # order)
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners

                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                # draw the Axes referring to the boundaries of box of the ArUCo detection
                cv2.line(images, topLeft, topRight, (255, 0, 0), 2)
                cv2.line(images, bottomLeft, topLeft, (0, 255, 0), 2)
                cv2.putText(images, str("X"), (bottomLeft[0]+5, bottomLeft[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(images, str("Y"), (topRight[0]-5, topRight[1]+15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                cv2.circle(images, topLeft, 4, (0, 0, 255), -1)
                
                # build pixel vector
                origin = np.array(topLeft)
                X = np.array(bottomLeft)
                Y = np.array(topRight)

                Xvec,Yvec,Zvec,OrigVec = coordinate_and_normal(origin,X,Y,intrd,depth)
                
                global T
                T = CameraToPoint(Xvec,Yvec,Zvec,OrigVec)
                # print("Matrice Trasformazione Camera-punto")
                # print(str(T))
                # possible_theta = [0, 5, 10, 15, 20, 25 ,30]print("\n")
                """
                print("\n"+str(Znorm))  
                zpoint = rs.rs2_project_point_to_pixel(intrd,(Znorm[0],Znorm[1],Znorm[2]))
                print("\n"+str(zpoint))
                zpoint = zpoint + origin
                zpoint = (int(zpoint[0]), int(zpoint[1]))
                cv2.line(images, topLeft, zpoint, (255, 255, 0), 2)
                cv2.putText(images, str("Z"), (bottomLeft[0], bottomLeft[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                """

                # compute and draw the center (x, y)-coordinates of the
                # ArUco marker
                

                # draw the ArUco marker ID on the frame
                # cv2.putText(images, str(markerID), (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            cv2.namedWindow('L515 Video')#, cv2.WINDOW_AUTOSIZE)
            cv2.imshow('L515 Video', images)

            
                    
            key = cv2.waitKey(1)
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break
    finally:
        pipeline.stop()
if __name__ == '__main__':
    main()
