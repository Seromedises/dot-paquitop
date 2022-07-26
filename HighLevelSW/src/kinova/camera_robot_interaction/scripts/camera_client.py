#!/usr/bin/env python3

import cv2
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
import math
from camera_robot_interaction.srv import *
import rospy

# function that convert camera info in in metric info
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

# funzione che riconosce il click del mouse
# Funzione che non mi serve
def click_event(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        global pix_x  
        global pix_y 
        global lclick
        pix_x = x
        pix_y = y
        lclick = True

    if event == cv2.EVENT_RBUTTONDOWN:
        global rclick
        rclick = True

#funzione che calcola la normale ad un punto
#Funzione che non mi serve
def MatrixCAM(pixel_x, pixel_y, original_depth, intrd):
    '''
    funzione che calcola le normale al piano tramite l'acquisizione di tre
    punti. I punti da cui si calcolano i vettori sono scelti appositamente 
    non uscire fuori dal matrice di punti che e' stata acquisita
    '''
    pix_space = 50
    low = pix_space -1 #49
    top = pix_space +1 #51
    
    dep = original_depth[pixel_y-1:pixel_y+1, pixel_x-1:pixel_x+1].astype(float)
    dist, _, _, _ = cv2.mean(dep)
    Center = np.array(convert_depth_pixel_to_metric_coordinate(dist, pixel_x, pixel_y, intrd))
    dep = original_depth[pixel_y-1:pixel_y+1, pixel_x-top:pixel_x-low].astype(float)
    dist, _, _, _ = cv2.mean(dep)
    Sx = np.array(convert_depth_pixel_to_metric_coordinate(dist, pixel_x-pix_space, pixel_y, intrd))
    dep = original_depth[pixel_y-top:pixel_y-low, pixel_x-1:pixel_x+1].astype(float)
    dist, _, _, _ = cv2.mean(dep)
    Low = np.array(convert_depth_pixel_to_metric_coordinate(dist, pixel_x, pixel_y-pix_space, intrd))
    Xvector = Sx - Center
    Yvector = Low - Center
   
    # ricorda: vettore tra due punti e' finale-iniziale
    # vengono calcolati i 4 possibili sistemi di riferimento in modo da evitare
    # che i vettori usati per il cacolo cadano fuori dal campo di calcolo 
    

    """
    if pix_y<= Row/2 and pix_x<= Col/2:
        # quadrante in alto a sx
        dep = original_depth[pixel_y-1:pixel_y+1, pixel_x+low:pixel_x+top].astype(float)
        dist, _, _, _ = cv2.mean(dep)
        Dx = np.array(convert_depth_pixel_to_metric_coordinate(dist, pixel_x+pix_space, pixel_y, intrd))

        dep = original_depth[pixel_y-top:pixel_y-low, pixel_x-1:pixel_x+1].astype(float)
        dist, _, _, _ = cv2.mean(dep)
        Low = np.array(convert_depth_pixel_to_metric_coordinate(dist, pixel_x, pixel_y-pix_space, intrd))

        Xvector = Dx - Center
        Yvector = Low - Center

    elif pix_y<= Row/2 and pix_x>= Col/2:
        # quadrante in alto a dx
        dep = original_depth[pixel_y-1:pixel_y+1, pixel_x-top:pixel_x-low].astype(float)
        dist, _, _, _ = cv2.mean(dep)
        Sx = np.array(convert_depth_pixel_to_metric_coordinate(dist, pixel_x-pix_space, pixel_y, intrd))

        dep = original_depth[pixel_y-top:pixel_y-low, pixel_x-1:pixel_x+1].astype(float)
        dist, _, _, _ = cv2.mean(dep)
        Low = np.array(convert_depth_pixel_to_metric_coordinate(dist, pixel_x, pixel_y-pix_space, intrd))

        Xvector = Low - Center
        Yvector = Sx - Center

    elif pix_y>= Row/2 and pix_x>= Col/2:
        # quadrante in basso a dx
        dep = original_depth[pixel_y-1:pixel_y+1, pixel_x-top:pixel_x-low].astype(float)
        dist, _, _, _ = cv2.mean(dep)
        Sx = np.array(convert_depth_pixel_to_metric_coordinate(dist, pixel_x-pix_space, pixel_y, intrd))

        dep = original_depth[pixel_y+low:pixel_y+top, pixel_x-1:pixel_x+1].astype(float)
        dist, _, _, _ = cv2.mean(dep)
        Top = np.array(convert_depth_pixel_to_metric_coordinate(dist, pixel_x, pixel_y+pix_space, intrd))

        Xvector = Sx - Center
        Yvector = Top - Center

    elif pix_y>= Row/2 and pix_x<= Col/2:
        # quadrante in basso a sx
        dep = original_depth[pixel_y-1:pixel_y+1, pixel_x+low:pixel_x+top].astype(float)
        dist, _, _, _ = cv2.mean(dep)
        Dx = np.array(convert_depth_pixel_to_metric_coordinate(dist, pixel_x+pix_space, pixel_y, intrd))

        dep = original_depth[pixel_y+low:pixel_y+top, pixel_x-1:pixel_x+1].astype(float)
        dist, _, _, _ = cv2.mean(dep)
        Top = np.array(convert_depth_pixel_to_metric_coordinate(dist, pixel_x, pixel_y+pix_space, intrd))

        Xvector = Top - Center
        Yvector = Dx - Center
    """
    # calculating the Z vector entering in the plane
    # removing error for dividing for zero
    np.seterr(divide='ignore', invalid='ignore')
    Zvector = np.cross(Xvector,Yvector)
    
    OrigVec = Center

    magnitude = np.linalg.norm(Xvector) 
    Xvec = Xvector/magnitude
    magnitude = np.linalg.norm(Yvector) 
    Yvec = Yvector/magnitude
    magnitude = np.linalg.norm(Zvector) 
    Zvec = Zvector/magnitude

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

class Quaternion():
	def __init__(self,w,x,y,z):
		self.w = w
		self.x = x
		self.y = y
		self.z = z

def MatrixToQuaternion(Mtx):
    r11 = Mtx[0][0]
    r22 = Mtx[1][1]
    r33 = Mtx[2][2]

    r32 = Mtx[2][1]
    r23 = Mtx[1][2]

    r13 = Mtx[0][2]
    r31 = Mtx[2][0]

    r21 = Mtx[1][0]
    r12 = Mtx[0][1]

    if 1+r11+r22+r33<10**(-32):
        w = 0
    else:
        w = 0.5*math.sqrt(1+r11+r22+r33)


    if 1+r11-r22-r33>0:
        x = 0.5*np.sign(r32-r23)*math.sqrt(1+r11-r22-r33)
    elif w != 0:
        x = (r32-r23)/(4*w)

    if 1-r11+r22-r33>=0:
        y = 0.5*np.sign(r13-r31)*math.sqrt(1-r11+r22-r33)
    elif w != 0:
        y = (r13-r31)/(4*w)

    if 1-r11-r22+r33>=0:
        z = 0.5*np.sign(r21-r12)*math.sqrt(1-r11-r22+r33)
    elif w != 0:
        z = (r21-r12)/(4*w)

    return Quaternion(w,x,y,z)


def main():
 
    rospy.init_node('camera_data_save')
    # rospy.Service("/my_gen3_lite/camera_server", SaveData, save_data)
    
    ## L515 Video Building
    # Create a pipeline
    pipeline = rs.pipeline()

    # Create a config and configure the pipeline to stream
    # different resolutions of color and depth streams
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

    gripper = 1
    global lclick
    global rclick
    lclick = False
    rclick = False

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
            depth = depth_image * depth_scale
            images = color_image 

            T = np.identity(4)

            cv2.namedWindow('L515 Video')#, cv2.WINDOW_AUTOSIZE)
            cv2.imshow('L515 Video', images)
            cv2.setMouseCallback('L515 Video', click_event)


            if lclick:
                T = MatrixCAM(pix_x,pix_y,depth,intrd)
               
                print("Object pose Matrix:")
                print(T)
                
            
            if rclick:
                if gripper < 0.7:
                    gripper = 1
                else:
                    gripper = 0

            if rclick or lclick:
                rospy.wait_for_service('/my_gen3_lite/robot_mover')
                try:
                    robot_mover = rospy.ServiceProxy('/my_gen3_lite/robot_mover', Movement)
                    
                    resp1 = robot_mover(T[0,0],T[0,1],T[0,2],T[0,3],T[1,0],T[1,1],T[1,2],T[1,3],T[2,0],T[2,1],T[2,2],T[2,3],gripper,0,0)
                    gripper = resp1.gripper_fb

                    if resp1.Success:
                        print("Point passed succesfully")

                except rospy.ServiceException as e:
                    print("Service call failed: %s"%e)
            
                rclick = False
                lclick = False  

            key = cv2.waitKey(1)
            if key & 0xFF == ord('r'):
                rospy.wait_for_service('/my_gen3_lite/robot_mover')
                try:
                    robot_mover = rospy.ServiceProxy('/my_gen3_lite/robot_mover', Movement)
                    T = np.identity(4)
                    rest = 1
                    resp1 = robot_mover(T[0,0],T[0,1],T[0,2],T[0,3],T[1,0],T[1,1],T[1,2],T[1,3],T[2,0],T[2,1],T[2,2],T[2,3],gripper,0,rest)
                    gripper = resp1.gripper_fb

                    if resp1.Success:
                        print("Point passed succesfully")

                except rospy.ServiceException as e:
                    print("Service call failed: %s"%e)
                    
            
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                rospy.wait_for_service('/my_gen3_lite/robot_mover')
                try:
                    robot_mover = rospy.ServiceProxy('/my_gen3_lite/robot_mover', Movement)
                    T = np.identity(4)
                    rest = 1
                    resp1 = robot_mover(T[0,0],T[0,1],T[0,2],T[0,3],T[1,0],T[1,1],T[1,2],T[1,3],T[2,0],T[2,1],T[2,2],T[2,3],gripper,0,rest)
                    gripper = resp1.gripper_fb

                    if resp1.Success:
                        print("Point passed succesfully")

                except rospy.ServiceException as e:
                    print("Service call failed: %s"%e)
                break
    finally:
        pipeline.stop()

if __name__ == '__main__':
    main()
