from __future__ import division
import rospy
import cv2
import tf
import roslib
import numpy as np
from numpy import asarray
from mavros_msgs.msg import State, PositionTarget

from geometry_msgs.msg import PoseStamped, Point, Quaternion, TransformStamped, PoseArray
from sensor_msgs.msg import CameraInfo, RegionOfInterest, Image
from image_geometry import PinholeCameraModel
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt
import math
import time
from std_msgs.msg import String, Header
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

from angle import angle
from obstacle_avoidance import obstacle_avoidance


from sensor_msgs.msg import CameraInfo

class OffbPosCtl:
    curr_pose = PoseStamped()
    des_pose = PoseStamped()
    cam_pose = PoseStamped()
    cam_img = PoseStamped()
    x_prev_error = 0
    y_prev_error = 0
    destination_x_previous = 0
    destination_y_previous = 0
    destination_z_previous = 0
    destination_pose = PoseStamped()
    waypointIndex = 0
    detections = []
    distThreshold= 2
    detection_count=0
    depth = Image()
    depth_matrix = []
    dimg = Image()
    KP= .005
    KD = 0.0004
    des_x = 0
    des_y = 0
    des_z = 0
    camera=PinholeCameraModel()
    saved_location = None
    isReadyToFly = False
    hover_loc = [-28,16.3,10,0,0,0,0]
    mode="HOVER"
    rgb_target = []
    ray_target = []
    target = []       # a column vector that stores the x,y,z target
    #dronename = rospy.get_param('iris_cam')
    vel_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)	
    pub = rospy.Publisher('/data', String, queue_size=10)


    def __init__(self):
        rospy.init_node('offboard_test', anonymous=True)
        self.loc = []
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback= self.mocap_cb)
        rospy.Subscriber('/mavros/state',State, callback= self.state_cb)
        rospy.Subscriber('/dreams/state',String,callback=self.update_state_cb)
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, callback=self.yolo)
	rospy.Subscriber('/darknet_ros/detection_image', Image, callback=self.detected_image)
	rospy.Subscriber('/camera/depth/image_raw',Image,callback=self.depth_image)
        #self.camera.fromCameraInfo(self.rgb_info())
	# Zhiang: use this instead	
	camera_info = rospy.wait_for_message("/camera/depth/camera_info", CameraInfo)  # Zhiang: make sure this is the correct camera
	#camera_info = rospy.wait_for_message("/camera/rgb/camera_info", CameraInfo) 
	self.pinhole_camera_model = PinholeCameraModel()
	self.pinhole_camera_model.fromCameraInfo(camera_info)

	#self.listener =  tf.TransformListener()  # linked to tf_static
        self.planner() 

	
    def mocap_cb(self,msg1):
        self.curr_pose = msg1	
        #br = tf.TransformBroadcaster()
        #br.sendTransform((msg1.pose.position.x + 0, msg1.pose.position.y + 0, msg1.pose.position.z - 0.2, msg1.pose.orientation.x + 0.3825 , msg1.pose.orientation.y + 0.0003 , msg1.pose.orientation.z + 0.924 , msg1.pose.orientation.w + 0.001),rospy.Time.now(),"world", self.dronename)



    def rgb_info(self):
        msg_header = Header()
        msg_header.frame_id = "camera_link"
        msg_roi = RegionOfInterest()
        msg_roi.x_offset = 0
        msg_roi.y_offset = 0
        msg_roi.height = 0
        msg_roi.width = 0
        msg_roi.do_rectify = 0
        msg = CameraInfo()
        msg.header = msg_header
        msg.height = 480
        msg.width = 640
        msg.distortion_model = 'plumb_bob'
        msg.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.K = [1.0, 0.0, 320.5, 0.0, 1.0, 240.5, 0.0, 0.0, 1.0]
        msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.P = [1.0, 0.0, 320.5, -0.0, 0.0, 1.0, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0]
        msg.binning_x = 0
        msg.binning_y = 0
        msg.roi = msg_roi
        return msg



    def depth_image(self,Img):
	self.depth = Img
	
	#print(self.depth.encoding)
	#print(self.depth_matrix)



    def depth_eval(self):
	cx = self.rgb_target[0]
	alpha1 = 0.7
	cy = self.rgb_target[1]
	
	x_left_lim = int(cx) - 3
	x_right_lim = int(cx) + 3
	y_down_lim = int(cy) - 3
	y_up_lim = int(cy) + 3
	sum = 0
	#print(x_left_lim,x_right_lim,y_down_lim,y_up_lim)
        
	self.depth_matrix = CvBridge().imgmsg_to_cv2(self.depth,desired_encoding= 'passthrough')
	#self.depth_matrix = np.array(self.depth.data, dtype=np.uint8) 
	cpimg = self.depth_matrix.copy().astype(float)
	#print(cpimg.shape)
	#mat = cv2.convertScaleAbs(cpimg, alpha = 255/cpimg.max())
	depth = cpimg[x_left_lim:x_right_lim, y_down_lim:y_up_lim] # Truncating mat
	#print depth
        if depth.shape != 0:
	    for i in range(6):
	        for j in range(6):
	            sum = sum + depth[i][j]
 	avg_depth = sum/(49) 
        # Evaluating x value from Z :

             
        #print(avg_depth)
	#K = np.array([[1.0, 0.0, 510],[0.0, 1.0, 384],[0.0, 0.0, 1.0]])
	#K_inv = np.array([[1.0, 0, -510],[0, 1.0, -384],[0, 0, 1.0]])
	#self.target = avg_depth *np.matmul(K_inv,self.rgb_target)
        #print(self.target)
        #x = ((self.rgb_target[0][0] - 510) * avg_depth)/#focal length from the pinhole camera model?     # fx is the focal length in the pixel coordinate
	#y = ((self.rgb_target[1][0] - 384) * avg_depth)/# focal length from the pinhole camera model?     # fy is the focal length in the pixel coordinate
	#print(self.ray_target[0])
	#print(self.ray_target[1])
	#print(avg_depth)
	#print(self.ray_target)
	x = self.ray_target[0]*avg_depth
	y = self.ray_target[1]*avg_depth
	z = avg_depth
	#print("_______________________________________________")
	#print x
	#print y
	#print z
	self.target = np.array([[x],[y],[z]])
	#hom_transformation = np.array([[0, -1, 0, 0],[-0.7071, 0, 0.7071, 0],[-.7071, 0, -.7071, 0],[0, 0, 0.2, 1]])
	hom_transformation = np.array([[0, -0.707, -0.707, 0],[-1, 0, 0, 0],[ 0, 0.707, -.707, 0],[0, 0, 0, 1]])
	
        homogeneous_coordinates = np.array([[self.target[0][0]],[self.target[1][0]],[self.target[2][0]],[1]])
        #print(homogeneous_coordinates)
        product =  np.matmul(hom_transformation,homogeneous_coordinates)
	self.cam_img.pose.position.x = product[0][0]
        self.cam_img.pose.position.y = product[1][0]
        self.cam_img.pose.position.z = product[2][0]
        #print(product[0][0])
	#print(product[1][0])
	#print(product[2][0])
	#self.cam_img.pose.position.z = self.target[2]
	self.cam_img.pose.orientation.x = 0
	self.cam_img.pose.orientation.y = 0
	self.cam_img.pose.orientation.z = 0
	self.cam_img.pose.orientation.w = 1
	self.destination_pose.pose.position.x = self.cam_img.pose.position.x + self.curr_pose.pose.position.x  
        self.destination_pose.pose.position.y = self.cam_img.pose.position.y + self.curr_pose.pose.position.y 
        self.destination_pose.pose.position.z = self.cam_img.pose.position.z + self.curr_pose.pose.position.z  
	self.destination_pose.pose.orientation.x = self.curr_pose.pose.orientation.x
	self.destination_pose.pose.orientation.y = self.curr_pose.pose.orientation.y
	self.destination_pose.pose.orientation.z = self.curr_pose.pose.orientation.z
	self.destination_pose.pose.orientation.w = self.curr_pose.pose.orientation.w
	self.destination_x = ((1 - alpha1)*self.destination_pose.pose.position.x) + (alpha1 * self.destination_x_previous)
	self.destination_y = ((1 - alpha1)*self.destination_pose.pose.position.y) +(alpha1 * self.destination_y_previous)
	self.destination_z = ((1 - alpha1)*self.destination_pose.pose.position.z) +(alpha1 * self.destination_z_previous)
	self.destination_x_previous = self.destination_x
	self.destination_y_previous = self.destination_y
	self.destination_z_previous = self.destination_z
	#print(self.destination_x,self.destination_y,self.destination_z)


	
    def yolo(self,data):
	while self.curr_pose.pose.position.z > 4 and self.curr_pose.pose.position.z < 14:
		for a in data.bounding_boxes:
		    if (a.Class == "stop sign" or a.Class =="kite" or a.Class=="cell phone" or a.Class=="car" or a.Class=="truck" or a.Class=="bus"):
	    	        self.detection_count = self.detection_count + 1
			#print(self.detection_count)
		        self.rgb_target = np.array([[a.xmin + (a.xmax - a.xmin)/2], [a.ymin + (a.ymax - a.ymin)/2], [1]])
	  		self.ray_target = self.pinhole_camera_model.projectPixelTo3dRay((a.xmin + (a.xmax - a.xmin)/2,a.ymin + (a.ymax - a.ymin)/2))
		        #print(self.rgb_target)
			#print(self.rgb_target[1])
			self.depth_eval()


       
    def detected_image(self,detection):
	try:
	   self.dimg = CvBridge().imgmsg_to_cv2(detection, "bgr8")
    	except CvBridgeError as e:
	    print(e)


         
    def copy_pose(self , pose):
        pt = pose.pose.position
        quat = pose.pose.orientation
        copied_pose = PoseStamped()
        copied_pose.header.frame_id = pose.header.frame_id
        copied_pose.pose.position = Point(pt.x, pt.y, pt.z)
        copied_pose.pose.orientation = Quaternion(quat.x , quat.y , quat.z , quat.w)
        return copied_pose
  

    
    def state_cb(self,msg):
        if msg.mode == 'OFFBOARD':
            self.isReadyToFly = True
        else:
            print msg.mode



    def update_state_cb(self,data):
        self.mode= data.data



    def attach(self):
	attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
	attach_srv.wait_for_service()
	print('Trying to attach')

	req = AttachRequest()
	req.model_name_1 = "iris"
	req.link_name_1 = "base_link"
	req.model_name_2 = "suv" # Insert the model name you want to attach to
	req.link_name_2  = "link" # Insert link name you want to attach to

	attach_srv.call(req)
	print('Attached')



    def detach(self):
	attach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
	attach_srv.wait_for_service()
	print('Trying to detach')

	req = AttachRequest()
	req.model_name_1 = "iris"
	req.link_name_1 = "base_link"
	req.model_name_2 = "suv" # Insert the model name you want to attach to
	req.link_name_2  = "link" # Insert link name you want to attach to

	attach_srv.call(req)
	print('Detached')



    def hover(self):
        location = self.hover_loc
        loc = [location,
               location,
               location,
               location,
               location,
               location,
               location,
               location,
               location]
      
        rate = rospy.Rate(10)
        shape = len(loc)
        pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size= 10)
        des_pose = self.copy_pose(self.curr_pose)
        waypoint_index = 0
        sim_ctr = 1
       	
        while self.mode=="HOVER" and self.detection_count < 100 and not rospy.is_shutdown():
	    #print(self.detection_count)
            if waypoint_index==shape:
                waypoint_index = 0            # changing the way point index to 0
                sim_ctr = sim_ctr + 1
                #print "HOVER STOP COUNTER:" + str(sim_ctr)
            if self.isReadyToFly:
                des_x = loc[waypoint_index][0]
                des_y = loc[waypoint_index][1]
                des_z = loc[waypoint_index][2]
                des_pose.pose.position.x = des_x
                des_pose.pose.position.y = des_y
                des_pose.pose.position.z = des_z
                des_pose.pose.orientation.x = loc[waypoint_index][3]
                des_pose.pose.orientation.y = loc[waypoint_index][4]
                des_pose.pose.orientation.z = loc[waypoint_index][5]
                des_pose.pose.orientation.w = loc[waypoint_index][6]
                curr_x = self.curr_pose.pose.position.x
                curr_y = self.curr_pose.pose.position.y
                curr_z = self.curr_pose.pose.position.z        
                dist = math.sqrt((curr_x - des_x)*(curr_x - des_x) + (curr_y - des_y)*(curr_y - des_y) + (curr_z - des_z)*(curr_z - des_z))
                if dist<self.distThreshold :
                    waypoint_index += 1
                  
            if sim_ctr == 50:
                pass
            #print('I am here')
            pose_pub.publish(des_pose)
	#print('I am here')
	rate.sleep() 
        if self.detection_count >= 100 :
	    self.mode = "SWOOP"
	    self.descent()
	    #print(self.mode) 
	 



    def get_descent(self,x,y,z):
	des_vel = PositionTarget()
	des_vel.header.frame_id = "world"
	des_vel.header.stamp=rospy.Time.from_sec(time.time())
        des_vel.coordinate_frame= 8
	des_vel.type_mask = 3527
	des_vel.velocity.x = y
	des_vel.velocity.y = x 
	des_vel.velocity.z = z
		
	return des_vel



    def descent(self):
        rate = rospy.Rate(10)  # 10 Hz
        # find tf static section under depth_eval
        #print self.mode
        while self.mode == "SWOOP" and self.curr_pose.pose.position.z > 1 and not rospy.is_shutdown():
            #print(" In while loop")
            err_x = self.des_x - self.curr_pose.pose.position.x
            err_y = self.des_y - self.curr_pose.pose.position.y
            err_z = self.curr_pose.pose.position.z - self.des_z # self.des_z is small - close to ground.
	    #print(err_y)
            #print(err_z)
            # print(err_y)
	    
            x_change = -(err_x * self.KP * 1)#-(((err_x - self.x_prev_error) * 70) * self.KD)
            y_change = -(err_y * self.KP * 0)#-(((err_y - self.y_prev_error) * 70) * self.KD)
	    z_change = -((err_z)/(math.sqrt((err_x)**2 + (err_y)**2)) ) * self.KP*200
            des = self.get_descent(x_change, y_change, z_change)
            self.vel_pub.publish(des)
	    self.x_prev_error = err_x
            self.y_prev_error = err_y
            self.pub.publish("PICKUP COMPLETE")
            rate.sleep()

	self.attach() # The drone gets attached to the target
	self.mode == "HOVER"
        self.hover()


    def planner(self):


        if self.mode == "HOVER":
	    self.hover()
	print('transitioning')
	#if self.mode == "SWOOP":
	#    self.descent()	    

if __name__=='__main__':
    OffbPosCtl()
