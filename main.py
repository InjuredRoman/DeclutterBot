#!/usr/bin/python3
import numpy as np
from autolab_core import RigidTransform
from frankapy import FrankaArm
import time
import rospy
from geometry_msgs.msg import PoseArray,Pose
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge
import numpy as np
import random
from vision_msgs.msg import Detection2DArray
import tf
import tf2_ros
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_matrix
from sensor_msgs.msg import CameraInfo


##flow of the project
#starts from the home position
#detects the object using sam and segment them
#convert the segmented bounding box to camera frame coordinates
#convert the camera frame coordinates to the world frame
#reduce the z cooridnates by a certain degree
#go to the pose found by the algorithm till now 
#After reaching the pose, run the point cloud generation and get the gripping pose
#go to the gripping pose and grasp the object 
#go to home position and run what bin the object needs to go (kit management)
#go to the bin location and drop the object
#repeat till condition  
#go to the home pose 


home = [ 1.96596364e-04, -7.85841667e-01, -3.06014654e-03, -2.35654641e+00, -4.62090277e-04,  1.57150903e+00,  7.85095747e-01]

class YSSR:
    def __init__(self):
        #initialise arm
        self.franka = FrankaArm()
        # self.franka.reset_joints()
        # self.franka.open_gripper()
        # self.franka.goto_joints(home)
        
        # self.bin0_pose = np.array([0.0, 0.0, 0.0])
        self.bin0_pose = PoseStamped()
        self.bin0_pose.pose.position.x = 0.4
        self.bin0_pose.pose.position.y = -0.3
        self.bin0_pose.pose.position.z = 0.3

        self.bin1_pose = np.array([0.0, 0.0, 0.0])
        self.bin1_pose = PoseStamped()
        self.bin1_pose.pose.position.x = 0.4
        self.bin1_pose.pose.position.y = -0.3
        self.bin1_pose.pose.position.z = 0.3



        #initialise camera
        # self.camera_intrinsic_matrix = [[908.3275146484375, 0.0, 627.3765258789062], [0.0, 905.9411010742188, 365.3042907714844], [0.0, 0.0, 1.0]] #size [3,3]
        self.K = np.array([[908.3275146484375, 0.0, 627.3765258789062], [0.0, 905.9411010742188, 365.3042907714844], [0.0, 0.0, 1.0]])
        
        self.camera_extrinsic_matrix = [[0.0591998,  0.9970382, -0.0490941, 0.0024620632570656435],[-0.9966598,  0.0562626, -0.0591937, 0.01992316194519951],[-0.0562562,  0.0524344,  0.9970385, -0.028317526414364302],[0,0,0,1]]#size [4,4]
        # rospy.Subscriber("/yolov7/detection",Detection2DArray,self.get_camera_frame_object)
        self.listener = tf.TransformListener()
        # rospy.Subscriber('/tf', TransformStamped, self.tf_callback)
        # rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_captured_cb)
        # rospy.Subscriber('/camera/color/image_raw', Image, self.image_captured_cb)
        # rospy.Subscriber('/camera/color/camera_info',CameraInfo,self.camera_info_cb)
        # self.depths_captured = list()
        # self.images_captured = list()
        #utils
        self.z_threshold = 0.5
        # self.tf_buffer = tf2_ros.Buffer()
        # self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.kit_management = {'bin0':[], 'bin1':[]}
        self.main()

    def camera_info_cb(self,msg):
        self.K = msg.K
    
    def transform_pose(self, input_pose, from_frame, to_frame):
        """
        Transforms a PoseStamped from one frame to another.

        Parameters:
        input_pose (geometry_msgs/PoseStamped): The pose to transform.
        from_frame (str): The current frame of the pose.
        to_frame (str): The target frame to transform the pose into.

        Returns:
        geometry_msgs/PoseStamped: The pose transformed into the target frame,
        or None if the transformation failed.
        """
        try:
            # Ensure that the pose's header.frame_id is correctly set
            input_pose.header.frame_id = from_frame

            # Get the current time and wait for the transformation to be available
            current_time = rospy.Time.now()
            self.listener.waitForTransform(to_frame, from_frame, current_time, rospy.Duration(4.0))
            
            # Perform the transformation
            transformed_pose = self.listener.transformPose(to_frame, input_pose)
            return transformed_pose
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.loginfo("Transform error: %s", e)
            return None







    def create_pose_msg(self, x, y, angle, z=0.5):
        pose_msg = PoseStamped()
        # pose_msg.header.frame_id = "camera_color_frame"

        pose_msg.header.frame_id = "camera_color_optical_frame"
        pose_msg.header.stamp = rospy.Time.now()
        
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]

        # Assuming cx, cy, fx, fy are properly defined
        X = (x - cx) * z / fx
        Y = (y - cy) * z / fy

        pose_msg.pose.position.x = X
        pose_msg.pose.position.y = Y
        pose_msg.pose.position.z = z

        # pose_msg.pose.position.x = z
        # pose_msg.pose.position.y = -X
        # pose_msg.pose.position.z = -Y 
        

        # Quaternion from Euler angles
        quaternion = tf.transformations.quaternion_from_euler(0, 0, angle)
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]

        to_frame = "panda_link0"
        from_frame = pose_msg.header.frame_id

        
        try:
            # Ensure that the pose's header.frame_id is correctly set
            # pose_msg.header.frame_id = from_frame

            # Get the current time and wait for the transformation to be available
            current_time = rospy.Time.now()
            self.listener.waitForTransform(to_frame, from_frame, current_time, rospy.Duration(4.0))
            
            # Perform the transformation
            transformed_pose = self.listener.transformPose(to_frame, pose_msg)
            return transformed_pose
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.loginfo("Transform error: %s", e)
            return None
        



        try:
            transform_timeout = rospy.Duration(1.0)
            robot_frame = "panda_link0"
            pose_transformed = self.tf_buffer.transform(pose_msg, robot_frame, timeout=transform_timeout)
            return pose_transformed
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            rospy.logerr("TF2 transform error: %s" % ex)
            return None

        
    def get_camera_frame_object(self,bbox):
        
        # print(bbox)
        box_coordinate = []
        for box in bbox.detections:
            pose_object = [box.bbox.center.x, box.bbox.center.y]
            # box_coordinate.append(np.array([box.bbox.center.x, box.bbox.center.y, 1]))
            
        # print(box_coordinate[0])
        # box_coordinate = np.array([box.position.x, box.position.y, box.position.z])
        # camera_frame_coordinates = np.linalg.inv(self.camera_intrinsic_matrix) @ box_coordinate[0]
        # camera_frame_coordinates = camera_frame_coordinates / camera_frame_coordinates[2]
        # print(camera_frame_coordinates)
        object = 'block'
        
        # tf_matrix = self.tf_callback()
        world_frame_box= self.create_pose_msg(pose_object[0],pose_object[1],0.0)

        return world_frame_box,object


    def get_world_frame_object(self, tf_matrix, camera_pose):
        camera_pose = np.hstack((camera_pose,[1]))
        camera_pose = camera_pose.reshape(-1,1)
        
        
        world_frame = tf_matrix@ camera_pose
        world_frame /= world_frame[3]
        print(world_frame)

        return world_frame[:,:3]


    def get_bin(self,object):
        for key, values in self.kit_management.items():
            if not (object in values):
                self.kit_management[key].append(object)
                return key
                
        random_key = random.choice(list(self.kit_management.keys()))
        self.kit_management[random_key].append(object)


    def transformPose(self,pose):
        extrated_pose = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]

        p0 = self.franka.get_pose()
        print("Get pose", p0)
        p0.position = np.array(extrated_pose)
    
        return p0


    def main(self):
        while True:
            print('hello')
            object_poses = rospy.wait_for_message("/yolov7/detection", Detection2DArray,timeout=5)
            
            world_frame_boxes, object = self.get_camera_frame_object(object_poses)
            
            
            # new_pose = [world_frame_boxes[0][0],world_frame_boxes[1][0], 0.0]

            # print(new_pose)
            # new_pose = np.array([0.175,0.0453, 0.0])
            # new_pose = np.array([0.0,0.0, 0.0])
            new_pose = self.transformPose(world_frame_boxes)
            print("[INFO] Successfully transformed the pose and exiting")
            
            self.franka.goto_pose(new_pose)
           
            # exit()
            #run point cloud search to find the gripping point
            # gripper_point = PointCloudSearch()
            # gripper_pose = self.transformPose(gripper_pose)
            
            # self.franka.goto_pose(gripper_point)
            self.franka.close_gripper()

            self.franka.goto_joints(home)
            
            selected_bin = self.get_bin(object)

            if (selected_bin == 'bin0'):
                bin = self.transformPose(self.bin0_pose)
                # self.franka.goto_pose(bin0Pose)
            else:
                bin = self.transformPose(self.bin1_pose)
                # self.franka.goto_pose(bin1Pose)

            self.franka.goto_pose(bin)

            self.franka.open_gripper()

            time.sleep(1)
            self.franka.goto_joints(home)
            time.sleep(2)
            if(not object_poses):
                break
                
        self.franka.goto_joints(home)
        self.franka.reset_joints()

if __name__ == '__main__':
    obj = YSSR()
    
    