#!/usr/bin/python3
import numpy as np
from autolab_core import RigidTransform
from frankapy import FrankaArm
import time
import rospy
from geometry_msgs.msg import PoseArray,Pose
import numpy as np
import random
from vision_msgs.msg import Detection2DArray
import tf
from geometry_msgs.msg import TransformStamped


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
        
        self.bin0_pose = []
        self.bin1_pose = []

        #initialise camera
        self.camera_intrinsic_matrix = [[908.3275146484375, 0.0, 627.3765258789062], [0.0, 905.9411010742188, 365.3042907714844], [0.0, 0.0, 1.0]] #size [3,3]
        self.camera_extrinsic_matrix = [[0.0591998,  0.9970382, -0.0490941, 0.0024620632570656435],[-0.9966598,  0.0562626, -0.0591937, 0.01992316194519951],[-0.0562562,  0.0524344,  0.9970385, -0.028317526414364302],[0,0,0,1]]#size [4,4]
        # rospy.Subscriber("/yolov7/detection",Detection2DArray,self.get_camera_frame_object)
        self.listener = tf.TransformListener()
        # rospy.Subscriber('/tf', TransformStamped, self.tf_callback)
        #utils
        self.z_threshold = 0.5
        self.kit_management = {'bin0':None, 'bin1':None}
        self.main()

    def tf_callback(self,data):
        trans, rot = self.listener.lookupTransform('/base_link', '/camera_link', rospy.Time(0))
        print("Translation:", trans)
        print("Rotation:", rot)

    def get_camera_frame_object(self,bbox):
        world_frame_box = []
        # print(bbox)
        box_coordinate = []
        for box in bbox.detections:
            box_coordinate.append(np.array([box.bbox.center.x, box.bbox.center.y, 1]))
            
        # print(box_coordinate[1])
        # box_coordinate = np.array([box.position.x, box.position.y, box.position.z])
        camera_frame_coordinates = np.linalg.inv(self.camera_intrinsic_matrix) @ box_coordinate[1]
        camera_frame_coordinates = camera_frame_coordinates / camera_frame_coordinates[2]
        print(camera_frame_coordinates)
        object = 'block'
        world_frame_box.append(self.get_world_frame_object(camera_frame_coordinates))

        return world_frame_box,object


    def get_world_frame_object(self, pose):
        camera_pose = np.hstack((pose,[1]))
        camera_pose = camera_pose.reshape(-1,1)
        # print(np.shape(camera_pose))
        world_frame = self.camera_extrinsic_matrix @ camera_pose
        # print(world_frame)

        return world_frame[:,:3]


    def get_bin(self,object):
        for key, values in self.kit_management.items():
            if not (object in values):
                self.kit_management[key].append(object)
                return key
                
        random_key = random.choice(list(self.kit_management.keys()))
        self.kit_management[random_key].append(object)


    def transformPose(self,pose):
        p0 = self.franka.get_pose()
        p1 = p0.copy()

        T_delta = RigidTransform(
        translation=np.array([pose[0], pose[1], pose[2]]),
        rotation=RigidTransform.z_axis_rotation(np.deg2rad(0)), 
                            from_frame=p1.from_frame, to_frame=p1.from_frame)
        
        p1 = p1 * T_delta
        return p1


    def main(self):
        while True:
            print('hello')
            object_poses = rospy.wait_for_message("/yolov7/detection",Detection2DArray,timeout=5)

            world_frame_boxes, object = self.get_camera_frame_object(object_poses)
            # print(world_frame_boxes)
            new_pose = [world_frame_boxes[0][0],world_frame_boxes[0][1], 0]

            print(new_pose)
            new_pose = np.array([0.175,0.0353,0.0])
            new_pose = self.transformPose(new_pose)
            self.franka.goto_pose(new_pose)
           
            #run point cloud search to find the gripping point
            # gripper_point = PointCloudSearch()

            self.franka.goto_poses(gripper_point)
            self.franka.close_gripper()

            self.franka.goto_joints(home)
            
            selected_bin = self.get_bin(object)

            if (selected_bin == 'bin0'):
                bin0Pose = self.transformPose(self.bin0_pose)
                self.franka.goto_poses(bin0Pose)
            else:
                bin1Pose = self.transformPose(self.bin1_pose)
                self.franka.goto_poses(bin1Pose)

            self.franka.open_gripper()

            self.franka.goto_joints(home)

            if(not object_poses):
                break
                
        self.franka.goto_joints(home)
        self.franka.reset_joints()

if __name__ == '__main__':
    obj = YSSR()
    
    