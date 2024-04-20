#!/usr/bin/python3
import numpy as np
from autolab_core import RigidTransform
from frankapy import FrankaArm
import time
import rospy
from geometry_msgs.msg import PoseArray,Pose
import numpy as np
import random

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


# fa = FrankaArm()
# fa.reset_joints()
# fa.open_gripper()
# fa.goto_joints(home)

# for obj in objects:
#     fa.goto_joints(home)
#     fa.goto_joints(obj)
#     # time.sleep(10)
#     fa.close_gripper()
#     fa.goto_joints(home)
#     fa.goto_joints(final_box)
#     fa.open_gripper()

home = [ 1.96596364e-04, -7.85841667e-01, -3.06014654e-03, -2.35654641e+00, -4.62090277e-04,  1.57150903e+00,  7.85095747e-01]

class YSSR:
    def __init__(self):
        #initialise arm
        self.franka = FrankaArm()
        self.franka.reset_joints()
        self.franka.open_gripper()
        self.franka.goto_joints(home)
        self.bin0_pose = []
        self.bin1_pose = []

        #initialise camera
        self.camera_intrinsic_matrix = #size [3,3]
        self.camera_extrinsic_matrix = #size [4,4]
        rospy.Subscriber("/sam_bbox_object",PoseArray,self.get_camera_frame_object)

        #utils
        self.z_threshold = 0.5
        self.kit_management = {'bin0':None, 'bin1':None}


    def get_camera_frame_object(self,msg):
        world_frame_box = []

        for box in msg:
            # box_msg = Pose()
            box_coordinate = np.array([box.poses.position.x, box.poses.position.y, box.poses.position.z])
            camera_frame_coordinates = self.camera_frame_coordinates @ box_coordinate
            object = box.object_name
            world_frame_box.append(self.get_world_frame_object(camera_frame_coordinates[:,:3]),object)

        return world_frame_box


    def get_world_frame_object(self, pose):
        camera_pose = np.hstack((pose,[1]))
        world_frame = self.camera_extrinsic_matrix @ camera_pose

        return world_frame[:,:3]
    
    def get_bin(self,object):
        for key, values in self.kit_management.items():
            if not (object in values):
                self.kit_management[key].append(object)
                return key
                
        random_key = random.choice(list(self.kit_management.keys()))
        self.kit_management[random_key].append(object)


    def main(self):
        while(not rospy.is_shutdown()):

            object_poses = rospy.wait_for_message('/sam_bbox_object',PoseArray,timeout=2)
            world_frame_boxes,object = self.get_camera_frame_object(object_poses)

            new_pose = [world_frame_boxes[0],world_frame_boxes[1], world_frame_boxes[2]+self.z_threshold]

            self.franka.goto_pose(new_pose)

            #run point cloud search to find the gripping point
            # gripper_point = PointCloudSearch()

            self.franka.goto_poses(gripper_point)
            self.franka.close_gripper()

            self.franka.goto_joints(home)
            
            selected_bin = self.get_bin(object)

            if (selected_bin == 'bin0'):
                self.franka.goto_poses(self.bin0_pose)
            else:
                self.franka.goto_poses(self.bin1_pose)

            self.franka.open_gripper()

            self.franka.goto_joints(home)

            if(not object_poses):
                break
        
        
        self.franka.goto_joints(home)
        self.franka.reset_joints()

if __name__ == '__main__':
    obj = YSSR()
    obj.main()
