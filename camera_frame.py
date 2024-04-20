#!/usr/bin/python3
import numpy as np
from autolab_core import RigidTransform
from frankapy import FrankaArm
import time
import rospy
from geometry_msgs.msg import PoseArray,Pose
import numpy as np

class cameraFrame:
    def __init__(self):
        self.camera_intrinsic_matrix = [3,3]
        rospy.init_node('cameraFrame',anonymous=True)
        self.rate = rospy.Rate(10)

        self.bbox_subscribe = rospy.Subscriber("/bbox_object",PoseArray,self.get_camera_frame)
        self.camera_frame_publisher = rospy.Publisher("/camera_bbox_object",PoseArray,queue_size=10)

    def get_camera_frame(self,msg):
        camera_frame_box = []
        camera_bboxes = PoseArray()

        for box in msg:
            box_msg = Pose()
            box_coordinate = np.array([box.poses.position.x, box.poses.position.y, box.poses.position.z])
            camera_frame_coordinates = self.camera_frame_coordinates @ box_coordinate
            box_msg.position.x = camera_frame_coordinates[0]
            box_msg.position.y = camera_frame_coordinates[1]
            box_msg.position.z = camera_frame_coordinates[2]
            camera_bboxes.append(box_msg)

        self.camera_frame_publisher.publish(camera_bboxes)
        self.rate.sleep()

if __name__ == '__main__':
    frame = cameraFrame()
        
