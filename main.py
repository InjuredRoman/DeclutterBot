#!/usr/bin/python3
import numpy as np
from autolab_core import RigidTransform
from frankapy import FrankaArm
import copy
import rospy
from geometry_msgs.msg import PoseArray,Pose
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge
import numpy as np
import random
from vision_msgs.msg import Detection2DArray
import tf
import tf2_ros
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_matrix, euler_matrix
from sensor_msgs.msg import CameraInfo, PointCloud2
import struct
import ctypes
import time

import open3d as o3d
def pctonp(pc):
    print('a')
    # from [here](https://answers.ros.org/question/344096/subscribe-pointcloud-and-convert-it-to-numpy-in-python/)
    xyz = np.array([[0,0,0]])
    rgb = np.array([[0,0,0]])
    print('a')
    gen = pc2.read_points(pc, skip_nans=True)
    print('a')
    int_data = list(gen)
    print('yo')
    for x in int_data:
        test = x[3] 
        # cast float32 to int so that bitwise operations are possible
        s = struct.pack('>f' ,test)
        i = struct.unpack('>l',s)[0]
        # you can get back the float value by the inverse operations
        pack = ctypes.c_uint32(i).value
        r = (pack & 0x00FF0000)>> 16
        g = (pack & 0x0000FF00)>> 8
        b = (pack & 0x000000FF)
        # prints r,g,b values in the 0-255 range
                    # x,y,z can be retrieved from the x[0],x[1],x[2]
        xyz = np.append(xyz,[[x[0],x[1],x[2]]], axis = 0)
        rgb = np.append(rgb,[[r,g,b]], axis = 0)
    print('a')
    return xyz, rgb
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

import ros_numpy.point_cloud2 as nppc2
home = [ 1.96596364e-04, -7.85841667e-01, -3.06014654e-03, -2.35654641e+00, -4.62090277e-04,  1.57150903e+00,  7.85095747e-01]
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

# The data structure of each point in ros PointCloud2: 16 bits = x + y + z + rgb
FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
FIELDS_XYZRGB = FIELDS_XYZ + \
    [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]

# Bit operations
BIT_MOVE_16 = 2**16
BIT_MOVE_8 = 2**8
from ctypes import * # convert float to uint32
convert_rgbUint32_to_tuple = lambda rgb_uint32: (
    (rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff)
)
convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
    int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
)

# Convert the datatype of point cloud from Open3D to ROS PointCloud2 (XYZRGB only)
def convertCloudFromOpen3dToRos(open3d_cloud, frame_id="odom"):
    # Set "header"
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id

    # Set "fields" and "cloud_data"
    points=np.asarray(open3d_cloud.points)
    if not open3d_cloud.colors: # XYZ only
        fields=FIELDS_XYZ
        cloud_data=points
    else: # XYZ + RGB
        fields=FIELDS_XYZRGB
        # -- Change rgb color from "three float" to "one 24-byte int"
        # 0x00FFFFFF is white, 0x00000000 is black.
        colors = np.floor(np.asarray(open3d_cloud.colors)*255) # nx3 matrix
        colors = colors[:,0] * BIT_MOVE_16 +colors[:,1] * BIT_MOVE_8 + colors[:,2]  
        cloud_data=np.c_[points, colors]
    
    # create ros_cloud
    return pc2.create_cloud(header, fields, cloud_data)
import random
class YSSR:
    def __init__(self):
        #initialise arm
        self.franka = FrankaArm()
        self.franka.stop_skill()
        self.franka.open_gripper()
        self.franka.reset_joints()
        self.cvb = CvBridge()
        

        # self.franka.reset_joints()
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
        self.depth_captures = []
        # '/camera/depth/color/points'
        # rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_captured_cb)
        # rospy.Subscriber('/camera/color/image_raw', Image, self.image_captured_cb)
        # rospy.Subscriber('/camera/color/camera_info',CameraInfo,self.camera_info_cb)
        rospy.Subscriber("/yolov7/cropped_detections", Image, self.cropped_image_cb, queue_size=1)
        self.cpcPub =  rospy.Publisher("/cropped_pointcloud", PointCloud2, queue_size=100)

        self.collected_pcs = list()
        # self.depths_captured = list()
        # self.images_captured = list()
        #utils
        self.z_threshold = 0.5
        # self.tf_buffer = tf2_ros.Buffer()
        # self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.kit_management = {'bin0':[], 'bin1':[]}
        self.yoloIdsToNames = {
            0: 'eraser',
            1: 'marker',
            2: 'cube',
            3: 'cylinder'
        }
        self.namesToBins= {
            'eraser': 3,
            'marker': 2,
            'cube': 1,
            'cylinder': 0
        }
        self.binPoses = {
            0: [0.35, 0.25, 0.1],
            1: [0.6, 0.3, 0.1],
            2: [0.35, -0.25, 0.1],
            3: [0.6, -0.3, 0.1]
        }
        self.resetBinPoses = [
            RigidTransform( translation = np.array([ 0.37589318, -0.09899138,  0.3897211 ]),
                rotation = np.array([[ 0.99640239, -0.05519584, -0.0641601 ],
                                        [-0.07412555, -0.203273,   -0.97631173],
                                        [ 0.04084633,  0.97755524, -0.2066371 ]]), from_frame='franka_tool', to_frame='world'),
            RigidTransform( translation= np.array([ 0.54663912, -0.12236179,  0.34385008]),
                rotation = np.array([[ 0.99981569, -0.00546223, -0.01787468],
                                    [-0.01846056, -0.13905769, -0.99011201],
                                    [ 0.0029226,   0.9902595,  -0.13913557]]) , from_frame='franka_tool', to_frame='world'),
            RigidTransform( translation= np.array([ 0.52805492, -0.02477458,  0.23431636]),
                rotation = np.array([[ 0.99937196, -0.03257488, -0.01324025],
                                    [ 0.01069844, -0.07699795,  0.99697379],
                                    [-0.03349578, -0.99648931, -0.07660257]]) , from_frame='franka_tool', to_frame='world'),
            RigidTransform( translation= np.array([ 0.39893177, -0.05065839,  0.19229496]),
                rotation = np.array([[ 0.99313767, -0.06340301,  0.0981762 ],
                                    [-0.0996791,  -0.02100013,  0.99479791],
                                    [-0.06101147, -0.99775739, -0.02717649]]) , from_frame='franka_tool', to_frame='world')
            ]
                #  Qtn: [-0.4795787   0.87515566 -0.03311425  0.05482217]
        # )

        self.main()

    def cropped_image_cb(self, img):
        img = np.asarray(self.cvb.imgmsg_to_cv2(img, desired_encoding='passthrough'))
        pc = []
        maxz = 0
        minz = 1000000
        # print(img.shape)
        for y in range(img.shape[1]):
            for x in range(img.shape[0]):
                z =  img[x][y]
                # camera_vec  = self.get_imagetocamera_vector(x, y, 0.0, z)
                # print(camera_vec)
                maxz = max(maxz, z)
                minz = min(minz, z)
                if (z < 50): continue
                camera_vec = [x, y, z]
                pc.append([camera_vec[0], camera_vec[1], camera_vec[2]])
        # print(maxz, minz)
        pc = np.reshape(np.array(pc), (-1, 3))
        # print(pc.shape)
        self.croppedPC = pc
        pcd = o3d.geometry.PointCloud()
        # print(self.lastPcPoints.shape)0]
        pcd.points = o3d.utility.Vector3dVector(self.croppedPC)
        #self.o3dCroppedPc = pcd
        self.collected_pcs.append(pcd)
        # o3d.io.write_point_cloud("collected_pointclouds/%i.ply" % int(random.random()*100), pcd)
        # pcMsg = convertCloudFromOpen3dToRos(pcd, frame_id="camera_color_optical_frame")
        
        # self.cpcPub.publish(pcMsg)
        # o3d.visualization.draw_geometries([pcd])
        # print(1/0)
        
    def makePoseFromPosArr(self, poseArray):
        x, y, z = poseArray
        res = PoseStamped()
        res.pose.position.x = x
        res.pose.position.y = y
        res.pose.position.z = z
        return res
    
    # def depth_captured_cb(self, pc):
    #     print(type(pc))
    #     rgb, depth = pctonp(pc)
    #     print(rgb.shape)
    #     # print(rgb.shape)
    #     print(depth.shape)
    #     # x = np.asarray(self.cvb.pointcloud2_to_cv2(pc))
    #     # print(x.shape)
    #     # print(depth.shape)
    #     # img = np.asarray(self.cvb.imgmsg_to_cv2(pc, desired_encoding='passthrough'))
    #     # print(pc.height, pc.width, pc.step, pc.encoding)
    #     # print(img.shape)
    #     # self.depth_captures.append(pc)
    
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
            
            # Perform the transformationcropped_pointcloud
            transformed_pose = self.listener.transformPose(to_frame, input_pose)
            return transformed_pose
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.loginfo("Transform error: %s", e)
            return None


    def get_imagetocamera_vector(self, x, y, angle, z=0.57):
        pose_msg = PoseStamped()
        # pose_msg.header.frame_id = "camera_color_frame"

        pose_msg.header.frame_id = "camera_color_optical_frame"
        pose_msg.header.stamp = rospy.Time.now()
        
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]

        # Assuming cx, cy, fx, fy are properly defined
        X = (x - cx) * z / fx
        Y = (y - cy) * z / fy
        Z = z

        return np.array([X, Y, Z])


    def create_pose_msg(self, x, y, angle, z=0.57):
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
        

        
    def get_camera_frame_object(self,bbox):
        # print(bbox)
        box_coordinate = []
        detectionArea = {'x': [220, 920]} #       detectionArea = {'x': [200, 920]}
        for i, box in enumerate(bbox.detections):
            if box.bbox.center.x > detectionArea['x'][1] or box.bbox.center.x < detectionArea['x'][0]:
                continue
            # print(box.bbox.center.x, box.bbox.center.y, self.yoloIdsToNames[bbox.detections[i].results[0].id])

            pose_object = [box.bbox.center.x, box.bbox.center.y]
            world_frame_box= self.create_pose_msg(pose_object[0],pose_object[1],0.0)
            item = self.yoloIdsToNames[bbox.detections[i].results[0].id]
            return world_frame_box, item
        # pose_object = [box.bbox.center.x, box.bbox.center.y]
        # world_frame_box= self.create_pose_msg(pose_object[0],pose_object[1],0.0)
        # item = self.yoloIdsToNames[bbox.detections[i].results[0].id]
        # return world_frame_box, item
            # box_coordinate.append(np.array([box.bbox.center.x, box.bbox.center.y, 1]))
        return
        # print(box_coordinate[0])
        # box_coordinate = np.array([box.position.x, box.position.y, box.position.z])
        # camera_frame_coordinates = np.linalg.inv(self.camera_intrinsic_matrix) @ box_coordinate[0]
        # camera_frame_coordinates = camera_frame_coordinates / camera_frame_coordinates[2]
        # print(camera_frame_coordinates)
        # for obj in bbox.results:
        #     object = self.yoloIdsToNames[obj.id]

        # object = 'block'
        
        # tf_matrix = self.tf_callback()

        return world_frame_box,object


    def get_world_frame_object(self, tf_matrix, camera_pose):
        camera_pose = np.hstack((camera_pose,[1]))
        camera_pose = camera_pose.reshape(-1,1)
        
        
        world_frame = tf_matrix@ camera_pose
        world_frame /= world_frame[3]
        # print(world_frame)

        return world_frame[:,:3]


    def get_bin(self,object):
        for key, values in self.kit_management.items():
            if not (object in values):
                self.kit_management[key].append(object)
                return key
                
        random_key = random.choice(list(self.kit_management.keys()))
        self.kit_management[random_key].append(object)


    def transformPose(self,pose, orientation=None):
        extrated_pose = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]

        p0 = self.franka.get_pose()
        p0.position = extrated_pose
        if not (orientation is None):
            print('happening')
            p0.orientation = [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]
        return p0
    
    def dropInBin(self, bin):
        binDroppingPose = self.binPoses[bin]
        self.franka.goto_pose(self.transformPose(self.makePoseFromPosArr(binDroppingPose)))
        self.franka.open_gripper()
    def goToBinResetPose(self, bin):
        self.franka.goto_pose(self.resetBinPoses[bin])

    def goToBinPickupPose(self, bin):
        ## grab edge of bin
        binCollectionPose = self.transformPose(self.makePoseFromPosArr(self.binPoses[bin]))
        # adjust y position to be slightly to the right
        yAdjustment = .04
        if (bin in [0, 1]):
            binCollectionPose.position[1] -= yAdjustment
        if (bin in [2, 3]):
            binCollectionPose.position[1] += yAdjustment
        # adjust z position to grasp the edge of the bin
        binCollectionPose.position[2] = 0.007
        self.franka.goto_pose(binCollectionPose)

    def emptyBin(self, bin):
        print('Emptying bin %i, comrade.' % (bin+1))
        self.goToBinPickupPose(bin)
        self.franka.close_gripper()
        time.sleep(1)

        ## drop items in center of table
        self.franka.goto_pose(self.scanningPose)
        self.goToBinResetPose(bin)
        self.franka.goto_pose(self.scanningPose)
        ## place bin back where it was
        self.goToBinPickupPose(bin)
        self.franka.open_gripper()
        time.sleep(1.5)
        self.franka.goto_pose(self.scanningPose)
    def main(self):
        # for k, v in self.binPoses.items():
        #     print('going to bin %i' % k)[3, 2, 1, 0]
        #     self.franka.goto_pose(self.transformPose(self.makePoseFromPosArr(v)))
        # exit()
        while True:
            print("""
                  Welcome back, comrade.""")
            # self.franka.reset_joints()
            # print('hello')

            # depthTopic = '/camera/depth/color/points' #sensor_msgs/PointCloud2
            # got to a view where we see more of the table, avoid artifacts from base of franka arm (offtable)
            scanningPose = self.franka.get_pose()
            scanningPose.position[0] += .18; scanningPose.position[2] += .05
            self.scanningPose = scanningPose
            self.resetBinPose = copy.deepcopy(scanningPose); self.resetBinPose.position[2] -= .1;
            self.franka.goto_pose(scanningPose)

            # continue
            # object_poses = rospy.wait_for_message("/yolov7/detection", Detection2DArray,timeout=5)
            # depthScan = rospy.wait_for_message('/camera/depth/color/points', PointCloud2, timeout=5)
            # # res = pc2.read_points(depthScan)
            # # res = pc2.pointcloud2_to_array(depthScan)
            # res = ss.pointcloud2_to_xyz_array(depthScan)
            # print(res.shape)
            # self.franka.open_gripper()
            def getNextItem():
                object_poses = rospy.wait_for_message("/yolov7/detection", Detection2DArray,timeout=5)
                try:
                    world_frame_boxes, item = self.get_camera_frame_object(object_poses)
                except: #no items left!
                    return
                new_pose = self.transformPose(world_frame_boxes)
                new_pose.position[2] = 0.011
                if (item == 'cube'):
                    new_pose.position[0] -= .02
                self.franka.goto_pose(new_pose)
                self.franka.close_gripper()
                return item

            def dropItemInBin(item):
                self.franka.goto_pose(scanningPose)
                self.dropInBin(self.namesToBins[item])
                self.franka.goto_pose(scanningPose)

            # go through our collected point clouds, find the items they match to
            while True:
                # print('trying for %i' % i)
                item = getNextItem()
                if (item is None):
                    print('No items are left, comrade.')
                    break
                print('Found %s, dropping now into bin %i, comrade.' % (item, self.namesToBins[item]+1))
                dropItemInBin(item)
            for bin in [0, 1, 2, 3]:
                self.emptyBin(bin)
                self.franka.reset_joints()
            print("Your room is clean, comrade.")
            # o3d.visualization.draw_geometries([pcd])
            break
            # for pc in self.collected_pcs:
            #     itemGraspingPose, item = self.match_pc_to_item(pc)
            #     self.franka.goto_pose(itemGraspingPose)
            #     self.franka.close_gripper()


            ## PCD looks like it might be our ticket
            o3d.visualization.draw_geometries([pcd])
            # ?rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.depth_captured_cb, queue_size=1, buff_size=52428800)

            
            # new_pose = [world_frame_boxes[0][0],world_frame_boxes[1][0], 0.0]

            # print(new_pose)
            # new_pose = np.array([0.175,0.0453, 0.0])
            # new_pose = np.array([0.0,0.0, 0.0])
            
            
            # print("[INFO] Successfully transformed the pose and exiting")
            # print("New pose", new_pose)
           
            #run point cloud search to find the gripping point
            # gripper_point = PointCloudSearch()
            # gripper_pose = self.transformPose(gripper_pose)
            
            # self.franka.goto_pose(gripper_point)
            # self.franka.goto_joints(home)
            
            # selected_bin = self.get_bin(object)

            # for keys, values in self.kit_management.items():
            #     print(keys, values)

            # if (selected_bin == 'bin0'):
            #     bin = self.transformPose(self.bin0_pose)
            #     # self.franka.goto_pose(bin0Pose)
            # else:
            #     bin = self.transformPose(self.bin1_pose)
            #     # self.franka.goto_pose(bin1Pose)

            # self.franka.goto_pose(bin)

            # self.franka.open_gripper()

            # time.sleep(1)
            # self.franka.goto_joints(home)
            # time.sleep(2)
            # if(not object_poses):
            #     break
                
        self.franka.goto_joints(home)
        self.franka.reset_joints()

if __name__ == '__main__':
    obj = YSSR()
    
    