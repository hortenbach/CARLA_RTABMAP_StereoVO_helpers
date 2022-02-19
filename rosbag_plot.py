# example for plotting ground truth and estitated tracetory from rosbag file (ROS2)
# usage: rosbag_plot.py <path-to-your-rosbag-folder>/<rosbag-name>.db3

#!/usr/bin/env python3
import sys

import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

import cv2
from cv_bridge import CvBridge as cvb

# source ~/carla-ros-bridge/install/setup.bash 

class BagFileParser():
    def __init__(self, bag_file):
        self.conn = sqlite3.connect(bag_file)
        self.cursor = self.conn.cursor()

        ## create a message type map
        topics_data = self.cursor.execute("SELECT id, name, type FROM topics").fetchall()
        self.topic_type = {name_of:type_of for id_of,name_of,type_of in topics_data}
        self.topic_id = {name_of:id_of for id_of,name_of,type_of in topics_data}
        try:
            self.topic_msg_message = {name_of:get_message(type_of) for id_of,name_of,type_of in topics_data}
        except:
            print("Error readinf bag file. Have you sourced carla-ros-brdige?")

    def __del__(self):
        self.conn.close()

    # Return [(timestamp0, message0), (timestamp1, message1), ...]
    def get_messages(self, topic_name):

        topic_id = self.topic_id[topic_name]
        # Get from the db
        rows = self.cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)).fetchall()
        # Deserialise all and timestamp them
        return [ (timestamp,deserialize_message(data, self.topic_msg_message[topic_name])) for timestamp,data in rows]

#=====================================
# Plot trajectories with subplots
#=====================================
def plot_trajectories(gt, vo, title=f"rosbag {sys.argv[1]}"):
    #===============
    # set up
    #===============
    fig = plt.figure(figsize=plt.figaspect(0.5))
    fig.suptitle(title, fontsize=12)
    figcnt = 1
    # ax.autoscale_view()
    # ax.set_zlim(0.0, 0.1)
    
    #===================
    # get difference between initial poses of ground thruth and vo
    #===================
    gt_init_pos = gt[0][1].pose.pose.position
    vo_init_pos = vo[0][1].pose.pose.position
    dx = (gt_init_pos.x-vo_init_pos.x)
    dy = (gt_init_pos.y-vo_init_pos.y)
    dz = (gt_init_pos.z-vo_init_pos.z)
    print(f"Initial pose difference [gt-vo]:\ndx: {dx}\tdy: {dy}\tdz: {dz}")

    #===================
    # plot gound truth
    #===================
    # get timestamps
    #t_des = [trajectory[i][0] for i in range(len(trajectory))]
    # prepare coordinates
    p_des_x = []
    p_des_y = []
    p_des_z = []

    for i in range(len(gt)):
        # get initial pose and use for normalizing trajectories
        p_des_x.append(gt[i][1].pose.pose.position.x-dx)
        p_des_y.append(gt[i][1].pose.pose.position.y-dy)
        p_des_z.append(gt[i][1].pose.pose.position.z-dz)
        
    ax = fig.add_subplot(1,2,1, projection='3d')
    ax.set_title('ground truth')
    ax.set_xlabel('X', fontsize=10)
    ax.set_ylabel('Y', fontsize=10)
    ax.set_zlabel('Z', fontsize=10)
    ax.yaxis._axinfo['label']['space_factor'] = 3.0
    ax.scatter(p_des_x, p_des_y, p_des_z, s=1)
    
 # plot vo estimation
    
    p_des_x = []
    p_des_y = []
    p_des_z = []

    for i in range(len(vo)):
        # get initial pose and use for normalizing trajectories
        p_des_x.append(vo[i][1].pose.pose.position.x)
        p_des_y.append(vo[i][1].pose.pose.position.y)
        p_des_z.append(vo[i][1].pose.pose.position.z)
    
    ax = fig.add_subplot(1,2,2, projection='3d')
    ax.set_title('stereo vo')
    ax.set_xlabel('X', fontsize=10)
    ax.set_ylabel('Y', fontsize=10)
    ax.set_zlabel('Z', fontsize=10)
    ax.yaxis._axinfo['label']['space_factor'] = 3.0
    ax.scatter(p_des_x, p_des_y, p_des_z, s=1)
    
    # the coordinate systems from carla and ros2 dont seem to match. 
    # clean way is to write a static transformation with tf2 (TODO)
    # quick and dirty plot:
    # ax.scatter(p_des_y, [-i for i in p_des_x], p_des_z, s=1)
    
    # ax = fig.add_subplot(1, 2, 2, projection='3d')
    # ax.plot(p_des_x, p_des_y)
    plt.show()
    
def viewImage(parser, frame):
    # use CVBridge
    imgL = parser.get_messages("/carla/ego_vehicle/rgb_left/image")
    cv_img = cvb.imgmsg_to_cv2(imgL, desired_encoding="passthrough")
    print(type(cv_img))


# timestamp at parser.get_messages("/carla/ego_vehicle/odometry")[*][0]
# nav_msgs/Odometry.msg at parser.get_messages("/carla/ego_vehicle/odometry")[*][1]

if __name__ == "__main__":

    try:    
        bag_file =  sys.argv[1]
    except IndexError:
        raise SystemExit("Usage: ./rosbag_plot.py <path_to_bagfile>")

    parser = BagFileParser(bag_file)

#         trajectory = parser.get_messages("/carla/ego_vehicle/odometry")[0][1] 
#         p_des_1 = [trajectory.points[i].positions[0] for i in range(len(trajectory.points))]
#         t_des = [trajectory.points[i].time_from_start.sec + trajectory.points[i].time_from_start.nanosec*1e-9 for i in range(len(trajectory.points))]
    gt = parser.get_messages("/carla/ego_vehicle/odometry") 
    vo = parser.get_messages("/odom") 
    # trajectory = parser.get_messages("/carla/ego_vehicle/odometry") 
    # viewImage(parser, 0)
    
    plot_trajectories(gt, vo, str(sys.argv[1])) 
