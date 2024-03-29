from launch import LaunchDescription
from launch.actions import ExecuteProcess

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtabmap_ros', executable='stereo_odometry', output="screen",
            parameters=[{
                "frame_id":'ego_vehicle',               #our base_link
                "publish_tf" : True,                    #~publish_tf (bool, default: "true") Publish TF from /odom to /base_link
                "Odom/Strategy": "1",                   # 1= Frame 2 Frame 
                "wait_for_transform": 2.0,
                "Reg/Force3DoF": "true",                # [Force 3 degrees-of-freedom transform (3Dof: x,y and yaw). Parameters z, roll and pitch will be set to 0.]
                "approx_sync": False,
                "Odom/AlignWithGround": "false",         #[Align odometry with the ground on initialization.]
                "tf_delay": 0.01, #(double, default: 0.05)    Rate at which the TF from /map to /odom is published (20 Hz)
                "queue_size": 15,
                "Odom/Holonomic" : 'false',             #[If the robot is holonomic (strafing commands can be issued). If not, y value will be estimated from x and yaw values (y=x*tan(yaw)).]
                "Odom/ResetCountdown" : '1',            #[Automatically reset odometry after X consecutive images on which odometry cannot be computed (value=0 disables auto-reset).]
                # ORB/EdgeThreshold = "19"              [This is size of the border where the features are not detected. It should roughly match the patchSize parameter.]
                # ORB/PatchSize = "31"                  [size of the patch used by the oriented BRIEF descriptor. Of course, on smaller pyramid layers the perceived image area covered by a feature will be larger.]
                # ORB/ScoreType = "0"                   [The default HARRIS_SCORE=0 means that Harris algorithm is used to rank features (the score is written to KeyPoint::score and is used to retain best nfeatures features); FAST_SCORE=1 is alternative value of the parameter that produces slightly less stable keypoints, but it is a little faster to compute.]
                # ORB/WTA_K = "2"                       [The number of points that produce each element of the oriented BRIEF descriptor. The default value 2 means the BRIEF where we take a random point pair and compare their brightnesses, so we get 0/1 response. Other possible values are 3 and 4. For example, 3 means that we take 3 random points (of course, those point coordinates are random, but they are generated from the pre-defined seed, so each element of BRIEF descriptor is computed deterministically from the pixel rectangle), find point of maximum brightness and output index of the winner (0, 1 or 2). Such output will occupy 2 bits, and therefore it will need a special variant of Hamming distance, denoted as NORM_HAMMING2 (2 bits per bin). When WTA_K=4, we take 4 random points to compute each bin (that will also occupy 2 bits with possible values 0, 1, 2 or 3).]
                # Odom/AlignWithGround = "false"        [Align odometry with the ground on initialization.]
                # Odom/FilteringStrategy = "0"          [0=No filtering 1=Kalman filtering 2=Particle filtering. This filter is used to smooth the odometry output.]
                # Odom/GuessMotion = "true"             [Guess next transformation from the last motion computed.]
                # Odom/GuessSmoothingDelay = "0"        [Guess smoothing delay (s). Estimated velocity is averaged based on last transforms up to this maximum delay. This can help to get smoother velocity prediction. Last velocity computed is used directly if "Odom/FilteringStrategy" is set or the delay is below the odometry rate.]
                # "OdomFovis/StereoMaxDisparity" : "128",
                "Stereo/Eps":"0.01",                    #[[Stereo/OpticalFlow = true] Epsilon stop criterion.]
                "Stereo/Iterations":"30",               # 30 [Maximum iterations.]
                "Stereo/MaxLevel":"3",                  #[Maximum pyramid level.]
                "Stereo/MinDisparity":"1",              #[Minimum disparity.]
                "Stereo/OpticalFlow":"true",            #true[Use optical flow to find stereo correspondences, otherwise a simple block matching approach is used.]
                "Stereo/SSD":"false",                    #[[Stereo/OpticalFlow = false] Use Sum of Squared Differences (SSD) window, otherwise Sum of Absolute Differences (SAD) window is used.]
                "Stereo/WinHeight":"3",                 #3[Window height.]
                "Stereo/WinWidth":"15",                 #15[Window width.]
                "Stereo/MaxDisparity" : "256.0",        #[Maximum disparity.]
                "Stereo/DenseStrategy" : "0",            #[0=cv::StereoBM, 1=cv::StereoSGBM]
                # "OdomORBSLAM/ThDepth" : "40.0",
                "Optimizer/Strategy" : "0",           #[Graph optimization strategy: 0=TORO, 1=g2o, 2=GTSAM and 3=Ceres.]
                # "Reg/Strategy" : "0",                 #[0=Vis, 1=Icp, 2=VisIcp]
                "Vis/EstimationType" : "1",             #[Motion estimation approach: 0:3D->3D, 1:3D->2D (PnP), 2:2D->2D (Epipolar Geometry)]
                "Vis/CorFlowMaxLevel" : "5",
                "Vis/BundleAdjustment" : "1",           #[Optimization with bundle adjustment: 0=disabled, 1=g2o, 2=cvsba, 3=Ceres.]
                "Vis/FeatureType" : "1"                 #[0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB 9=KAZE 10=ORB-OCTREE 11=SuperPoint 12=SURF/FREAK 13=GFTT/DAISY 14=SURF/DAISY 15=PyDetector]
                }],
            remappings=[
                ("left/image_rect", "/carla/ego_vehicle/rgb_left/image"),
                ("right/image_rect", "/carla/ego_vehicle/rgb_right/image" ),
                ("left/camera_info", "/carla/ego_vehicle/rgb_left/camera_info"),
                ("right/camera_info", "/right/camera_info"),
                ("/odom", "/odom_stereo_01")
                ],
            arguments=["--delete_db_on_start"]
        )
    ]) 
