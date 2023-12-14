from plane_grasp import PlaneGraspClass
import time
import rospy
# from sensor_msgs.msg import Image, CameraInfo
import sensor_msgs
from cv_bridge import CvBridge
import cv2
import moveit_commander
import sys
from copy import deepcopy


# 定义路径规划函数
def plan_and_execute(waypoints):
    fraction = 0.0   #路径规划覆盖率
    maxtries = 100   #最大尝试规划次数
    attempts = 0     #已经尝试规划次数

    while fraction < 1.0 and attempts < maxtries:
                (plan, fraction) = rokae.compute_cartesian_path (
                                    waypoints,   # waypoint poses，路点列表
                                    0.01,        # eef_step，终端步进值
                                    0.0,         # jump_threshold，跳跃阈值
                                    True)        # avoid_collisions，避障规划  
                # 尝试次数累加
                attempts += 1
                    
                    # 打印运动规划进程
                if attempts % 10 == 0:
                    rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                            
                    # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
                if fraction == 1.0:
                    rospy.loginfo("Path computed successfully. Moving the arm.")
                    rokae.execute(plan)
                    rospy.loginfo("Path execution complete.")
                # 如果路径规划失败，则打印失败信息
                else:
                    rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  



if __name__ == '__main__':
    rospy.init_node("plane_grasp", anonymous=True)
    bridge = CvBridge()
    moveit_commander.roscpp_initialize(sys.argv)

    rokae = moveit_commander.MoveGroupCommander("rokae")
    rokae.set_pose_reference_frame("xMateCR7_base")
    inspire = moveit_commander.MoveGroupCommander("inspire")

    # 允许重新规划
    rokae.allow_replanning(True)

    # 设置位置和姿态的允许误差
    rokae.set_goal_position_tolerance(0.1)
    rokae.set_goal_orientation_tolerance(0.1)
    inspire.set_goal_position_tolerance(0.1)
    inspire.set_goal_orientation_tolerance(0.1)

    # 设置最大速度和加速度的缩放因子
    rokae.set_max_acceleration_scaling_factor(0.8)
    rokae.set_max_velocity_scaling_factor(0.8)
    inspire.set_max_acceleration_scaling_factor(0.8)
    inspire.set_max_velocity_scaling_factor(0.8)

    g = PlaneGraspClass(
        saved_model_path='trained-models/jacquard-rgbd-grconvnet3-drop0-ch32/epoch_48_iou_0.93',
        visualize=True,
        include_rgb=True
    )

    # 初始化路点列表
    waypoints = []
    
    while True:
        #清空列表
        waypoints.clear()
        # 回到home姿态，方便拍照
        rokae.set_named_target("home")
        rokae.go(wait=True)

        # 获取深度图像
        color_image_msg = rospy.wait_for_message('/camera/color/image_raw', sensor_msgs.msg.Image)
        cv_color_image = bridge.imgmsg_to_cv2(color_image_msg, desired_encoding='bgr8')

        # 获取深度图像
        depth_image_msg = rospy.wait_for_message('/camera/depth/image_raw_left', sensor_msgs.msg.Image)
        cv_depth_image = bridge.imgmsg_to_cv2(depth_image_msg, desired_encoding='passthrough')

        #运动到抓取初始点
        rokae.set_named_target("demo_point")
        rokae.go(wait=True)
        demo_point_pose = rokae.get_current_pose().pose

        grasp_pose, grasps_length=g.generate(cv_color_image, cv_depth_image)

        demo_point_pose.position.x = grasp_pose[0]
        demo_point_pose.position.y = grasp_pose[1]
        demo_point_pose.position.z = grasp_pose[2]+0.20
        waypoints.append(deepcopy(demo_point_pose))

        plan_and_execute(waypoints)
        
        time.sleep(4)