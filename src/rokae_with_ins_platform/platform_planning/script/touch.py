import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import tf
import numpy as np
import tf.transformations as tf_trans
import threading
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from copy import deepcopy
try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import threading


# 全局变量
cv_color_image = None
cv_depth_image = None
cam_intrinsics = None
camera2robot = None

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

click_point_pix = ()
def mouseclick_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        global click_point_pix, waypoints

        click_point_pix = (x,y)

        # print(click_point_pix)

        # Get click point in camera coordinates
        # 获取深度值 depth.get_value(x, y)返回的是一个元组，元组的第0位是状态，第1位才是数值
        click_z = cv_depth_image[y, x]
        # print(f"click_z:{click_z }")
        # print("cam_intrinsics:")
        # print(cam_intrinsics)

        click_x = np.multiply(x-cam_intrinsics[0][2],click_z/cam_intrinsics[0][0])
        click_y = np.multiply(y-cam_intrinsics[1][2],click_z/cam_intrinsics[1][1])
        if click_z == 0:
            return
        click_point = np.asarray([click_x,click_y,click_z])
        click_point.shape = (3,1)
        # 图片中被点击的点在相机坐标系下的X,Y,Z为
        click_point = click_point/1000
        print("图片中被点击的点在相机坐标系下的X,Y,Z为:")
        print(click_point)
        
        # print(camera2robot)
        target_position = np.dot(camera2robot[0:3,0:3],click_point) + camera2robot[0:3,3:]
        print("图片中被点击的点在机械臂base坐标系下的X,Y,Z:")
        print(target_position)

        # 初始化路点列表
        waypoints = []
        demo_point_pose.position.x = target_position[0][0]
        demo_point_pose.position.y = target_position[1][0]
        demo_point_pose.position.z = target_position[2][0]+0.20
        waypoints.append(deepcopy(demo_point_pose))

        # 在回调函数中启动新的线程进行路径规划和机械臂控制
        thread = threading.Thread(target=plan_and_execute)
        thread.start()


        
# 定义路径规划函数
def plan_and_execute():
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


def main():
    global cam_intrinsics
    global camera2robot
    global cv_depth_image # 声明为全局变量
    global cv_color_image
    global rokae
    global demo_point_pose

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("img_viewer", anonymous=True)

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

    # 设置目标为"home"
    rokae.set_named_target("demo_point")

    # 运动到目标位置
    rokae.go(wait=True)

    demo_point_pose = rokae.get_current_pose().pose
    print(demo_point_pose)


    bridge = CvBridge()
    # 创建tf监听器
    listener = tf.TransformListener()
    # 等待tf变换的可用性
    listener.waitForTransform('/xMateCR7_base','/camera_depth_optical_frame', rospy.Time(), rospy.Duration(4.0))

    # 获取相机坐标系到机械臂基底坐标系的转换矩阵
    try:
        # 获取最新的相机坐标系到机械臂基底坐标系的转换
        (trans, rot) = listener.lookupTransform('/xMateCR7_base','/camera_depth_optical_frame',rospy.Time(0))

        # 将平移和旋转转换为变换矩阵
        transform_matrix = tf_trans.concatenate_matrices(tf_trans.translation_matrix(trans), tf_trans.quaternion_matrix(rot))

        # 打印转换矩阵
        print("相机坐标系到机械臂基底坐标系的转换矩阵:")
        print(transform_matrix)

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr("无法获取变换矩阵。{}".format(str(e)))
    
    camera2robot = transform_matrix
    #保存相机坐标系到基底坐标系的变换矩阵
    np.savetxt('camera2base_matrix.txt', camera2robot, delimiter=' ')

    
     # 获取相机信息
    camera_info_msg = rospy.wait_for_message('/camera/color/camera_info', CameraInfo)

    # 获取相机内参矩阵
    cam_intrinsics = np.reshape(camera_info_msg.K, (3, 3))
    #保存相机的内参矩阵
    np.savetxt('internal_reference_matrix.txt', cam_intrinsics, delimiter=' ')


    # 打印相机内参矩阵
    print("相机内参矩阵:")
    print(cam_intrinsics)

    cv2.namedWindow('color')
    cv2.setMouseCallback('color', mouseclick_callback)
    cv2.namedWindow('depth')

    while True:

        # 获取深度图像
        color_image_msg = rospy.wait_for_message('/camera/color/image_raw', Image)
        cv_color_image = bridge.imgmsg_to_cv2(color_image_msg, desired_encoding='bgr8')

        # 获取深度图像
        depth_image_msg = rospy.wait_for_message('/camera/depth/image_raw_left', Image)
        cv_depth_image = bridge.imgmsg_to_cv2(depth_image_msg, desired_encoding='passthrough')

        # 调整深度图像尺寸以匹配彩色图像
        # cv_depth_image = cv2.resize(cv_depth_image, (cv_color_image.shape[1], cv_color_image.shape[0]))

        if len(click_point_pix) != 0:
            cv_color_image = cv2.circle(cv_color_image, click_point_pix, 7, (0,0,255), 2)

        cv2.imshow('color', cv_color_image )
        # 归一化深度图像到0-255范围
        normalized_depth = cv2.normalize(
            cv_depth_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U
        )
        cv2.imshow('depth', normalized_depth)


        
        if cv2.waitKey(1) == ord('c'):
            break

    cv2.destroyAllWindows()
    # 关闭MoveIt
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()