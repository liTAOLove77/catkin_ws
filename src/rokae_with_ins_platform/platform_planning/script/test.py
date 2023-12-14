import os
import time

import torch
from utils.data.camera_data import CameraData
import numpy as np
from inference.post_process import post_process_output
from utils.dataset_processing.grasp import detect_grasps
from utils.visualisation.plot import plot_grasp
from PIL import Image
import torchvision.transforms as transforms
import matplotlib.pyplot as plt

import rospy
# from sensor_msgs.msg import Image, CameraInfo
import sensor_msgs
from cv_bridge import CvBridge
import cv2

from plane_grasp import PlaneGraspClass


rospy.init_node("img_viewer", anonymous=True)
bridge = CvBridge()


# 定义保存路径和文件名的前缀
save_path = 'data/'
color_prefix = 'color_image_'
depth_prefix = 'depth_image_'
for i in range(10):
    # 获取深度图像
    color_image_msg = rospy.wait_for_message('/camera/color/image_raw', sensor_msgs.msg.Image)
    cv_color_image = bridge.imgmsg_to_cv2(color_image_msg, desired_encoding='bgr8')

    # 获取深度图像
    depth_image_msg = rospy.wait_for_message('/camera/depth/image_raw_left', sensor_msgs.msg.Image)
    cv_depth_image = bridge.imgmsg_to_cv2(depth_image_msg, desired_encoding='passthrough')
    # 保存彩色图像和深度图像
    color_filename = color_prefix + str(i) + '.jpg'
    depth_filename = depth_prefix + str(i) + '.png'
    cv2.imwrite(save_path + color_filename, cv_color_image)
    cv2.imwrite(save_path + depth_filename, cv_depth_image)

# cv_depth_image= np.expand_dims(cv_depth_image, axis=2)


# print(cv_color_image.shape)
# print(cv_depth_image.shape)

# visualize = False

# cam_data = CameraData(include_rgb=True,include_depth=True,output_size=300,width=1280,height=720)
# # x, depth_img, rgb_img = cam_data.get_data(rgb=cv_color_image, depth=cv_depth_image)
# # print(x.shape)
# # print(depth_img.shape)
# # print(rgb_img.shape)

# saved_model_path = 'trained-models/jacquard-rgbd-grconvnet3-drop0-ch32/epoch_48_iou_0.93'
# model = torch.load(saved_model_path)
# device = "cuda:0" 

# num =2 # change me num=[1:6],and you can see the result in '/result' file
# rgb_path = f"data/color_image_{num}.jpg"
# depth_path = f"data/depth_image_{num}.png"
# rgb = np.array(Image.open(rgb_path))
# depth = np.array(Image.open(depth_path)).astype(np.float32)
# depth = depth/1000
# depth= np.expand_dims(depth, axis=2)
# x, depth_img, rgb_img = cam_data.get_data(rgb=rgb, depth=depth)

# # rgb = cv2.cvtColor(rgb,cv2.COLOR_BGR2RGB)

# with torch.no_grad():
#     xc = x.to(device)
#     pred = model.predict(xc)
# q_img, ang_img, width_img = post_process_output(pred['pos'], pred['cos'], pred['sin'], pred['width'])

# grasps = detect_grasps(q_img, ang_img, width_img)
# if len(grasps) ==0:
#     print("Detect 0 grasp pose!")


# plot_grasp(fig=plt.figure(figsize=(6, 6)), rgb_img=cam_data.get_rgb(rgb, False), grasps=grasps, save=True)

num =2
rgb_path = f"data/color_image_{num}.jpg"
depth_path = f"data/depth_image_{num}.png"
rgb = np.array(Image.open(rgb_path))
depth = np.array(Image.open(depth_path)).astype(np.float32)

g = PlaneGraspClass(
        saved_model_path='trained-models/jacquard-rgbd-grconvnet3-drop0-ch32/epoch_48_iou_0.93',
        visualize=True,
        include_rgb=True
    )

g.generate(rgb, depth)
# cv2.imwrite("results/q_img.png",q_img)
# q_img = cv2.normalize(
#             q_img, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U
#         )
# print(q_img.dtype)
# q_img = cv2.applyColorMap(q_img, cv2.COLORMAP_RAINBOW)
# cv2.imwrite("results/q_img_normalize.png",q_img)

# cv2.imwrite("results/ang_img.png",ang_img)
# ang_img = cv2.normalize(
#             ang_img, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U
#         )
# ang_img = cv2.applyColorMap(ang_img, cv2.COLORMAP_RAINBOW)
# cv2.imwrite("results/ang_img_normalize.png",ang_img)

# cv2.imwrite("results/width_img.png",width_img)
# width_img = cv2.normalize(
#             width_img, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U
#         )
# width_img = cv2.applyColorMap(width_img, cv2.COLORMAP_RAINBOW)
# cv2.imwrite("results/width_img_normalize.png",width_img)