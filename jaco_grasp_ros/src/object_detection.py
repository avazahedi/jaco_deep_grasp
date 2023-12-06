#!/usr/bin/env python

import cv2
from ultralytics import YOLO
import pyrealsense2 as rs
import numpy as np
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from jaco_grasp_ros_interfaces.msg import BboxCoords
# import torch
# import rospkg

class ObjectDetection:
    def __init__(self):
        # realsense subscribers
        self.rgb_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_callback)
        # self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_callback)
        self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback)
        self.intr_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.intr_callback)

        # passthrough filter publisher
        self.passthrough_pub = rospy.Publisher("/passthrough_filter_vals", BboxCoords, queue_size=10)
        self.passthrough_vals = BboxCoords()

        self.intr = None
        self.color_image = None
        self.depth_image = None

        self.bridge = CvBridge()
        self.model = YOLO('yolov8n-seg.pt')

        # bottle, cup, bowl, etc.
        # self.desired_object = "bottle"  # this will be able to be set by user
        self.desired_object = rospy.get_param("/desired_object")    # defaults to "bottle"

        # print(self.model.names)

        self.frequency = 30.0
        self.timer = rospy.Timer(rospy.Duration(1.0/self.frequency), self.timer_callback)

    def timer_callback(self, event=None):
        if self.intr is None or self.color_image is None or self.depth_image is None:
            return

        self.results = self.model.predict(self.color_image, conf=0.7, verbose=False, save_txt=False)
        annotated_img = self.results[0].plot()
        cv2.imshow("YOLOv8 Inference", annotated_img)   # same as show=True in self.model.predict

        # print(self.results[0].boxes.xywhn) # x_center, y_center, width, height (normalized by original image size)
        # print(self.results[0].boxes.xywh) # x_center, y_center, width, height

        # for r in self.results:
        #     for c in r.boxes.cls:
        #         print(self.model.names[int(c)])

        # RealSense is 1280x720
        # check if the name of the class matches what we want to grasp
        for r in self.results:
            for c in r.boxes.cls:
                if (self.model.names[int(c)]) == self.desired_object:
                    xywh = r.boxes.xywh.tolist()[0]     # unpacking PyTorch Tensor
                    # xcenter, ycenter, width, height = xywh
                    xcenter, ycenter, width, height = [int(i) for i in xywh]

                    # print(f"xcenter: {xcenter}")
                    # print(f"ycenter: {ycenter}")
                    # print(f"width: {width}")
                    # print(f"height: {height}")

                    # print(f"self.intr.width: {self.intr.width}")
                    # print(f"self.intr.height: {self.intr.height}")

                    # either send xywh values to grasp_detection.cpp for processing,
                    # or process values before sending

                    # pixel values to be deprojected [xleft, xright, ytop, ybottom]
                    xleft = int(xcenter - width/2)
                    xright = int(xcenter + width/2)
                    ytop = int(ycenter + height/2)
                    ybottom = int(ycenter - height/2)

                    # print(f"xright: {xright}")
                    # print(f"xleft: {xleft}")
                    # print(f"ytop: {ytop}")
                    # print(f"ybottom: {ybottom}")

                    # use depth of center of bounding box
                    depth = self.depth_image[int(ycenter)][int(xcenter)]

                    if depth == 0:
                        return

                    # convert pixels to points
                    # each point is a list [x, y, z] coordinates
                    left = rs.rs2_deproject_pixel_to_point(self.intr, [xleft, ycenter], depth)
                    right = rs.rs2_deproject_pixel_to_point(self.intr, [xright, ycenter], depth)
                    top = rs.rs2_deproject_pixel_to_point(self.intr, [xcenter, ytop], depth)
                    bottom = rs.rs2_deproject_pixel_to_point(self.intr, [xcenter, ybottom], depth)

                    # convert to m
                    self.passthrough_vals.left = [i/1000. for i in left]
                    self.passthrough_vals.right = [i/1000. for i in right]
                    self.passthrough_vals.top = [i/1000. for i in top]
                    self.passthrough_vals.bottom = [i/1000. for i in bottom]
                    self.passthrough_vals.center_depth = depth/1000.
                    self.passthrough_pub.publish(self.passthrough_vals)

        # press ESC or 'q' to closearray
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()


    def rgb_callback(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg)
        self.color_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB) # convert BGR to RGB
        # cv2.imshow("raw image", self.color_image)


    def depth_callback(self, msg):
        # Get depth image
        self.depth_image = self.bridge.imgmsg_to_cv2(msg)

    def intr_callback(self, msg):
        # Set camera intrinsics
        self.intr = rs.intrinsics()
        self.intr.height = msg.height
        self.intr.width = msg.width
        self.intr.fx = msg.K[0]
        self.intr.fy = msg.K[4]
        self.intr.ppx = msg.K[2]
        self.intr.ppy = msg.K[5]

        # Set distortion model and coefficients
        if msg.distortion_model == 'plumb_bob':
            self.intr.model = rs.distortion.brown_conrady
        elif msg.distortion_model == 'equidistant':
            self.intr.model = rs.distortion.kannala_brandt4

        self.intr.coeffs = [coeff for coeff in msg.D]


def main():
    # initialize the object detection node
    rospy.init_node("object_detection", anonymous=True)

    obj_det = ObjectDetection()

    rospy.spin()

if __name__ == '__main__': 
    try: 
        main() 
    except rospy.ROSInterruptException: 
        pass
