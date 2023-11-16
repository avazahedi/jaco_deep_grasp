#!/usr/bin/env python

import cv2
from ultralytics import YOLO
import pyrealsense2 as rs
import numpy as np
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
# import torch
# import rospkg

class ObjectDetection:
    def __init__(self):
        # realsense subscriber
        self.rgb_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_callback)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_callback)
        self.intr_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.intr_callback)
        self.timer = rospy.Timer(rospy.Duration(1), self.timer_callback)

        self.intr = None
        self.color_image = None
        self.depth_image = None

        self.bridge = CvBridge()
        self.model = YOLO('yolov8n-seg.pt')

        # training on fridge dataset takes a LONG time
        # self.model = YOLO('yolov8n.pt')
        # self.rospack = rospkg.RosPack()
        # self.rospack.get_path('jaco_grasp_ros')
        # model_path = self.rospack.get_path('jaco_grasp_ros') + "/fridge_detection_YOLO_dataset/data.yaml"
        # print(model_path)
        # self.results = self.model.train(data=model_path, epochs=100, imgsz=640)

        self.desired_object = "bottle"  # this will be able to be set by user

    def timer_callback(self):
        pass

    def rgb_callback(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg)
        self.color_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB) # convert BGR to RGB
        cv2.imshow("raw image", self.color_image)

        # self.results = self.model(image)   # same as self.model.predict(image)
        self.results = self.model.predict(self.color_image, conf=0.7, verbose=False, save_txt=False)
        annotated_img = self.results[0].plot()
        cv2.imshow("YOLOv8 Inference", annotated_img)   # same as show=True in self.model.predict

        # print(self.results[0].boxes.xywhn) # x_center, y_center, width, height (normalized by original image size)
        # print(self.results[0].boxes.xywh) # x_center, y_center, width, height

        # print(self.model.names)

        # for r in self.results:
        #     for c in r.boxes.cls:
        #         print(self.model.names[int(c)])

        # RealSense is 1280x720
        # check if the name of the class matches what we want to grasp
        for r in self.results:
            for c in r.boxes.cls:
                if (self.model.names[int(c)]) == self.desired_object:
                    xywh = r.boxes.xywh.tolist()[0]     # unpacking PyTorch Tensor
                    xcenter, ycenter, width, height = xywh

                    print(f"xcenter: {xcenter}")
                    print(f"ycenter: {ycenter}")

                    # either send xywh values to grasp_detection.cpp for processing,
                    # or process values before sending
                    xright = xcenter + width/2.0
                    xleft = xcenter - width/2.0
                    ytop = ycenter + height/2.0
                    ybot = ycenter - height/2.0

                    rs.rs2_deproject_pixel_to_point()

                    pen_coords_wrt_camera = rs.rs2_deproject_pixel_to_point(intr, [centroid_x, centroid_y], pen_depth)


        # press ESC or 'q' to close
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()


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
    obj_det = ObjectDetection()

    # initialize the object detection node
    rospy.init_node("object_detection", anonymous=True)
    rospy.spin()

if __name__ == '__main__': 
    try: 
        main() 
    except rospy.ROSInterruptException: 
        pass
