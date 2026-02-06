#
# Author: Li Zhenxuan
# Date: 2026/2/4
# Purpose: Ros node to detect objects using YOLO model


import os
import sys
import threading
import cv2
import numpy as np

# ROS related imports
import rospy
import rospkg
from std_msgs.msg import String , Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

# param service in ROS1
from dynamic_reconfigure.server import Server
from your_pkg.cfg import DetectorConfig

# import YOLO
import torch
from ultralytics import YOLO

# set as param
object_classes = ["bear", "clock","glass"]
object_classes_color = [(255, 0, 0), (0, 255, 0), (0, 0, 255)] # BGR the cv2 order

def draw_detections(image, detections):
    debug_image = image.copy()

    if not detections:
        return debug_image

    # make debug_image
    for det in detections:
        x1, y1, x2, y2 = map(int, det["bbox"])
        cls_id = det["cls_id"]
        if cls_id >= len(object_classes):
            continue
        score = det["score"]

        # bbox
        cv2.rectangle(
            debug_image,
            (x1, y1),
            (x2, y2),
            object_classes_color[cls_id % len(object_classes_color)],
            2
        )

        label = f"{object_classes[cls_id % len(object_classes_color)]}:{score:.2f}"
        (tw, th), _ = cv2.getTextSize(
            label,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            1
        )
        y_text = max(th + 4, y1)

        # label background
        cv2.rectangle(
            debug_image,
            (x1, y_text - th - 4),
            (x1 + tw + 2, y_text),
            object_classes_color[cls_id % len(object_classes_color)],
            -1
        )

        # draw label text
        cv2.putText(
            debug_image,
            label,
            (x1 + 1, y_text - 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
            cv2.LINE_AA
        )

    return debug_image


class Detector:
    def __init__(self):
        # set YOLO
        self.model_path = "yolo26n.pt"
        self.model = YOLO(self.model_path)

        # default values
        self.is_draw_debug_image = True
        self.iou = 0.45
        self.conf_thres = 0.25

        # dynamic reconfigure server
        self.cfg_srv = Server(DetectorConfig, self.reconfig_cb)

        self.image_sub = rospy.Subscriber("image", Image, self.image_cb, queue_size=1, buff_size=2 ** 24)
        self.bridge = CvBridge()
        self.object_pub = rospy.Publisher("objects", Detection2DArray, queue_size=1)
        self.image_pub = rospy.Publisher("debug_image", Image, queue_size=1)

    # param service
    def reconfig_cb(self, config, level):
        if config.model_path != self.model_path:
            try:
                self.load_model(config.model_path)
                rospy.loginfo("YOLO model switched successfully")
            except Exception as e:
                rospy.logerr(f"Failed to load model: {e}")
                # Rollback parameters 
                # (to prevent the GUI from showing that the switch has been made when it actually hasn't succeeded)
                config.model_path = self.model_path
        
        rospy.loginfo(
            f"[reconfig] draw={config.draw}, "
            f"iou={config.iou}, conf={config.conf_thres}"
        )
        self.draw = config.draw
        self.iou = config.iou
        self.conf_thres = config.conf_thres

        return config

    def load_model(self, model_path):
        rospy.loginfo(f"[Detector] Loading YOLO model: {model_path}")
        self.model = YOLO(model_path)
        self.model_path = model_path

        threading.Thread(
            target=self.warmup,
            daemon=True
            ).start()
    
    def warmup(self, img_shape=(640, 480)):
        rospy.loginfo("[Detector] YOLO warmup...")
        dummy = np.zeros((img_shape[1], img_shape[0], 3), dtype=np.uint8)
        with torch.no_grad():
            _ = self.model.predict(
                dummy,
                verbose=False
            )
        rospy.loginfo("[Detector] YOLO warmup done")

    def object_detect(self, image):
        # detect
        results = self.model.predict(
            image, 
            iou=self.iou, 
            conf=self.conf_thres, 
            verbose=False
            )
        # one result, one image
        boxes = results[0].boxes

        # detection product
        detections = []
        debug_image = None

        if boxes is None or len(boxes) == 0:
            return detections, debug_image

        # tensor -> scalar
        xyxy = boxes.xyxy.cpu().numpy()
        cls = boxes.cls.cpu().numpy()
        conf = boxes.conf.cpu().numpy()
        for i in range(len(xyxy)):
            x1, y1, x2, y2 = xyxy[i]
            cls_id = int(cls[i])
            score = float(conf[i])
            # if use value in boxes straightly, just use boxes.cls[i].item(), 
            # which means get the value from scalar tensor, safe for next step
            # but this cls has been translated into a list, not a tensor
            detections.append({
                "bbox": (x1, y1, x2, y2),
                "cls_id": cls_id,
                "score": score,
            })

        if self.is_draw_debug_image:
            # ------ draw debug_image ------
            debug_image = image.copy()
            debug_image = draw_detections(debug_image, detections)

        return detections, debug_image

    def image_cb(self, data):
        # make msg
        objArray = Detection2DArray()
        objArray.detections = []
        objArray.header = data.header
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        detections, draw_image_result = self.object_detect(cv_image)

        if detections:
            for det in detections:
                obj = Detection2D()
                obj.header = data.header

                # ------ bbox ------
                # yolo box: [x_min, y_min, x_max, y_max]
                x1,y1,x2,y2 = det["bbox"]
                obj.bbox.center.x = (x1 + x2) / 2
                obj.bbox.center.y = (y1 + y2) / 2
                obj.bbox.size_x = float(x2 - x1)
                obj.bbox.size_y = float(y2 - y1)
                obj.bbox.center.angle = 0

                # ------ class & confidence ------
                hyp = ObjectHypothesisWithPose()
                hyp.id = str(det["cls_id"])  # must use string
                hyp.score = float(det["score"])

                obj.results.append(hyp)
                objArray.detections.append(obj)

        self.object_pub.publish(objArray)
        
        if self.is_draw_debug_image:
            debug_image_out = Image()
            debug_image_out.header = data.header
            try:
                debug_image_out = self.bridge.cv2_to_imgmsg(draw_image_result, "bgr8")
                debug_image_out.header = data.header
            except CvBridgeError as e:
                rospy.logwarn("cv2_to_imgmsg failed, publish empty Image")
                debug_image_out.height = 0
                debug_image_out.width = 0
                debug_image_out.encoding = ""
                debug_image_out.data = b""
                debug_image_out.step = 0
            self.image_pub.publish(debug_image_out)

def main(args):
    rospy.init_node('detector_node')
    det=Detector()
    # warnup yolo model predict
    threading.Thread(target=det.warmup, daemon=True).start()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("ShutDown")

if __name__=='__main__':
    main(sys.argv)