#!/usr/bin/env python3
from __future__ import division
import roslib
import sys
import rospy

import numpy as np
import math
import os
#import pandas as pd
from spark_carry_object.msg import *
from std_msgs.msg import String
from sensor_msgs.msg import Image
from swiftpro.msg import *
from cv_bridge import CvBridge, CvBridgeError
from sklearn.linear_model import LinearRegression
import argparse
import logging
import time
import threading
import cv2
import torch
from tf_pose.estimator import TfPoseEstimator
from tf_pose.networks import get_graph_path, model_wh
from tf_pose.gesture_pose import Poseclassification

logger = logging.getLogger('TfPoseEstimator-WebCam')
logger.setLevel(logging.DEBUG)
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)
formatter = logging.Formatter('[%(asctime)s] [%(name)s] [%(levelname)s] %(message)s')
ch.setFormatter(formatter)
logger.addHandler(ch)

def str2bool(v):
    return v.lower() in ("yes", "true", "t", "1")


def image_callback_swap(data):
	global pub_new
	global finish_bit
	#image = CvBridge().imgmsg_to_cv2(data, "bgr8")
	if(finish_bit):
		finish_bit = 0
		pub_new.publish(data)
		
def image_callback(data):
	global xc, yc
	global w, h
	global gesture_net
	global ee
	global fps_time
	global pub_arm
	global pos
	global first_frame
	global finish_bit
	if(first_frame == 1):
		finish_bit = 1
		first_frame = 0
		pos = position()
		pub_arm = rospy.Publisher('position_write_topic', position, queue_size=5)
		pos.x = 180
		pos.y = 0
		pos.z = -20
		pub_arm.publish(pos)
		w, h = model_wh("432x368")
		gesture_net = Poseclassification
		pose_classifie_path = '/home/spark/spark_noetic/src/spark_app/spark_tf_arm_move/nodes/models/pose_classifier.pth'
		gesture_net = torch.load(pose_classifie_path, map_location=torch.device('cpu'))
		gesture_net.eval()
		if w > 0 and h > 0:
			ee = TfPoseEstimator(get_graph_path("mobilenet_thin"), target_size=(w, h), trt_bool=str2bool("False"))
		else:
			ee = TfPoseEstimator(get_graph_path("mobilenet_thin"), target_size=(640, 480),
								trt_bool=str2bool("False"))
		logger.debug('cam read+')
		fps_time = 0
	# change to opencv
	try:
		image = CvBridge().imgmsg_to_cv2(data, "bgr8")
		logger.debug('image process+')
		humans = ee.inference(image, resize_to_default=(w > 0 and h > 0), upsample_size=4.0)
		logger.debug('postprocess+')

		image, label, angle = TfPoseEstimator.draw_humans(image, gesture_net, humans, imgcopy=False)
		if (label == "left"):
			pos.x = 180
			pos.y = -200
			pos.z = -20
			pub_arm.publish(pos)

		elif (label == "right"):
			pos.x = 180
			pos.y = 200
			pos.z = -20
			pub_arm.publish(pos)
		logger.debug('show+')
		cv2.putText(image,
					"FPS: %f" % (1.0 / (time.time() - fps_time)),
					(10, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
					(0, 255, 0), 2)
		cv2.imshow('tf-pose-estimation result', image)
		fps_time = time.time()
		finish_bit = 1
		if cv2.waitKey(1) == 27:
			return
		logger.debug('finished+')
	except CvBridgeError as e:
		print(e)

def main():
	global first_frame
	global pub_new
	global finish_bit
	finish_bit = 1
	first_frame = 1
	rospy.init_node('image_converter', anonymous=True)
	pub_new = rospy.Publisher("/image_raw_new", Image, queue_size=1)
	sub1 = rospy.Subscriber("/camera/color/image_raw", Image, image_callback_swap, queue_size=1)
	sub2 = rospy.Subscriber("/image_raw_new", Image, image_callback, queue_size=1)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main()
