#!/usr/bin/env python
## Author: Rohit
## Date: July, 25, 2017
# Purpose: Ros node to detect objects using tensorflow

import os
import sys
import cv2
import numpy as np
try:
    import tensorflow as tf
except ImportError:
    print("unable to import TensorFlow. Is it installed?")
    print("  sudo apt install python-pip")
    print("  sudo pip install tensorflow")
    sys.exit(1)

# ROS related imports
import rospy
import rospkg
from std_msgs.msg import String , Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

# Object detection module imports
import object_detection
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

# SET FRACTION OF GPU YOU WANT TO USE HERE
# 在此处设置您想要使用的 GPU 的比例。
GPU_FRACTION = 0.4

# get an instance of RosPack with the default search paths
# 获取一个带有默认搜索路径的 RosPack 实例
rospack = rospkg.RosPack()
# list all packages, equivalent to rospack list
# 列出所有包，相当于 rospack list 命令的功能。
rospack.list() 
# get the file path for tensorflow_object_detector
# 获取用于 tensorflow_object_detector 的文件路径
PACKAGE_PATH = os.path.join(rospack.get_path('tensorflow_object_detector'))

######### Set model here ############
MODEL_NAME =  'ssd_mobilenet_v1_coco_11_06_2017'
# By default models are stored in data/models/
MODEL_PATH = os.path.join(PACKAGE_PATH,'data','models' , MODEL_NAME)
# Path to frozen detection graph. This is the actual model that is used for the object detection.
PATH_TO_CKPT = MODEL_PATH + '/frozen_inference_graph.pb'
######### Set the label map file here ###########
LABEL_NAME = 'mscoco_label_map.pbtxt'
# By default label maps are stored in data/labels/
PATH_TO_LABELS = os.path.join(PACKAGE_PATH,'data','labels', LABEL_NAME)
######### Set the number of classes here #########
NUM_CLASSES = 90


# 加载tensorflow计算图
detection_graph = tf.compat.v1.Graph()
with detection_graph.as_default():
    od_graph_def = tf.compat.v1.GraphDef()
    with tf.io.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.compat.v1.import_graph_def(od_graph_def, name='')

## Loading label map
# Label maps map indices to category names, so that when our convolution network predicts `5`,
# we know that this corresponds to `airplane`.  Here we use internal utility functions,。
# but anything that returns a dictionary mapping integers to appropriate string labels would be fine
# 标签映射将索引与类别名称进行关联，因此当我们的卷积网络预测出“5”时，
# 我们知道这与“飞机”相对应。在这里，我们使用了内部的辅助函数
# 但任何能返回一个将整数映射到相应字符串标签的字典的数据结构都是可以接受的。
label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)

# Setting the GPU options to use fraction of gpu that has been set
# 将 GPU 选项设置为使用已设定的 GPU 的一部分资源。
config = tf.compat.v1.ConfigProto()
config.gpu_options.per_process_gpu_memory_fraction = GPU_FRACTION

# Detection

class Detector:

    def __init__(self):
        self.image_pub = rospy.Publisher("debug_image",Image, queue_size=1)
        self.object_pub = rospy.Publisher("objects", Detection2DArray, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("image", Image, self.image_cb, queue_size=1, buff_size=2**24)
        # 绑定session到指定的计算图
        self.sess = tf.compat.v1.Session(graph=detection_graph,config=config)

    # 图片订阅的回调函数
    def image_cb(self, data):

        # 创建容器，装预测结果
        objArray = Detection2DArray()
        try:
            # ROS -> OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
        except CvBridgeError as e:
            print(e)
        # BGR -> RGB tf模型通常需要RGB
        image=cv2.cvtColor(cv_image,cv2.COLOR_BGR2RGB)

        # the array based representation of the image will be used later in order to prepare the
        # result image with boxes and labels on it.
        # 之后将会使用基于数组的图像表示形式，以便生成带有方框和标签的结果图像。
        image_np = np.asarray(image)
        # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
        # 扩展维度，因为该模型期望图像的形状为：[1, H, W, 3]
        image_np_expanded = np.expand_dims(image_np, axis=0)

        # 取计算图中的 Tensor
        image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
        # Each box represents a part of the image where a particular object was detected.
        # 每个方框代表图像中某一特定物体被检测到的那部分区域。
        boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
        # Each score represent how level of confidence for each of the objects.
        # 每个分数都代表了对每个对象的置信程度。
        # Score is shown on the result image, together with the class label.
        # 得分会显示在结果图像上，同时还会显示类别标签。
        scores = detection_graph.get_tensor_by_name('detection_scores:0')
        classes = detection_graph.get_tensor_by_name('detection_classes:0')
        num_detections = detection_graph.get_tensor_by_name('num_detections:0')

        # 推理
        (boxes, scores, classes, num_detections) = self.sess.run([boxes, scores, classes, num_detections],
            feed_dict={image_tensor: image_np_expanded})

        # 可视化 + 整理objects
        objects=vis_util.visualize_boxes_and_labels_on_image_array(
            image,
            np.squeeze(boxes),
            np.squeeze(classes).astype(np.int32),
            np.squeeze(scores),
            category_index,
            use_normalized_coordinates=True,
            line_thickness=10
            )

        # 构建Detection2DArray
        objArray.detections =[]
        objArray.header=data.header
        object_count=1

        # 逐个生成Detection2D
        for i in range(len(objects)):
            object_count+=1
            objArray.detections.append(self.object_predict(objects[i],data.header,image_np,cv_image))

        # 发布检测结果
        self.object_pub.publish(objArray)
        img=cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
        image_out = Image()
        try:
            image_out = self.bridge.cv2_to_imgmsg(img,"bgr8")
        except CvBridgeError as e:
            print(e)
        image_out.header = data.header
        
        # 发布调试图像
        self.image_pub.publish(image_out)

    # 模型输出 -> ROS Detection2D
    def object_predict(self,object_data, header, image_np,image):
        image_height,image_width,channels = image.shape
        # 创建消息
        obj=Detection2D()
        obj_hypothesis= ObjectHypothesisWithPose()

        # 拆数据
        object_id=object_data[0]
        object_score=object_data[1]
        dimensions=object_data[2]

        # 填充类别和置信度
        obj.header=header
        obj_hypothesis.id = object_id
        obj_hypothesis.score = object_score
        obj.results.append(obj_hypothesis)
        # 计算像素级 bbox
        obj.bbox.size_y = int((dimensions[2]-dimensions[0])*image_height)
        obj.bbox.size_x = int((dimensions[3]-dimensions[1] )*image_width)
        # 中心点
        obj.bbox.center.x = int((dimensions[1] + dimensions [3])*image_width/2)
        obj.bbox.center.y = int((dimensions[0] + dimensions[2])*image_height/2)

        return obj

def main(args):
    rospy.init_node('detector_node')
    obj=Detector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("ShutDown")
    cv2.destroyAllWindows()

if __name__=='__main__':
    main(sys.argv)
