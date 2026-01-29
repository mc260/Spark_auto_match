#!/usr/bin/env python3
"""
teleop_opencv.py

同时进行键盘控制（发布 `/cmd_vel`）和使用 OpenCV 自动录视频的 ROS 节点。

用法示例:
  rosrun spark_teleop teleop_opencv.py --camera 0 --linear 0.2 --angular 0.6 --out /tmp/robot_$(date +%s).avi

依赖:
  sudo apt install python3-opencv (或 pip3 install opencv-python)
  ROS (rospy, geometry_msgs)

脚本说明:
  - 使用 termios + select 非阻塞读取键盘（WASD 控制）并发布 geometry_msgs/Twist 到 `/cmd_vel`。
  - 使用单独线程从相机采集并写入视频文件，文件名可通过参数指定。
  - 优雅处理退出，释放摄像头和恢复终端状态。
"""
from __future__ import print_function
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import threading
import cv2
import time
from datetime import datetime
import argparse
import sys
import select
import termios
import tty
import os
import signal


class KeyboardTeleop(object):
    def __init__(self, linear=0.2, angular=0.6, topic='/cmd_vel'):
        self.pub = rospy.Publisher(topic, Twist, queue_size=1)
        self.linear = linear
        self.angular = angular
        self.keep_running = True
        self._old_attrs = None

    def _get_char(self, timeout=0.1):
        # 非阻塞读取单个字符，返回 '' 如果超时
        dr, dw, de = select.select([sys.stdin], [], [], timeout)
        if dr:
            return sys.stdin.read(1)
        return ''

    def _setup_terminal(self):
        if sys.stdin.isatty():
            self._old_attrs = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())

    def _restore_terminal(self):
        if self._old_attrs is not None:
            termios.tcsetattr(sys.stdin, termios.TCSANOW, self._old_attrs)

    def run(self):
        self._setup_terminal()
        try:
            twist = Twist()
            print("控制说明: W/S 前后, A/D 左右转, Q/E 斜前斜转, 空格 停止, CTRL-C 退出")
            while not rospy.is_shutdown() and self.keep_running:
                c = self._get_char(timeout=0.1)
                if not c:
                    continue
                c = c.lower()
                if c == 'w':
                    twist.linear.x = self.linear
                    twist.angular.z = 0.0
                elif c == 's':
                    twist.linear.x = -self.linear
                    twist.angular.z = 0.0
                elif c == 'a':
                    twist.linear.x = 0.0
                    twist.angular.z = self.angular
                elif c == 'd':
                    twist.linear.x = 0.0
                    twist.angular.z = -self.angular
                elif c == 'q':
                    twist.linear.x = self.linear
                    twist.angular.z = self.angular
                elif c == 'e':
                    twist.linear.x = self.linear
                    twist.angular.z = -self.angular
                elif c == ' ':
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                elif ord(c) == 3:  # Ctrl-C
                    rospy.signal_shutdown('user')
                    break
                else:
                    # 未识别键保持当前
                    continue
                self.pub.publish(twist)
        finally:
            # 退出时发布停止
            try:
                stop = Twist()
                self.pub.publish(stop)
            except Exception:
                pass
            self._restore_terminal()


class CameraRecorder(threading.Thread):
    def __init__(self, camera_index=0, out_dir=None, mode='video', fps=20.0, interval=1.0, image_topic='/camera/color/image_raw'):
        super(CameraRecorder, self).__init__()
        self.camera_index = camera_index
        self.fps = fps
        self.mode = mode  # 'video' or 'images'
        self.interval = interval
        self.out_dir = out_dir
        self.keep_running = True
        self.writer = None
        self.image_index = 1
        self.bridge = CvBridge()
        self.last_frame = None
        self.frame_lock = threading.Lock()
        self.image_topic = image_topic
        self.sub = None
        self._first_frame_event = threading.Event()

    def stop(self):
        self.keep_running = False
        try:
            if self.sub:
                self.sub.unregister()
        except Exception:
            pass

    def _ensure_out_dir(self):
        if not self.out_dir:
            date = datetime.now().strftime('%Y%m%d')
            self.out_dir = os.path.join('/tmp', 'Camera_{}'.format(date))
        os.makedirs(self.out_dir, exist_ok=True)

    def _image_cb(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            rospy.logerr('cv_bridge error: %s' % str(e))
            return
        with self.frame_lock:
            self.last_frame = cv_image
        self._first_frame_event.set()

    def run(self):
        self._ensure_out_dir()

        # Subscribe to image topic
        try:
            self.sub = rospy.Subscriber(self.image_topic, Image, self._image_cb, queue_size=1, buff_size=2**24)
        except Exception as e:
            rospy.logerr('订阅图像话题失败: %s' % str(e))
            return

        # 等待第一帧
        if not self._first_frame_event.wait(timeout=5.0):
            rospy.logerr('未收到图像话题的第一帧: %s' % self.image_topic)
            return

        # 获取第一帧的尺寸
        with self.frame_lock:
            frame = self.last_frame.copy() if self.last_frame is not None else None

        if frame is None:
            rospy.logerr('收到空帧，退出')
            return

        height, width = frame.shape[:2]

        if self.mode == 'video':
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            timestamp = int(time.time())
            video_name = 'video_{}.avi'.format(timestamp)
            out_path = os.path.join(self.out_dir, video_name)
            self.writer = cv2.VideoWriter(out_path, fourcc, self.fps, (width, height))
            if not self.writer.isOpened():
                rospy.logerr('打开视频写入器失败: {}'.format(out_path))
                return

            rospy.loginfo('开始录制视频: {} ({}x{} @ {}fps)'.format(out_path, width, height, self.fps))
            interval = 1.0 / max(self.fps, 1.0)
            while not rospy.is_shutdown() and self.keep_running:
                with self.frame_lock:
                    f = None if self.last_frame is None else self.last_frame.copy()
                if f is None:
                    rospy.logwarn_throttle(5, '暂无图像帧')
                    time.sleep(0.05)
                    continue
                self.writer.write(f)
                time.sleep(interval)

            rospy.loginfo('停止录制: {}'.format(out_path))
            try:
                self.writer.release()
            except Exception:
                pass

        elif self.mode == 'images':
            rospy.loginfo('开始按间隔 {:.2f}s 保存图片到: {}'.format(self.interval, self.out_dir))
            existing = [f for f in os.listdir(self.out_dir) if f.lower().endswith(('.jpg', '.jpeg', '.png'))]
            if existing:
                nums = []
                for f in existing:
                    name = os.path.splitext(f)[0]
                    try:
                        nums.append(int(name))
                    except Exception:
                        pass
                if nums:
                    self.image_index = max(nums) + 1

            while not rospy.is_shutdown() and self.keep_running:
                with self.frame_lock:
                    f = None if self.last_frame is None else self.last_frame.copy()
                if f is None:
                    rospy.logwarn_throttle(5, '暂无图像帧')
                    time.sleep(0.1)
                    continue
                filename = os.path.join(self.out_dir, '{:06d}.jpg'.format(self.image_index))
                cv2.imwrite(filename, f)
                rospy.loginfo_throttle(5, '保存图片: {}'.format(filename))
                self.image_index += 1
                time.sleep(self.interval)

        try:
            if self.sub:
                self.sub.unregister()
        except Exception:
            pass


def main():
    parser = argparse.ArgumentParser(description='Keyboard teleop + OpenCV recorder')
    parser.add_argument('--camera', type=int, default=0, help='camera index (default 0)')
    parser.add_argument('--out', type=str, default=None, help='output directory (default /tmp/Camera_YYYYMMDD)')
    parser.add_argument('--mode', type=str, choices=['video', 'images'], default='video', help='capture mode: video or images')
    parser.add_argument('--fps', type=float, default=20.0, help='video fps (only for video mode)')
    parser.add_argument('--interval', type=float, default=1.0, help='image interval seconds (only for images mode)')
    parser.add_argument('--linear', type=float, default=0.2, help='linear speed')
    parser.add_argument('--angular', type=float, default=0.6, help='angular speed')
    args, unknown = parser.parse_known_args()

    rospy.init_node('teleop_opencv', anonymous=False)

    # 摄像头线程
    cam = CameraRecorder(camera_index=args.camera, out_dir=args.out, mode=args.mode, fps=args.fps, interval=args.interval)
    cam.start()

    teleop = KeyboardTeleop(linear=args.linear, angular=args.angular, topic='/cmd_vel')

    # 捕获 SIGINT 以优雅退出
    def _sigint_handler(signum, frame):
        rospy.loginfo('捕获 SIGINT，准备退出')
        cam.stop()
        teleop.keep_running = False
        rospy.signal_shutdown('sigint')

    signal.signal(signal.SIGINT, _sigint_handler)

    try:
        teleop.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        cam.stop()
        cam.join(timeout=2.0)


if __name__ == '__main__':
    main()
