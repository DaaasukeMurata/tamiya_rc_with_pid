#!/usr/bin/env python
# # -*- coding: utf-8 -*-

import rospy
import cv2
import math
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import UInt16MultiArray


class RcPid(object):

    def __init__(self):
        self.__cv_bridge = CvBridge()
        image_node = rospy.get_param("~image", "/usb_cam_node/image_raw")
        self.__img_sub = rospy.Subscriber(image_node, Image, self.callback, queue_size=1)
        self.__img_pub = rospy.Publisher('image_pid', Image, queue_size=1)
        self.__steer_pub = rospy.Publisher('servo', UInt16MultiArray, queue_size=1)

        # for pid
        self.before_val = 0.0
        self.now_val = 0.0
        self.integral = 0.0
        self.integral_time = 0.033

        # for RC control
        self.steer = 90
        self.spd_hi = rospy.get_param("~speed_hi", 75)
        self.spd_lo = rospy.get_param("~speed_low", 83)

    def callback(self, image_msg):
        cv_img = self.__cv_bridge.imgmsg_to_cv2(image_msg, 'bgr8')

        line_pos = self.line_detect(cv_img)

        if not math.isnan(line_pos):
            speed = self.spd_hi
            self.steer = 90 - self.steer_pid(line_pos * 180, 90)
            rospy.loginfo("line_pos=%4.2f  steer=%d", line_pos, self.steer)
        else:
            speed = self.spd_lo

        rc_cntr = UInt16MultiArray()
        rc_cntr.data = [self.steer, speed]
        self.__steer_pub.publish(rc_cntr)

    # 0.5をcenterとして、0.0 〜 1.0の間の値を返す
    def line_detect(self, cv_img):
        DEBUG_SCREENOUT = True
        WIDTH = 480

        # トリミング
        start_w = (1280 - WIDTH) / 2
        cv_img = cv_img[320:350, start_w:start_w + WIDTH]

        # 2値化
        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        cv_img = cv2.GaussianBlur(cv_img, (3, 3), 0)
        cv_img = cv2.Canny(cv_img, 30, 120)
        contours, hierarchy = cv2.findContours(cv_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(cv_img, contours, -1, (255, 255, 255), 10)

        # 白線位置計算、CENTER_RANGEの範囲を見てなければ全体で確認
        CENTER_RANGE = 240
        CENTER_START = (WIDTH - CENTER_RANGE) / 2
        wk_img = np.hsplit(cv_img, [CENTER_START, CENTER_START + CENTER_RANGE])
        center_img = wk_img[1]  # 3分割した2個目がcenter

        sum_array = center_img.sum(axis=0)
        pos = np.sum(sum_array * range(CENTER_START, CENTER_START + CENTER_RANGE)) / np.sum(sum_array) / WIDTH

        if math.isnan(pos):
            sum_array = cv_img.sum(axis=0)
            pos = np.sum(sum_array * range(WIDTH)) / np.sum(sum_array) / WIDTH

        if DEBUG_SCREENOUT:
            # 中心線描画
            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_GRAY2BGR)
            if not math.isnan(pos):  # NaN check
                cv2.line(cv_img, (int(pos * WIDTH), 0), (int(pos * WIDTH), 30), [0, 0, 255], 10)

            # self.__img_pub.publish(self.__cv_bridge.cv2_to_imgmsg(cv_img, 'mono8'))
            self.__img_pub.publish(self.__cv_bridge.cv2_to_imgmsg(cv_img, 'bgr8'))

        return pos

    def steer_pid(self, val, target):
        DELTA_T = 0.033

        KP = rospy.get_param("~KP", 1.66)
        KI = KP / self.integral_time
        KD = KP * DELTA_T

        self.before_val = self.now_val
        self.now_val = target - val

        self.integral += (self.before_val + self.now_val) / 2.0 * DELTA_T
        self.integral_time += DELTA_T

        p = KP * self.now_val
        i = KI * self.integral
        d = KD * (self.now_val - self.before_val) / DELTA_T

        # rospy.loginfo("integral=%5.1f   integral_time=%5.2f", self.integral, self.integral_time)
        # rospy.loginfo("KP=%3.1f  KI=%3.1f  KD=%3.1f", KP, KI, KD)
        rospy.loginfo("p=%4.1f   i=%4.1f   d=%4.1f", p, i, d)

        return max(min(90, p + i + d), -90)

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('rc_pid')

    process = RcPid()
    process.main()
