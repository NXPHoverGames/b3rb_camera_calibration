# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import cv2
import cv_bridge
import message_filters
import numpy
import os
import rclpy
from rclpy.node import Node
import sensor_msgs.msg
import std_msgs.msg
import sensor_msgs.srv
import threading
import time
from camera_calibration.calibrator import MonoCalibrator, Patterns
try:
    from queue import Queue
except ImportError:
    from Queue import Queue
from camera_calibration.calibrator import CAMERA_MODEL
from rclpy.qos import qos_profile_system_default
from rclpy.qos import QoSProfile


class BufferQueue(Queue):
    """Slight modification of the standard Queue that discards the oldest item
    when adding an item and the queue is full.
    """
    def put(self, item, *args, **kwargs):
        # The base implementation, for reference:
        # https://github.com/python/cpython/blob/2.7/Lib/Queue.py#L107
        # https://github.com/python/cpython/blob/3.8/Lib/queue.py#L121
        with self.mutex:
            if self.maxsize > 0 and self._qsize() == self.maxsize:
                self._get()
            self._put(item)
            self.unfinished_tasks += 1
            self.not_empty.notify()


class SpinThread(threading.Thread):
    """
    Thread that spins the ros node, while imshow runs in the main thread
    """

    def __init__(self, node):
        threading.Thread.__init__(self)
        self.node = node

    def run(self):
        rclpy.spin(self.node)


class ConsumerThread(threading.Thread):
    def __init__(self, queue, function):
        threading.Thread.__init__(self)
        self.queue = queue
        self.function = function

    def run(self):
        while True:
            m = self.queue.get()
            self.function(m)


class CalibrationNode(Node):
    def __init__(self, name, boards, service_check = True, synchronizer = message_filters.TimeSynchronizer, flags = 0,
                 fisheye_flags = 0, pattern=Patterns.Chessboard, camera_name='', checkerboard_flags = 0,
                 max_chessboard_speed = -1, queue_size = 1):
        super().__init__(name)

        self._stop=False

        self._boards = boards
        self._calib_flags = flags
        self._fisheye_calib_flags = fisheye_flags
        self._checkerboard_flags = checkerboard_flags
        self._pattern = pattern
        self._camera_name = camera_name
        self._max_chessboard_speed = max_chessboard_speed


        msub = message_filters.Subscriber(self, sensor_msgs.msg.Image, 'image', qos_profile=self.get_topic_qos("image"))
        msub.registerCallback(self.queue_monocular)

        #Publish calibration Image through ROS
        self.camera_pub = self.create_publisher(sensor_msgs.msg.Image, 'calibration_camera_feed', 10)
        #Subscription to mange the control of calibration through ROS msgs
        self.calibration_control_sub = self.create_subscription(std_msgs.msg.String,'calibration_control',self.calibration_control_callback,10)

        self.q_mono = BufferQueue(queue_size)

        self.c = None

        self.br = cv_bridge.CvBridge()

        self._last_display = None

        mth = ConsumerThread(self.q_mono, self.handle_monocular)
        mth.setDaemon(True)
        mth.start()

    def publish_monocular(self, *args):
        pass
    def print_monocular(self, *args):
        pass
    def calibration_control_callback(self, *args):
        pass
    def queue_monocular(self, msg):
        self.q_mono.put(msg)


    def handle_monocular(self, msg):
        if self.c == None:
            if self._camera_name:
                self.c = MonoCalibrator(self._boards, self._calib_flags, self._fisheye_calib_flags, self._pattern, name=self._camera_name,
                                        checkerboard_flags=self._checkerboard_flags,
                                        max_chessboard_speed = self._max_chessboard_speed)
            else:
                self.c = MonoCalibrator(self._boards, self._calib_flags, self._fisheye_calib_flags, self._pattern,
                                        checkerboard_flags=self.checkerboard_flags,
                                        max_chessboard_speed = self._max_chessboard_speed)

        # This should just call the MonoCalibrator
        drawable = self.c.handle_msg(msg)
        self.displaywidth = drawable.scrib.shape[1]
        self.print_monocular(drawable)
        self.publish_monocular(drawable)

    def get_topic_qos(self, topic_name: str) -> QoSProfile:
        """!
        Given a topic name, get the QoS profile with which it is being published.
        Replaces history and depth settings with default values since they cannot be retrieved
        @param topic_name (str) the topic name
        @return QosProfile the qos profile with which the topic is published. If no publishers exist
        for the given topic, it returns the sensor data QoS. returns None in case ROS1 is being used
        """
        topic_name = self.resolve_topic_name(topic_name)
        topic_info = self.get_publishers_info_by_topic(topic_name=topic_name)
        if len(topic_info):
            qos_profile = topic_info[0].qos_profile
            qos_profile.history = qos_profile_system_default.history
            qos_profile.depth = qos_profile_system_default.depth
            return qos_profile
        else:
            self.get_logger().warn(f"No publishers available for topic {topic_name}. Using system default QoS for subscriber.")
            return qos_profile_system_default


class OpenCVCalibrationNode(CalibrationNode):
    """ Calibration node with an OpenCV Gui """
    FONT_FACE = cv2.FONT_HERSHEY_SIMPLEX
    FONT_SCALE = 0.6
    FONT_THICKNESS = 2

    def __init__(self, *args, **kwargs):

        CalibrationNode.__init__(self, *args, **kwargs)

    def spin(self):
        sth = SpinThread(self)
        sth.setDaemon(True)
        sth.start()

        while not self._stop:
            pass

    @classmethod
    def putText(cls, img, text, org, color = (0,0,0)):
        cv2.putText(img, text, org, cls.FONT_FACE, cls.FONT_SCALE, color, thickness = cls.FONT_THICKNESS)

    @classmethod
    def getTextSize(cls, text):
        return cv2.getTextSize(text, cls.FONT_FACE, cls.FONT_SCALE, cls.FONT_THICKNESS)[0]

    def on_model_change(self, model_select_val):
        if self.c == None:
            print("Cannot change camera model until the first image has been received")
            return

        self.c.set_cammodel( CAMERA_MODEL.PINHOLE if model_select_val < 0.5 else CAMERA_MODEL.FISHEYE)

    def on_model_change(self, model_select_val):
        self.c.set_cammodel( CAMERA_MODEL.PINHOLE if model_select_val < 0.5 else CAMERA_MODEL.FISHEYE)

    def on_scale(self, scalevalue):
        if self.c.calibrated:
            self.c.set_alpha(scalevalue / 100.0)

    def button(self, dst, label, enable):
        dst.fill(255)
        size = (dst.shape[1], dst.shape[0])
        if enable:
            color = (155, 155, 80)
        else:
            color = (224, 224, 224)
        cv2.circle(dst, (size[0] // 2, size[1] // 2), min(size) // 2, color, -1)
        (w, h) = self.getTextSize(label)
        self.putText(dst, label, ((size[0] - w) // 2, (size[1] + h) // 2), (255,255,255))

    def buttons(self, display):
        x = self.displaywidth
        self.button(display[180:280,x:x+100], "CALIBRATE", self.c.goodenough)
        self.button(display[280:380,x:x+100], "SAVE", self.c.calibrated)
        self.button(display[380:480,x:x+100], "COMMIT", self.c.calibrated)

    def y(self, i):
        """Set up right-size images"""
        return 30 + 40 * i

    def screendump(self, im):
        i = 0
        while os.access("/tmp/dump%d.png" % i, os.R_OK):
            i += 1
        cv2.imwrite("/tmp/dump%d.png" % i, im)
        print("Saved screen dump to /tmp/dump%d.png" % i)
    
    def display_progress_bar(self,progress,label):
        bar_length = 20  # You can adjust this to change the progress bar length
        block = '-'
        empty = ' '       
        filled_length = int(bar_length * progress)
        bar = block * filled_length + empty * (bar_length - filled_length)
        # Display the label and the progress bar
        label_position = (bar_length - len(label)) // 2
        label_padding = ' ' * label_position
        print(f"{label_padding}{label}")
        print(f"[{bar}] {progress * 100:.2f}%")
        
    def print_monocular(self,drawable):
        if not self.c.calibrated:
            if drawable.params and drawable.good_sample:
                print("\n")
                for i, (label, lo, hi, progress) in enumerate(drawable.params):
                    self.display_progress_bar(progress,label)
                print("\nCALIBRATION STATUS:")
                if self.c.goodenough:
                    print("READY TO CALIBRATE - Publish ROS2 msg 'start_calibration' to begin.")
                else:
                    print("CALIBRATION NEEDED - Continue sampling.")
                if self.c.calibrated:
                    print("CALIBRATION COMPLETE - Publish ROS2 msg 'save_results' to save.")

    
    def calibration_control_callback(self, msg: std_msgs.msg.String):
        if msg.data == 'start_calibration' and self.c.goodenough:
            print('\n', end='')
            self.c.do_calibration()
            print("\nCALIBRATION FINISHED - Ready to save. Publish ROS2 msg 'save_results'.")
        elif msg.data == 'save_results' and self.c.calibrated:
            print('\n', end='')
            self.c.do_save()
            print("\nRESULTS SAVED - Calibration data saved. Publish ROS2 msg 'stop' to end session.")
        elif msg.data == 'stop':
            self._stop= True
            print("\nCALIBRATION STOPPED - Session ended.")

    def publish_monocular(self, drawable):
        height = drawable.scrib.shape[0]
        width = drawable.scrib.shape[1]

        display = numpy.zeros((max(480, height), width + 100, 3), dtype=numpy.uint8)
        display[0:height, 0:width,:] = drawable.scrib
        display[0:height, width:width+100,:].fill(255)

        self.buttons(display)
        if not self.c.calibrated:
            if drawable.params:
                 for i, (label, lo, hi, progress) in enumerate(drawable.params):
                    (w,_) = self.getTextSize(label)
                    self.putText(display, label, (width + (100 - w) // 2, self.y(i)))
                    color = (0,255,0)
                    if progress < 1.0:
                        color = (0, int(progress*255.), 255)
                    cv2.line(display,
                            (int(width + lo * 100), self.y(i) + 20),
                            (int(width + hi * 100), self.y(i) + 20),
                            color, 4)

        else:
            self.putText(display, "lin.", (width, self.y(0)))
            linerror = drawable.linear_error
            if linerror is None or linerror < 0:
                msg = "?"
            else:
                msg = "%.2f" % linerror
                #print "linear", linerror
            self.putText(display, msg, (width, self.y(1)))

        msg = self.br.cv2_to_imgmsg(display, "bgr8")
        self.camera_pub.publish(msg)
