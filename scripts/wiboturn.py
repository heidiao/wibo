#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from middleme.msg import SoundLoc
from middleme.msg import MotorCtl
from rospy import loginfo as ROS_INFO
import socket
import sys
import time
import os
import ctypes

class robot_wiboturn():
    node = "wiboturn"
    motor_topic = node+"/motorctl"
    sound_topic = node+"/soundloc"

    def __init__(self):
        rospy.init_node(self.node) 

        rospy.on_shutdown(self.shutdown)

        self.rate = rospy.get_param('~rate', 10)
        self.sockfile = rospy.get_param('~sockfile', '/tmp/soundloc_sockets')

        # get motor control value from brain center
        rospy.Subscriber(self.motor_topic, MotorCtl, self.cmd_motor_handler)

        libstm_ctl = os.path.join(os.path.dirname(os.path.realpath(__file__)), "libstm32_ctrl.so")
        print libstm_ctl

        # include shared libaries
        self.motor = ctypes.CDLL(libstm_ctl)

        # publish sound localization value
        soundloc_pub = rospy.Publisher(self.sound_topic, SoundLoc, queue_size=10)

        rate = rospy.Rate(self.rate)

        # create socket file for communication with java-side code
        if os.path.exists(self.sockfile):
            os.remove(self.sockfile)
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.bind(self.sockfile)
        self.sock.listen(1)

        ROS_INFO("Waiting for sound localization socket ...")
        while not rospy.is_shutdown():
            try:
                self.conn, self.addr = self.sock.accept()
                while True:
                    # read 1024 character
                    data = self.conn.recv(1024)
                    if data != "":
                        package = data.split('|')
                        if len(package) == 2:
                            timestamp, angle = package
                            obj = SoundLoc()
                            obj.timestamp = rospy.Time.from_sec(float(timestamp))
                            obj.angle = float(angle)
                            
                            #ROS_INFO(obj)
                                
                            # publish sound localization value to subscriber
                            soundloc_pub.publish(obj)
                    else:
                        break
            except Exception, e:
                pass
            rate.sleep()


    # 
    # get motor control value from brain center
    # transfer the data to low-level mcu control
    def cmd_motor_handler(self, data):
        self.motor.welcome_msg('Hi C, im python')
        ROS_INFO(data)

    def shutdown(self):
        self.sock.close()
        ROS_INFO('Shutting down wiboturn node...')


if __name__ == '__main__':
    try:
        wb = robot_wiboturn()
    except rospy.ROSInterruptException:
        pass
