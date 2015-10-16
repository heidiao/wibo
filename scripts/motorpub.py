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
import threading
from random import randint

MOTORTOPIC= "wiboturn/motorctl"

def motorctl():
    ROS_INFO("motor publisher test...")
    pub = rospy.Publisher(MOTORTOPIC, MotorCtl, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        obj = MotorCtl()
        obj.x = randint(0, 180)
        obj.y = randint(0,180)
#        obj.accel = randint(1,10)
        ROS_INFO(obj)
        pub.publish(obj)
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node("publishermotor") 

        #soundlocthread = threading.Thread(target = soundloc, args=("soundloc_thread",1))    
        #soundlocthread.setDaemon(True)
        #soundlocthread.start()

        motorctl()
    except rospy.ROSInterruptException:
        #soundlocthread.join(0)
        pass
