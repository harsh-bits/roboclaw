#!/usr/bin/env python
#@package foxbot_node
#This package defines a ROS node for the foxbot robot. In order to run this code, please load the fxscript defined in ../fxscript/ROSnode/ros.pac on the foxbot and then run the script after initializing the robot. Once the script is running, run the rosnode with rosrun foxbot robot_node.py. 
#
#The node spawns 2 socket connections with the ros node behaving as the client. The first socket created is the logger socket which sets up a repeated stream of joint and cartesian configurations and publishes on the JointsLog and CartesianLog topics. The second socket is a service socket. The ROS node advertises services that can then be used to command the robot. 
#


import rospy 
import struct
import threading
import socket
import json 
import sys
import math
import signal
import logging
import numpy as np
from collections import defaultdict
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped,TransformStamped
import tf
import tf.transformations as transform
from foxbot.srv import *
from foxbot.msg import BoolStamped

rospy.init_node('roboclaw_node', anonymous=True)


class DelayedKeyboardInterrupt(object):
    def __enter__(self):
        self.signal_received = False
        self.old_handler = signal.getsignal(signal.SIGINT)
        signal.signal(signal.SIGINT, self.handler)

    def handler(self, signal, frame):
        self.signal_received = (signal, frame)
        logging.debug('SIGINT received. Delaying KeyboardInterrupt.')

    def __exit__(self, type, value, traceback):
        signal.signal(signal.SIGINT, self.old_handler)
        if self.signal_received:
            self.old_handler(*self.signal_received)


class Roboclaw():
    #Transforms
    broadcast = tf.TransformBroadcaster()
    listener = tf.TransformListener()


    def __init__(self):
        self.connect()
        self.advertise()
        # self.logPublisherJ = rospy.Publisher('robot_JointsLog', JointState, queue_size=10)
        # self.logPublisherC = rospy.Publisher('robot_CartesianLog', PoseStamped, queue_size=10)
        # self.isMovingPublisher = rospy.Publisher('robot_IsMoving', BoolStamped, queue_size=10)


    def connect(self):
        #Connect to roboclaw and check get version here. 
        #TODO

    def disconnect(self):
        #Disconnect roboclaw here
        #TODO
    def run_logger(self):
        #Log encoder values here
        #TODO

    # Service Handlers
    def handle_get_position(self,req)
    #TODO
        return GetPositionResponse()
    def handle_set_position(self,req)
    #TODO
        return SetPositionResponse()
    def handle_get_velocity(self,req)
    #TODO
        return GetVelocityResponse()
    def handle_set_velocity(self,req)
    #TODO
        return SetVelocityResponse()
    def handle_get_current(self,req)
    #TODO
        return GetCurrentResponse()
    def handle_set_current(self,req)
    #TODO
        return SetCurrentResponse()
    def handle_set_position_gains(self,req)
    #TODO
        return SetPositionGainsResponse()
    def handle_get_position_gains(self,req)
    #TODO
        return GetPositionGainsResponse()
    def handle_set_velocity_gains(self,req)
    #TODO
        return SetVelocityGainsResponse()
    def handle_get_velocity_gains(self,req)
    #TODO
        return GetVelocityGainsResponse()
    def handle_home(self,req)
    #TODO
        return HomeResponse()
    def handle_set_zeros(self,req)
    #TODO
        return SetZerosResponse()
    def handle_set_mode(self,req)
    #TODO
        return SetModeResponse()

# Socket Functions
    def sendrecv(self, hSocket, hLock, data):
        sendMsg = pack(data)
        hLock.acquire()
        hSocket.sendall(sendMsg)
        recvMsg = hSocket.recv(MESSAGE_LEN)
        hLock.release()
        rospy.loginfo('Sent {0} bytes, recv {1} bytes'.format(len(data), len(recvMsg)))
        return unpack(recvMsg)


    def advertise(self):
        #Advertise all roboclaw services
        try:
            rospy.loginfo('Advertising services')

            self.getPositionSrv = rospy.Service('GetPosition',GetPosition,self.handle_get_position)
            self.setPositionSrv = rospy.Service('SetPosition',SetPosition,self.handle_set_position)

            self.getVelocitySrv = rospy.Service('GetVelocity',GetVelocity,self.handle_get_velocity)
            self.setVelocitySrv = rospy.Service('SetVelocity',SetVelocity,self.handle_set_velocity)

            self.getCurrentSrv = rospy.Service('GetCurrent',GetCurrent,self.handle_get_current)
            self.setCurrentSrv = rospy.Service('SetCurrent',SetCurrent,self.handle_set_current)

            self.setPositionGainsSrv = rospy.Service('SetPositionGains',SetPositionGains, self.handle_set_position_gains)
            self.getPositionGainsSrv = rospy.Service('GetPositionGains',GetPositionGains, self.handle_get_position_gains)

            self.setVelocityGainsSrv = rospy.Service('SetVelocityGains',SetVelocityGains, self.handle_set_velocity_gains)
            self.getVelocityGainsSrv = rospy.Service('GetVelocityGains',GetVelocityGains, self.handle_get_velocity_gains)
            self.homeSrv = rospy.Service('Home',Home, self.handle_home)
            self.setZerosSrv = rospy.Service('SetZeros',SetZeros,self.handle_set_zeros)
            self.setModeSrv = rospy.Service('SetMode',SetMode,self.handle_set_mode)
        except:
            rospy.logerr('Unable to advertise services')
            raise

def main():
    rospy.loginfo('Node Started')
    while not rospy.is_shutdown():
        node = Roboclaw()
        try:
            while not rospy.is_shutdown():
                node.run_logger()
        except Exception as ex:
            rospy.logerr("Caught error: " + str(ex))
            node.disconnect()

    rospy.loginfo('Shutting Down')
    node.disconnect()




if __name__ == '__main__':
    main()
