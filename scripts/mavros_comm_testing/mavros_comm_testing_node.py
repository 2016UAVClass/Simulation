#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from mavroscommunicator import MavrosCommunicator

rospy.init_node("mavros_comm_node")
obj = MavrosCommunicator()
rospy.spin()
print "Done communicating with mavros."
obj.stop()
print "Done stopping mavros communication."
