#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from decoupling_observer import DecouplingObserver

rospy.init_node("macros_decoupling_monitor")
obj = DecouplingObserver()
rospy.spin()
print "Done observing magnetic coupling"
obj.stop()
print "Done stopping observation"
