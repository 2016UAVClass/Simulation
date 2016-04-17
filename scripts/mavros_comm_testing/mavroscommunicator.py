from __future__ import division
import rospy
import numpy as np
import numpy.linalg as LA
# from tf.transformations import quaternion_matrix
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
from mavros_msgs.msg import ActuatorControl

class MavrosCommunicator(object):
    def __init__(self):
        print "Instantiating..."

        self._imu_msg = Imu()
        self._lin_vel = Vector3()
        self._ang_vel = Vector3()
        self._orient = Quaternion()
        self._actuator_control = ActuatorControl()
        self._rate = rospy.Rate(10)

        # Instantiate publishers and subscribers
        self._imu_pub = rospy.Publisher('/mavros/imu/data', Imu, queue_size=1000)
        self._imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, self._imu_cb)
        self._imu_sub = rospy.Subscriber("/mavros/actuator_control", ActuatorControl, self._actuator_control_cb)

        # Initialize timer
        pub_timer = rospy.Timer(rospy.Duration(0.01), self._main)

    def _imu_cb(self, capsule_pose_data):
        print "Callback called."
        return None

    def _actuator_control_cb(self, data):
        print "Callback called."
        return None

    def _main(self, ev): #if no ev, get error "takes exactly 1 arg"
        print "Running main loop. Should be publishing data."

        # Quaternion Orientation
        self._orient.w = 4
        self._orient.x = 1
        self._orient.y = 2
        self._orient.z = 3

        # Linear Velocity
        self._lin_vel.x = 10
        self._lin_vel.y = 10
        self._lin_vel.z = 10

        # Angular Velocity
        self._ang_vel.x = 10
        self._ang_vel.y = 10
        self._ang_vel.z = 10
        
        # Populate variable
        self._imu_msg.orientation = self._orient
        self._imu_msg.linear_acceleration = self._lin_vel
        self._imu_msg.angular_velocity  = self._ang_vel

        self._imu_pub.publish(self._imu_msg)

        return None

    def stop(self):
        pass
