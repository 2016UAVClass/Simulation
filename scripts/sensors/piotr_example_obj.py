from __future__ import division
import rospy
import numpy as np
import numpy.linalg as LA
from tf.transformations import quaternion_matrix
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3, Wrench
from macros_controller.utils.kdl_utils import kdl_to_numpy
from macros_controller.utils.tf_utils import quaternion_msg_to_tf, rot_mat_x, rot_mat_y, rot_mat_z
from std_msgs.msg import String, Float64
from macros_controller.magnetic_model.mac_dipole import DipoleField
import macros_controller.robot.constants as SC
from scipy.signal import butter, lfilter

# Now files used for forward kinematics
import PyKDL as KDL
import tf
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model

class DecouplingObserver(object):
    def __init__(self):
        print "Instantiating..."

        self._joint_vec = np.zeros(6)
        self._rate = rospy.Rate(10)
        self._pc = None
        self._pc_prev = np.array([0,0,0])
        self._R_capsule_world = None
        self._capsule_lin_vel = None
        self._capsule_ang_vel = None
        self._pa = None
        self._pa_prev = np.array([0,0,0])
        self._R_EPM_world = None
        self._fm = np.zeros(3)
        self._fm_prev = np.zeros(3)
        self._tm = np.zeros(3)
        self._tm_prev = np.zeros(3)
        self._coupling_status_msg = Float64()
        self._df_dx_msg = Vector3()
        self._df_dx_raw_msg = Vector3()
        self._dcapz_dEPMz_msg = Vector3()
        self._dcapz_dEPMz_raw_msg = Vector3()
        self._mag_wrench_msg = Wrench()

        #print "To publish: " , self._to_publish
        self._dipole = DipoleField(1.48, 1.48, SC.ACTUATOR_MAG_H, SC.CAPSULE_MAG_H)
        #print "Mag info:", SC.ACTUATOR_MAG_H

        # Initial conditions for Butterworth fulter
        fs = 100.0
        nyq = 0.5 * fs
        cutoff = 0.5
        order = 2
        self._butter_zi_dfdx = np.zeros((3, order))
        self._butter_params_dfdx = butter(order, cutoff/nyq, btype='lowpass', analog=False)
        self._butter_zi_dz = np.zeros((3, order))
        self._butter_params_dz = butter(order, cutoff/nyq, btype='lowpass', analog=False)

        # Instantiate publishers and subscribers
        self._coupling_status_pub = rospy.Publisher('/MAC/coupling_status_topic', Float64, queue_size=1000)
        self._df_dx_pub = rospy.Publisher('/MAC/df_dx_topic', Vector3, queue_size=1000)
        self._df_dx_raw_pub = rospy.Publisher('/MAC/df_dx_raw_topic', Vector3, queue_size=1000)
        self._mag_wrench_pub = rospy.Publisher('/MAC/mag_wrench_topic', Wrench, queue_size=1000)
        self._dcapz_dEPMz_pub = rospy.Publisher('/MAC/dcapz_dEPMz_topic', Vector3, queue_size=1000)
        self._dcapz_dEPMz_raw_pub = rospy.Publisher('/MAC/dcapz_dEPMz_raw_topic', Vector3, queue_size=1000)
        self._robot_sub = rospy.Subscriber("/mitsubishi_arm/joint_states", JointState, self._robot_pose_cb)
        self._capsule_sub = rospy.Subscriber("/MAC/mac/odom", Odometry, self._capsule_pose_cb)
        
        # Robot info
        self._robot = self._wait_and_get_robot()
        self._tree = kdl_tree_from_urdf_model(self._robot)
        self._chain_to_magnet = self._tree.getChain("base_link", "magnet_center")
        self._fksolver_magnet = KDL.ChainFkSolverPos_recursive(self._chain_to_magnet)
        self._q_cur = KDL.JntArray(self._chain_to_magnet.getNrOfJoints())

        # Initialize timer
        pub_timer = rospy.Timer(rospy.Duration(0.01), self._determine_coupling_state)

    def _wait_and_get_robot(self):
        t_init = rospy.Time.now()
        t_timeout = rospy.Duration(5) # 10 seconds
        
        while (rospy.Time.now() - t_init < t_timeout):
            try:
                robot = URDF.from_parameter_server("/mitsubishi_arm/robot_description")
                print "*** Obtained robot_description ***"
                return robot
            except KeyError:
                print "Key error."
                rospy.sleep(rospy.Duration(1))

        raise KeyError("robot_description was not found")

    def _capsule_pose_cb(self, capsule_pose_data):
        # Save past data to compute rate of change later
        self._pc_prev = self._pc
        self._R_capsule_world_prev = self._R_capsule_world
        
        # Get new capsule pose
        pos = capsule_pose_data.pose.pose.position
        quat = capsule_pose_data.pose.pose.orientation

        linear = capsule_pose_data.twist.twist.linear
        angular = capsule_pose_data.twist.twist.angular

        self._pc = np.array([pos.x, pos.y, pos.z])
        self._R_capsule_world = quaternion_matrix([quat.x, quat.y, quat.z, quat.w])[:3,:3]

        self._capsule_lin_vel = np.array([linear.x, linear.y, linear.z])
        self._capsule_ang_vel = np.array([angular.x, angular.y, angular.z])
        
        #print "Capsule Position: ", self._pc
        #print "Capsule Orientation: ", self._R_capsule_world
        #print " ---------- "

        return None

    def _robot_pose_cb(self, robot_jnt_data):
        q_now = KDL.JntArray(6)
        cur_frame = KDL.Frame()

        for i in range(0, 6):
            q_now[i] = robot_jnt_data.position[i]
        
        self._fksolver_magnet.JntToCart(q_now, cur_frame)
        self._pa = kdl_to_numpy(cur_frame.p)
        self._R_EPM_world = kdl_to_numpy(cur_frame.M)

        #print "EE Position: " , self._pa
        #print "EE Orientation: ", self._R_EPM_world
        #print " ---------- "

        return None

    def _determine_coupling_state(self, ev): #if no ev, get error "takes exactly 1 arg"
        
        if (self._pc is None) or (self._pa is None) or (self._R_EPM_world is None) or (self._R_capsule_world is None):
            print "Lacking data. Cannot compute.  Waiting..."
            return
       
        print " *** Got all necessary data. Ready for decoupling analysis. *** "
        
        ###
        # Decoupling analysis will consider the following:
        # (1) Increase in magnitude of df/dx
        # (2) Minimal threshold of magnetic force applied
        # (3) Increased z-motion of capsule without significant z motion of EPM
        ###
        
        # Compute new force and torque
        self._dipole.set_mag_poses(self._pa, self._pc, self._R_EPM_world, self._R_capsule_world)
        self._fm = self._dipole.mag_force()
        self._tm = self._dipole.mag_torque()
        
        # (1) Get df/dx
        self._get_dfdx_data()
        
        # (2) Check if minimal threshold of fm is met
        if abs(self._fm[2]) < 0.3:
            self._coupling_status_msg = 0 # NOT coupled
        else:
            self._coupling_status_msg = 1 # Coupled

        # (3) Get d_capz/d_EPMz
        self._get_dcapz_dEPMz()

        # Publish results
        self._publish_results()

        # To view what is being published, in another window: "rostopic echo -c /MAC/decouplilng_status_topic" . the -c clears the screen, can ommit.

        # print "I published"
        #rospy.sleep(.001)
        return None

 #*************************************
    def _get_dcapz_dEPMz(self):
        # Current positions
        pa = self._pa
        pc = self._pc

        # dc/dEE
        dpa = pa - self._pa_prev
        dpc = pc - self._pc_prev
        self._dcapz_dEPMz_raw = np.array([dpc[0]/dpa[0], dpc[1]/dpa[1], dpc[2]/dpa[2]])

        # Filter
        b, a = self._butter_params_dz
        self._dcapz_dEPMz,  self._butter_zi_dz = lfilter(b,a,self._dcapz_dEPMz_raw.reshape(-1,1),axis=1,zi=self._butter_zi_dz)

        self._pa_prev = pa
        self._pc_prev = pc

        return None

    def _get_dfdx_data(self):
        # Compute raw derivative df/dx
        self._dfm = self._fm - self._fm_prev
        self._dpc = self._pc - self._pc_prev
        self._df_dx_raw = np.array([self._dfm[0]/self._dpc[0], self._dfm[1]/self._dpc[1], self._dfm[2]/self._dpc[2]])

        # Save these values
        self._fm_prev = self._fm
        self._tm_prev = self._tm
        #idk_what_this_is = self._dipole.mag_field()
   
        # Apply Butterworth filter for df/dx
        b, a = self._butter_params_dfdx
        self._df_dx, self._butter_zi_dfdx = lfilter(b,a,self._df_dx_raw.reshape(-1,1),axis=1,zi=self._butter_zi_dfdx)

        return None

    def _publish_results(self):
        # Publish coupling status (std_msgs/Float64)
        self._coupling_status_pub.publish(self._coupling_status_msg)
        # Publish raw df/dx data (geometry_msgs/Vector3)
        self._df_dx_raw_msg.x = self._df_dx_raw[0]
        self._df_dx_raw_msg.y = self._df_dx_raw[1]
        self._df_dx_raw_msg.z = self._df_dx_raw[2]
        self._df_dx_raw_pub.publish(self._df_dx_raw_msg) 
        # Publish filtered df/dx data (geometry_msgs/Vector3)
        self._df_dx_msg.x = self._df_dx[0]
        self._df_dx_msg.y = self._df_dx[1]
        self._df_dx_msg.z = self._df_dx[2]
        self._df_dx_pub.publish(self._df_dx_msg) 
        # Publish magnetic wrench data (geometry_msgs/Wrench)
        self._mag_wrench_msg.force.x = self._fm[0]
        self._mag_wrench_msg.force.y = self._fm[1] 
        self._mag_wrench_msg.force.z = self._fm[2]
        self._mag_wrench_msg.torque.x = self._tm[0]
        self._mag_wrench_msg.torque.y = self._tm[1]
        self._mag_wrench_msg.torque.z = self._tm[2]
        self._mag_wrench_pub.publish(self._mag_wrench_msg)
        # Publish raw d_EPMz/d_capsulez data (geometry_msgs/Vector3)
        self._dcapz_dEPMz_msg.x = 0
        self._dcapz_dEPMz_msg.y = 0
        self._dcapz_dEPMz_msg.z = self._dcapz_dEPMz[2]
        self._dcapz_dEPMz_pub.publish(self._dcapz_dEPMz_msg)
        # Publish raw d_EPMz/d_capsulez data (geometry_msgs/Vector3)
        self._dcapz_dEPMz_raw_msg.x = 0
        self._dcapz_dEPMz_raw_msg.y = 0
        self._dcapz_dEPMz_raw_msg.z = self._dcapz_dEPMz[2]     
        self._dcapz_dEPMz_raw_pub.publish(self._dcapz_dEPMz_raw_msg)

        return None

    def stop(self):
        pass
