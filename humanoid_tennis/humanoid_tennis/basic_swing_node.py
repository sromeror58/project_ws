import rclpy
import numpy as np
import tf2_ros

from math               import pi, sin, cos, acos, atan2, sqrt, fmod, exp

from asyncio            import Future
from rclpy.node         import Node
from geometry_msgs.msg  import PoseStamped, TwistStamped
from geometry_msgs.msg  import TransformStamped
from sensor_msgs.msg    import JointState
from std_msgs.msg       import Header
from std_msgs.msg import Float64, Float64MultiArray
from rclpy.duration import Duration


# Grab the Utilities
from utils.TransformHelpers     import *
from utils.TrajectoryUtils      import *
from hw5sols.KinematicChainSol     import KinematicChain
from humanoid_tennis.utils.TransformHelpers import *



class SwingNode(Node):
    def __init__(self):
        super().__init__('swing_node')
        self.joint_names = [
            'left_hip_pitch_joint',
            'left_hip_roll_joint',
            'left_hip_yaw_joint',
            'left_knee_joint',
            'left_ankle_pitch_joint',
            'left_ankle_roll_joint',
            'right_hip_pitch_joint',
            'right_hip_roll_joint',
            'right_hip_yaw_joint',
            'right_knee_joint',
            'right_ankle_pitch_joint',
            'right_ankle_roll_joint',
            'waist_yaw_joint',
            'waist_roll_joint',
            'waist_pitch_joint',
            'left_shoulder_pitch_joint',
            'left_shoulder_roll_joint',
            'left_shoulder_yaw_joint',
            'left_elbow_joint',
            'left_wrist_roll_joint',
            'left_wrist_pitch_joint',
            'left_wrist_yaw_joint',
            'right_shoulder_pitch_joint',
            'right_shoulder_roll_joint',
            'right_shoulder_yaw_joint',
            'right_elbow_joint',
            'right_wrist_roll_joint',
            'right_wrist_pitch_joint',
            'right_wrist_yaw_joint'
        ]
        # joints we care about for basic swinging with right arm (7 joints for right arm all at the end)
        # right_shoulder_pitch_joint 
        # right_shoulder_roll_joint
        # right_shoulder_yaw_joint
        # right_elbow_joint
        # right_wrist_roll_joint
        # right_wrist_pitch_joint
        # right_wrist_yaw_joint
        N_ra = 7
        self.n_joints = len(self.joint_names)
        def indexlist(start, end): return [i for i in range(start, end)]
        self.i_ra = indexlist(self.n_joints - 7, self.n_joints)
        self.Jblank = np.zeros((3, N_ra))
        
        self.chain = KinematicChain(self, 'torso_link', 'right_rubber_hand', [self.joint_names[i] for i in self.i_ra])
        

        #gains and parameters
        self.lam = 10.0 # error correction gain for ikin
        self.gamma = 0.1 # damping factor
        self.dt = 0.01  #100hz


        #state machine
        self.state = "IDLE"     
        self.idle_wait = 0.5

        
        self.start_time = None
        self.swing_duration = 2.0 #seconds to hit

        #trajectory waypoints
        self.q_0 = [0] * self.n_joints
        self.q_0[-4] = pi/2 # elbow joint
        (self.p_start, self.R_start, _, _) = self.chain.fkin(self.q_0[self.i_ra[0]:self.i_ra[-1]+1])
        self.p_idle = self.p_start
        self.p_swing = np.array([0.8, -0.2, 1.0]) ###########################################################################################
        # Rotation of -90 degrees around Z axis
        self.R_target = Rotz(-np.pi/2) 

        #stored command/error states
        self.qc = np.zeros(self.n_joints)
        # self.ep = np.zeros(3)
        # self.eR = np.zeros(3)
        self.have_state = False

        #ros publishers
        self.pub_cmd = self.create_publisher(Float64MultiArray, '/forward_command_controller/commands', 10)
        self.tfbroad = tf2_ros.TransformBroadcaster(self)

        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        #timer
        self.t = 0.0
        self.now = self.get_clock().now()
        self.timer = self.create_timer(self.dt, self.update)
        self.get_logger().info("Swing Node init")
        
    def joint_state_callback(self, msg):
        if not self.have_state:
            self.qc = self.extract_joints(msg)
            self.have_state = True
            self.get_logger().info("Received first joint state!")
        else:
            self.qc = self.extract_joints(msg)

    def extract_joints(self, msg):
        q = np.zeros(self.n_joints)
        for i in range(self.n_joints):
            try:
                index = msg.name.index(self.joint_names[i])
                q[i] = msg.position[index]
            except ValueError:
                return None
        return q


    def shutdown(self):
        self.timer.destroy()
        self.destroy_node()
    
    def update(self):
        if not self.have_state:
            return 

        self.t   += self.dt
        self.now = self.now + rclpy.time.Duration(seconds=self.dt)
        
        if self.state == "IDLE":
            if self.t >= self.idle_wait:
                self.state = "SWINGING"
                self.get_logger().info("Started Swinging!")
                self.t = 0.0
                return
        elif self.state == "SWINGING":
            if self.t <= self.swing_duration:
                s, sdot = goto(self.t, self.swing_duration, 0.0, 1.0)
                pd = self.p_idle + (self.p_swing - self.p_idle) * s
                vd = (self.p_swing - self.p_idle) * sdot
                #orientation is just constant
                Rd = Rinter(self.R_start, self.R_target, s)
                wd = winter(self.R_start, self.R_target, sdot)

            else:
                #end of phase 1
                self.state = "RETURN"
                self.get_logger().info("Swing complete, Returning!")
                self.t = 0.0
                return
            self.ik(pd, vd, Rd, wd)
        elif self.state == "RETURN":
            if self.t <= self.swing_duration:
                s, sdot = goto(self.t, self.swing_duration, 0.0, 1.0)
                pd = self.p_swing + (self.p_idle - self.p_swing) * s
                vd = (self.p_idle - self.p_swing) * sdot

                # Spline orientation back to start
                Rd = Rinter(self.R_target, self.R_start, s)
                wd = winter(self.R_target, self.R_start, sdot)
            else:
                self.state = "IDLE"
                self.get_logger().info("Returning to idle!")
                self.t = 0.0
                return
            self.ik(pd, vd, Rd, wd)

    def ik(self, pd, vd, Rd, wd):
        q_ra = self.qc[self.i_ra[0]:self.i_ra[-1]+1]
        (pc, Rc, Jv, Jw) = self.chain.fkin(q_ra)
        
        err_p = ep(pd, pc)
        err_R = eR(Rd, Rc) #orientation error

        #reference velocity
        vr = vd + self.lam * err_p
        wr = wd + self.lam * err_R

        x_dot_ref = np.concatenate((vr, wr))
        J = np.vstack((Jv, Jw))

        damp = (self.gamma**2) * np.eye(6)
        qdot_ra = J.T @ np.linalg.inv(J @ J.T + damp) @ x_dot_ref

        # change joint command
        self.qc[self.i_ra[0]:self.i_ra[-1]+1] += qdot_ra * self.dt
        #self.get_logger().info(f"Published command: {self.qc[self.i_ra[0]:self.i_ra[-1]+1].tolist()}")  
        
        # Publish command
        msg = Float64MultiArray()
        msg.data = self.qc.tolist()
        self.pub_cmd.publish(msg)
        # self.get_logger().info(f"Published command: {msg.data[:5]}...")

#
#  Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Create a future object to signal when the trajectory ends.

    # Initialize the trajectory generator node.
    trajectory = SwingNode()

    # Spin, meaning keep running (taking care of the timer callbacks
    # and message passing), until interrupted or the trajectory is
    # complete (as signaled by the future object).
    rclpy.spin(trajectory)

    # Shutdown the node and ROS.
    trajectory.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
