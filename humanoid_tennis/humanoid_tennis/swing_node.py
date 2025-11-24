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
# Import the format for the condition number message
from std_msgs.msg import Float64

# Grab the Utilities
from utils.TransformHelpers     import *
from utils.TrajectoryUtils      import *
from hw5code.KinematicChain     import KinematicChain



class SwingNode(Node):
    def __init__(self):
        super().__init__('swing_node')
        self.chain = KinematicChain(self, 'torso_link', 'hand_palm_link')
        self.joint_names = ['l_sole_x', 'l_sole_y', 'l_sole_z', 'r_sole_x', 'r_sole_y', 'r_sole_z']
        self.n_joints = len(self.joint_names)

        #gains and parameters
        self.lam = 10.0 # error correction gain for ikin
        self.gamma = 0.1 # damping factor
        self.dt = 0.01  #100hz


        #state machine
        self.state = "IDLE" #idle, swinging, recover
        self.start_time = 0.0
        self.swing_duration = 0.8 #seconds to hit

        #trajectory waypoints
        self.p_start = None
        self.R_start = None
        self.p_ball = None
        self.p_follow = None

        #stored command/error states
        self.qc = np.zeros(self.n_joints)
        self.ep = np.zeros(3)
        self.eR = np.zeros(3)
        self.received_first_state = False

        #ros publishers
        self.pub_cmd = self.create_publisher(Float64MultiArray, '/forward_command_controller/commands', 10)
        self.pub_pose = self.create_publisher(PoseStamped, '/debug_pose', 10)
        self.tfbroad = tf2_ros.TransformBroadcaster(self)

        self.create_subscription(PointStamped, '/tennis_ball/position', self.ball_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        #timer
        self.t = 0.0
        self.now = self.get_clock().now()
        self.timer = self.create_timer(self.dt, self.update)
        self.get_logger().info("Swing Node init")
        
    def joint_state_callback(self, msg):
        if self.state == "IDLE":
            current_q = self.extract_joints(msg)
            if current_q is not None:
                self.qc = current_q
                self.received_first_state = True
    
    def ball_callback(self, msg):
        if self.state == "IDLE" and self.received_first_state:
            self.get_logger().info(f"ball @ {msg.point}")

            (self.p_start, self.R_start, _, _) = self.chain.fkin(self.qc)
            self.p_ball = np.array([msg.point.x, msg.point.y, msg.point.z])

            swing_vec = self.p_ball - self.p_start
            self.p_follow = self.p_ball + (swing_vec * 0.5)

            #transition state
            self.state = "SWINGING"
            self.t = 0.0
            self.ep = np.zeros(3) #reset errors
            self.eR = np.zeros(3)


    def shutdown(self):
        self.timer.destroy()
        self.destroy_node()
    
    def update(self):
        if self.state == "IDLE" or not self.received_first_state:
            return

        self.t   += self.dt
        self.now = self.now + rclpy.time.Duration(seconds=self.dt)

        if self.state == "SWINGING":
            if self.t <= self.swing_duration:
                # go to spline help to get the path varible from 0 to 1
                #returnign position scaling and velocity scaling 
                (s , sdot) = goto(self.t, self.swing_duration, 0.0, 1.0)

                #pd = start + (ball - start) * s
                pd = self.p_start + (self.p_ball - self.p_start) * s

                #vd = ball - start * sdot
                vd = (self.p_ball - self.p_start) * sdot

                #orientation is just constant
                Rd = self.R_start
                wd = np.zeros(3)

            else:
                #end of phase 1
                self.state = "DONE"
                self.get_logger().info("ball is hit lets go ban")
                return
            
            #i kin stuff
            (pc, Rc, Jv, Jw) = self.chain.fkin(self.qc)

            #error -- task space
            err_p = ep(pd, pc) #translation error
            err_R = eR(Rd, Rc) #orientation error

            #reference velocity
            vr = vd + self.lam * err_p
            wr = wd + self.lam * err_R

            x_dot_ref = np.concatenate((vr, wr))

            J = np.vstack((Jv, Jw))

            #damped inverse
            damp = (self.gamma**2) * np.eye(6)
            inv_term = np.linalg.inv( J @ J.T + damp)
            qcdot = J.T @ inv_term @ x_dot_ref

            #integrate
            self.qc = self.qc + self.dt * qcdot

            #publish
            msg = Float64MultiArray()
            msg.data = self.qc.tolist()
            self.pub_cmd.publish(msg)

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
    

    # Report the reason for shutting down.
    if future.done():
        trajectory.get_logger().info("Stopping: " + future.result())
    else:
        trajectory.get_logger().info("Stopping: Interrupted")

    # Shutdown the node and ROS.
    trajectory.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()