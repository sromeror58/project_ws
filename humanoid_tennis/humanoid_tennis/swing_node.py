#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import tf2_ros
import numpy as np
from math import pi, sin, cos, acos, atan2, sqrt

from humanoid_tennis.utils.TransformHelpers import *
from humanoid_tennis.utils.TrajectoryUtils import *
from hw5sols.KinematicChainSol import KinematicChain

class SwingNode(Node):
    def __init__(self):
        super().__init__('swing_node')
        self.joint_names = [
            'left_hip_pitch_joint', #0
            'left_hip_roll_joint', #1
            'left_hip_yaw_joint', #2
            'left_knee_joint', #3
            'left_ankle_pitch_joint', #4
            'left_ankle_roll_joint', #5
            'right_hip_pitch_joint', #6
            'right_hip_roll_joint', #7
            'right_hip_yaw_joint', #8
            'right_knee_joint', #9
            'right_ankle_pitch_joint', #10
            'right_ankle_roll_joint', #11
            'waist_yaw_joint', #12
            'waist_roll_joint', #13
            'waist_pitch_joint', #14
            'left_shoulder_pitch_joint', #15
            'left_shoulder_roll_joint', #16
            'left_shoulder_yaw_joint', #17
            'left_elbow_joint', #18
            'left_wrist_roll_joint', #19
            'left_wrist_pitch_joint', #20
            'left_wrist_yaw_joint', #21
            'right_shoulder_pitch_joint', #22
            'right_shoulder_roll_joint', #23
            'right_shoulder_yaw_joint', #24
            'right_elbow_joint', #25
            'right_wrist_roll_joint', #26   
            'right_wrist_pitch_joint', #27
            'right_wrist_yaw_joint' #28
        ]
        
        # Indices for waist (3) + right arm (7)
        # Waist joints: 12, 13, 14
        # Right arm joints: 22, 23, 24, 25, 26, 27, 28
        # self.i_waist = [12, 13, 14]
        self.i_arm = [22, 23, 24, 25, 26, 27, 28]
        self.i_chain = self.i_arm # Only arm
        self.n_joints = len(self.joint_names)
        
        # Kinematic Chain from torso_link to right_rubber_hand
        self.chain = KinematicChain(self, 'torso_link', 'right_rubber_hand', [self.joint_names[i] for i in self.i_chain])
        
        # TF Buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
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
        self.q_0 = [0.0] * self.n_joints
        self.q_0[-4] = pi/2 # elbow joint
        
        # Calculate initial pose
        # We need to extract the chain joints from q_0
        q_chain_init = np.array([self.q_0[i] for i in self.i_chain])
        (self.p_start, self.R_start, _, _) = self.chain.fkin(q_chain_init)
        
        self.p_idle = self.p_start
        self.p_swing = self.p_idle # Will be updated by ball callback
        
        # Rotation of -90 degrees around Z axis
        self.R_target = Rotz(-np.pi/2) 

        #stored command/error states
        self.qc = np.zeros(self.n_joints)
        self.have_state = False
        
        # Ball tracking
        self.ball_position = None
        self.new_ball = False
        self.create_subscription(PoseStamped, '/ball_pose', self.ball_callback, 10)
        
        # Visualization
        self.marker_pub = self.create_publisher(Marker, '/swing_markers', 10)
        self.trajectory_points = []

        #ros publishers
        self.pub_cmd = self.create_publisher(Float64MultiArray, '/forward_command_controller/commands', 10)
        self.tfbroad = tf2_ros.TransformBroadcaster(self)

        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        #timer
        self.t = 0.0
        self.now = self.get_clock().now()
        self.timer = self.create_timer(self.dt, self.update)
        self.get_logger().info("Swing Node init")

    def ball_callback(self, msg):
        self.ball_position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        # Only trigger if we are IDLE? Or update target if swinging?
        # For now, let's just flag it. The update loop will handle the trigger.
        self.new_ball = True
        # self.get_logger().info(f"Ball detected at {self.ball_position}")

    def extract_joints(self, msg):
        q = np.zeros(self.n_joints)
        for i in range(self.n_joints):
            try:
                index = msg.name.index(self.joint_names[i])
                q[i] = msg.position[index]
            except ValueError:
                return None
        return q

    def joint_state_callback(self, msg):
        if not self.have_state:
            self.qc = self.extract_joints(msg)
            self.have_state = True
            self.get_logger().info("Received first joint state!")
        else:
            self.qc = self.extract_joints(msg)

    def publish_markers(self, target_pos, current_pos):
        # Target Marker
        marker_target = Marker()
        marker_target.header.frame_id = "torso_link"
        marker_target.header.stamp = self.get_clock().now().to_msg()
        marker_target.ns = "target"
        marker_target.id = 0
        marker_target.type = Marker.SPHERE
        marker_target.action = Marker.ADD
        marker_target.pose.position.x = target_pos[0]
        marker_target.pose.position.y = target_pos[1]
        marker_target.pose.position.z = target_pos[2]
        marker_target.scale.x = 0.1
        marker_target.scale.y = 0.1
        marker_target.scale.z = 0.1
        marker_target.color.a = 1.0
        marker_target.color.r = 1.0
        marker_target.color.g = 0.0
        marker_target.color.b = 0.0
        self.marker_pub.publish(marker_target)

        # Trajectory Marker
        self.trajectory_points.append(Point(x=current_pos[0], y=current_pos[1], z=current_pos[2]))
        marker_traj = Marker()
        marker_traj.header.frame_id = "torso_link"
        marker_traj.header.stamp = self.get_clock().now().to_msg()
        marker_traj.ns = "trajectory"
        marker_traj.id = 1
        marker_traj.type = Marker.LINE_STRIP
        marker_traj.action = Marker.ADD
        marker_traj.scale.x = 0.02
        marker_traj.color.a = 1.0
        marker_traj.color.r = 0.0
        marker_traj.color.g = 1.0
        marker_traj.color.b = 0.0
        marker_traj.points = self.trajectory_points
        self.marker_pub.publish(marker_traj)

    def update(self):
        if not self.have_state:
            return 

        self.t   += self.dt
        self.now = self.now + rclpy.time.Duration(seconds=self.dt)
        
        if self.state == "IDLE":
            # Check for ball
            if self.new_ball:
                # Transform ball to torso_link
                try:
                    # Look up transform from pelvis (source) to torso_link (target)
                    # The ball is in pelvis frame.
                    # We want p_torso.
                    trans = self.tf_buffer.lookup_transform('torso_link', 'pelvis', rclpy.time.Time())
                    
                    # Extract transform
                    t_p = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
                    r_q = np.array([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
                    R_mat = R_from_quat(r_q)
                    
                    # p_torso = R * p_pelvis + t
                    self.p_swing = R_mat @ self.ball_position + t_p
                    
                    self.state = "SWINGING"
                    self.get_logger().info(f"Started Swinging at {self.p_swing} (torso frame)!")
                    self.t = 0.0
                    self.new_ball = False # Reset flag
                    self.trajectory_points = [] # Clear trajectory
                    return
                except Exception as e:
                    self.get_logger().warn(f"Could not transform ball: {e}")
                    return
                
        elif self.state == "SWINGING":
            if self.t <= self.swing_duration:
                s, sdot = goto(self.t, self.swing_duration, 0.0, 1.0)
                pd = self.p_idle + (self.p_swing - self.p_idle) * s
                vd = (self.p_swing - self.p_idle) * sdot
                
                #orientation spline
                Rd = Rinter(self.R_start, self.R_target, s)
                wd = winter(self.R_start, self.R_target, sdot)
                
                # Visualize
                q_chain = np.array([self.qc[i] for i in self.i_chain])
                (pc, _, _, _) = self.chain.fkin(q_chain)
                self.publish_markers(self.p_swing, pc)

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
        # Extract current chain configuration
        q_chain = np.array([self.qc[i] for i in self.i_chain])
        
        (pc, Rc, Jv, Jw) = self.chain.fkin(q_chain)
        
        err_p = ep(pd, pc) #translation error
        err_R = eR(Rd, Rc) #orientation error

        #reference velocity
        vr = vd + self.lam * err_p
        wr = wd + self.lam * err_R

        x_dot_ref = np.concatenate((vr, wr))
        J = np.vstack((Jv, Jw))

        damp = (self.gamma**2) * np.eye(6)
        qdot_chain = J.T @ np.linalg.inv(J @ J.T + damp) @ x_dot_ref

        # change joint command
        # Map chain velocities back to full joint vector
        for k, idx in enumerate(self.i_chain):
            self.qc[idx] += qdot_chain[k] * self.dt
            


        # Clamp elbow joint
        self.qc[25] = np.clip(self.qc[25], -np.pi/2, np.pi/2)    
        
        # Clamp right shoulder pitch
        self.qc[23] = np.clip(self.qc[23], -np.pi, 0)

        # Clamp hip joints
        # Roll (X) - Index 26
        self.qc[13] = np.clip(self.qc[13], -np.pi/6, np.pi/6)
        # Pitch (Y) - Index 27
        self.qc[14] = np.clip(self.qc[14], -np.pi/6, np.pi/6)
        # Yaw (Z) - Index 28
        self.qc[12] = np.clip(self.qc[12], -2*np.pi, 2*np.pi)
            
        # Publish command
        msg = Float64MultiArray()
        msg.data = self.qc.tolist()
        self.pub_cmd.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SwingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
