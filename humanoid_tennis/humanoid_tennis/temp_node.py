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
# from hw5code.KinematicChain import KinematicChain

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
        
        # Joint limits for the chain (Right Arm)
        # Indices: 22, 23, 24, 25, 26, 27, 28
        # Limits from URDF (approx) + User constraints
        self.chain_limits = [
            (-3.08, 2.67), # Shoulder Pitch
            (-2.25, 1.58), # Shoulder Roll
            (-2.61, 2.61), # Shoulder Yaw
            (-1.04, 2.09), # Elbow
            (-1.97, 1.97), # Wrist Roll
            (-1.04, 1.04), # Wrist Pitch (User constrained)
            (-1.04, 1.04)  # Wrist Yaw (User constrained)
        ]
        
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
        self.swing_duration = 2.5 #seconds to hit

        #trajectory waypoints
        self.q_0 = [0.0] * self.n_joints
        self.q_0[-4] = pi/2 # elbow joint
        
        # Calculate initial pose
        # We need to extract the chain joints from q_0
        q_chain_init = np.array([self.q_0[i] for i in self.i_chain])
        (self.p_start, self.R_start, _, _) = self.chain.fkin(q_chain_init)
        
        self.p_idle = self.p_start
        self.p_swing = self.p_idle # Will be updated by ball callback
        self.q_target = np.array([self.q_0[i] for i in self.i_chain])
        self.q_start = np.array([self.q_0[i] for i in self.i_chain])

        
        # Rotation of -90 degrees around Z axis
        #self.R_target = Rotz(-np.pi/2) 
        self.R_target = Reye()

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
        marker_target.color.r = 0.0
        marker_target.color.g = 1.0
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
        marker_traj.color.g = 0.0
        marker_traj.color.b = 1.0
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
                    self.q_target = self.newton_raphson(self.p_swing, self.R_target, self.q_start)
                    
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
            # p_swing -> q_swing
        
            if self.t <= self.swing_duration:
                # 1. Get interpolation factor s (0 to 1)
                s, sdot = goto(self.t, self.swing_duration, 0.0, 1.0)
                
                # 2. Interpolate manually using s
                q_d = self.q_start + (self.q_target - self.q_start) * s
                qdot_d = (self.q_target - self.q_start) * sdot
                
                # 3. Forward Kinematics
                (pd, Rd, Jv, Jw) = self.chain.fkin(q_d)
                
                vd = Jv @ qdot_d
                wd = Jw @ qdot_d
                
                # Visualize
                self.publish_markers(self.p_swing, pd)

            else:
                #end of phase 1
                self.state = "RETURN"
                self.get_logger().info("Swing complete, Returning!")
                self.t = 0.0
                return
            self.ik(pd, vd, Rd, wd, track_orientation=True)
            
        elif self.state == "RETURN":
            if self.t <= self.swing_duration:
                # 1. Get interpolation factor s (0 to 1)
                s, sdot = goto(self.t, self.swing_duration, 0.0, 1.0)
                
                # 2. Interpolate manually using s (Joint Space)
                # Return to q_start (which is q_0)
                q_d = self.q_target + (self.q_start - self.q_target) * s
                qdot_d = (self.q_start - self.q_target) * sdot

                # 3. Forward Kinematics
                (pd, Rd, Jv, Jw) = self.chain.fkin(q_d)
                vd = Jv @ qdot_d
                wd = Jw @ qdot_d
            else:
                self.state = "IDLE"
                self.get_logger().info("Returning to idle!")
                self.t = 0.0
                return
            self.ik(pd, vd, Rd, wd, track_orientation=False)


    def newton_raphson(self, p_target, R_target, q_guess, max_iter=10, tol=1e-6):
        """
        Calculates joint angles (q) for a given target position (p) and rotation (R).
        """
        # q = np.array(q_guess) # Start from a guess (e.g., current position)
        q = q_guess
        for i in range(20): # Try 20 iterations
            # 1. Calculate where we are now
            (p_curr, R_curr, Jv, Jw) = self.chain.fkin(q)
            
            # 2. Calculate error
            err_p = p_target - p_curr
            err_R = eR(R_target, R_curr) # Orientation error
            
            error = np.concatenate((err_p, err_R))
            
            # 3. Check if we are close enough
            if np.linalg.norm(error) < 1e-4:
                return q
                
            # 4. Calculate step using Jacobian
            J = np.vstack((Jv, Jw))
            damp = 1e-3 * np.eye(6) # Damping to prevent instability
            
            # dq = J_pinv * error
            dq = J.T @ np.linalg.inv(J @ J.T + damp) @ error
            
            # 5. Update q
            q += dq

            # 6. Wrap to stay close to guess (prevent spinning)
            # This ensures we pick the solution closest to our start configuration
            q = q_guess + (q - q_guess + np.pi) % (2 * np.pi) - np.pi
            
            # 7. Clamp to limits
            for j in range(len(q)):
                q[j] = np.clip(q[j], self.chain_limits[j][0], self.chain_limits[j][1])
            
        return q # Return best effort after 20 tries
    def ik(self, pd, vd, Rd, wd, track_orientation=True):
        # Extract current chain configuration
        q_chain = np.array([self.qc[i] for i in self.i_chain])
        
        (pc, Rc, Jv, Jw) = self.chain.fkin(q_chain)
        
        err_p = ep(pd, pc) #translation error

        if track_orientation:
            err_R = eR(Rd, Rc)
        else:
            err_R = np.zeros(3)



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
            # integrate
            q_next = self.qc[idx] + qdot_chain[k] * self.dt

            # apply per-joint limits for the chain
            q_min, q_max = self.chain_limits[k]
            self.qc[idx] = float(np.clip(q_next, q_min, q_max))

            


        # Clamp elbow joint
        self.qc[25] = np.clip(self.qc[25], -np.pi/2, np.pi/2)    
        
        # Clamp right shoulder pitch
        self.qc[23] = np.clip(self.qc[23], -np.pi, 0)

        # Clamp wrist joints
        # Roll - Index 26
        self.qc[26] = np.clip(self.qc[26], -1.97, 1.97)
        # Pitch - Index 27
        self.qc[27] = np.clip(self.qc[27], -1.04, 1.04)
        # Yaw - Index 28
        self.qc[28] = np.clip(self.qc[28], -np.pi/3, np.pi/3)
        
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
