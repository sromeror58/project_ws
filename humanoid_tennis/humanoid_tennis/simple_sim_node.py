import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile
import numpy as np
import time

class SimpleSimNode(Node):
    def __init__(self):
        super().__init__('simple_sim_node')

        # Define joint names (must match URDF)
        self.joint_names = [
            'left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint', 'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint',
            'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint', 'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint',
            'waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint',
            'left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_shoulder_yaw_joint', 'left_elbow_joint', 'left_wrist_roll_joint', 'left_wrist_pitch_joint', 'left_wrist_yaw_joint',
            'right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint', 'right_elbow_joint', 'right_wrist_roll_joint', 'right_wrist_pitch_joint', 'right_wrist_yaw_joint'
        ]
        self.n_joints = len(self.joint_names)
        self.q = np.zeros(self.n_joints)
        
        # Set initial pose (optional, can be all zeros)
        # Example: self.q[index] = value

        self.dt = 0.01 # 100Hz

        # Publisher for joint states
        qos_profile = QoSProfile(depth=10)
        self.pub_joint_state = self.create_publisher(JointState, '/joint_states', qos_profile)

        # Subscriber for commands
        self.create_subscription(Float64MultiArray, '/forward_command_controller/commands', self.command_callback, 10)

        # Timer for simulation loop
        self.create_timer(self.dt, self.update)
        
        self.get_logger().info("Simple Sim Node Started")

    def command_callback(self, msg):
        # Assuming msg.data contains positions for all joints in order
        # Or if it's partial, we need to know the mapping. 
        # The basic_swing_node sends a full list of joints (self.qc), so we can just update directly.
        if len(msg.data) == self.n_joints:
            self.q = np.array(msg.data)
        else:
            # If the command is partial, we might need more logic. 
            # For now, assume full state command as per basic_swing_node implementation.
            pass

    def update(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.q.tolist()
        self.pub_joint_state.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
