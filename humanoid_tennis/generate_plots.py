#!/usr/bin/env python3
import sys
import numpy as np
import matplotlib.pyplot as plt
from math import pi

# Try to import necessary modules from the workspace
try:
    from humanoid_tennis.swing_node import SwingNode
    from humanoid_tennis.utils.TransformHelpers import pxyz, Rotz, eR, ep
    import rclpy
    from hw5sols.KinematicChainSol import KinematicChain
except ImportError as e:
    print("Error importing modules. Make sure you actived your ROS2 workspace!")
    print(f"Details: {e}")
    # We will try to mock minimal components if possible, but KinematicChain is essential.
    sys.exit(1)

def generate_tracking_error_plot():
    print("Generating Tracking Error Plot...")
    
    # Initialize basic node (without spinning ROS to avoid threading issues if possible)
    rclpy.init()
    node = SwingNode()
    
    # Manually trigger a "swing" simulation
    # We will step through the update() loop logic without ROS time essentially
    # Or we can just use the math functions directly.
    
    # Setup Target
    node.state = "SWINGING"
    node.t = 0.0
    node.swing_duration = 2.0
    
    # Mock Ball Position (roughly in front)
    node.p_swing = np.array([0.5, -0.2, 0.8]) 
    
    # Setup Start (Rest)
    q_start = node.q_0
    # Actually q_start is set in IDLE. Let's force it.
    node.qc = np.array(node.q_0) # Current state = Rest
    node.q_start = np.array([node.qc[i] for i in node.i_chain])
    
    # Calculate Target Joints (Inverse Kinematics for Goal)
    # Re-using internal function
    node.q_target = node.newton_raphson(node.p_swing, node.R_target, node.q_start.copy())
    
    # Simulation Loop
    dt = 0.01
    times = []
    errors = []
    
    # We need to simulate the feedback loop. 
    # In the real node, 'qc' comes from joint_states. 
    # Here we must integrate qdot manually or assume perfect tracking of 'qc' 
    # BUT the IK controller computes 'qdot'. We should update 'qc' based on that.
    
    # Copy current chain config
    current_q_chain = node.q_start.copy()
    
    for t in np.arange(0, node.swing_duration, dt):
        node.t = t
        
        # --- LOGIC FROM UPDATE LOOP ---
        # 1. Trajectory Interpolation
        # s, sdot = goto(t, duration, ...) -> Implement minimalist goto here or use node's if accessible? 
        # The node imports TrajectoryUtils. Let's use the formula directly:
        # Quintic spline 0 to 1
        tf = node.swing_duration
        if t > tf: s = 1.0; sdot = 0.0
        else:
            tau = t/tf
            s = 10*tau**3 - 15*tau**4 + 6*tau**5
            sdot = (30*tau**2 - 60*tau**3 + 30*tau**4) / tf
            
        q_d = node.q_start + (node.q_target - node.q_start) * s
        qdot_d = (node.q_target - node.q_start) * sdot
        
        # 2. FK
        (pd, Rd, Jv, Jw) = node.chain.fkin(q_d)
        vd = Jv @ qdot_d
        wd = Jw @ qdot_d
        
        # 3. IK (Calculate command velocity)
        # ik() uses 'self.qc' as current state. We must ensure self.qc is updated.
        # We need to inject 'current_q_chain' into 'self.qc'
        for k, idx in enumerate(node.i_chain):
            node.qc[idx] = current_q_chain[k]
            
        # Call IK (computes qdot and publishes, but we want the internal values)
        # The ik() function in SwingNode calculates qdot_chain and updates self.qc via integration!
        # wait, SwingNode.ik() lines 386-392: "q_next = self.qc[idx] + qdot_chain[k] * dt"
        # So calling ik() actually UPDATES self.qc (simulating the robot moving).
        # Perfect!
        
        node.ik(pd, vd, Rd, wd)
        
        # 4. Measure Error
        # Actual Position
        new_q_chain = np.array([node.qc[i] for i in node.i_chain])
        (p_act, _, _, _) = node.chain.fkin(new_q_chain)
        
        pos_error = np.linalg.norm(pd - p_act)
        
        times.append(t)
        errors.append(pos_error)
        
        current_q_chain = new_q_chain # Sync
        
    rclpy.shutdown()
    
    # Plot
    plt.figure()
    plt.plot(times, errors)
    plt.title("End-Effector Tracking Error vs Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Position Error (m)")
    plt.grid(True)
    plt.savefig('tracking_error.png')
    print("Saved tracking_error.png")

def generate_workspace_plot():
    print("Generating Workspace Reachability Plot...")
    
    # We need a node to initialize chain? 
    # Or just KinematicChain class if we can instantiate it standalone.
    # SwingNode instantiates it with 'self'. KinematicChain likely takes a node for logging or params.
    # Let's try to mock the node interface if needed, or just use the one we have.
    
    rclpy.init()
    node = SwingNode()
    
    points = []
    
    print("Sampling 1000 configurations...")
    for _ in range(1000):
        # Random configuration within limits
        q_rand = []
        for low, high in node.chain_limits:
            val = np.random.uniform(low, high)
            q_rand.append(val)
        
        q_rand = np.array(q_rand)
        
        # FK
        (p, _, _, _) = node.chain.fkin(q_rand)
        points.append(p)
        
    points = np.array(points)
    
    # Plot X-Z (Side view) and X-Y (Top view)
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    
    # X vs Z
    ax1.scatter(points[:,0], points[:,2], s=5, alpha=0.5)
    ax1.set_title("Workspace Side View (X-Z)")
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Z (m)")
    ax1.axis('equal')
    
    # X vs Y
    ax2.scatter(points[:,0], points[:,1], s=5, alpha=0.5)
    ax2.set_title("Workspace Top View (X-Y)")
    ax2.set_xlabel("X (m)")
    ax2.set_ylabel("Y (m)")
    ax2.axis('equal')
    
    plt.savefig('workspace_reachability.png')
    print("Saved workspace_reachability.png")
    
    # Calculate Volume (Convex Hull) estimate
    try:
        from scipy.spatial import ConvexHull
        hull = ConvexHull(points)
        print(f"Estimated Workspace Volume (Convex Hull): {hull.volume:.4f} m^3")
    except ImportError:
        print("Scipy not installed/found. Skipping volume calculation.")

    rclpy.shutdown()

if __name__ == "__main__":
    generate_tracking_error_plot()
    generate_workspace_plot()
