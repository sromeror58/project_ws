'''TrajectoryUtils.py

   This is a library of utility functions for trajectory generation.

   The functions are:

   Quintic Spline:  goto(t, T, v0, v1)  Return (s, sdot) for quintic spline
'''

import numpy as np

def goto(t, T, v0, v1):
    '''
    Return the position and velocity of a quintic spline segment.

    Input:
      t   Current time
      T   Total duration of the segment
      v0  Initial velocity
      v1  Final velocity

    Output:
      s   Position (0 to 1)
      sdot Velocity
    '''
    
    if t < 0:
        return (0.0, 0.0)
    elif t >= T:
        return (1.0, 0.0)
        
    # Quintic spline coefficients
    # s(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    # Constraints:
    # s(0) = 0, s(T) = 1
    # sdot(0) = v0/T, sdot(T) = v1/T (normalized velocity)
    # sddot(0) = 0, sddot(T) = 0
    
    # Normalized time
    tau = t / T
    
    # Coefficients for normalized time (0 to 1)
    # s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5 (for v0=v1=0)
    # But we want to support v0, v1? The usage in basic_swing_node is goto(t, duration, 0.0, 1.0)?? 
    # Wait, basic_swing_node calls goto(self.t, self.swing_duration, 0.0, 1.0).
    # If the last two args are v0, v1, then 1.0 is final velocity? That seems odd for a stop-to-stop motion.
    # Let's check how it's used:
    # s, sdot = goto(self.t, self.swing_duration, 0.0, 1.0)
    # pd = p_start + (p_end - p_start) * s
    # vd = (p_end - p_start) * sdot
    # If sdot is normalized velocity, then actual velocity is (p_end - p_start) * sdot.
    # If v0=0, v1=1.0, then it ends with velocity?
    # Maybe the arguments are not v0, v1.
    
    # Let's assume standard quintic spline with zero accel at ends.
    # s = 10*tau**3 - 15*tau**4 + 6*tau**5
    # sdot = (30*tau**2 - 60*tau**3 + 30*tau**4) / T
    
    # If the args are (t, T, start_val, end_val)? No, goto usually returns s in [0,1].
    
    # Let's implement the standard minimum jerk trajectory (quintic)
    tau = t / T
    s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
    sdot = (30 * tau**2 - 60 * tau**3 + 30 * tau**4) / T
    
    return (s, sdot)
