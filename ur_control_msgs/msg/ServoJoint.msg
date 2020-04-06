std_msgs/Header header

float64[] position      # joint angles
float64 lookahead_time  # smoothens the trajectory, range [0.03, 0.2] sec
float64 gain            # proportional gain for following target, range [100, 2000]
