string  name              # motor name from config
float64 position          # rad    — target position
float64 speed             # rad/s  — max speed during motion (default 2 rad/s)
float64 acceleration      # rad/s² — acceleration ramp     (default 10 rad/s²)
float64 deceleration      # rad/s² — deceleration ramp     (0 = same as acceleration)
float64 torque_limit      # N·m    — 0 = motor default (T_MAX)
