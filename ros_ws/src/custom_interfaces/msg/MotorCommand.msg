# Motor command message.
# command_type selects the control mode: "position" | "velocity" | "torque" | "current" | "operation"

string  name         # target motor name
string  command_type

# Scalar value for simple command types.
# position → rad   velocity → rad/s   torque → N·m   current → A
float64 value

# Fields for operation-control mode (comm type 1):
# t_ref = torque_ff + Kp*(position - p_actual) + Kd*(velocity - v_actual)
# Kp and Kd are set via write_param service (ParamIndex.LOC_KP / SPD_KP).
float64 position     # rad
float64 velocity     # rad/s
float64 torque_ff    # N·m
