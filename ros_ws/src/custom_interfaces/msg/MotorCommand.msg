# Operation-control mode command (comm type 1).
# t_ref = Kd*(velocity - v_actual) + Kp*(position - p_actual) + torque_ff

string  name         # target motor name

# command_type identifies which kind of command this message carries.
# Expected values: "position" | "velocity" | "torque" | "current" | "operation"
string  command_type

# value holds the primary scalar for the command_type above.
# position  → rad        velocity → rad/s
# torque    → N·m        current  → A
# operation → unused (use the fields below instead)
float64 value

float64 position     # rad       target angle
float64 velocity     # rad/s     target angular velocity
float64 torque_ff    # N·m       torque feedforward
float64 kp           # 0–500
float64 kd           # 0–5
