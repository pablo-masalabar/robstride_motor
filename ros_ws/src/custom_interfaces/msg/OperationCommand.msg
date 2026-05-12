string  name         # motor name from config
float64 position     # rad       — target angle
float64 velocity     # rad/s     — target angular velocity
float64 torque_ff    # N·m       — torque feedforward
# Kp and Kd are persistent firmware parameters set via write_param service
