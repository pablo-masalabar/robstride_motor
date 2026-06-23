string  name         # motor name from config
float64 position     # rad       — target angle
float64 velocity     # rad/s     — target angular velocity
float64 torque_ff    # N·m       — torque feedforward
float64 kp           # position gain  (0 → use node config value)
float64 kd           # velocity gain  (0 → use node config value)
