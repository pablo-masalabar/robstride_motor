std_msgs/Header header

string  name          # joint / motor name from config

float64 position      # rad
float64 velocity      # rad/s
float64 torque        # N·m

float32 temperature   # °C  (winding)
uint8   mode          # 0=reset  1=calibration  2=run
uint32  fault         # bitmask — see FaultBit in motor_base.py
bool    enabled
