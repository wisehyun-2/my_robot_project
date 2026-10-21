# One moteus-style servo command (same fields as wbr.common.servo.ServoCommand):
#   tau = clamp(feedforward_torque + kp * kp_scale * (position - q) + kd * kd_scale * (velocity - qd), +-maximum_torque)
# A NaN position disables the position term (velocity mode); kp_scale = kd_scale = 0 with a zero
# feedforward torque is a "stop" (zero torque) command.
string name                 # joint / servo name (e.g. left_wheel)
float64 position            # rad (NaN -> no position feedback)
float64 velocity            # rad/s
float64 feedforward_torque  # N m
float64 kp_scale            # multiplier on the servo's position gain
float64 kd_scale            # multiplier on the servo's velocity gain
float64 maximum_torque      # N m
