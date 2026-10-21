# Observation produced by the spine at every control cycle (upkie spine observers):
#   base_pitch            pitch of the base w.r.t. the world vertical (rad, positive = leaning forward);
#                         the spine's pitch_offset setting ("model") subtracts the model's equilibrium
#                         pitch (IMU zero at the balance point), "none" gives the raw observer pitch
#   base_angular_velocity body angular velocity in the base frame (rad/s)
#   ground_position       wheel odometry position (m), integrated while the wheels touch the floor
#   ground_velocity       wheel odometry velocity (m/s)
#   floor_contact         robot-level contact flag (any wheel in contact, or large upper-leg torques)
#   servos                measured servo states (position, velocity, torque)
#   imu                   raw IMU reading (orientation of the IMU frame in the world, gyro, accelerometer)
std_msgs/Header header
float64 base_pitch
float64[3] base_angular_velocity
float64 ground_position
float64 ground_velocity
bool floor_contact
ServoState[] servos
sensor_msgs/Imu imu
