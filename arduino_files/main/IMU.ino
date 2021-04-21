#ifdef READ_IMU
void get_yaw_angle() {
  GY80_scaled val = IMU.read_scaled();       // get values from all sensors of the GY-80
  yaw_w = val.g_z;                           // yaw angular speed
  yaw += yaw_w;                              // yaw angle
}
#endif
