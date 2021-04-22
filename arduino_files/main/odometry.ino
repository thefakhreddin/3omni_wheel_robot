void calculate_odometry(float time_elapsed) {
  double dt = time_elapsed * 0.001;                                   // elapsed time in secs
  double delta_x = (vx_ac * cos(alpha) - vy_ac * sin(alpha)) * dt;    // chage in x & y position with respect to the space
  double delta_y = (vx_ac * sin(alpha) + vy_ac * cos(alpha)) * dt;
  double delta_alpha = w_ac * dt;                                     // chage in robot's orientaion

  x_pos += delta_x;                                                   // apply changes to keep track of the position & orientation
  y_pos += delta_y;
  alpha += delta_alpha;
}
