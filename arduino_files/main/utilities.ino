void refresh_timers() {
  timer.tick();                                                                   // refresh the timer for sampling

  #ifdef PID_TUNUNG
  tuning_setpoint_timer.tick();                                                   // refresh the timer for pid tuning
  #endif
}

boolean refresh_contorls(void *) {
  long time_elapsed = millis() - last_time;                                     // the sampling timing window
  calculate_motor_speed_from_encoder(time_elapsed);                             // read motor speed from encoders
  motor_1_speed_pid.Compute();                                                  // update pid effort
  motor_2_speed_pid.Compute();
  motor_3_speed_pid.Compute();
  calculate_robot_velocity();                                                   // calculate robots velocity
  calculate_odometry(time_elapsed);                                             // calculate robots odometry
  last_time = millis();                                                         // log the last sampling time
  return true;                                                                  // is necessary for the timer to resume
}

boolean setpoint_generator(void *) {
  for (int i = 0; i < 3; i++)
    wheel_w_ds[i] = random(-27, 27);                                              // random omega (rad/s)
  return true;
}

void monitor_motor_speed() {
  for (int i = 0; i < 3; i++) {
    //      int i = 0;
    Serial.print(wheel_w_ds[i]);
    Serial.print(" ");
    Serial.print(wheel_w[i]);
    Serial.print(" ");
  }
  Serial.println();
}
