void refresh_timers() {
  timer.tick();                                                                   // refresh the timer for sampling
  tuning_setpoint_timer.tick();                                                   // refresh the timer for pid tuning
}

boolean update_pid_controllers(void *) {
  for (int i = 0; i < 3; i++) {                                                   // for each wheel
    float en_counted = en_counter_old[i] - en_counter[i];                         // determine how many paulses has elapsed
    long en_time_elapsed = millis() - en_counter_time;                            // the sampling timing window
    en_counter_old[i] = en_counter[i];                                            // update the counter log for each wheel
    wheel_w[i] = ((en_counted / rotation_pls) * 2000 * PI) / en_time_elapsed;     // calculate the omega of each wheel
  }
  en_counter_time = millis();                                                     // log the last sampling time
  motor_1_speed_pid.Compute();                                                    // update pid effort
  motor_2_speed_pid.Compute();
  motor_3_speed_pid.Compute();
  yaw_angle_compensator.Compute();
  return true;                                                                    // is necessary for the timer to resume
}

boolean setpoint_generator(void *) {
  for (int i = 0; i < 3; i++)
    wheel_w_ds[i] = random(-27, 27);                                              // random omega (rad/s)
  return true;
}

void monitor_motor_speed() {
  for (int i = 0; i < 3; i++) {
    //  int i = 0;
    Serial.print(wheel_w_ds[i]);
    Serial.print(" ");
//    Serial.print(wheel_w[i]);
//    Serial.print(" ");
  }
  Serial.println();
}
