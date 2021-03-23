boolean setpoint_generator(void *) {
  for (int i = 0; i < 3; i++)
    wheel_w_ds[i] = random(-27, 27);      // random omega (rad/s)
  return true;
}

void monitor_motor_speed() {
  for (int i = 0; i < 3; i++) {
    //  int i = 0;
    Serial.print(wheel_w_ds[i]);
    Serial.print(" ");
    Serial.print(wheel_w[i]);
    Serial.print(" ");
  }
  Serial.println();
}

void refresh_timers() {
  timer.tick();                    // refresh the timer for sampling
  tuning_setpoint_timer.tick();    // refresh the timer for pid tuning
}

void apply_to_motors() {           // apply controller effort on the motors
  for (int i = 0; i < 3; i++)
    turn_wheel(i + 1, motor_pwm[i]);
}
