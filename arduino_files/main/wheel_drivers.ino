void turn_wheel(int wheel_no, int omega) {
  int a, b, pwm;
  if (wheel_no == 1) {
    a = motor_1_a;
    b = motor_1_b;
    pwm = motor_1_pwm;
  } else if (wheel_no == 2) {
    a = motor_2_a;
    b = motor_2_b;
    pwm = motor_2_pwm;
  } else if (wheel_no == 3) {
    a = motor_3_a;
    b = motor_3_b;
    pwm = motor_3_pwm;
  }

  digitalWrite(a, omega > 0 ? HIGH : LOW);
  digitalWrite(b, omega > 0 ? LOW : HIGH);
  analogWrite(pwm, abs(omega));
}
