void perform_circle_menoeuvre() {
  double q = millis();
  q /= 5000;
  vx = -0.3 * sin(2 * PI * q);
  vy = 0.3 * cos(2 * PI * q);

  //  Serial.print(vx);
  //  Serial.print(" ");
  //  Serial.println(vy);
}

void perform_straight_line_once() {
  vy = 0;
  double m = millis() - 500;
  double t_change = 1000;
  double pick = 0.5;
  double width = 4000;
  if (m > 0) {
    if (m < t_change)vx = (pick / t_change) * m;
    else if (m < t_change + width)vx = pick;
    else if (m < 2 * t_change + width)vx = pick - (pick / t_change) * (m - (t_change + width));
    else vx = 0;
    //        Serial.println(vx);
  }
}
