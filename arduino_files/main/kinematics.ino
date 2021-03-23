const double r = 4.5 * 0.01;                                // radios of the wheels (m)
const double d = 15.5 * 0.01;                               // distance from side wheels and the front wheel from center (m)
const double theta = (2 * PI - (2 * 59.03 * PI / 180)) / 2; // angle between front and side wheels (rad)

void calculate_wheel_w(float w, float vx, float vy) {
  wheel_w_ds[1] = (-1 / r) * (-d * w + vx);
  wheel_w_ds[2] = (-1 / r) * (-d * w + cos(-theta) * vx + sin(-theta) * vy);
  wheel_w_ds[0] = (-1 / r) * (-    d * w + cos(+theta) * vx + sin(+theta) * vy);
}
