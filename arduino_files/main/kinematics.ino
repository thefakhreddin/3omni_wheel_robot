const double r = 4.5 * 0.01;                                // radios of the wheels (m)
const double d0 = (124+40) * 0.001;                         // front arm (m)
const double d1 = (108+40) * 0.001;                         // side  arms (m)
const double theta = (2 * PI - (2 * 60.25 * PI / 180)) / 2; // angle between front and side wheels (rad)

void calculate_wheel_w(float w, float vx, float vy) {
  wheel_w_ds[1] = (-1 / r) * (-d0 * w + vx);
  wheel_w_ds[2] = (-1 / r) * (-d1 * w + cos(-theta) * vx + sin(-theta) * vy);
  wheel_w_ds[0] = (-1 / r) * (-d1 * w + cos(+theta) * vx + sin(+theta) * vy);
}
