const float r = 4.5 * 0.01;                                // radios of the wheels (m)
const float d0 = (124 + 40) * 0.001;                                                          // front arm (m)
const float d1 = (108 + 40) * 0.001;                                                          // side  arms (m)
const float theta = (2 * PI - (2 * 60.25 * PI / 180)) / 2;                                    // angle between front and side wheels (rad)

const double j[3][3] {                                                                        // jacobian matrix
  {d0 / r, -1 / r, 0},
  {d1 / r, -cos(-theta) / r, -sin(-theta) / r},
  {d1 / r, -cos(+theta) / r, -sin(+theta) / r}
};

const float jic = -r / (-2 * d0 * cos(theta) * sin(theta) + 2 * d1 * sin(theta));             // jacobian inverse matrix
const double ji[3][3] {
  {jic * 2 * cos(theta)*sin(theta), -jic * sin(theta), -jic * sin(theta)},
  {jic * 2 * d1 * sin(theta), -jic*d0 * sin(theta), -jic*d0 * sin(theta)},
  {0, jic*(d0 * cos(theta) - d1), jic*(d1 - d0 * cos(theta))}
};


void calculate_wheel_w() {                                                                    // caculating inverse kinematics
  wheel_w_ds[1] = j[0][0] * w + j[0][1] * vx;
  wheel_w_ds[2] = j[1][0] * w + j[1][1] * vx + j[1][2] * vy;
  wheel_w_ds[0] = j[2][0] * w + j[2][1] * vx + j[2][2] * vy;
}

void calculate_robot_velocity() {                                                            // calculating forward kinematics
  w_ac  = ji[0][0] * wheel_w[1] + ji[0][1] * wheel_w[2] + ji[0][2] * wheel_w[0];
  vx_ac = ji[1][0] * wheel_w[1] + ji[1][1] * wheel_w[2] + ji[1][2] * wheel_w[0];
  vy_ac = ji[2][0] * wheel_w[1] + ji[2][1] * wheel_w[2] + ji[2][2] * wheel_w[0];
}
