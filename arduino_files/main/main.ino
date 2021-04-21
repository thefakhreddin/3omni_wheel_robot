#define USE_USBCON
//#define PID_TUNING
//#define READ_IMU

#include <PID_v1.h>
#include <arduino-timer.h>

#include <ros.h>
#include <geometry_msgs/Twist.h>

#ifdef READ_IMU
#include <Wire.h>
#include <GY80.h>
#endif

#define encoder_1_a 22
#define encoder_1_b 23
#define encoder_2_a 24
#define encoder_2_b 25
#define encoder_3_a 26
#define encoder_3_b 27

#define motor_1_a   28
#define motor_1_b   29
#define motor_2_a   30
#define motor_2_b   31
#define motor_3_a   32
#define motor_3_b   33

#define motor_1_pwm 2
#define motor_2_pwm 3
#define motor_3_pwm 4

const long rotation_pls = 750;        // number of paulses in one complete rotation

long en_counter[] = {0, 0, 0};        // paulse counter for encoders
long en_counter_old[] = {0, 0, 0};    // paulse counter for encoders log
long en_counter_time;                 // encoders counter sampling time
double wheel_w[] = {0, 0, 0};         // acctual omega of the wheels (rad/s)
double motor_pwm[] = {0, 0, 0};       // controller effort
double wheel_w_ds[] = {0, 0, 0};      // desigered omega of the wheels (rad/s)
double w = 0 , vx = 0, vy = 0;        // robot's desigered status
#ifdef READ_IMU
double yaw_w = 0;                                    // robot's actual yaw angular vel (yaw dot)
double yaw_w_ds = 0;                                 // robot's desigered yaw anglular vel (yaw dot)
double yaw = 0;                                      // robot's actual yaw angle in degrees
#endif

const double Kp_wheel = 4, Ki_wheel = 50, Kd_wheel = 0;         // pid controller for wheels speed control
const double pidSampelingTime = 20;                             // pid Sampeling Time (ms)

auto timer = timer_create_default();                    // timer object for sampling the encoder

#ifdef PID_TUNING
auto tuning_setpoint_timer = timer_create_default();    // setpoint change for pid tuning
#endif

PID motor_1_speed_pid(&wheel_w[0], &motor_pwm[0], &wheel_w_ds[0], Kp_wheel, Ki_wheel, Kd_wheel, DIRECT);   // pid controller init for each wheel speed control
PID motor_2_speed_pid(&wheel_w[1], &motor_pwm[1], &wheel_w_ds[1], Kp_wheel, Ki_wheel, Kd_wheel, DIRECT);
PID motor_3_speed_pid(&wheel_w[2], &motor_pwm[2], &wheel_w_ds[2], Kp_wheel, Ki_wheel, Kd_wheel, DIRECT);

#ifdef READ_IMU
GY80 IMU = GY80();                                                                       // IMU object of GY80 class
#endif

ros::NodeHandle  nh;                                                                     // ROS node handler

void update_cmd_pos( const geometry_msgs::Twist& cmd_vel) {                             // input command state handler function
  float x = cmd_vel.linear.x;
  float y = cmd_vel.linear.y;
  float th = cmd_vel.angular.z;
  vx = x;
  vy = y;
  w = th;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &update_cmd_pos);                  // command state (x-dot y-dot theta-dot) listener

void setup() {
#ifdef PID_TUNING
  Serial.begin(9600);
#endif

  pinMode(encoder_1_a, INPUT_PULLUP);
  pinMode(encoder_1_b, INPUT_PULLUP);
  pinMode(encoder_2_a, INPUT_PULLUP);
  pinMode(encoder_2_b, INPUT_PULLUP);
  pinMode(encoder_3_a, INPUT_PULLUP);
  pinMode(encoder_3_b, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoder_1_a), encoder_handler_1_a, CHANGE);    // quadrature encoder with a pair of intrrupts for each wheel
  attachInterrupt(digitalPinToInterrupt(encoder_1_b), encoder_handler_1_b, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder_2_a), encoder_handler_2_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder_2_b), encoder_handler_2_b, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder_3_a), encoder_handler_3_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder_3_b), encoder_handler_3_b, CHANGE);

  timer.every(pidSampelingTime, update_pid_controllers);                // sampling period

#ifdef PID_TUNING
  tuning_setpoint_timer.every(3000, setpoint_generator);                // setpoint change interval for pid tuning
#endif

  motor_1_speed_pid.SetMode(AUTOMATIC);                                 // pid init
  motor_2_speed_pid.SetMode(AUTOMATIC);
  motor_3_speed_pid.SetMode(AUTOMATIC);

  motor_1_speed_pid.SetOutputLimits(-255, 255);                         // controller effort limit
  motor_2_speed_pid.SetOutputLimits(-255, 255);
  motor_3_speed_pid.SetOutputLimits(-255, 255);

#ifdef READ_IMU
  IMU.begin();                                                          // init GY-80
#endif

  nh.initNode();                                                        // ROS interface init
  nh.subscribe(sub);

  pinMode(LED_BUILTIN, OUTPUT);                                         // on-board LED for debugging

}


void loop() {
  nh.spinOnce();                     // referesh ROS interface
  calculate_wheel_w(w, vx, vy);      // calculate motors omega
  refresh_timers();                  // update timers for sampling and contorlling
  apply_to_motors();                 // apply controller's effort on the motors

#ifdef PID_TUNING
  monitor_motor_speed();             // monitor desigered and acctual speed of the wheels
#endif
}
