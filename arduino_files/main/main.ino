#include <PID_v1.h>
#include <arduino-timer.h>


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

const double Kp = 8, Ki = 130, Kd = 0.0;        // pid controller for wheels speed control
const double pidSampelingTime = 50;             // pid Sampeling Time (ms)

auto timer = timer_create_default();                    // timer object for sampling the encoder

auto tuning_setpoint_timer = timer_create_default();    // setpoint change for pid tuning

PID motor_1_speed_pid(&wheel_w[0], &motor_pwm[0], &wheel_w_ds[0], Kp, Ki, Kd, DIRECT);   // pid controller init for each wheel speed control
PID motor_2_speed_pid(&wheel_w[1], &motor_pwm[1], &wheel_w_ds[1], Kp, Ki, Kd, DIRECT);
PID motor_3_speed_pid(&wheel_w[2], &motor_pwm[2], &wheel_w_ds[2], Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(9600);

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

  timer.every(pidSampelingTime, update_wheel_speed);                 // sampling period

//  tuning_setpoint_timer.every(2000, setpoint_generator);            // setpoint change interval for pid tuning

  motor_1_speed_pid.SetMode(AUTOMATIC);                             // pid init
  motor_2_speed_pid.SetMode(AUTOMATIC);
  motor_3_speed_pid.SetMode(AUTOMATIC);

  motor_1_speed_pid.SetOutputLimits(-255, 255);                     // controller effort limit
  motor_2_speed_pid.SetOutputLimits(-255, 255);
  motor_3_speed_pid.SetOutputLimits(-255, 255);

}


void loop() {
  monitor_motor_speed();             // monitor desigered and acctual speed of the wheels
  refresh_timers();                  // update timers for sampling and contorlling
  apply_to_motors();                 // apply controller's effort on the motors
}
