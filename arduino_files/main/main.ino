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

double Kp = 2, Ki = 0, Kd = 0;        // pid controller for wheels speed control

auto timer = timer_create_default();  // timer object for sampling the encoder

PID motor_1_speed_pid(&wheel_w[0], &motor_pwm[0], &motor_1_w_ds, Kp, Ki, Kd, DIRECT);   // pid controller init for each wheel speed control

void setup() {
  Serial.begin(9600);

  pinMode(encoder_1_a, INPUT_PULLUP);
  pinMode(encoder_1_b, INPUT_PULLUP);
  pinMode(encoder_2_a, INPUT_PULLUP);
  pinMode(encoder_2_b, INPUT_PULLUP);
  pinMode(encoder_3_a, INPUT_PULLUP);
  pinMode(encoder_3_b, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoder_1_a), encoder_handler_1_a, CHANGE);     // quadrature encoder with a pair of intrrupts for each wheel
  attachInterrupt(digitalPinToInterrupt(encoder_1_b), encoder_handler_1_b, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder_2_a), encoder_handler_2_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder_2_b), encoder_handler_2_b, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder_3_a), encoder_handler_3_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder_3_b), encoder_handler_3_b, CHANGE);

  timer.every(10, update_wheel_speed);      // timer object init

  motor_1_speed_pid.SetMode(AUTOMATIC);     // pid init
}

void loop() {
  timer.tick();    // refresh the timer
}
