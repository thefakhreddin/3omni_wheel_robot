void encoder_handler_1_a() {                                                      // get called when signal "a" status changes
  if (digitalRead(encoder_1_a)) {                                                 // if "a" is rised
    if (!digitalRead(encoder_1_b)) en_counter[0]++;                               // if the "b" signal is low increment counter
    else en_counter[0]--;                                                         // if the "b" signal is high decrement counter
  } else {                                                                        // vice versa
    if (digitalRead(encoder_1_b)) en_counter[0]++;
    else en_counter[0]--;
  }
}
void encoder_handler_1_b() {
  if (digitalRead(encoder_1_b)) {
    if (digitalRead(encoder_1_a)) en_counter[0]++;
    else en_counter[0]--;
  } else {
    if (!digitalRead(encoder_1_a)) en_counter[0]++;
    else en_counter[0]--;
  }
}

void encoder_handler_2_a() {
  if (digitalRead(encoder_2_a)) {
    if (!digitalRead(encoder_2_b)) en_counter[1]++;
    else en_counter[1]--;
  } else {
    if (digitalRead(encoder_2_b)) en_counter[1]++;
    else en_counter[1]--;
  }
}
void encoder_handler_2_b() {
  if (digitalRead(encoder_2_b)) {
    if (digitalRead(encoder_2_a)) en_counter[1]++;
    else en_counter[1]--;
  } else {
    if (!digitalRead(encoder_2_a)) en_counter[1]++;
    else en_counter[1]--;
  }
}

void encoder_handler_3_a() {
  if (digitalRead(encoder_3_a)) {
    if (!digitalRead(encoder_3_b)) en_counter[2]--;
    else en_counter[2]++;
  } else {
    if (digitalRead(encoder_3_b)) en_counter[2]--;
    else en_counter[2]++;
  }
}
void encoder_handler_3_b() {
  if (digitalRead(encoder_3_b)) {
    if (digitalRead(encoder_3_a)) en_counter[2]--;
    else en_counter[2]++;
  } else {
    if (!digitalRead(encoder_3_a)) en_counter[2]--;
    else en_counter[2]++;
  }
}

void calculate_motor_speed_from_encoder(float time_elapsed) {
  for (int i = 0; i < 3; i++) {                                                   // for each wheel
    float en_counted = en_counter_old[i] - en_counter[i];                         // determine how many paulses has elapsed
    en_counter_old[i] = en_counter[i];                                            // update the counter log for each wheel
    wheel_w[i] = ((en_counted / rotation_pls) * 2000 * PI) / time_elapsed;        // calculate the omega of each wheel
  }
}
