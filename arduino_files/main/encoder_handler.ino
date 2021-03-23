boolean update_wheel_speed(void *) {
  for (int i = 0; i < 3; i++) {                                                   // for each wheel
    float en_counted = abs(en_counter[i] - en_counter_old[i]);                    // determine how many paulses has elapsed
    long en_time_elapsed = millis() - en_counter_time;                            // the sampling timing window
    en_counter_old[i] = en_counter[i];                                            // update the counter log for each wheel
    wheel_w[i] = ((en_counted / rotation_pls) * 2000 * PI) / en_time_elapsed;     // calculate the omega of each wheel
  }
  en_counter_time = millis();                                                     // log the last sampling time
  return true;                                                                    // is necessary for the timer to resume
}

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
    if (!digitalRead(encoder_3_b)) en_counter[2]++;
    else en_counter[2]--;
  } else {
    if (digitalRead(encoder_3_b)) en_counter[2]++;
    else en_counter[2]--;
  }
}
void encoder_handler_3_b() {
  if (digitalRead(encoder_3_b)) {
    if (digitalRead(encoder_3_a)) en_counter[2]++;
    else en_counter[2]--;
  } else {
    if (!digitalRead(encoder_3_a)) en_counter[2]++;
    else en_counter[2]--;
  }
}
