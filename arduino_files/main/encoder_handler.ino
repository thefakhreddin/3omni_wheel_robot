void encoder_handler_1_a() {
  if (digitalRead(encoder_1_a)) {
    if (!digitalRead(encoder_1_b)) en_counter[0]++;
    else en_counter[0]--;
  } else {
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
