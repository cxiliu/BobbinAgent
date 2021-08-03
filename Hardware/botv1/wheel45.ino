// BACK - LEFT BACK WHEEL - LEFT CHIP IN1IN2 (in1 CC)
// RIGHT - RIGHT BACK WHEEL - LEFT CHIP IN3IN4
// LEFT - LEFT FRONT WHEEL - RIGHT CHIP IN1IN2 (in1 CC)
// FRONT - RIGHT FRONT WHEEL - RIGHT CHIP IN3IN4 (in1 C)

//BL and BR is flipped in the motor signals


// ******* distance specified in CM ******* //

float distanceToSignals(int distance){
  float scale = 2.82; // somewhat arbitrary ratio
  return int(40.0 / scale * 10 * distance * sqrt(2) / 150.8);
}

float degreeToSignals(int degree){
  float scale = 2.0; // somewhat arbitrary ratio
  return int(40.0 / scale * (487.0 * degree / 360.0) / 150.8);
}

void GoForward45(int distance) {
  SetpointFL = distanceToSignals(distance);
  SetpointFR = SetpointFL;
  SetpointBR = SetpointFL;
  SetpointBL = SetpointFL;
  if (disableMotors) return;
  // FL and BL are cc
  FL_direction = false;//cc
  FR_direction = true;//c
  BL_direction = false;//cc
  BR_direction = true;//c
  SetFLWheel(true, FL_direction);
  SetFRWheel(true, FR_direction);
  SetBLWheel(true, BL_direction);
  SetBRWheel(true, BR_direction);
  currentDirection = "forward";
}

void GoBackward45(int distance) {
  SetpointFL = distanceToSignals(distance);
  SetpointFR = SetpointFL;
  SetpointBR = SetpointFL;
  SetpointBL = SetpointFL;
  // FR and BR are cc
  FL_direction = true;//c
  FR_direction = false;//cc
  BL_direction = true;//c
  BR_direction = false;//cc
  if (disableMotors) return;
  SetFLWheel(true, FL_direction);
  SetFRWheel(true, FR_direction);
  SetBLWheel(true, BL_direction);
  SetBRWheel(true, BR_direction);
  currentDirection = "backward";
}

void GoRight45(int distance) {
  // FL and FR are cc, but actually BL and BR
  SetpointFL = distanceToSignals(distance);
  SetpointFR = SetpointFL;
  SetpointBR = SetpointFL;
  SetpointBL = SetpointFL;
  FL_direction = false;//cc
  FR_direction = false;//cc
  BL_direction = true;//c
  BR_direction = true;//c
  if (disableMotors) return;
  SetFLWheel(true, FL_direction);
  SetFRWheel(true, FR_direction);
  SetBLWheel(true, BL_direction);
  SetBRWheel(true, BR_direction);
  currentDirection = "go right";
}

void GoLeft45(int distance) {
  // BL and BR are cc, but actually FL and FR
  SetpointFL = distanceToSignals(distance);
  SetpointFR = SetpointFL;
  SetpointBR = SetpointFL;
  SetpointBL = SetpointFL;
  FL_direction = true;//c
  FR_direction = true;//c
  BL_direction = false;//cc
  BR_direction = false;//cc
  if (disableMotors) return;
  SetFLWheel(true, FL_direction);
  SetFRWheel(true, FR_direction);
  SetBLWheel(true, BL_direction);
  SetBRWheel(true, BR_direction);
  currentDirection = "go left";
}

void TurnRight45(int degree) {
  SetpointFL = degreeToSignals(degree);
  SetpointFR = SetpointFL;
  SetpointBR = SetpointFL;
  SetpointBL = SetpointFL;
  // EVERYTHING cc
  FL_direction = false;//cc
  FR_direction = false;//cc
  BL_direction = false;//cc
  BR_direction = false;//cc
  if (disableMotors) return;
  SetFLWheel(true, FL_direction);
  SetFRWheel(true, FR_direction);
  SetBLWheel(true, BL_direction);
  SetBRWheel(true, BR_direction);
  currentDirection = "turn right";
}

void TurnLeft45(int degree) {
  SetpointFL = degreeToSignals(degree);
  
//  Serial.print("rotate left ");
//  Serial.print(degree);
//  Serial.print("  signal ");
//  Serial.println(int(40 * (487 * degree / 360.f) / 150.8));
//  Serial.print("  assigned ");
//  Serial.println(SetpointFL);
  
  SetpointFR = SetpointFL;
  SetpointBR = SetpointFL;
  SetpointBL = SetpointFL;
  // EVERYTHING c
  FL_direction = true;//c
  FR_direction = true;//c
  BL_direction = true;//c
  BR_direction = true;//c
  if (disableMotors) return;
  SetFLWheel(true, FL_direction);
  SetFRWheel(true, FR_direction);
  SetBLWheel(true, BL_direction);
  SetBRWheel(true, BR_direction);
  currentDirection = "turn left";
  
}

void Stop() {
  if (disableMotors) return;
  SetFLWheel(false, false);
  SetFRWheel(false, false);
  SetBLWheel(false, false);
  SetBRWheel(false, false);
  currentDirection = "stopped";
}

//int F_motorSpeed = 98;//0-255, FR
//int L_motorSpeed = 92;//0-255, FL
//int R_motorSpeed = 100;//0-255, BR
//int B_motorSpeed = 112;//0-255, BL

void SetFLWheel(bool on, bool clockwise) {
  //void SetBLWheel(bool on, bool clockwise) {
  if (!on) {
    digitalWrite(L_in1, LOW);
    digitalWrite(L_in2, LOW);
  }
  else {
    if (!clockwise) {
      digitalWrite(L_in1, LOW);
      digitalWrite(L_in2, HIGH);
    } else {
      digitalWrite(L_in1, HIGH);
      digitalWrite(L_in2, LOW);
    }
  }
}

//void SetFLWheel(bool on, bool clockwise) {
void SetFRWheel(bool on, bool clockwise) {
  if (!on) {
    digitalWrite(F_in1, LOW);
    digitalWrite(F_in2, LOW);
  }
  else {
    if (clockwise) {
      digitalWrite(F_in1, HIGH);
      digitalWrite(F_in2, LOW);
      //      digitalWrite(F_in1, LOW);
      //      digitalWrite(F_in2, HIGH);
    } else {
      digitalWrite(F_in1, LOW);
      digitalWrite(F_in2, HIGH);
      //      digitalWrite(F_in1, HIGH);
      //      digitalWrite(F_in2, LOW);
    }
  }
}

void SetBLWheel(bool on, bool clockwise) {
  //  void SetBRWheel(bool on, bool clockwise) {
  if (!on) {
    digitalWrite(B_in1, LOW);
    digitalWrite(B_in2, LOW);
  }
  else {
    if (!clockwise) {
      digitalWrite(B_in1, LOW);
      digitalWrite(B_in2, HIGH);
    } else {
      digitalWrite(B_in1, HIGH);
      digitalWrite(B_in2, LOW);
    }
  }
}

void SetBRWheel(bool on, bool clockwise) {
  //void SetFRWheel(bool on, bool clockwise) {
  if (!on) {
    digitalWrite(R_in1, LOW);
    digitalWrite(R_in2, LOW);
  }
  else {
    if (clockwise) {
      digitalWrite(R_in1, LOW);
      digitalWrite(R_in2, HIGH);
    } else {
      digitalWrite(R_in1, HIGH);
      digitalWrite(R_in2, LOW);
    }
  }
}

void SetInitialMotorTargets() {
  SetpointFR = 20;
  SetpointFL = 20;
  SetpointBR = 20;
  SetpointBL = 20;
}

// ENCODER INTERRUPT CALLBACKS

void FR_callback() {
  elapsedTimeFR = millis() - previousTimeFR;
  if (elapsedTimeFR >= MIN_ELAPSED_TIME)
  {
    previousTimeFR = millis();
    
    FR_count += FR_reversed ? -1 : 1;
  } else {
    filterActiveCountFR += 1;
    filterSignalSpeedFR = (elapsedTimeFR + filterSignalSpeedFR) / 2.0;
  }
}

void FL_callback() {
  elapsedTimeFL = millis() - previousTimeFL;
  if (elapsedTimeFL >= MIN_ELAPSED_TIME)
  {
    previousTimeFL = millis();
    FL_count += FL_reversed ? -1 : 1;
  }
  else {
    filterActiveCountFL += 1;
    filterSignalSpeedFL = (elapsedTimeFL + filterSignalSpeedFL) / 2.0;
  }
}

void BR_callback() {
  elapsedTimeBR = millis() - previousTimeBR;
  if (elapsedTimeBR >= MIN_ELAPSED_TIME)
  {
    previousTimeBR = millis();
    BR_count += BR_reversed ? -1 : 1;
  }
  else {
    filterActiveCountBR += 1;
    filterSignalSpeedBR = (elapsedTimeBR + filterSignalSpeedBR) / 2.0;
  }
}

void BL_callback() {
  elapsedTimeBL = millis() - previousTimeBL;
  if (elapsedTimeBL >= MIN_ELAPSED_TIME)
  {
    previousTimeBL = millis();
    BL_count += BL_reversed ? -1 : 1;
  }
  else {
    filterActiveCountBL += 1;
    filterSignalSpeedBL = (elapsedTimeBL + filterSignalSpeedBL) / 2.0;
  }
}

// ENCODER HELPER FUNCTIONS

void ResetEncoders() {
  FR_count = 0;
  FL_count = 0;
  BR_count = 0;
  BL_count = 0;
  filterActiveCountFR = 0;
  filterActiveCountFL = 0;
  filterActiveCountBR = 0;
  filterActiveCountBL = 0;
  filterSignalSpeedFR = 0;
  filterSignalSpeedFL = 0;
  filterSignalSpeedBR = 0;
  filterSignalSpeedBL = 0;
}

void PrintEncoderInfo(String prefix) {
  // http://androminarobot-english.blogspot.com/2017/03/encoder-and-arduinotutorial-about-ir.html
  // 562J 250v
  if (false) {
    Serial.print(digitalRead(FR_encoder));
    Serial.print(" ");
    Serial.print(digitalRead(FL_encoder));
    Serial.print(" ");
    Serial.print(digitalRead(BR_encoder));
    Serial.print(" ");
    Serial.println(digitalRead(BL_encoder));
  }
  else {
    Serial.print(prefix + ":  ");
    Serial.print(FR_count);
    Serial.print("  ");
    Serial.print(FL_count);
    Serial.print("  ");
    Serial.print(BR_count);
    Serial.print("  ");
    Serial.println(BL_count);
  }
}
