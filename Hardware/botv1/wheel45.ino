// BACK - LEFT BACK WHEEL - LEFT CHIP IN1IN2 (in1 CC)
// RIGHT - RIGHT BACK WHEEL - LEFT CHIP IN3IN4
// LEFT - LEFT FRONT WHEEL - RIGHT CHIP IN1IN2 (in1 CC)
// FRONT - RIGHT FRONT WHEEL - RIGHT CHIP IN3IN4 (in1 C)

//ps: FR wheel is backwards, reverse pin signals

void GoForward45() {
  if (disableMotors) return;
  SetFLWheel(true, false);
  SetFRWheel(true, true);
  SetBLWheel(true, false);
  SetBRWheel(true, true);
  currentDirection = "forward";
}

void GoBackward45() {
  if (disableMotors) return;
  SetFLWheel(true, true);
  SetFRWheel(true, false);
  SetBLWheel(true, true);
  SetBRWheel(true, false);
  currentDirection = "backward";
}

void GoLeft45() {
  if (disableMotors) return;
  SetFLWheel(true, true);
  SetFRWheel(true, true);
  SetBLWheel(true, false);
  SetBRWheel(true, false);
  currentDirection = "left";
}

void GoRight45() {
  if (disableMotors) return;
  SetFLWheel(true, false);
  SetFRWheel(true, false);
  SetBLWheel(true, true);
  SetBRWheel(true, true);
  currentDirection = "right";
}

void TurnLeft45() {
  if (disableMotors) return;
  SetFLWheel(true, true);
  SetFRWheel(true, true);
  SetBLWheel(true, true);
  SetBRWheel(true, true);
  currentDirection = "turn left";
}

void TurnRight45() {
  if (disableMotors) return;
  SetFLWheel(true, false);
  SetFRWheel(true, false);
  SetBLWheel(true, false);
  SetBRWheel(true, false);
  currentDirection = "turn right";
}


void SetBLWheel(bool on, bool clockwise) {
  analogWrite(L_enA, L_motorSpeed);
  if (!on) {
    digitalWrite(L_in1, LOW);
    digitalWrite(L_in2, LOW);
  }
  else {
    if (clockwise) {
      digitalWrite(L_in1, LOW);
      digitalWrite(L_in2, HIGH);
    } else {
      digitalWrite(L_in1, HIGH);
      digitalWrite(L_in2, LOW);
    }
  }
}

void SetFLWheel(bool on, bool clockwise) {
  analogWrite(F_enA, F_motorSpeed);
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

void SetBRWheel(bool on, bool clockwise) {
  analogWrite(B_enA, B_motorSpeed);
  if (!on) {
    digitalWrite(B_in1, LOW);
    digitalWrite(B_in2, LOW);
  }
  else {
    if (clockwise) {
      digitalWrite(B_in1, LOW);
      digitalWrite(B_in2, HIGH);
    } else {
      digitalWrite(B_in1, HIGH);
      digitalWrite(B_in2, LOW);
    }
  }
}

void SetFRWheel(bool on, bool clockwise) {
  analogWrite(R_enA, R_motorSpeed);
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
