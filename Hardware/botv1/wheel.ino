const int STUCK_COUNT = 64;

bool IsStuck() {
  if (abs(frontDistance - lastFrontDistance) < 3) {
    stuckCounter += 1;
  } else {
    stuckCounter = 0;
    lastFrontDistance = frontDistance;
  }
  if (stuckCounter >= STUCK_COUNT) {
    return true;
  } else {
    return false;
  }
}

void PickSideAndTurn(int count) {
  if (leftDistance > rightDistance) {
    TurnLeft();
    delay(count);
  } else {
    TurnRight();
    delay(count);
  }
}

void Locomote() {
  // if the bot is stuck, do not move until human moves it
  if (IsStuck()) {
    Stop();
    return;
  }

  // collision avoidance
  measureDistance();

  // if either one reading is below collision threshold, do something
  if ((frontDistance < COLLISION_THRESHOLD) ||
      (rightDistance < COLLISION_THRESHOLD) ||
      (leftDistance < COLLISION_THRESHOLD)) {

    // if both left and right sides are tight, go back and turn to more open side
    if ((rightDistance < COLLISION_THRESHOLD)
        && (leftDistance < COLLISION_THRESHOLD)) {
      GoBackward();
      delay(SMALL_STEP);
      PickSideAndTurn(TINY_STEP);
    } else {
      // if only one side is tight, back up a bit and turn to the other side
      if (rightDistance < COLLISION_THRESHOLD) {
        GoBackward();
        delay(TINY_STEP);
        TurnLeft();
        delay(SMALL_STEP);
      }
      else if (leftDistance < COLLISION_THRESHOLD) {
        GoBackward();
        delay(TINY_STEP);
        TurnRight();
        delay(SMALL_STEP);
      }
      // if only front is blocked
      // the bot is correctly aligned with a bobbin! approach and grip
      else {
        // move forward or backward set distance
        int distance = frontDistance - PICKING_DISTANCE;
        if ( distance > 0) {
          GoForward();
          delay(TINY_STEP * distance);
        } else if (distance < 0) {
          GoBackward();
          delay(-TINY_STEP * distance);
        }
        // if gripping is not successful, go back and turn a random side
        if (!tryGrip()) {
          //          GoForward();
          //          delay(SMALL_STEP);
          //          if (!tryGrip()) {
          GoBackward();
          delay(SMALL_STEP);
          PickSideAndTurn(SMALL_STEP);
        }
      }
    }
  } else {
    // if all clear, go forward
    GoForward();
    delay(100);
  }


  Stop();
  delay(50);
}


// BACK - LEFT BACK WHEEL - LEFT CHIP IN1IN2 (in1 CC)
// RIGHT - RIGHT BACK WHEEL - LEFT CHIP IN3IN4
// LEFT - LEFT FRONT WHEEL - RIGHT CHIP IN1IN2 (in1 CC)
// FRONT - RIGHT FRONT WHEEL - RIGHT CHIP IN3IN4 (in1 C)

//ps: front wheel is soldered backwards so reverse that one

bool disableMotors = false;

void Stop() {
  if (disableMotors) return;
  SetLeftWheel(false, false);
  SetRightWheel(false, false);
  SetFrontWheel(false, false);
  SetBackWheel(false, false);
  currentDirection = "stopped";
}

void GoForward() {
  if (disableMotors) return;
  SetLeftWheel(true, false);
  SetRightWheel(true, true);
  SetFrontWheel(false, false);
  SetBackWheel(false, false);
  currentDirection = "forward";
}

void GoBackward() {
  if (disableMotors) return;
  SetLeftWheel(true, true);
  SetRightWheel(true, false);
  SetFrontWheel(false, false);
  SetBackWheel(false, false);
  currentDirection = "backward";
}

void GoLeft() {
  if (disableMotors) return;
  SetLeftWheel(false, false);
  SetRightWheel(false, false);
  SetFrontWheel(true, false);
  SetBackWheel(true, false);
  currentDirection = "left";
}

void GoRight() {
  if (disableMotors) return;
  SetLeftWheel(false, false);
  SetRightWheel(false, false);
  SetFrontWheel(true, true);
  SetBackWheel(true, true);
  currentDirection = "right";
}

void TurnLeft() {
  if (disableMotors) return;
  SetLeftWheel(true, true);
  SetRightWheel(true, true);
  SetFrontWheel(true, false);
  SetBackWheel(true, true);
  currentDirection = "turn left";
}

void TurnRight() {
  if (disableMotors) return;
  SetLeftWheel(true, false);
  SetRightWheel(true, false);
  SetFrontWheel(true, true);
  SetBackWheel(true, false);
  currentDirection = "turn right";
}

void SetLeftWheel(bool on, bool clockwise) {
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

void SetFrontWheel(bool on, bool clockwise) {
  analogWrite(F_enA, F_motorSpeed);
  if (!on) {
    digitalWrite(F_in1, LOW);
    digitalWrite(F_in2, LOW);
  }
  else {
    if (clockwise) {
      digitalWrite(F_in1, LOW);
      digitalWrite(F_in2, HIGH);
    } else {
      digitalWrite(F_in1, HIGH);
      digitalWrite(F_in2, LOW);
    }
  }
}

void SetBackWheel(bool on, bool clockwise) {
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

void SetRightWheel(bool on, bool clockwise) {
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
