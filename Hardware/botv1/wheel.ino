//const int STUCK_COUNT = 32;
//const int SMALL_STEP = 125;
//const int LARGE_STEP = 250;
const int STUCK_COUNT = 16;
const int TINY_STEP = 50;
const int SMALL_STEP = 100;
const int LARGE_STEP = 200;

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
  if ((frontDistance < COLLISION_THRESHOLD) ||
      (rightDistance < COLLISION_THRESHOLD) ||
      (leftDistance < COLLISION_THRESHOLD)) {
    GoBackward();
    delay(TINY_STEP);

    // if both left and right sides are tight
    if ((rightDistance < COLLISION_THRESHOLD)
        && (leftDistance < COLLISION_THRESHOLD)) {
      GoBackward();
    } else {
      // if only one side is tight, turn to the other side
      if (rightDistance < COLLISION_THRESHOLD) {
        //        GoLeft();
        TurnLeft();
      }
      else if (leftDistance < COLLISION_THRESHOLD) {
        //        GoRight();
        TurnRight();
      }
      // if only front is blocked, pick a more open side and turn
      else {
        PickSideAndTurn(TINY_STEP);
      }
    }
    return;
  }

  // if all clear, go forward
  GoForward();
}

//  // otherwise, it may roam
//
//  if (frontDistance <= AVOIDANCE_THRESHOLD) {
//
//    // if blocked but right side is clear, turn right
//    if (rightDistance > AVOIDANCE_THRESHOLD) {
//      GoRight();
//      return;
//    }
//
//    // if left side is clear, turn left
//    if (leftDistance > AVOIDANCE_THRESHOLD) {
//      // turn left if that's the only option
//      GoLeft();
//      return;
//    }
//
//    // if nothing is clear, back up
//    GoBackward();
//    delay(LARGE_STEP);
//    return;
//
//  }


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
