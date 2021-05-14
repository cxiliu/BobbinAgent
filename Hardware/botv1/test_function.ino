
// turn 360 degrees and stop
void twist() {
  TurnLeft45();
  delay(TURN_90_STEP * 4);
  Stop();
}

// turn, approach bobbin, close right gripper, move away, turn back, and stop
void pickup() {
  TurnLeft45();
  delay(TURN_90_STEP / 2);
  Stop();
  delay(SMALL_STEP);

  GoRight();
  delay(SMALL_STEP);
  Stop();
  delay(SMALL_STEP);
  actuateRightGripper(true);
  delay(SMALL_STEP);
  GoLeft();
  delay(SMALL_STEP);
  Stop();
  delay(SMALL_STEP);

  TurnRight45();
  delay(TURN_90_STEP / 2);
  Stop();

  actuateRightGripper(false);
}

// open right gripper, move away 5cm, and stop
void dropoff() {
  actuateRightGripper(false);
  delay(SMALL_STEP);
  GoLeft();
  delay(LARGE_STEP);
  Stop();
}

// turn defined degrees and stop
void rotateDegree(int degree) {
  if (degree > 0) {
    TurnLeft45();
    delay(int(degree*1.5 * (TURN_90_STEP / 90)));
    Stop();
  }
  else {
    TurnRight45();
    delay(int(-degree*1.5 * (TURN_90_STEP / 90)));
    Stop();
  }
}

// move forward in centimeters
void moveForwardDistance(int distance) {
  GoForward45();
  delay(int(distance * LARGE_STEP / 5));
  Stop();
  //  delay(SMALL_STEP);
}
