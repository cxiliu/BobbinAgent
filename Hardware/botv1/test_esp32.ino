
// turn 360 degrees and stop
void twist() {
  TurnLeft45(90);
  delay(TURN_90_STEP * 4);
  Stop();
}

// turn, approach bobbin, close right gripper, move away, turn back, and stop
void pickup() {
  TurnLeft45(90);
  Stop();
  delay(SMALL_STEP);

  GoRight45(5);
  Stop();
  delay(SMALL_STEP);

  actuateRightGripper(true);
  delay(SMALL_STEP);
  
  GoLeft45(5);
  Stop();
  delay(SMALL_STEP);

  TurnRight45(90);
  Stop();
  delay(SMALL_STEP);

  actuateRightGripper(false);
  delay(SMALL_STEP);
}

// open right gripper, move away 5cm, and stop
void dropoff() {
  actuateRightGripper(false);
  delay(SMALL_STEP);
  GoLeft45(5);
  delay(LARGE_STEP);
  Stop();
}

// turn defined degrees and stop
void rotateDegree(int degree) {
  if (degree > 0) {
    TurnLeft45(degree);
  }
  else {
    TurnRight45(degree);
  }
}

// move forward in centimeters
void moveForwardDistance(int distance) {
  GoForward45(distance);
}
