
// turn 360 degrees
void twist() {
  StartLocomotion();
  TurnLeft45(360);
  LoopPID();
  return;
}

// ============================
// RIGHT GRIPPER FUNCTIONS
// ============================
// after approach bobbin from front, rotate to align gripper
void alignRightGripper(int degree){
    StartLocomotion();
    TurnLeft45(85 + degree);
    LoopPID();
    return;
}

// after turning to align with gripper, move closer to the bobbin
void approachRightGripper(){
  StartLocomotion();
  GoRight45(5);
  LoopPID();
  return;
}

// lower gripper, close, raise gripper
void pickupRight() {
  actuateRightGripper(true);
  delay(SMALL_STEP);
}

// lower gripper, open, raise gripper
void dropoffRight() {
  actuateRightGripper(false);
  delay(SMALL_STEP);
}
// ============================

// ============================
// LEFT GRIPPER FUNCTIONS
// ============================
// after approach bobbin from front, rotate to align gripper
void alignLeftGripper(int degree){
    StartLocomotion();
    TurnRight45(90 + abs(degree));
    LoopPID();
    return;
}

// after turning to align with gripper, move closer to the bobbin
void approachLeftGripper(int dist){
  StartLocomotion();
  GoLeft45(7);
  LoopPID();
  return;
}

// lower gripper, close, raise gripper
void pickupLeft() {
  actuateLeftGripper(true);
  delay(SMALL_STEP);
}

// lower gripper, open, raise gripper
void dropoffLeft() {
  actuateLeftGripper(false);
  delay(SMALL_STEP);
}
// ============================

// ============================
// LOCOMOTION FUNCTIONS
// ============================
// turn defined degrees and stop
void rotateDegree(int degree) {
  if (degree > 0) {
    StartLocomotion();
    TurnLeft45(degree);
    LoopPID();
    return;
  }
  else {
    StartLocomotion();
    TurnRight45(-degree);
    LoopPID();
    return;
  }
}

// move forward in centimeters
void moveForwardDistance(int distance) {
  StartLocomotion();
  GoForward45(distance);
  LoopPID();
  return;
}

// move left in centimeters
void moveLeftDistance(int distance) {
  StartLocomotion();
  GoLeft45(distance);
  LoopPID();
  return;
}

// move right in centimeters
void moveRightDistance(int distance) {
  StartLocomotion();
  GoRight45(distance);
  LoopPID();
  return;
}
