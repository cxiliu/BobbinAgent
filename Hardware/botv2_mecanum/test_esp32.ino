
// turn 360 degrees
void twist() {
  setGrippers();
  StartLocomotion();
  TurnLeft(360);
  LoopPID();
  setGrippers();
  return;
}

// ============================
// RIGHT GRIPPER FUNCTIONS
// ============================
// after approach bobbin from front, rotate to align gripper
void alignRightGripper(int degree) {
  setGrippers();
  StartLocomotion();
  TurnLeft(85 + degree);
  LoopPID();
  setGrippers();
  return;
}

// after turning to align with gripper, move closer to the bobbin
void approachRightGripper() {
  setGrippers();
  StartLocomotion();
  GoRight(5);
  LoopPID();
  setGrippers();
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
void alignLeftGripper(int degree) {
  setGrippers();
  StartLocomotion();
  TurnRight(90 + abs(degree));
  LoopPID();
  setGrippers();
  return;
}

// after turning to align with gripper, move closer to the bobbin
void approachLeftGripper(int dist) {
  setGrippers();
  StartLocomotion();
  GoLeft(7);
  LoopPID();
  setGrippers();
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
  setGrippers();
  if (degree > 0) {
    StartLocomotion();
    TurnLeft(degree); //counterclockwise
    LoopPID();
    setGrippers();
    return;
  }
  else {
    StartLocomotion();
    TurnRight(-degree); //clockwise
    LoopPID();
    setGrippers();
    return;
  }
}

// move forward in centimeters
void moveForwardDistance(int distance) {
  setGrippers();
  StartLocomotion();
  GoForward(distance);
  LoopPID();
  setGrippers();
  return;
}

// move back in centimeters
void moveBackwardDistance(int distance) {
  setGrippers();
  StartLocomotion();
  GoBackward(distance);
  LoopPID();
  setGrippers();
  return;
}

// move left in centimeters
void moveLeftDistance(int distance) {
  setGrippers();
  StartLocomotion();
  GoLeft(distance);
  LoopPID();
  setGrippers();
  return;
}

// move right in centimeters
void moveRightDistance(int distance) {
  setGrippers();
  StartLocomotion();
  GoRight(distance);
  LoopPID();
  setGrippers();
  return;
}

// cross motion for robot on the left (facing same way as robot)
void Cross1() {
  moveBackwardDistance(CROSS_DISTANCE_V);
  delay(500);
  moveRightDistance(CROSS_DISTANCE_H);
  delay(200);
  moveRightDistance(CROSS_DISTANCE_H);
  delay(500);
  moveForwardDistance(CROSS_DISTANCE_V);
  delay(500);
  rotateDegree(-180);
}

// cross motion for robot on the right (facing same way as robot)
void Cross2() {
  moveForwardDistance(CROSS_DISTANCE_V);
  delay(500);
  moveLeftDistance(CROSS_DISTANCE_H);
  delay(200);
  moveLeftDistance(CROSS_DISTANCE_H);
  delay(500);
  moveBackwardDistance(CROSS_DISTANCE_V);
  delay(500);
  rotateDegree(-180);
}
