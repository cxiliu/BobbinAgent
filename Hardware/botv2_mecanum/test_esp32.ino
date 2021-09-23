
// turn 360 degrees
void twist() {
  StartLocomotion();
  TurnLeft(360);
  LoopPID();
  return;
}

// ============================
// RIGHT GRIPPER FUNCTIONS
// ============================
// after approach bobbin from front, rotate to align gripper
void alignRightGripper(int degree){
    StartLocomotion();
    TurnLeft(85 + degree);
    LoopPID();
    return;
}

// after turning to align with gripper, move closer to the bobbin
void approachRightGripper(){
  StartLocomotion();
  GoRight(5);
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
    TurnRight(90 + abs(degree));
    LoopPID();
    return;
}

// after turning to align with gripper, move closer to the bobbin
void approachLeftGripper(int dist){
  StartLocomotion();
  GoLeft(7);
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
    TurnLeft(degree); //counterclockwise
    LoopPID();
    return;
  }
  else {
    StartLocomotion();
    TurnRight(-degree); //clockwise
    LoopPID();
    return;
  }
}

// move forward in centimeters
void moveForwardDistance(int distance) {
  StartLocomotion();
  GoForward(distance);
  LoopPID();
  return;
}

// move back in centimeters
void moveBackwardDistance(int distance) {
  StartLocomotion();
  GoBackward(distance);
  LoopPID();
  return;
}

// move left in centimeters
void moveLeftDistance(int distance) {
  StartLocomotion();
  GoLeft(distance);
  LoopPID();
  return;
}

// move right in centimeters
void moveRightDistance(int distance) {
  StartLocomotion();
  GoRight(distance);
  LoopPID();
  return;
}


void Cross1(){
  moveBackwardDistance(10);
  delay(500);
  moveRightDistance(10);
  delay(200);
  moveRightDistance(10);
  delay(500);
  moveForwardDistance(10);
  delay(500);
  rotateDegree(-180);
}

void Cross2(){
  moveForwardDistance(10);
  delay(500);
  moveLeftDistance(10);
  delay(200);
  moveLeftDistance(10);
  delay(500);
  moveBackwardDistance(10);
  delay(500);
  rotateDegree(-180);
}
