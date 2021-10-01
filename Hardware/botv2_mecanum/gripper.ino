const int APPROACH_DISTANCE = 15;//10; // distance robot moves sideways to reach the bobbin
const int SIDE_DISTANCE_GRAB = 4; // gripper reaches the bobbin best at this distance
const int SWEEP_STEPS = 2;
const int ANGLE_INCREMENT = 16;

void pickupRightAdaptive(bool withLift) {
  // add front distance check here?
  rightGripper.write(OPEN_DEG_R);
  rotateDegree(90);
  delay(200);
  moveBackwardDistance(2);
  delay(200);
  findOptimalAngle(false);
  //Serial.println(rightDistance);
  if (rightDistance < SIDE_DISTANCE_GRAB + APPROACH_DISTANCE) {
    moveRightDistance(rightDistance - SIDE_DISTANCE_GRAB);
    delay(200);
    if (withLift) {
      actuateRightGripper(true);
    } else {
      rightGripper.write(CLOSE_DEG_R);
    }
  } else {
    //Serial.println("bobbin is too far to grip right");
  }
}

void pickupLeftAdaptive(bool withLift) {
  // add front distance check here?
  leftGripper.write(OPEN_DEG_L);
  rotateDegree(-90);
  delay(200);
  moveBackwardDistance(2);
  delay(200);
  findOptimalAngle(true);
  //Serial.println(leftDistance);
  if (leftDistance < SIDE_DISTANCE_GRAB + APPROACH_DISTANCE) {
    moveLeftDistance(leftDistance - SIDE_DISTANCE_GRAB);
    delay(200);
    if (withLift) {
      actuateLeftGripper(true);
    } else {
      leftGripper.write(CLOSE_DEG_L);
    }
    //Serial.println("gripped !");
  } else {
    //Serial.println("bobbin is too far to grip left");
  }
}

///////////////////////////////////////////////////////////////////////////////////////////

void actuateRightGripper(bool grippp) {
  if (grippp) {
    lowerBobbins();
    rightGripper.write(CLOSE_DEG_R);
    delay(1000);
    liftBobbins();
  }
  else {
    lowerBobbins();
    rightGripper.write(OPEN_DEG_R);
    delay(1000);
    liftBobbins();
  }
}

void actuateLeftGripper(bool grippp) {
  if (grippp) {
    lowerBobbins();
    leftGripper.write(CLOSE_DEG_L);
    delay(1000);
    liftBobbins();
  }
  else {
    lowerBobbins();
    leftGripper.write(OPEN_DEG_L);
    delay(1000);
    liftBobbins();
  }
}

///////////////////////////////////////////////////////////////////////////////////////////

bool LowerSwitchTriggered() {
  if (!digitalRead(R_LOW_POS)) {
    lowerTriggered = true;
    //Serial.println("low switch triggered");
    return lowerTriggered;
  }
  if (!digitalRead(L_LOW_POS)) {
    lowerTriggered = true;
    //Serial.println("low switch triggered");
    return lowerTriggered;
  }
  lowerTriggered = false;
  return lowerTriggered;
}

bool TopSwitchTriggered() {
  if (!digitalRead(TOP_POS)) {
    upperTriggered = true;
    //Serial.println("top switch triggered");
  } else {
    upperTriggered = false;
  }
  return upperTriggered;
}

int NumOfSteps = 1600;
//int NumOfSteps = 780; // FULL RANGE 850
const int STEPPER_DELAY = 2;
//const int STEPPER_DELAY = 1;
//int NumOfSteps = 425;
//int NumOfSteps = 50;

void liftBobbins() {
  int count = 0;
  digitalWrite(DIR, LOW);
  delay(100);
  while (count <= NumOfSteps) { // 20mm
    if (TopSwitchTriggered()) {
      //Serial.println("warning: top limit switch preventing movement");
      //Serial.println(count);
      return;
    }
    digitalWrite(DIR, LOW);
    digitalWrite(STEP, HIGH);
    delay(STEPPER_DELAY);
    digitalWrite(STEP, LOW);
    delay(STEPPER_DELAY);
    count += 1;
  }
  delay(800);
}

void lowerBobbins() {
  int count = 0;
  digitalWrite(DIR, HIGH);
  delay(100);
  while (count <= NumOfSteps) { // 20mm
    if (LowerSwitchTriggered()) {
      //Serial.println("warning: lower limit switch preventing movement");
      //Serial.println(count);
      return;
    }
    digitalWrite(DIR, HIGH);
    digitalWrite(STEP, HIGH);
    delay(STEPPER_DELAY);
    digitalWrite(STEP, LOW);
    delay(STEPPER_DELAY);
    count += 1;
  }
  delay(800);
}

void findOptimalAngle(bool isLeft) {
  // go to the starting point
  for (int i = 0; i < SWEEP_STEPS; i++) {
    rotateDegree(isLeft? ANGLE_INCREMENT : -ANGLE_INCREMENT);
    delay(100);
  }

  // populate reading into the list of distances
  int distanceField[SWEEP_STEPS * 2 + 1];
  for (int i = 0; i < SWEEP_STEPS * 2 + 1; i++) {
    //Serial.print("--- position "); Serial.print(i); Serial.print(" : ");
    if (isLeft) {
      measureDistanceLeftFiltered();
      distanceField[i] = leftDistance;
    } else {
      measureDistanceRightFiltered();
      distanceField[i] = rightDistance;
    }
    //Serial.println(distanceField[i]);
    rotateDegree(isLeft? -ANGLE_INCREMENT : ANGLE_INCREMENT);
    delay(200);
  }

  // find the smallest distance and angle value
  int minIndex = SWEEP_STEPS;
  int minDistance = 999;
  for (int i = 0; i < SWEEP_STEPS * 2 + 1; i++) {
    if (distanceField[i] < minDistance) {
      minDistance = distanceField[i];
      minIndex = i;
    }
  }
  //Serial.print(minIndex); Serial.print(" is smallest at "); Serial.println(minDistance);

  // align to the best angle
  //Serial.println("starting to align");
  minDistance += 1; // account for wiggle room
  int attempts = 0;
  if (isLeft) {
    measureDistanceLeftFiltered();
    Serial.println(leftDistance);
  } else {
    measureDistanceRightFiltered();
    Serial.println(rightDistance);
  }
  while (attempts < SWEEP_STEPS * 2 + 1) {
    if (isLeft) {
      if (leftDistance <= minDistance) {
        //Serial.print(leftDistance); Serial.println(" reached!");
        break;
      } else {
        rotateDegree(isLeft? ANGLE_INCREMENT : -ANGLE_INCREMENT);
        delay(100);
        measureDistanceLeftFiltered();
        //Serial.println(leftDistance);
      }
    } else {
      if (rightDistance <= minDistance) {
        //Serial.print(rightDistance); Serial.println(" reached!");
        break;
      } else {
        rotateDegree(isLeft? ANGLE_INCREMENT : -ANGLE_INCREMENT);
        delay(100);
        measureDistanceRightFiltered();
        //Serial.println(rightDistance);
      }
    }
    attempts += 1;
    delay(100);
  }
  //Serial.print("alignment complete after: "); Serial.print(attempts); Serial.println(" turns");
}

//bool tryGrip() {
//  if (frontDistance == PICKING_DISTANCE) {
//    if ((leftDistance > frontDistance + TARGET_OFFSET)
//        && (rightDistance > frontDistance + TARGET_OFFSET)) {
//      Stop();
//      delay(1000);
//      // gripper cycle:
//      // empty -1> left gripper -2> left+right gripper -3> right gripper -4> empty
//      if (leftGripperClosed) {
//        if (rightGripperClosed) {
//          //3: full -> right only
//          //Serial.println("open left");
//          currentMode = "openL";
//          actuateLeftGripper(false);
//          leftGripperClosed = false;
//        } else {
//          // 2: left only -> full
//          //Serial.println("grip right");
//          currentMode = "gripR";
//          actuateRightGripper(true);
//          rightGripperClosed = true;
//        }
//      } else {
//        if (rightGripperClosed) {
//          //4: right only -> empty
//          //Serial.println("open right");
//          currentMode = "openR";
//          actuateRightGripper(false);
//          rightGripperClosed = false;
//        } else {
//          //1: empty -> left only
//          //Serial.println("grip left");
//          currentMode = "gripL";
//          actuateLeftGripper(true);
//          leftGripperClosed = true;
//        }
//      }
//      writeData();
//      // end of gripper cycle
//      return true;
//    }
//  }
//  return false;
//}
