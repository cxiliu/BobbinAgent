void actuateRightGripper(bool grippp) {
  if (grippp) {
    lowerBobbins();
    rightGripper.write(180 - CLOSE_DEG);
    delay(1000);
    liftBobbins();
  }
  else {
    lowerBobbins();
    rightGripper.write(180 - OPEN_DEG);
    delay(1000);
    liftBobbins();
  }
}


void actuateLeftGripper(bool grippp) {
  if (grippp) {
    lowerBobbins();
    leftGripper.write(CLOSE_DEG);
    delay(1000);
    liftBobbins();
  }
  else {
    lowerBobbins();
    leftGripper.write(OPEN_DEG);
    delay(1000);
    liftBobbins();
  }
}

bool LowerSwitchTriggered() {
  if (!digitalRead(R_LOW_POS)) {
    lowerTriggered = true;
    //Serial.println("right low switch triggered");
    return lowerTriggered;
  }
  if (!digitalRead(L_LOW_POS)) {
    lowerTriggered = true;
    //Serial.println("left low switch triggered");
    return lowerTriggered;
  }
  lowerTriggered = false;
  return lowerTriggered;
}

bool TopSwitchTriggered() {
  if (!digitalRead(TOP_POS)) {
    upperTriggered = true;
    Serial.println("top switch triggered");
  } else {
    upperTriggered = false;
  }
  return upperTriggered;
}

int NumOfSteps = 850; // FULL RANGE 850
//const int STEPPER_DELAY = 2;
const int STEPPER_DELAY = 2;
//int NumOfSteps = 425;
//int NumOfSteps = 50;

void liftBobbins() {
  //  if (TopSwitchTriggered()) {
  //    Serial.println("warning: top limit switch preventing movement");
  //    return;
  //  }
  int count = 0;
  digitalWrite(DIR, LOW);
  delay(100);
  while (count <= NumOfSteps) { // 20mm
    if (TopSwitchTriggered()) {
      Serial.println("warning: top limit switch preventing movement");
      Serial.println(count);
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
  //  if (LowerSwitchTriggered()) {
  //    Serial.println("warning: lower limit switch preventing movement");
  //    return;
  //  }
  int count = 0;
  digitalWrite(DIR, HIGH);
  delay(100);
  while (count <= NumOfSteps) { // 20mm
    if (LowerSwitchTriggered()) {
      Serial.println("warning: lower limit switch preventing movement");
      Serial.println(count);
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

void runGripperCycle() {
  Stop();
  delay(1000);
  // gripper cycle:
  // empty -1> left gripper -2> left+right gripper -3> right gripper -4> empty
  if (leftGripperClosed) {
    if (rightGripperClosed) {
      //3: full -> right only
      //Serial.println("open left");
      currentMode = "openL";
      actuateLeftGripper(false);
      leftGripperClosed = false;
    } else {
      // 2: left only -> full
      //Serial.println("grip right");
      currentMode = "gripR";
      actuateRightGripper(true);
      rightGripperClosed = true;
    }
  } else {
    if (rightGripperClosed) {
      //4: right only -> empty
      //Serial.println("open right");
      currentMode = "openR";
      actuateRightGripper(false);
      rightGripperClosed = false;
    } else {
      //1: empty -> left only
      //Serial.println("grip left");
      currentMode = "gripL";
      actuateLeftGripper(true);
      leftGripperClosed = true;
    }
  }

  writeData();
  // end of gripper cycle
}

//bool tryGrip() {
//  if (frontDistance == PICKING_DISTANCE) {
//    if ((leftDistance > frontDistance + TARGET_OFFSET)
//        && (rightDistance > frontDistance + TARGET_OFFSET)) {
//
//      runGripperCycle();
//      return true;
//    }
//  }
//  return false;
//}

//void grabRight() {
//  TurnLeft();
//  delay(TURN_90_STEP);
//  Stop();
//  delay(200);
//  GoRight();
//  delay(200);
//  Stop();
//  delay(600);
//  actuateRightGripper(true);
//  delay(600);
//  GoLeft();
//  delay(TURN_90_STEP);
//  //  TurnRight();
//  //  delay(200);
//  Stop();
//}
//
//void grabLeft() {
//  TurnRight();
//  delay(TURN_90_STEP);
//  Stop();
//  delay(200);
//  GoLeft();
//  delay(200);
//  Stop();
//  delay(600);
//  actuateLeftGripper(true);
//  delay(600);
//  TurnLeft();
//  delay(TURN_90_STEP);
//  Stop();
//}
