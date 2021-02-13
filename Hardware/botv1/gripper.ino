const int TURN_COUNT = 300;
void runGripperCycle() {
  // gripper cycle:
  // empty -1> left gripper -2> left+right gripper -3> right gripper -4> empty
  if (leftGripperClosed) {
    if (rightGripperClosed) {
      //3: full -> right only
      Serial.println("open left");
      currentMode = "openL";
      actuateLeftGripper(false);
      leftGripperClosed = false;
    } else {
      // 2: left only -> full
      Serial.println("grip right");
      currentMode = "gripR";
      //      actuateRightGripper(true);
      grabRight();
      rightGripperClosed = true;
    }
  } else {
    if (rightGripperClosed) {
      //4: right only -> empty
      Serial.println("open right");
      currentMode = "openR";
      actuateRightGripper(false);
      rightGripperClosed = false;
    } else {
      //1: empty -> left only
      Serial.println("grip left");
      currentMode = "gripL";
      //      actuateLeftGripper(true);
      grabLeft();
      leftGripperClosed = true;
    }
  }

  writeData();
  // end of gripper cycle
}

void grabRight() {
  TurnLeft();
  delay(TURN_COUNT);
  Stop();
  delay(200);
  GoRight();
  delay(200);
  Stop();
  delay(600);
  actuateRightGripper(true);
  delay(600);
  GoLeft();
  delay(TURN_COUNT);
  //  TurnRight();
  //  delay(200);
  Stop();
}

void grabLeft() {
  TurnRight();
  delay(TURN_COUNT);
  Stop();
  delay(200);
  GoLeft();
  delay(200);
  Stop();
  delay(600);
  actuateLeftGripper(true);
  delay(600);
  TurnLeft();
  delay(TURN_COUNT);
  Stop();
}

void actuateRightGripper(bool grippp) {
  if (grippp) {
    rightGripper.write(CLOSE_DEG);
    delay(1000);
    rightLifter.write(HIGH_DEG);
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else {
    rightGripper.write(OPEN_DEG);
    delay(1000);
    rightLifter.write(LOW_DEG);
    digitalWrite(LED_BUILTIN, LOW);
  }
}


void actuateLeftGripper(bool grippp) {
  if (grippp) {
    leftGripper.write(CLOSE_DEG);
    delay(1000);
    leftLifter.write(HIGH_DEG);
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else {
    leftGripper.write(OPEN_DEG);
    delay(1000);
    leftLifter.write(LOW_DEG);
    digitalWrite(LED_BUILTIN, LOW);
  }
}
