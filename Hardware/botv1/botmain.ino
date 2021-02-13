void loop() {
  // if bot is waiting for server, do nothing
  if (waiting) {
    return;
  }

  // get sensor inputs
  //  sensorCounter += 1;
  //  if (sensorCounter == 30) {
  measureDistance();
  //    headingData = measureHeading();
  sensorCounter = 0;
  //    Serial.println("new readings! ");
  //  }

  // listen for joystick press
  int toggle = digitalRead(sw);
  if (toggle == 0) {
    pressed = true;
  }
  else {
    if (pressed == true) {
      pressed = false;

      // either use joystick to toggle manual / auto
      autoMode = !autoMode;
      if (autoMode) {
        currentMode = "auto";
      } else {
        Stop();
        currentMode = "manual";
      }

      // or use it to run gripper cycle
      //            runGripperCycle();
    }
  }

  // grip if possible
  if (frontDistance == PICKING_DISTANCE) {
    if ((leftDistance > frontDistance)
        && (rightDistance > frontDistance)) {
      runGripperCycle();
    }
  }

  // locomotion
  if (autoMode) {
    // obstacle avoidance
    Locomote();
  } else {
    // listen for joystick pushes
    joystickCommand = getJoystickDirection();
    if (joystickCommand != lastJoystickCommand) {
      lastJoystickCommand = joystickCommand;
      SetMotors(joystickCommand);
      //      Serial.println(DirName);
    }
  }

  // show info on lcd
  writeData();

  delay(10);

  //  if (Serial.available() > 0) {
  //    // read the incoming byte:
  //    incomingByte = Serial.read();
  //
  //      //s 115
  //      //d 100
  //      //encode incoming bytes to bot instructions
  //
  //      if (incomingByte == 115){
  //      }
  //      // say what you got:
  //    Serial.print("I received: ");
  //    Serial.println(incomingByte, DEC);
  //  }
}
