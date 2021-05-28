void loop() {

  if (LOCOMOTION_ACTIVE) {
    MotorPID();
  }

  // get sensor inputs
  if (DistanceOn) {
    measureDistance();
    //  measureDistanceFront();
  }

  //  Serial.println(analogRead(STALL_DETECTION));


  // listen for joystick pressw
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
  //  tryGrip()

  // locomotion
  //  if (autoMode) {
  //    Locomote();
  //  } else {
  //    // listen for joystick pushes
  //    joystickCommand = getJoystickDirection();
  //    if (joystickCommand != lastJoystickCommand) {
  //      lastJoystickCommand = joystickCommand;
  //      if (joystickCommand != 5) {
  //        int on = 30;
  //        int off = 20;
  //        // six cycles
  //        for (int i = 0; i < 6; i++) {
  //          SetMotors(joystickCommand);
  //          delay(on);
  //          SetMotors(5);
  //          delay(off);
  //        }
  //        delay(500);
  //      }
  //      else {
  //        SetMotors(joystickCommand );
  //      }
  //    }
  //  }

  // show info on lcd
  if (DisplayOn) {
    writeData();
  }

  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    //    Serial.print("Received: ");
    //    Serial.println(incomingByte, DEC);

    LocomotionTest(incomingByte);
    //    CommunicationIntegrationTest(incomingByte);
  }

  //  Serial.println(digitalRead(FR_encoder));

  delay(10);

}