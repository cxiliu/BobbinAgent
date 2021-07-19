bool PID_DEBUG = false;


void LoopPID(){
  while (LOCOMOTION_ACTIVE){
    MotorPID();
  }
}


void MotorPID() {
  // visualise encoder increments during movement
  //    PrintEncoderInfo(" ");
  unsigned long currentMicros = micros();
  float timeElapsed = currentMicros - previousMicros;
  previousMicros = currentMicros;

  motorUpdateTime += timeElapsed;

  // do PID every 10 ms
  if (motorUpdateTime > 10000) {

    // prevent disasters
    if (badCounter >= 150) {
      LOCOMOTION_ACTIVE = false;
      Stop();
      ResetEncoders();
      PrintEncoderInfo("--- Force Stop ---"); // FR FL BR BL
      badCounter = 0;
      return;
    }

    //    float secEllpased = motorUpdateTime / 1000000.f;
    motorUpdateTime -= 10000;

    if (FR_count == SetpointFR) {
      FR_reached = true;
      SetFRWheel(false, FR_direction);
    }
    else {
      FR_reached = false;
    }

    // FR CONTROL
    if (!FR_reached) {
      InputFR = FR_count;
      pidFR.Compute();
      int currentPwmFR = OutputFR;
      if (currentPwmFR > 0) {
        FR_reversed = false;
        currentPwmFR = map(currentPwmFR, 0, 100, minPwm, maxPwm);
        analogWrite(F_enA, currentPwmFR);
        SetFRWheel(true, FR_direction);
      }
      else {
        FR_reversed = true;
        currentPwmFR = map(-currentPwmFR, 0, 100, minPwm, maxPwm);
        analogWrite(F_enA, currentPwmFR);
        SetFRWheel(true, !FR_direction);
      }
      if (PID_DEBUG) {
        Serial.print("FR_Input: ");
        Serial.print(InputFR);
        Serial.print(",FR_PWM: ");
        Serial.println(currentPwmFR);
      }
    }

    // FL CONTROL
    if (FL_count == SetpointFL) {
      FL_reached = true;
      SetFLWheel(false, FL_direction);
    }
    else {
      FL_reached = false;
    }
    if (!FL_reached) {
      InputFL = FL_count;
      pidFL.Compute();
      int currentPwmFL = OutputFL;
      if (currentPwmFL > 0) {
        FL_reversed = false;
        currentPwmFL = map(currentPwmFL, 0, 100, minPwm, maxPwm);
        analogWrite(L_enA, currentPwmFL);
        SetFLWheel(true, FL_direction);
      }
      else {
        FL_reversed = true;
        currentPwmFL = map(-currentPwmFL, 0, 100, minPwm, maxPwm);
        analogWrite(L_enA, currentPwmFL);
        SetFLWheel(true, !FL_direction);
      }
      if (true) {
        Serial.print(",FL_Input: ");
        Serial.print(InputFL);
        Serial.print(",FL_PWM: ");
        Serial.println(currentPwmFL);
      }
    }

    // BR CONTROL
    if (BR_count == SetpointBR) {
      BR_reached = true;
      SetBRWheel(false, BR_direction);
    }
    else {
      BR_reached = false;
    }
    if (!BR_reached) {
      InputBR = BR_count;
      pidBR.Compute();
      int currentPwmBR = OutputBR;
      if (currentPwmBR > 0) {
        BR_reversed = false;
        currentPwmBR = map(currentPwmBR, 0, 100, minPwm, maxPwm);
        analogWrite(R_enA, currentPwmBR);
        SetBRWheel(true, BR_direction);
      }
      else {
        BR_reversed = true;
        currentPwmBR = map(-currentPwmBR, 0, 100, minPwm, maxPwm);
        analogWrite(R_enA, currentPwmBR);
        SetBRWheel(true, !BR_direction);
      }
      if (PID_DEBUG) {
        Serial.print(",BR_Input: ");
        Serial.print(InputBR);
        Serial.print(",BR_Pwm: ");
        Serial.println(currentPwmBR);
      }
    }

    // BL CONTROL
    if (BL_count == SetpointBL) {
      BL_reached = true;
      SetBLWheel(false, BL_direction);
    }
    else {
      BL_reached = false;
    }
    if (!BL_reached) {
      InputBL = BL_count;
      pidBL.Compute();
      int currentPwmBL = OutputBL;
      if (currentPwmBL > 0) {
        BL_reversed = false;
        currentPwmBL = map(currentPwmBL, 0, 100, minPwm, maxPwm);
        analogWrite(B_enA, currentPwmBL);
        SetBLWheel(true, BL_direction);
      }
      else {
        BL_reversed = true;
        currentPwmBL = map(-currentPwmBL, 0, 100, minPwm, maxPwm);
        analogWrite(B_enA, currentPwmBL);
        SetBLWheel(true, !BL_direction);
      }
      if (PID_DEBUG) {
        Serial.print(",BL_Input: ");
        Serial.print(InputBL);
        Serial.print(",BL_PWM: ");
        Serial.println(currentPwmBL);
      }
    }

    if (FR_reached && FL_reached && BR_reached && BL_reached) {
      LOCOMOTION_ACTIVE = false;
      Stop();
      //Serial.print("FR filtered "); Serial.print(filterActiveCountFR); Serial.print("  noise frequency: "); Serial.println((float)1000.0 / filterSignalSpeedFR);
      //Serial.print("FL filtered "); Serial.print(filterActiveCountFL); Serial.print("  noise frequency: "); Serial.println((float)1000.0 / filterSignalSpeedFL);
      //Serial.print("BR filtered "); Serial.print(filterActiveCountBR); Serial.print("  noise frequency: "); Serial.println((float)1000.0 / filterSignalSpeedBR);
      //Serial.print("BL filtered "); Serial.print(filterActiveCountBL); Serial.print("  noise frequency: "); Serial.println((float)1000.0 / filterSignalSpeedBL);
      //      if (true) {
      //        PrintEncoderInfo("--- Motion Complete ---");
      //      }
      ResetEncoders();
      badCounter = 0;
    }

    badCounter += 1;
//    Serial.println(badCounter);
    return;
  }
}
