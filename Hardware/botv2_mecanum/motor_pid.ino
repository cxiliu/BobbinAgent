bool PID_DEBUG_VEL = false; // setting this to true affects loop performance
bool PID_DEBUG_POS = false; // setting this to true affects loop performance
int STEP_TOLERANCE = 1;
//const int AGG_TOLERANCE = 1;
const int APPROACH_TOLERANCE = 4;
const int FORCE_STOP_COUNT = 10;//200

// sub loop used during locomotion
void LoopPID() {
  while (LOCOMOTION_ACTIVE) {
    // the effects of pausing interrupts can be observed by inspecting the PID frequency
    // (disable the serial print statements)
    cli();
    MotorPID();
    sei();
  }
}

void StartLocomotion() {
  STEP_TOLERANCE = 1;
  now = micros();
  LOCOMOTION_ACTIVE = true;
  ResetEncoders();
  sei();
  // TODO: set locomotion goal here
}

void EndLoop(bool forceStop) {
  cli();
  LOCOMOTION_ACTIVE = false;
  Stop();
  if (!PID_DEBUG_POS && !PID_DEBUG_VEL) {
//    if (forceStop) {
//      PrintEncoderInfo("--- Force Stop ---"); // FR FL BR BL
//    }
//    else {
//      PrintEncoderInfo("--- Motion Complete ---");
//    }
//    Serial.print("loop completed in "); Serial.println(micros() - now);
  }
  ResetEncoders();
  badCounter = 0;
}

void MotorPID() {
  // execution speed of this block is important for PID to work correctly
  unsigned long currentMicros = micros();
  float timeElapsed = currentMicros - previousMicros;
  previousMicros = currentMicros;
  motorUpdateTime += timeElapsed;

  // do PID every 10 ms (while loop is around 1 ms)
  if (motorUpdateTime > 10000) {
    // prevent disasters
    if (badCounter >= FORCE_STOP_COUNT) {
      EndLoop(true);
      return;
    } else if (badCounter >= 5){
      STEP_TOLERANCE = 1;
    } 

    setGrippers();
    
    //    float secEllpased = motorUpdateTime / 1000000.f;
    motorUpdateTime -= 10000;
    if (PID_DEBUG_POS) {
      Serial.print("Target:");
      Serial.print(SetpointFR);
    }

    // FR Control
    if (abs(FR_count - SetpointFR) < STEP_TOLERANCE) {
      FR_reached = true;
      SetFRWheel(false, FR_direction);
    }
//    else if (abs(FR_count - SetpointFR) < AGG_TOLERANCE) {
//      FR_reached = false;
//      pidFR.SetTunings(Kp_con, Ki_con, Kd_con);
//    }
    else {
      FR_reached = false;
      pidFR.SetTunings(Kp_agg, Ki_agg, Kd_agg);
    }
    if (!FR_reached)
      //    if (true)  // true PID should always be active? 
    {
      InputFR = FR_count;
      pidFR.Compute();
      int currentPwmFR = OutputFR;
      if (currentPwmFR > 0) {
        FR_reversed = false;
        currentPwmFR = map(currentPwmFR, 0, 100, minPwm_front, maxPwm_front);
        SetFRWheel(true, FR_direction);
        analogWrite(F_enA, currentPwmFR);
      }
      else {
        FR_reversed = true;
        currentPwmFR = map(-currentPwmFR, 0, 100, minPwm_front, maxPwm_front);
        SetFRWheel(true, !FR_direction);
        analogWrite(F_enA, currentPwmFR);
      }
      if (PID_DEBUG_VEL) {
        Serial.print(",FR_PWM:");
        Serial.print(FR_reversed ? -currentPwmFR : currentPwmFR);
      }
    }
    if (PID_DEBUG_POS) {
      Serial.print(",FR_Input:");
      Serial.print(InputFR);
    }

    // FL CONTROL
    if (abs(FL_count - SetpointFL) < STEP_TOLERANCE) {
      FL_reached = true;
      SetFLWheel(false, FL_direction);
    }
//    else if (abs(FL_count - SetpointFL) < AGG_TOLERANCE) {
//      FL_reached = false;
//      pidFL.SetTunings(Kp_con, Ki_con, Kd_con);
//    }
    else {
      FL_reached = false;
      pidFL.SetTunings(Kp_agg, Ki_agg, Kd_agg);
    }
    if (!FL_reached)
      //    if (true)  // true PID should always be active? 
    {
      InputFL = FL_count;
      pidFL.Compute();
      int currentPwmFL = OutputFL;
      if (currentPwmFL > 0) {
        FL_reversed = false;
        currentPwmFL = map(currentPwmFL, 0, 100, minPwm_front, maxPwm_front);
        SetFLWheel(true, FL_direction);
        analogWrite(L_enA, currentPwmFL);
      }
      else {
        FL_reversed = true;
        currentPwmFL = map(-currentPwmFL, 0, 100, minPwm_front, maxPwm_front);
        SetFLWheel(true, !FL_direction);
        analogWrite(L_enA, currentPwmFL);
      }
      if (PID_DEBUG_VEL) {
        Serial.print(",FL_PWM:");
        Serial.print(FL_reversed ? -currentPwmFL : currentPwmFL);
      }
    }
    if (PID_DEBUG_POS) {
      Serial.print(",FL_Input:");
      Serial.print(InputFL);
    }



    // BR CONTROL
    // // disable BR
    //    BR_reached = true;
    //    SetBRWheel(false, false);
    if (abs(BR_count - SetpointBR) < STEP_TOLERANCE) {
      BR_reached = true;
      SetBRWheel(false, BR_direction);
    }
//    else if (abs(BR_count - SetpointBR) < AGG_TOLERANCE) {
//      BR_reached = false;
//      pidBR.SetTunings(Kp_con, Ki_con, Kd_con);
//    }
    else {
      BR_reached = false;
      pidBR.SetTunings(Kp_agg, Ki_agg, Kd_agg);
    }
    if (!BR_reached)
      //    if (true)  // true PID should always be active? 
    {
      InputBR = BR_count;
      pidBR.Compute();
      int currentPwmBR = OutputBR;
      if (currentPwmBR > 0) {
        BR_reversed = false;
        currentPwmBR = map(currentPwmBR, 0, 100, minPwm_BR, maxPwm_BR);
        SetBRWheel(true, BR_direction);
        analogWrite(R_enA, currentPwmBR);
      }
      else {
        BR_reversed = true;
        currentPwmBR = map(-currentPwmBR, 0, 100, minPwm_BR, maxPwm_BR);
        SetBRWheel(true, !BR_direction);
        analogWrite(R_enA, currentPwmBR);
      }
      if (PID_DEBUG_VEL) {
        Serial.print(",BR_Pwm:");
        Serial.print(BR_reversed ? -currentPwmBR : currentPwmBR);
      }
    }
    if (PID_DEBUG_POS) {
      Serial.print(",BR_Input:");
      Serial.print(InputBR);
    }


    // BL CONTROL
    if (abs(BL_count - SetpointBL) < STEP_TOLERANCE) {
      BL_reached = true;
      SetBLWheel(false, BL_direction);
    }
//    else if (abs(BL_count - SetpointBL) < AGG_TOLERANCE) {
//      BL_reached = false;
//      pidBL.SetTunings(Kp_con, Ki_con, Kd_con);
//    }
    else {
      BL_reached = false;
      pidBL.SetTunings(Kp_agg, Ki_agg, Kd_agg);
    }
    if (!BL_reached)
      //    if (true)  // true PID should always be active? 
    {
      InputBL = BL_count;
      pidBL.Compute();
      int currentPwmBL = OutputBL;
      if (currentPwmBL > 0) {
        BL_reversed = false;
        currentPwmBL = map(currentPwmBL, 0, 100, minPwm_BL, maxPwm_BL);
        SetBLWheel(true, BL_direction);
        analogWrite(B_enA, currentPwmBL);
      }
      else {
        BL_reversed = true;
        currentPwmBL = map(-currentPwmBL, 0, 100, minPwm_BL, maxPwm_BL);
        SetBLWheel(true, !BL_direction);
        analogWrite(B_enA, currentPwmBL);
      }
      if (PID_DEBUG_VEL) {
        Serial.print(",BL_PWM:");
        Serial.print(BL_reversed ? -currentPwmBL : currentPwmBL);
      }
    }
    if (PID_DEBUG_POS) {
      Serial.print(",BL_Input:");
      Serial.println(InputBL);
    }


    if (FR_reached && FL_reached && BR_reached && BL_reached) {
      EndLoop(false);
    }

    if ((abs(FL_count - SetpointFL) < APPROACH_TOLERANCE) || (abs(BL_count - SetpointBL) < APPROACH_TOLERANCE)
    || (abs(FR_count - SetpointFR) < APPROACH_TOLERANCE) || (abs(BR_count - SetpointBR) < APPROACH_TOLERANCE)) 
    {badCounter += 1;}
    //    Serial.println(badCounter);
    return;
  }
  //  else{
  //    Serial.println("idle");
  //  }
}
