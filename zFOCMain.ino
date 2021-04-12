
// Desired D and Q-axis currents
float FOC_Iq = 1;
float FOC_Id = 0;

// For PID
const float FOC_kAdj = 0.18;

float FOC_kp1 = FOC_kAdj * 0.035;
float FOC_ki1 = FOC_kAdj * 0.014;
float FOC_kd1 = FOC_kAdj * 0.015;

float FOC_kp2 = FOC_kAdj * FOC_kp1;
float FOC_ki2 = FOC_kAdj * FOC_ki1;
float FOC_kd2 = FOC_kAdj * FOC_kd1;

float FOC_kpM = 0.0001;
float FOC_kiM = 0.00001;


// PID memory
float FOC_I1 = 0;
float FOC_D1 = 0;
float FOC_I2 = 0;
float FOC_D2 = 0;
float FOC_angM = 0;

// In this file `Voltage` means 0=0v, 1=Full Battery Voltage
void FOCMain() {
  static Vector2 alphaBetaCurrent;
  static Vector2 dqCurrent; // Measured D and Q axis currents
  static Vector2 dqVoltage; // D and Q axis voltage
  static Vector2 alphaBetaVoltage; // Alpha and Beta voltage
  static Vector3 abcVoltage; // Phase voltage / PMM

  static float rawBemfAngle;
  static float pllBemfAngle;

  
  static uint32_t framecount = 0;

  // ---- ANGLE DETERMINATION
  rawBemfAngle = atan2(alphaBetaVoltage.y, alphaBetaVoltage.x);
  pllBemfAngle = angSmooth(pllBemfAngle, rawBemfAngle, FOC_angM, FOC_kiM, FOC_kpM, 0);

  float ang = pllBemfAngle;
  float ang2 = fmod(millis() / 6.0 , 6.28);
  float ang3 = HallsGetRealAngle1(true);

  ang = ang3;

  // ---- INPUT CALCULATION
  alphaBetaCurrent = PWMGetPhaseCurrents2(true);
  dqCurrent = park(alphaBetaCurrent, ang);


  // ---- PID CONTROL
//  float in, float out, float setpoint, float kp, float ki, float kd, float &integral, float &perr, float minOut, float maxOut
  dqVoltage.x = PID(dqCurrent.x, dqVoltage.x, FOC_Id, FOC_kp1, FOC_ki1, FOC_kd1, FOC_I1, FOC_D1, -1, 1);
  dqVoltage.y = PID(dqCurrent.y, dqVoltage.y, FOC_Iq, FOC_kp2, FOC_ki2, FOC_kd2, FOC_I2, FOC_D2, -1, 1);
  // ---- OUTPUT CALCULATION
  alphaBetaVoltage = inversePark(dqVoltage, ang);
  abcVoltage = generateSVPWM(alphaBetaVoltage, SVPWM_HOLDLOW);


  if (Serial.availableForWrite() > 120) {
    if ((framecount % 12) == 0) {
//      Serial.printf("%f %f %f %f %f 0 50\n", dqVoltage.x * 20, dqVoltage.y * 20, dqCurrent.x, dqCurrent.y, alphaBetaVoltage.x*30, alphaBetaVoltage.y*30);

//      Serial.printf("%.2f %.2f %.2f %.2f %.2f\n", alphaBetaVoltage.x * 20, alphaBetaVoltage.y * 20, ang + 10, pllBemfAngle + 10, rawBemfAngle + 10);
    }
  }

  PWMSetOutput(abcVoltage.x, abcVoltage.y, abcVoltage.z);


  framecount++;
}

void FOCSetDQ(float d, float q){
  FOC_Id = d;
  FOC_Iq = q;
}
