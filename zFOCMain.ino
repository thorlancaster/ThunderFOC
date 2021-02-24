

bool FOCPILEN = true; // Pilot tone enable for inductance / saliency

int8_t FOCPILCNT = 0; // Pilot tone counter
float FOCPILAMT = 0.1; // Amount of pilot tone amplitude

// Saliency calculation variables
float FOCSalA[6];
float FOCSalPA = 0;
float FOCSalB[6];
float FOCSalPB = 0;
float FOCSalC[6];
float FOCSalPC = 0;
// Saliency result variables
float FOCSalRawA = 0;
float FOCSalRawB = 0;
float FOCSalRawC = 0;
float FOCSalMeasA = 0;
float FOCSalMeasB = 0;
float FOCSalMeasC = 0;

float FOC_SAL_RAWANG = 0; // Goes around twice every rotation
float FOC_SAL_ANG = 0; // Goes around once every rotation
float FOC_SAL_RAWANG_OFFSET = PI; // Determined by experimentation

float FOC_IPID_kp = 0.015;
float FOC_IPID_ki = 0.003;
float FOC_IPID_kd = 0.02;
float FOC_IPID_inta = 0;
float FOC_IPID_dera = 0;
float FOC_IPID_intb = 0;
float FOC_IPID_derb = 0;

float FOC_Iq = 3;
float FOC_Id = 0;

void FOCMain() {
  static uint32_t framecount = 0;
  framecount++;

  // Dummy commands
  uint32_t cycler = framecount % 200000;
  if(cycler < 15000)
    FOC_Iq = 6;
  else if(cycler > 100000 && cycler < 115000)
    FOC_Iq = -6;
  else
    FOC_Iq = 0;

  // Assume current pole is correct, follow it
  FOC_SAL_ANG = FOCFollowSaliency(FOC_SAL_ANG, FOC_SAL_RAWANG + FOC_SAL_RAWANG_OFFSET);

  float ang = FOC_SAL_ANG;
  float desIa = FOC_Iq * cos(ang);
  float desIb = FOC_Iq * cos(ang - TWO_PI / 3);


  Vector3 current = PWMGetPhaseCurrents(true);
  Vector3 currentSmooth = PWMGetPhaseCurrents(false);

  float Va = 0.50;
  float Vb = 0.50;
  PID(currentSmooth.x, Va, desIa, FOC_IPID_kp, FOC_IPID_ki, FOC_IPID_kd, FOC_IPID_inta, FOC_IPID_dera, 0, 1);
  PID(currentSmooth.y, Vb, desIb, FOC_IPID_kp, FOC_IPID_ki, FOC_IPID_kd, FOC_IPID_intb, FOC_IPID_derb, 0, 1);


  if ((framecount % 50) == 0) // Saliency and position graph
    Serial.printf("%f %f %f %f %f\n",  FOC_SAL_ANG + 20, ang + 25, FOCSalMeasA * 3, FOCSalMeasB * 3, FOCSalMeasC * 3);

  //  if ((framecount % 20) == 0)
  //    Serial.printf("%f %f %f\n", FP_ISENSE_A_VAL, FP_ISENSE_B_VAL, FP_ISENSE_C_VAL);


  //  if ((framecount % 500) == 0){ // Raw ADC graph
  //    int len = min(FP_ISENSE_A_RAWBUF_SZ, min(FP_ISENSE_B_RAWBUF_SZ, FP_ISENSE_C_RAWBUF_SZ));
  //    for(int x = 0; x < len; x++){
  //      Serial.printf("%d %d %d\n", FP_ISENSE_A_RAWBUF[x], FP_ISENSE_B_RAWBUF[x], FP_ISENSE_C_RAWBUF[x]);
  //    }
  //  }

  //  Va = 0.52;
  //  Vb = 0.52;

  bool useSVPWM = false;
  FOCPILEN = true; // TODO xxx

  float Vc = 1.5 - (Va + Vb);
  if (FOCPILEN) {
    if (FOCPILCNT == 0) {
      FOCSalA[0] = current.x;
      FOCSalB[0] = current.y;
      FOCSalC[0] = current.z;
      //      currLow = current.x;
      PWMSetOutput(Va + FOCPILAMT, Vb - FOCPILAMT, Vc + FOCPILAMT, useSVPWM);
    }
    else if (FOCPILCNT == 1) {
      FOCSalPB = FOCSalB[1];
      FOCSalA[1] = current.x;
      FOCSalB[1] = current.y;
      FOCSalC[1] = current.z;
      PWMSetOutput(Va + FOCPILAMT, Vb - FOCPILAMT, Vc - FOCPILAMT, useSVPWM);

      // Calculate phase B saliency
      float dcVal = (FOCSalB[0] + FOCSalB[1] + FOCSalB[2] + FOCSalB[3] + FOCSalB[4] + FOCSalB[5]) / 6;
      float slope = (FOCSalB[1] - FOCSalPB) / 6;
      FOCSalRawB = (FOCSalB[3] - dcVal - 1.5 * slope) - (FOCSalB[0] - dcVal + 1.5 * slope);
      if (!isnan(FOCSalRawB))
        FOCSalMeasB = FOCSalRawB * 0.12 + FOCSalMeasB * 0.88;
    }

    else if (FOCPILCNT == 2) {
      FOCSalA[2] = current.x;
      FOCSalB[2] = current.y;
      FOCSalC[2] = current.z;
      PWMSetOutput(Va + FOCPILAMT, Vb + FOCPILAMT, Vc - FOCPILAMT, useSVPWM);
    }
    else if (FOCPILCNT == 3) {
      FOCSalPC = FOCSalC[3];
      FOCSalA[3] = current.x;
      FOCSalB[3] = current.y;
      FOCSalC[3] = current.z;
      PWMSetOutput(Va - FOCPILAMT, Vb + FOCPILAMT, Vc - FOCPILAMT, useSVPWM);

      // Calculate phase C saliency
      float dcVal = (FOCSalC[0] + FOCSalC[1] + FOCSalC[2] + FOCSalC[3] + FOCSalC[4] + FOCSalC[5]) / 6;
      float slope = (FOCSalC[3] - FOCSalPC) / 6;
      FOCSalRawC = (FOCSalC[5] - dcVal - 1.5 * slope) - (FOCSalC[2] - dcVal + 1.5 * slope);
      if (!isnan(FOCSalRawC))
        FOCSalMeasC = FOCSalRawC * 0.12 + FOCSalMeasC * 0.88;
    }

    else if (FOCPILCNT == 4) {
      FOCSalA[4] = current.x;
      FOCSalB[4] = current.y;
      FOCSalC[4] = current.z;
      PWMSetOutput(Va - FOCPILAMT, Vb + FOCPILAMT, Vc + FOCPILAMT, useSVPWM);
    }
    else {
      FOCPILCNT = -1;
      FOCSalPA = FOCSalA[5];
      FOCSalA[5] = current.x;
      FOCSalB[5] = current.y;
      FOCSalC[5] = current.z;
      PWMSetOutput(Va - FOCPILAMT, Vb - FOCPILAMT, Vc + FOCPILAMT, useSVPWM);

      // Calculate phase A saliency
      float dcVal = (FOCSalA[0] + FOCSalA[1] + FOCSalA[2] + FOCSalA[3] + FOCSalA[4] + FOCSalA[5]) / 6;
      float slope = (FOCSalA[5] - FOCSalPA) / 6;
      FOCSalRawA = (FOCSalA[1] - dcVal - 1.5 * slope) - (FOCSalA[4] - dcVal + 1.5 * slope);
      if (!isnan(FOCSalRawA))
        FOCSalMeasA = FOCSalRawA * 0.12 + FOCSalMeasA * 0.88;

      // Calculate angle
      float salAvg = (FOCSalMeasA + FOCSalMeasB + FOCSalMeasC) / 3;
      float Ia = FOCSalMeasA - salAvg;
      float Ib = FOCSalMeasB - salAvg;
      float Ic = FOCSalMeasC - salAvg;

      float Id = (Ib - Ic) / 1.732;
      FOC_SAL_RAWANG = atan2(Ia, Id);
      if (dbgScopeExtReq) {
        dbgScopeExt[0] = FOCSalMeasA;
        dbgScopeExt[1] = FOCSalMeasB;
        dbgScopeExt[2] = FOCSalMeasC;
        dbgScopeExt[3] = FOC_SAL_RAWANG;
      }
    }
    FOCPILCNT++;
  } else {
    FOCPILCNT = 0;
    PWMSetOutput(Va, Vb, Vc, useSVPWM);
  }
}

// Follow a pole of saliency around the rotor.
// The output value rotates once for every two rotations of SalVal
float FOCFollowSaliency(float existing, float salVal) {
  salVal = fmod(salVal + 2*TWO_PI, TWO_PI) / 2;

  // Determine whether 0 or 180 offset is closer to current value
  float err0 = abs(existing - salVal);
  if (err0 > PI) // if error > 180 degrees
    err0 = abs(TWO_PI - err0);
    
  float err180 = abs(existing - (PI + salVal));
  if (err180 > PI) // if error > 180 degrees
    err180 = abs(TWO_PI - err180);

  // Stay with closest value
  if (err0 < err180)
    return salVal;
  else
    return PI + salVal;
}

//Vector3 FOCGetDi() {
//  Vector3 rtn;
//  rtn.x = FOC_DIA;
//  rtn.y = FOC_DIB;
//  rtn.z = FOC_DIC;
//  return rtn;
//}
