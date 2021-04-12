const float SQRT_3 = 1.73205080757;
const float SQRT_3I = 1 / SQRT_3;

const int SVPWM_NONE = 0;
const int SVPWM_HOLDLOW = 1;
const int SVPWM_ALTREV = 2;

/**
   PLL to smooth an unsteady angle measurement
   @param existing Existing smoothed value from last iteration
   @param instant Instant measured value of the angle (noisy)
   @param accel Pointer to variable for maintaining PLL acceleration
   @param bandwidth How fast the internal state changes
   @param maxSlop Max distance in radians between the instant and smoothed values
*/
float angSmooth(float existing, float instant, float &accel, float Ival, float Pval, float maxSlop) {
  float err = angDiff(existing, instant);
  accel -= err * Ival;
  accel = constrain(accel, -PI / 6, PI / 6);
  existing = fmod(existing + accel - (err * Pval), TWO_PI);
  if(maxSlop > 0)
    existing = angConstrain(existing, instant - maxSlop, instant + maxSlop);
  return existing;
}

/**
   Constrain an angle (radians) to be within a range
   If out-of-range the closest valid angle is returned
   @param ang input angle
   @param lower Lower bound
   @param upper Upper bound
*/
float angConstrain(float ang, float lower, float upper) {
  float lowErr = angDiff(lower, ang);
  float hiErr = angDiff(ang, upper);
  if (lowErr > 0 && hiErr <= 0)
    ang += lowErr;
  else if (hiErr > 0 && lowErr <= 0)
    ang -= hiErr;
  else if (lowErr > 0 || hiErr > 0) {
    if (lowErr < hiErr)
      ang += lowErr;
    else
      ang -= hiErr;
  }
  return fmod((ang + TWO_PI), TWO_PI);
}

/**
   @return the difference between two angles (radians)
*/
float angDiff(float ang1, float ang2) {
  return fmod(((ang1 - ang2) + (5 * PI)), TWO_PI) - PI;
}

/**
 * Follow a pole of saliency around the motor
 * NOTE: This function has two stable angles (+- 180 degrees)
 * and the correct pole to follow must be determined and passed
 * into `existing` before using saliency.
 * @param existing current angle estimation
 * @param salVal angle of saliency
 * @return an angle that rotates half as fast as salVal
 */
float followSaliency(float existing, float salVal) {
  salVal = fmod(salVal + 2 * TWO_PI, TWO_PI) / 2;

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


/**
   Clarke transform converts a and b (implicit Ic = 0-Ia-Ib) into Alpha and Beta
*/
inline Vector2 clarke(Vector2 in) {
  Vector2 rtn;
  float c = -(in.x + in.y);
  rtn.x = in.x;
  rtn.y = (in.y - c) * SQRT_3I;
  return rtn;
}

/**
   Park transform converts Alpha and Beta into D and Q and requires an angle
*/
inline Vector2 park(Vector2 in, float ang) {
  Vector2 rtn;
  float s = sin(ang);
  float c = cos(ang);
  rtn.x = in.x * c + in.y * s;
  rtn.y = in.y * c - in.x * s;
  return rtn;
}

/**
   Inverse Park transform converts D and Q into Alpha and Beta and requires an angle
*/
inline Vector2 inversePark(Vector2 in, float ang) {
  Vector2 rtn;
  float s = sin(ang);
  float c = cos(ang);
  rtn.x = in.x * c - in.y * s;
  rtn.y = in.y * c + in.x * s;
  return rtn;
}

/**
 * SVPWM generates phase (A, B, C) PWM signals from Alpha and Beta voltages
 * @param type Type of SVPWM to generate
 */
inline Vector3 generateSVPWM(Vector2 in, int type){
  Vector3 rtn;
  rtn.x = in.x + 0.5;
  rtn.y = 0.5*(SQRT_3 * in.y - in.x) + 0.5;
  rtn.z = 0.5*(-SQRT_3 * in.y - in.x) + 0.5;

  // Hold-low SVPWM brings the lowest phase to negative
  if(type == SVPWM_HOLDLOW){
    float low = min(rtn.x, min(rtn.y, rtn.z));
    rtn.x -= low;
    rtn.y -= low;
    rtn.z -= low;
  }
  // Alternate-Reverse SVPWM brings the closest phase to the rail,
  // more even FET heating but can cause gate driver UVLO at low speeds
  if(type == SVPWM_ALTREV){
    // TODO implement Alternate-Reverse SVPWM correctly
    float low = min(rtn.x, min(rtn.y, rtn.z));
    rtn.x -= low;
    rtn.y -= low;
    rtn.z -= low;
  }
  return rtn;
}
