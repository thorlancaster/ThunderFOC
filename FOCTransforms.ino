//
//
//const float DEG_120 = 2.0944;
//
//bool loadFOCTransforms(){
//  // TODO load learned LUT from EEPROM
////  generateMockBEMFLUT(TEMP_LUT, LUT_SZ);
//  generateSinLUT(TEMP_LUT, LUT_SZ);
//  generateSVPWMLUT(TEMP_LUT, SVPWM_LUT, LUT_SZ);
//  return true;
//}
//
//// ------------------------------------------------------------ LOOKUP TABLE GENERATION
//
//// Generate a sinusoidal look-up table
//FLASHMEM void generateSinLUT(float* output, int sz) {
//  for (int x = 0; x < sz; x++) {
//    float rad = (x * TWO_PI) / sz;
//    float xOut = sin(rad);
//    float yOut = sin(rad + DEG_120);
//    float zOut = sin(rad + 2 * DEG_120);
//    output[x * 3] = xOut * 2;
//    output[x * 3 + 1] = yOut * 2;
//    output[x * 3 + 2] = zOut * 2;
//  }
//}
//
//// Generate a mock BEMF look-up table for Mitsuba Solar Car Motor
//FLASHMEM void generateMockBEMFLUT(float* output, int sz) {
//  for (int x = 0; x < sz; x++) {
//    float rad = (x * TWO_PI) / sz;
//    float xOut = GBLSIN(rad);
//    float yOut = GBLSIN(rad + DEG_120);
//    float zOut = GBLSIN(rad + 2 * DEG_120);
//    output[x * 3] = xOut;
//    output[x * 3 + 1] = yOut;
//    output[x * 3 + 2] = zOut;
//  }
//}
//FLASHMEM float GBLSIN(float x) {
//  float b = 0;
//  float c = 0.12;
//  float d = 0;
//  float f = -0.05;
//  float g = 0;
//  float h = -0.07;
//  float div = 1.0223;
//  return (sin(x) + b * sin(2 * x) + c * sin(3 * x) + d * sin(4 * x) + f * sin(5 * x) + g * sin(6 * x) + h * sin(7 * x)) / div;
//}
//
//
//// Generate a lookup table of PWM, given a lookup table of BEMF / voltage
//// Output values are normalized between 0 and 1
//FLASHMEM void generateSVPWMLUT(float* input, float* output, int sz) {
//  for (int i = 0; i < sz; i++) {
//    // X, Y, and Z are the voltages on each phase
//    float x = input[i * 3];
//    float y = input[i * 3 + 1];
//    float z = input[i * 3 + 2];
//    // M is the minimum of the three. For this version of SVPWM,
//    // It will be clamped at zero to minimize switching losses while allowing
//    // the high-side gate drives to cycle
//    float m = min(x, min(y, z));
//    float xOut = x - m;
//    float yOut = y - m;
//    float zOut = z - m;
//    output[i * 3] = xOut;
//    output[i * 3 + 1] = yOut;
//    output[i * 3 + 2] = zOut;
//  }
//  normalize(output, output, sz * 3);
//}
//
//
//// ------------------------------------------------------------ TRANSFORMATIONS
//Vector3 rawSine(float angle) {
//  Vector3 output;
//  output.x = sin(angle);
//  output.y = sin(angle + DEG_120);
//  output.z = sin(angle + 2 * DEG_120);
//  return output;
//}
//
///**
//   Given a magnitude and direction of the desired output voltage magnitude (0-1) and angle (0-2PI)
//   return a 3-vector of the required PWM values. Uses the global SVPWM_LUT table
//*/
//Vector3 angToPWM(float ang, float mag) {
//  int iAng = (ang * LUT_SZ / (2 * PI));
//  iAng = (iAng % LUT_SZ) * 3;
//  // iAng is angle where 2PI radians spans the lookup table
//  Vector3 output;
//  output.x = SVPWM_LUT[iAng] * mag;
//  output.y = SVPWM_LUT[iAng + 1] * mag;
//  output.z = SVPWM_LUT[iAng + 2] * mag;
//  return output;
//}
//
//Vector3 angToLUT(float* LUT, int sz, float ang, float mag) {
//  int iAng = (ang * sz / (2 * PI));
//  iAng = (iAng % sz) * 3;
//  // iAng is angle where 2PI radians spans the lookup table
//  Vector3 output;
//  output.x = LUT[iAng] * mag;
//  output.y = LUT[iAng + 1] * mag;
//  output.z = LUT[iAng + 2] * mag;
//  return output;
//}
//
//
//// ------------------------------------------------------------ UTILITIES
///*
//   Utility function that calculates current of a purely resistive "motor" from voltages
//   Useful for simulation
//*/
//Vector3 voltageToCurrent(Vector3 voltage) {
//  Vector3 output;
//  float vGnd = (voltage.x + voltage.y + voltage.z) / 3; // Virtual Ground
//  output.x = voltage.x - vGnd;
//  output.y = voltage.y - vGnd;
//  output.z = voltage.z - vGnd;
//  return output;
//}
//
//
//
//float deg2rad(float angle) {
//  return angle * 0.0174533;
//}
