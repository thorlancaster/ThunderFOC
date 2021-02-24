#include <ADC.h>
#include "DataStructs.h"

// TODO this goes back in FOCTransforms
// Each entry in the lookup table consists of a, b, and c. LUT size in bytes is 12x this
#define LUT_SZ 768
// Compressed LUT size for EEPROM. Size in bytes is 3x this
#define EEP_LUT_SZ 256
float SVPWM_LUT[LUT_SZ * 3]; // Lookup table for calculating PWM to match motor's back EMF
float TEMP_LUT[LUT_SZ * 3]; // Lookup table of BEMF. Used for calibration
#define PWM_MODE_STANDARD 0 // PWM signals are center-aligned and in phase
#define PWM_MODE_SVPWM 1  // PWM signals are center-aligned and in phase but modulate to reduce FET switching losses
#define PWM_MODE_PILOTTONE 2 // PWM signals are offset by an amount (0-1) set by calling PWMSetPilotTone()


volatile bool FOCEN = false; // True to drive motor, false to leave half bridge outputs floating


/* Stuff to note
  asm volatile ("wfi"); // Halt processor until interrupt. Reduces power usage
  FASTRUN vs FLASHMEM - FASTRUN for important stuff, FLASHMEM for LUT generation etc
*/


const int LED = 13;
const int FET_EN = 7; // FET output enable, active low

bool nerr = true;
void setup() {
  Serial.begin(19200);
  pinMode(LED, OUTPUT);
  pinMode(FET_EN, OUTPUT);
  digitalWrite(FET_EN, HIGH);
  delay(400);

  //  nerr &= loadFOCTransforms();
  nerr &= PWMSetFrequency(20000);
  nerr &= FPCalibrateADC();
  nerr &= VM_init();
  nerr &= PWMSetOutput(0, 0, 0);

  FOCEN = true;

  digitalWrite(13, nerr);
}


float dbgScopeExt[1024];
bool dbgScopeExtReq = false;


uint64_t frameCount = 0;
void loop() {

//  Vector3 current = PWMGetPhaseCurrents();
//    Vector3 inductance = FOCGetDi();
//
//  //  float theta = fmod(millis() / 4000.0, 1) * TWO_PI;
//  //  PWMSetOutput(0.5 + 0.035 * sin(theta), 0.5 + 0.035 * sin(theta + TWO_PI / 3), 0.5 + 0.035* sin(theta + TWO_PI * 2 / 3));
//
//  if (frameCount % 2 == 0) {
//    //    Serial.println(FPIGetCount());
//    Serial.printf("%f %f\n", current.x, current.y);
//    //    Serial.printf("%f %f %f\n", inductance.x, inductance.y, inductance.z);
//    //    dbgScopeExtReq = true;
//  }
//  //    if (frameCount % 20 == 2) {
//  //       Serial.printf("%f %f %f %f\n",
//  //        dbgScopeExt[0], dbgScopeExt[1], dbgScopeExt[2], dbgScopeExt[3]);
//  //    }
//
//  delayMicroseconds(500);

  digitalWrite(LED, nerr & millis() % 1000 > 500);
  frameCount++;
}
