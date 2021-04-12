#include <ADC.h>
#include "DataStructs.h"

#define PWM_MODE_STANDARD 0 // PWM signals are center-aligned and in phase
#define PWM_MODE_SVPWM 1  // PWM signals are center-aligned and in phase but modulate to reduce FET switching losses
#define PWM_MODE_PILOTTONE 2 // PWM signals are offset by an amount (0-1) set by calling PWMSetPilotTone()

#define THROTTLE A9


volatile bool FOCEN = false; // True to drive motor, false to leave half bridge outputs floating


/* Stuff to note
  asm volatile ("wfi"); // Halt processor until interrupt. Reduces power usage
  FASTRUN vs FLASHMEM - FASTRUN for important stuff, FLASHMEM for LUT generation etc
*/


const int LED = 13;
const int FET_EN = 7; // FET output enable, active low
const int FET_EN_ENABLE = LOW;

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
  nerr &= Halls_init();
  nerr &= PWMSetOutput(0, 0, 0);

   FOCEN = true;

  digitalWrite(FET_EN, LOW);
  digitalWrite(13, nerr);
}

float staminaPaVal = 0; // Virtual motor temperature
float staminaPaMax = 63; // Max continuous phase amps
float staminaPaTimeConst = 120; // Thermal time constant of motor (better to underestimate here)



uint64_t frameCount = 0;
void loop() {
  float volts = VMGetBattVoltage();
  float throttleAmps = VMGetThrottle() * 135; // Phase Amps full throttle
  float staminaMult = constrain(1.0 - (staminaPaVal - 0.95)*20, 0, 1);
  float voltMult = constrain((volts - 38), 0, 1);
  throttleAmps *= staminaMult;
  if(volts > 30)
    throttleAmps *= voltMult;
  calcStamina(throttleAmps / staminaPaMax, staminaPaVal, staminaPaTimeConst * 1000, 20);
  FOCSetDQ(0, throttleAmps);
  
//  digitalWrite(LED, nerr & millis() % 1000 > 500);
  delay(20);
  frameCount++;
}
