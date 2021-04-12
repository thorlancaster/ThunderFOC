#include "imxrt.h"
#include "core_pins.h"

#define FP_DEBUG_PIN 12 // Used for CPU usage debugging during development

#define FORCE_PHASE_CURRENTS_ZERO true // Makes phase readings more precise. Comment out during calibration

// ### NOTE: Everything beginning with FP_ should only be written to in THIS file

// Known usage of FlexPWM - see pwm.c in hardware/teensy/avr/cores/teensy4
// ------------------------
//   FlexPWM2_0    PWM pin 4, 33     (4 = PHASE_A)
//   FlexPWM2_1    PWM pin 5, NC     (5 = PHASE_B)
//   FlexPWM2_2    PWM pin 6, 9      (6 = PHASE_C)


#define FP_ISENSE_BUF_SZ 64 // Enough for min PWM frequency of 10KHz
uint16_t FP_ISENSE_A_BUF [FP_ISENSE_BUF_SZ]; // Used only for acquiring data
uint16_t FP_ISENSE_A_BUF_PTR = 0;
uint16_t FP_ISENSE_B_BUF [FP_ISENSE_BUF_SZ];
uint16_t FP_ISENSE_B_BUF_PTR = 0;
uint16_t FP_ISENSE_C_BUF [FP_ISENSE_BUF_SZ];
uint16_t FP_ISENSE_C_BUF_PTR = 0;

uint16_t FP_ISENSE_A_RAWBUF [FP_ISENSE_BUF_SZ]; // Used as working copy of above buffers for calculation
uint16_t FP_ISENSE_A_RAWBUF_SZ = 0;             // (After denoise)
uint16_t FP_ISENSE_B_RAWBUF [FP_ISENSE_BUF_SZ];
uint16_t FP_ISENSE_B_RAWBUF_SZ = 0;
uint16_t FP_ISENSE_C_RAWBUF [FP_ISENSE_BUF_SZ];
uint16_t FP_ISENSE_C_RAWBUF_SZ = 0;

// Size of below buffers is the same as above buffers, don't need redundant variables
uint16_t FP_ISENSE_A_AVGBUF [FP_ISENSE_BUF_SZ]; // Sorted copy of above buffers, for calculation
uint16_t FP_ISENSE_B_AVGBUF [FP_ISENSE_BUF_SZ];
uint16_t FP_ISENSE_C_AVGBUF [FP_ISENSE_BUF_SZ];



#define FP_INTERRUPT         IRQ_FLEXPWM2_0 // IRQ that fires off when PWM cycle completes
#define FP_INTERRUPT_CLEAR  FLEXPWM2_SM0STS // Used to clear FP_INTERRUPT to prevent continuous firing
#define FP_INTERRUPT_EN   FLEXPWM2_SM0INTEN // Used to enable the above IRQ

// These do the same thing for each phase but on different pins
// PHASE A = Module 2, Submodule 0, Pin A
#define FP_PIN_A_SMNUM 0 // Submodule number for this pin
#define FP_PIN_A_START FLEXPWM2_SM0VAL2 // [Module] [Submodule] [Start] to bring the pin high
#define FP_PIN_A_STOP FLEXPWM2_SM0VAL3 // [Module] [Submodule] [Stop] to bring the pin low
#define FP_PIN_A_PIN 4  // Arduino pin number of the PWM pin
#define FP_PIN_A_MUXVAL 1 // Muxval for selecting flexPWM as signal source -  see pwm.c
// Analog pins
#define FP_ISENSE_A A2 // Analog pin from current sensor
// #define FP_VSENSE_A
float FP_ISENSE_A_OFFSET = 1024; // Initial calibration values for the current sensors
float FP_ISENSE_A_RATIO = -0.748;  // More accurate values are stored in EEPROM during factory calubration
float FP_VSENSE_A_RATIO = 0.1;

// PHASE B = Module 2, Submodule 1, Pin A
#define FP_PIN_B_SMNUM 1
#define FP_PIN_B_START FLEXPWM2_SM1VAL2
#define FP_PIN_B_STOP FLEXPWM2_SM1VAL3
#define FP_PIN_B_PIN 5
#define FP_PIN_B_MUXVAL 1
// Analog pins
#define FP_ISENSE_B A3
// #define FP_VSENSE_B
float FP_ISENSE_B_OFFSET = 1024;
float FP_ISENSE_B_RATIO = -0.835;
float FP_VSENSE_B_RATIO = 0.1;

// PHASE C = Module 2, Submodule 2, Pin A
#define FP_PIN_C_SMNUM 2
#define FP_PIN_C_START FLEXPWM2_SM2VAL2
#define FP_PIN_C_STOP FLEXPWM2_SM2VAL3
#define FP_PIN_C_PIN 6
#define FP_PIN_C_MUXVAL 2
// Analog pins
#define FP_ISENSE_C A4
// #define FP_VSENSE_C
float FP_ISENSE_C_OFFSET = 1024;
float FP_ISENSE_C_RATIO = 0.736;
float FP_VSENSE_C_RATIO = 0.1;

// Bitmask to set LDOK and CLDOK for updating the PWM
#define FP_LCLDOK_ALL ((1 << FP_PIN_A_SMNUM)  | (1 << FP_PIN_B_SMNUM) | (1 << FP_PIN_C_SMNUM))

// Set by PWM
uint32_t FP_PERIOD = 0;
uint32_t FP_FREQUENCY = 0;
float FP_PILOT = 0; // Amount of pilot tone (0-1) for offset pilot tone mode



const int FP_ISENSE_WIGGLE = 2; // For noise reduction purposes
volatile float FP_ISENSE_A_VAL = 0;
volatile float FP_ISENSE_B_VAL = 0;
volatile float FP_ISENSE_C_VAL = 0;
volatile float FP_ISENSE_A_RAW = 0;
volatile float FP_ISENSE_B_RAW = 0;
volatile float FP_ISENSE_C_RAW = 0;
volatile int FP_ISENSE_COUNT = 0;
volatile int FP_MODE = 0;


int FPIGetCount() {
  return FP_ISENSE_COUNT;
}

ADC *FP_ADC = new ADC();

boolean FPCalibrateADC() {
  float a = 0;
  float b = 0;
  float c = 0;
  for (int x = 0; x < 5000; x++) {
    a += FP_ISENSE_A_VAL;
    b += FP_ISENSE_B_VAL;
    c += FP_ISENSE_C_VAL;
    delayMicroseconds(50);
  }
  FP_ISENSE_A_OFFSET = (a / 5000);
  FP_ISENSE_B_OFFSET = (b / 5000);
  FP_ISENSE_C_OFFSET = (c / 5000);
  return true;
}


// ISR that fires off whenever an ADC has a reading. Used for realtime FOC current measurement
void FP_ADC0_ISR() {
  static uint8_t turn = 0;
#if defined(__IMXRT1062__)  // Teensy 4.0
  uint8_t pin = ADC::sc1a2channelADC0[ADC1_HC0 & 0x1f]; // the bits 0-4 of ADC0_SC1A have the channel
#else
  uint8_t pin = ADC::sc1a2channelADC0[ADC0_SC1A & ADC_SC1A_CHANNELS]; // the bits 0-4 of ADC0_SC1A have the channel
#endif
  // add value to correct buffer
  if (pin == FP_ISENSE_A) {
    FP_ISENSE_A_BUF[FP_ISENSE_A_BUF_PTR] = FP_ADC->adc0->readSingle();
    if (FP_ISENSE_A_BUF_PTR < FP_ISENSE_BUF_SZ - 2)
      FP_ISENSE_A_BUF_PTR++;
  }
  else if (pin == FP_ISENSE_B) {
    FP_ISENSE_B_BUF[FP_ISENSE_B_BUF_PTR] = FP_ADC->adc0->readSingle();
    if (FP_ISENSE_B_BUF_PTR < FP_ISENSE_BUF_SZ - 2)
      FP_ISENSE_B_BUF_PTR++;
  }
  else if (pin == FP_ISENSE_C) {
    FP_ISENSE_C_BUF[FP_ISENSE_C_BUF_PTR] = FP_ADC->adc0->readSingle();
    if (FP_ISENSE_C_BUF_PTR < FP_ISENSE_BUF_SZ - 2)
      FP_ISENSE_C_BUF_PTR++;
  }
  else { // clear interrupt anyway
    FP_ADC->adc0->readSingle();
  }

  // restore ADC config if it was in use before being interrupted by the analog timer
  if (FP_ADC->adc0->adcWasInUse) {
    // restore ADC config, and restart conversion
    FP_ADC->adc0->loadConfig(&FP_ADC->adc0->adc_config);
    // avoid a conversion started by this isr to repeat itself
    FP_ADC->adc0->adcWasInUse = false;
  }

  // Start the next conversion interrupt
  // Read scheduling
  uint8_t readPin0 = 0;
  if (turn == 0) {
    readPin0 = FP_ISENSE_A;
  } else if (turn == 1) {
    readPin0 = FP_ISENSE_B;
  } else if (turn == 2) {
    readPin0 = FP_ISENSE_C;
  }
  FP_ADC->adc0->startSingleRead(readPin0);
  turn = (turn + 1) % 3;

#if defined(__IMXRT1062__)  // Teensy 4.0
  asm("DSB");
#endif
}


// Set the FlexPWM frequency. Also initializes the FlexPWM the first time it is called
// The frequency and period are limited in range. Guaranteed to work 5k-100khz.
// Read the FP_FREQUENCY and FP_PERIOD variables to read the actual values
bool PWMSetFrequency(int frequency)
{
  static boolean hasInit = false;

  FP_FREQUENCY = frequency;
  uint32_t currentCycles = (uint32_t)((float)F_BUS_ACTUAL / frequency);

  if (currentCycles > 65535) {
    currentCycles = 65535;
  }
  currentCycles = (currentCycles >> 2) * 4; // Ensure divisibility by 4. Ensures perfect timing alignment

  if (currentCycles < 256) {
    FP_FREQUENCY = 0;
    FP_PERIOD = 0;
    return false;
  }
  FP_FREQUENCY = (uint32_t)((float)F_BUS_ACTUAL / currentCycles);
  FP_PERIOD = currentCycles;

  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_CLDOK(FP_LCLDOK_ALL);  // Clear load OK flag to prevent race condition

  FLEXPWM2_SM0CTRL = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_PRSC(0);
  FLEXPWM2_SM0VAL1 = currentCycles - 1;

  FLEXPWM2_SM1CTRL = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_PRSC(0);
  FLEXPWM2_SM1VAL1 = currentCycles - 1;

  FLEXPWM2_SM2CTRL = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_PRSC(0);
  FLEXPWM2_SM2VAL1 = currentCycles - 1;

  // Everything below only has to be called once
  if (hasInit) {
    return true;
  }
  pinMode(FP_PIN_A_PIN, OUTPUT);
  pinMode(FP_PIN_B_PIN, OUTPUT);
  pinMode(FP_PIN_C_PIN, OUTPUT);

  // All outputs start at 0
  FP_PIN_A_START = 0;
  FP_PIN_A_STOP = 0;
  FP_PIN_B_START = 0;
  FP_PIN_B_STOP = 0;
  FP_PIN_C_START = 0;
  FP_PIN_C_STOP = 0;

  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_LDOK(FP_LCLDOK_ALL); // Set load OK flag to update PWM

  // Start synchronized ISR (IntervalTimer)
  pinMode(FP_DEBUG_PIN, OUTPUT);
  attachInterruptVector(FP_INTERRUPT, FP_PWM_ISR);
  NVIC_ENABLE_IRQ(FP_INTERRUPT);
  FP_INTERRUPT_EN |= FLEXPWM_SMINTEN_RIE; // Reload Interrupt Enable

  // Configure ADC for REALLY HIGH SPEED. A bit noisier but worth it's weight in averages
  FP_ADC->adc0->setAveraging(1); // set number of averages
  FP_ADC->adc0->setResolution(11); // set bits of resolution
  FP_ADC->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED); // change the conversion speed
  FP_ADC->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED); // change the sampling speed
  FP_ADC->adc0->enableInterrupts(FP_ADC0_ISR);
  FP_ADC->adc0->startSingleRead(FP_ISENSE_A); // Get the ISR loop running

  // Enable chip outputs
  *(portConfigRegister(FP_PIN_A_PIN)) = FP_PIN_A_MUXVAL;
  *(portConfigRegister(FP_PIN_B_PIN)) = FP_PIN_B_MUXVAL;
  *(portConfigRegister(FP_PIN_C_PIN)) = FP_PIN_C_MUXVAL;


  // Enable timer outputs
  FLEXPWM2_OUTEN |= FLEXPWM_OUTEN_PWMA_EN(1 << FP_PIN_A_SMNUM);
  FLEXPWM2_OUTEN |= FLEXPWM_OUTEN_PWMA_EN(1 << FP_PIN_B_SMNUM);
  FLEXPWM2_OUTEN |= FLEXPWM_OUTEN_PWMA_EN(1 << FP_PIN_C_SMNUM);

  // Set up IRQ priorities
  NVIC_SET_PRIORITY(IRQ_ADC1, 64); // High priority for ADC
  //  NVIC_SET_PRIORITY(, 0) // TODO HIGHEST priority for FAULT
  NVIC_SET_PRIORITY(FP_INTERRUPT, 96); // Medium-high priority for FOC

  hasInit = true;
  return true;
}

void PWMSetPilotTone(float amt) {
  FP_PILOT = constrain(amt, 0, 1);
}

void PWMDisable(){
  digitalWrite(FET_EN, !FET_EN_ENABLE);
  FOCEN = false;
}


bool PWMSetOutput(float a, float b, float c) {
  // Period must be set before turning on PWM
  if (FP_PERIOD == 0)
    return false;

  a = constrain(a, 0.0, 1.0);
  b = constrain(b, 0.0, 1.0);
  c = constrain(c, 0.0, 1.0);

  uint32_t halfPeriod = FP_PERIOD / 2;

  // Compute cycle timing from value and deadtime
  int32_t HV_A = max(0, a * halfPeriod);
  int32_t HV_B = max(0, b * halfPeriod);
  int32_t HV_C = max(0, c * halfPeriod);

  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_CLDOK(FP_LCLDOK_ALL);

  FP_PIN_A_START = halfPeriod - HV_A;
  FP_PIN_A_STOP  = halfPeriod + HV_A;

  FP_PIN_B_START = halfPeriod - HV_B;
  FP_PIN_B_STOP  = halfPeriod + HV_B;

  FP_PIN_C_START = halfPeriod - HV_C;
  FP_PIN_C_STOP  = halfPeriod + HV_C;

  // TODO maybe spinlock here if a few cycles from ticking over... Or not

  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_LDOK(FP_LCLDOK_ALL);

  return true;
}


uint32_t PWMGetResolution() {
  return FP_PERIOD;
}

Vector3 PWMGetPhaseCurrents() {
  return PWMGetPhaseCurrents(false);
}

// Same as getPhaseCurrents but returns values in Alpha-Beta instead of a-b-c
Vector2 PWMGetPhaseCurrents2(bool useInstant) {
  Vector2 rtn;
  Vector3 current = PWMGetPhaseCurrents(useInstant);
  rtn.x = current.x;
  rtn.y = current.y;
  rtn = clarke(rtn);
  return rtn;
}

Vector3 PWMGetPhaseCurrents(bool useInstant) {
  Vector3 output;
  float a = useInstant ? (FP_ISENSE_A_RAW - FP_ISENSE_A_OFFSET) : (FP_ISENSE_A_VAL - FP_ISENSE_A_OFFSET);
  float b = useInstant ? (FP_ISENSE_B_RAW - FP_ISENSE_B_OFFSET) : (FP_ISENSE_B_VAL - FP_ISENSE_B_OFFSET);
  float c = useInstant ? (FP_ISENSE_C_RAW - FP_ISENSE_C_OFFSET) : (FP_ISENSE_C_VAL - FP_ISENSE_C_OFFSET);

  output.x = a * FP_ISENSE_A_RATIO;
  output.y = b * FP_ISENSE_B_RATIO;
  output.z = c * FP_ISENSE_C_RATIO;

#ifdef FORCE_PHASE_CURRENTS_ZERO
  float sum = (output.x + output.y + output.z) / 3;
  output.x -= sum;
  output.y -= sum;
  output.z -= sum;
#endif
  return output;
}


// TODO this is more of a smoothed median than an interquartile average, rename
float FPInterquartileAvg(uint16_t* buffer, int len) {
  if (len > 17) len = 17; // Otherwise bubble sort is too slow
  boolean bumped = true;
  for (int i = 0; i < len; i++) {
    bumped = false;
    // TODO bubble sort is slow (but T4.x has plenty of horsepower)
    for (int x = 0; x < len - 1; x++) {
      if (buffer[x + 1] < buffer[x]) {
        uint16_t temp = buffer[x + 1];
        buffer[x + 1] = buffer[x];
        buffer[x] = temp;
        bumped = true;
      }
    }
    if(!bumped)
      break; // XXX is this correct?
  }
  int half = len / 2;
  int start = max(0, half - 1);
  int end = min(len - 1, half + 1);
  float rtn = 0;
  int cnt = 0;
  for (int x = start; x <= end; x++) {
    rtn += buffer[x];
    cnt++;
  }
  rtn /= cnt;
  return rtn;
}

// Measure the values of the FOC current, inductance, etc
// This function should ALWAYS be called when PWM is active to prevent malfunction / damage
void FPMeasureValues() {
  // The ADC sample buffers should be read as close to the same time as possible
  // and held up for as short of time as possible to ensure in-phase readings
  // and a maximum number of samples
  FP_ISENSE_A_RAWBUF_SZ = FP_ISENSE_A_BUF_PTR; // BEGIN buffer copying, ADC samples after this discarded
  FP_ISENSE_B_RAWBUF_SZ = FP_ISENSE_B_BUF_PTR;
  FP_ISENSE_C_RAWBUF_SZ = FP_ISENSE_C_BUF_PTR;
  memcpy(FP_ISENSE_A_RAWBUF, FP_ISENSE_A_BUF, FP_ISENSE_A_RAWBUF_SZ * sizeof(int16_t));
  memcpy(FP_ISENSE_B_RAWBUF, FP_ISENSE_B_BUF, FP_ISENSE_B_RAWBUF_SZ * sizeof(int16_t));
  memcpy(FP_ISENSE_C_RAWBUF, FP_ISENSE_C_BUF, FP_ISENSE_C_RAWBUF_SZ * sizeof(int16_t));
  FP_ISENSE_A_BUF_PTR = 0;
  FP_ISENSE_B_BUF_PTR = 0;
  FP_ISENSE_C_BUF_PTR = 0; // END buffer copying, ADC samples after this used in next round
  FP_ISENSE_COUNT = FP_ISENSE_A_RAWBUF_SZ;

  // TODO filter raw buffers
  // Make copy of raw data buffer for averaging
  memcpy(FP_ISENSE_A_AVGBUF, FP_ISENSE_A_RAWBUF, FP_ISENSE_A_RAWBUF_SZ * sizeof(int16_t));
  memcpy(FP_ISENSE_B_AVGBUF, FP_ISENSE_B_RAWBUF, FP_ISENSE_B_RAWBUF_SZ * sizeof(int16_t));
  memcpy(FP_ISENSE_C_AVGBUF, FP_ISENSE_C_RAWBUF, FP_ISENSE_C_RAWBUF_SZ * sizeof(int16_t));

  float ia = FPInterquartileAvg(FP_ISENSE_A_AVGBUF, FP_ISENSE_A_RAWBUF_SZ);
  float ib = FPInterquartileAvg(FP_ISENSE_B_AVGBUF, FP_ISENSE_B_RAWBUF_SZ);
  float ic = FPInterquartileAvg(FP_ISENSE_C_AVGBUF, FP_ISENSE_C_RAWBUF_SZ);

  if (!isnan(ia)) {
    FP_ISENSE_A_VAL = constrain(FP_ISENSE_A_VAL * 0.66 + ia * 0.34, ia - FP_ISENSE_WIGGLE, ia + FP_ISENSE_WIGGLE);
    FP_ISENSE_A_RAW = ia;
  }
  if (!isnan(ib)) {
    FP_ISENSE_B_VAL = constrain(FP_ISENSE_B_VAL * 0.66 + ib * 0.34, ib - FP_ISENSE_WIGGLE, ib + FP_ISENSE_WIGGLE);
    FP_ISENSE_B_RAW = ib;
  }
  if (!isnan(ic)) {
    FP_ISENSE_C_VAL = constrain(FP_ISENSE_C_VAL * 0.66 + ic * 0.34, ic - FP_ISENSE_WIGGLE, ic + FP_ISENSE_WIGGLE);
    FP_ISENSE_C_RAW = ic;
  }
}

// ISR that fires off once every PWM period
void FP_PWM_ISR() {
  pinMode(FP_DEBUG_PIN, OUTPUT);
  digitalWriteFast(FP_DEBUG_PIN, HIGH); // CPU usage tracking
  // Clear Reload flag to prevent continuous interrupt
  FP_INTERRUPT_CLEAR |= FLEXPWM_SMSTS_RF;

  if (true) { // Always measure current regardless of whether FOC is enabled or not
    FPMeasureValues();
  }
  if (FOCEN) {
    FOCMain();
  }
  else {
    // PWMSetOutput(0, 0, 0);
  }
  digitalWriteFast(FP_DEBUG_PIN, LOW);
}

int FPGetFrequency() {
  return FP_FREQUENCY;
}

int FPGetPeriod() {
  return FP_PERIOD;
}
