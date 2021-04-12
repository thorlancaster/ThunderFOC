const int VM_PIN_VBATT = A12;
const int VM_PIN_THROTTLE = A9;

int VM_PIN_LAST;

volatile int VM_THROTTLE_ZERO = 200;
volatile int VM_THROTTLE_FULL = 820;

volatile float VM_VBATT_RAW = 0;
volatile float VM_VBATT_CAL = 0.1023;
volatile float VM_THROTTLE_RAW = 0;
int VM_HVC_VAL = 65 / VM_VBATT_CAL; // Voltage exceeds this = shutdown for safety

ADC *VM_ADC = new ADC();

bool VM_init() {
  VM_ADC->adc1->setAveraging(1); // set number of averages
  VM_ADC->adc1->setResolution(11); // set bits of resolution
  VM_ADC->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED); // change the conversion speed
  VM_ADC->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED); // change the sampling speed
  VM_ADC->adc1->enableInterrupts(VM_ADC1_ISR);
  VM_ADC->adc1->startSingleRead(VM_PIN_VBATT); // Get the ISR loop running
  return true;
}


// ISR that fires off whenever an ADC has a reading. Used for realtime FOC current measurement
void VM_ADC1_ISR() {
  static uint8_t turn = 0;
  static uint8_t ovCount = 0; // To suppress noise, require multiple samples to confirm overvoltage
#if defined(__IMXRT1062__)  // Teensy 4.0
  uint8_t channel = ADC1_HC1 & 0x1f;
#else
  uint8_t channel = ADC1_SC1A & ADC_SC1A_CHANNELS;
#endif
  uint8_t pin = ADC::sc1a2channelADC1[channel];

  // Store value in correct location
  if (VM_PIN_LAST == VM_PIN_VBATT) {
    int val = VM_ADC->adc1->readSingle();
    // Shut down if voltage too high
    VM_VBATT_RAW = val * 0.005 + VM_VBATT_RAW * 0.995;
    if (val > VM_HVC_VAL) {
      ovCount++;
      if (ovCount > 8) { // 20 microseconds of overvoltage
        PWMDisable();
        ovCount = 0;
      }
    } else if (ovCount > 0) {
      ovCount--;
    }
  }
  else if (VM_PIN_LAST == VM_PIN_THROTTLE) {
    VM_THROTTLE_RAW = VM_THROTTLE_RAW * 0.95 + VM_ADC->adc1->readSingle() * 0.05;
  }
  else { // Clear interrupt
    VM_ADC->adc1->readSingle();
  }

  //  else { // clear interrupt anyway
  //    VM_VBATT_RAW = channel;
  //    VM_ADC->adc1->readSingle();
  //  }

  // restore ADC config if it was in use before being interrupted by the analog timer
  if (VM_ADC->adc1->adcWasInUse) {
    // restore ADC config, and restart conversion
    VM_ADC->adc1->loadConfig(&VM_ADC->adc1->adc_config);
    // avoid a conversion started by this isr to repeat itself
    VM_ADC->adc1->adcWasInUse = false;
  }

  // Start the next conversion interrupt
  // Read scheduling
  uint8_t readPin0 = 0;
  if (turn == 0) {
    readPin0 = VM_PIN_VBATT;
  } else if (turn == 1) {
    readPin0 = VM_PIN_VBATT;
  } else if (turn == 2) {
    readPin0 = VM_PIN_THROTTLE;
  }
  VM_PIN_LAST = readPin0;
  VM_ADC->adc1->startSingleRead(readPin0);
  turn = (turn + 1) % 3;

#if defined(__IMXRT1062__)  // Teensy 4.0
  asm("DSB");
#endif
}

float VMGetThrottle() {
  return constrain((VM_THROTTLE_RAW - VM_THROTTLE_ZERO) / (VM_THROTTLE_FULL - VM_THROTTLE_ZERO), 0, 1);
}

float VMGetBattVoltage() {
  return VM_VBATT_RAW * VM_VBATT_CAL;
}
