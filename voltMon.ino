const int VM_PIN_VBATT = A10;
volatile float VM_VBATT_RAW = 20;
volatile float VM_VBATT_CAL = 1.0;

bool VM_init(){
  FP_ADC->adc1->setAveraging(1); // set number of averages
  FP_ADC->adc1->setResolution(11); // set bits of resolution
  FP_ADC->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED); // change the conversion speed
  FP_ADC->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED); // change the sampling speed
  FP_ADC->adc1->enableInterrupts(VM_ADC1_ISR);
  FP_ADC->adc1->startSingleRead(VM_PIN_VBATT); // Get the ISR loop running
  return true;
}


// ISR that fires off whenever an ADC has a reading. Used for realtime FOC current measurement
void VM_ADC1_ISR() {
  VM_VBATT_RAW = 40;
  static uint8_t turn = 0;
#if defined(__IMXRT1062__)  // Teensy 4.0
  uint8_t pin = ADC::sc1a2channelADC1[ADC1_HC1 & 0x1f]; // the bits 0-4 of ADC0_SC1A have the channel
#else
  uint8_t pin = ADC::sc1a2channelADC1[ADC1_SC1A & ADC_SC1A_CHANNELS]; // the bits 0-4 of ADC0_SC1A have the channel
#endif
  // Store value in correct location
  if (pin == VM_PIN_VBATT) {
    int val = FP_ADC->adc1->readSingle();
    // TODO if voltage too high, read just voltage until we're sure
    // TODO shut down if voltage too high
    VM_VBATT_RAW = val;
  }
//  else if (pin == XXX) {
//    var = FP_ADC->adc1->readSingle();
//  }

  else { // clear interrupt anyway
    FP_ADC->adc1->readSingle();
  }

  // restore ADC config if it was in use before being interrupted by the analog timer
  if (FP_ADC->adc1->adcWasInUse) {
    // restore ADC config, and restart conversion
    FP_ADC->adc1->loadConfig(&FP_ADC->adc1->adc_config);
    // avoid a conversion started by this isr to repeat itself
    FP_ADC->adc1->adcWasInUse = false;
  }

  // Start the next conversion interrupt
  // Read scheduling
  uint8_t readPin0 = 0;
  if (turn == 0) {
    readPin0 = VM_PIN_VBATT;
  } else if (turn == 1) {
    readPin0 = VM_PIN_VBATT;
  } else if (turn == 2) {
    readPin0 = VM_PIN_VBATT;
  }
  FP_ADC->adc1->startSingleRead(readPin0);
  turn = (turn + 1) % 3;

#if defined(__IMXRT1062__)  // Teensy 4.0
  asm("DSB");
#endif
}

float VMGetBattVoltage(){
  return VM_VBATT_RAW;
}
