
#define HALLS_A 8
#define HALLS_B 9
#define HALLS_C 10
const uint32_t HALLS_MININT = F_CPU / 2000.0; // Successive hall pulses within 500us are ignored for noise resistance

volatile uint32_t HALLSTMR = 0;
volatile bool HALLSCHG = false;

uint32_t HALLSPRD = 0, HALLSPRD1 = 0; // Halls period
uint8_t HALLSPANG = 0, HALLSPANG1 = 0; // Halls pAng
uint32_t HALLSLUD = 0, HALLSLUD1 = 0; // Hals Last Update time



volatile uint8_t HALLSVAL = 0;
bool Halls_init() {
  // Enable CPU counter for halls
  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

  // Initialize Hall sensor pins
  pinMode(HALLS_A, INPUT);
  pinMode(HALLS_B, INPUT);
  pinMode(HALLS_C, INPUT);

  // Initialize hall interrupts
  attachInterrupt(HALLS_A, HallsAISR, CHANGE);
  attachInterrupt(HALLS_B, HallsBISR, CHANGE);
  attachInterrupt(HALLS_C, HallsCISR, CHANGE);

  // Set initial hall state
  bitWrite(HALLSVAL, 0, digitalReadFast(HALLS_A));
  bitWrite(HALLSVAL, 1, digitalReadFast(HALLS_B));
  bitWrite(HALLSVAL, 2, digitalReadFast(HALLS_C));
  return true;
}

void HallsAISR() {
  bool val = digitalReadFast(HALLS_A);
  if (ARM_DWT_CYCCNT - HALLSTMR < HALLS_MININT)
    return;
  if (bitRead(HALLSVAL, 0) != val) {
    noInterrupts();
    bitWrite(HALLSVAL, 0, val);
    HALLSTMR = ARM_DWT_CYCCNT;
    HALLSCHG = true;
    interrupts();
  }
}
void HallsBISR() {
  bool val = digitalReadFast(HALLS_B);
  if (ARM_DWT_CYCCNT - HALLSTMR < HALLS_MININT)
    return;
  if (bitRead(HALLSVAL, 1) != val) {
    noInterrupts();
    bitWrite(HALLSVAL, 1, val);
    HALLSTMR = ARM_DWT_CYCCNT;
    HALLSCHG = true;
    interrupts();
  }
}
void HallsCISR() {
  bool val = digitalReadFast(HALLS_C);
  if (ARM_DWT_CYCCNT - HALLSTMR < HALLS_MININT)
    return;
  if (bitRead(HALLSVAL, 2) != val) {
    noInterrupts();
    bitWrite(HALLSVAL, 2, val);
    HALLSTMR = ARM_DWT_CYCCNT;
    HALLSCHG = true;
    interrupts();
  }
}


inline uint8_t getHallSector() {
  return HALLSVAL;
}
inline uint32_t getHallTime() {
  return HALLSTMR;
}
inline uint8_t getHallAngle() {
  switch (HALLSVAL) {
    case 1:
      return 6;
    case 5:
      return 5;
    case 4:
      return 4;
    case 6:
      return 3;
    case 2:
      return 2;
    case 3:
      return 1;
    default:
      return 0;
  }
}

float HallsGetRealAngle() {
  noInterrupts();
  uint8_t ang = getHallAngle();
  bool chg = HALLSCHG;
  uint32_t tmr = HALLSTMR;
  interrupts();
  if (chg) {
    HALLSCHG = false;
    HALLSPRD = ARM_DWT_CYCCNT - HALLSLUD; // Halls Period
    HALLSLUD = tmr; // Halls Last Update
    HALLSPANG = ang; // Halls previous angle
  }
  float addend = ((ARM_DWT_CYCCNT - HALLSLUD) * 1.0) / HALLSPRD;
  if (HALLSPANG == 0 || addend > 2)
    addend = 0;
  if(addend > 1)
    addend = 1;
  else if ((ang - HALLSPANG + 6) % 6 == -1)
    addend = -addend;

  if (ang == 0)
    return 0;

   return fmod((ang - 1 - addend) * (TWO_PI / 6) + 1.96, TWO_PI);
}

// The following code is for running off only 1 sensor

float HALLS1_OFFSET = 3.71; // Tweak via experiment, find value that results in no torque. That +- PI/2 is correct value

// Return the angle (0 or 1) of the hall sensor
inline uint8_t getHallAngle1(){
  if(HALLSVAL == 7)
    return 0;
  else
    return 1;
}

/** Get an estimated angle of the rotor with only one sensor
 * @param dir true for positive rotation, false for negative
 * @return angle in Radians
 */
float HallsGetRealAngle1(bool dir){
  noInterrupts();
  uint8_t ang = getHallAngle1();
  bool chg = HALLSCHG;
  uint32_t tmr = HALLSTMR;
  interrupts();
  if (chg) {
    HALLSCHG = false;
    HALLSPRD1 = ARM_DWT_CYCCNT - HALLSLUD1; // Halls Period
    HALLSLUD1 = tmr; // Halls Last Update
    HALLSPANG1 = ang; // Halls previous angle
  }
  float addend = ((ARM_DWT_CYCCNT - HALLSLUD1) * 1.0) / HALLSPRD1;
  if(addend > 2) addend = 0;
  if(addend > 1) addend = 1;
  if(!dir) addend = -addend;
  
  
  return fmod((ang - 1 + addend) * (TWO_PI / 2) + TWO_PI + HALLS1_OFFSET, TWO_PI);
}
