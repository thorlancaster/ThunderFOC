
#define HALLS_A 8
#define HALLS_B 7
#define HALLS_C 10

volatile uint8_t HALLSVAL = 0;
bool hallsInit(){
  pinMode(HALLS_A, INPUT);
  pinMode(HALLS_B, INPUT);
  pinMode(HALLS_C, INPUT);

  attachInterrupt(HALLS_A, HallsAISR, CHANGE);
  attachInterrupt(HALLS_B, HallsBISR, CHANGE);
  attachInterrupt(HALLS_C, HallsCISR, CHANGE);

  bitWrite(HALLSVAL, 0, digitalReadFast(HALLS_A));
  bitWrite(HALLSVAL, 1, digitalReadFast(HALLS_B));
  bitWrite(HALLSVAL, 2, digitalReadFast(HALLS_C));
  return true;
}

void HallsAISR(){
  bitWrite(HALLSVAL, 0, digitalReadFast(HALLS_A));
}
void HallsBISR(){
  bitWrite(HALLSVAL, 1, digitalReadFast(HALLS_B));
}
void HallsCISR(){
  bitWrite(HALLSVAL, 2, digitalReadFast(HALLS_C));
}


inline uint8_t getHallSector(){
  return HALLSVAL;
}
