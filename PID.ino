void PID(float in, float &out, float setpoint, float kp, float ki, float kd, float &integral, float &perr, float minOut, float maxOut){
  float error = setpoint - in;
  integral = constrain(integral + error * ki, minOut, maxOut);
  float derivative = (error - perr) * kd;
  out = constrain(error * kp + integral + derivative, minOut, maxOut);
  perr = error;
}
