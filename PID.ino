// PID and related control algorithms


float PID(float in, float out, float setpoint, float kp, float ki, float kd, float &integral, float &perr, float minOut, float maxOut){
  float error = setpoint - in;
  integral = constrain(integral + error * ki, minOut, maxOut);
  float derivative = (error - perr) * kd;
  out = constrain(error * kp + integral + derivative, minOut, maxOut);
  perr = error;
  return out;
}

/** I2R Stamina calculation, for basic thermal modelling
 * Used to limit brief excursions above rated power for fun without FETs or battery going blowie blowie
 * @param in input (1 = max continuous rating)
 * @param wheel Float pointer to hold stamina wheel
 * @param timeConst number of seconds to increase to 60% of final value
 * @param timeStep Change in time from last invocation. Should be much smaller than timeConst
 * @return 0.0 = cold, 1.0 = at max temperature (STAAAHP THE TRAIN!!!)
 */
float calcStamina(float in, float &wheel, float timeConst, float timeStep){
  timeStep = min(timeStep, timeConst);
  float finalTemp = in * in;
  float newPct = timeStep / timeConst;
  wheel = wheel * (1-newPct) + finalTemp * newPct;
  return wheel;
}
