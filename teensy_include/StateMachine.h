#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include "RANSAC.h"

class StateMachine {
  public:
    enum AGVSTATE { STOP, INBETWEEN_ROWS, SEARCHING_FOR_WALLS/*, END_OF_ROW, DETECTED_WEED*/ };
    AGVSTATE STATE;
    
    
    //Cases
    void stop();
    void inbetween_rows(float sideDifference, float gyro);
    void searching_for_walls();
    //void end_of_row();
    //void detected_weed();
    //Proportional Integral Derivative
    void PID(float sideDifference, float gyro);

    //TP RANSAC ransac;

	const float& GetPIDResult() const { return pidResult; };
  private:
  //Tuning Variables
    const float Kp = .35; //0.35
    const float Ki = 0.0;
    const float Kd = 0.05; //0.7
  //desiredState at 0 means the difference between the distances from both walls is 0, aka they're even distance apart.
    const float desiredState = 0;
  //Errors
    float prevError = 0;
    float kiTotal = 0;
  //Used in inbetween_rows.
    int pwmSpeed = 100;
    float pidResult = 0;
  //Used in searching_for_rows
    int leftCurrentSpeed = 0;
    int rightCurrentSpeed = 0;
};

#endif
