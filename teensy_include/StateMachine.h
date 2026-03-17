#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include "RANSAC.h"

#include "datalayer.h"

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
	void Configure(const DriveControl& config, const PIDControl& PIDConfig);
	const float& GetPIDResult() const { return pidResult; };
  private:
  //Tuning Variables
    float Kp = .35; //0.35
    float Ki = 0.0;
    float Kd = 0.05; //0.7
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

    // Tuneable variables    
    int minMotorPWM = 40;
    int maxMotorPWM = 255;
    float aggressiveThreshold = 200.0f;
    float aggressiveMultiplier = 1.5f;
	float steeringLimitRatio = 0.8f;
    };

#endif
