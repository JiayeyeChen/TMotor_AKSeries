#ifndef AK10_9_V2_TESTING_H
#define AK10_9_V2_TESTING_H

#include "system_periphrals.h"
#include "tmotor_ak10-9_v2.h"
#include "my_math.h"
#include <math.h>
#include "exoskeleton.h"
#include "usb.h"

enum ControlMode
{
  AK10_9_MODE_CURRENT,
  AK10_9_MODE_POSITION,
  AK10_9_MODE_VELOCITY,
  AK10_9_MODE_BRAKE
};

void MotorInit(void);
void AK10_9_MotorProfiling_Function1(AK10_9Handle* hmotor);
void AK10_9_MotorProfiling_Function2_CurrentControlStepResponse(AK10_9Handle* hmotor);
void AK10_9_Calculate_velocity_current_AVG(AK10_9Handle* hmotor);
void AK10_9_ImpedanceControl(AK10_9Handle* hmotor, float spring_constant, float damping_constant, float center_position);

void AK10_9_DataLog_Update_Data_Slots(AK10_9Handle* hmotor, BNO055Handle* himu);
void AK10_9_DataLog_Manager(AK10_9Handle* hmotor, BNO055Handle* himu);
void AK10_9_Set_DataLog_Label(void);

extern AK10_9Handle hAKMotorLeftHip, hAKMotorLeftKnee, hAKMotorRightHip, hAKMotorRightKnee;
extern AK10_9Handle* hMotorPtrManualControl;
extern uint8_t ifMotorProfilingStarted;
extern uint32_t timeDifference;
extern float manualControlValue;
extern enum ControlMode controlMode;
extern uint8_t ifManualControlStarted;
extern float impedance_control_spring_constant;
extern float impedance_control_damping_constant;
extern uint8_t ifImpedanceControlStarted;
extern uint8_t ifIMUFeedbackStarted;
#endif
