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

typedef struct
{
  uint8_t ifTestingStarted;
  uint8_t ifTestingFinished;
  uint8_t curSamplingCount;
  float   curSetIq;
  uint8_t ifCurStable;
  uint32_t  curNewSetTimeStamp;
  float   torqueMeasured;
  
}TMotorStaticTorqueConstantHandle;

void MotorInit(void);
void AK10_9_MotorProfiling_Function1_Half_Sin(AK10_9Handle* hmotor, float frequency);
void AK10_9_MotorProfiling_Function2_CurrentControlStepResponse(AK10_9Handle* hmotor);
void AK10_9_Calculate_velocity_current_AVG(AK10_9Handle* hmotor);
void AK10_9_ImpedanceControl(AK10_9Handle* hmotor, float spring_constant, float damping_constant, float center_position);

void AK10_9_DataLog_Update_Data_Slots(AK10_9Handle* hmotor, BNO055Handle* himu);
void AK10_9_DataLog_Manager(AK10_9Handle* hmotor, BNO055Handle* himu);
void AK10_9_Set_DataLog_Label_Acceleration_Observer(void);
void AK10_9_Set_DataLog_Label_Torque_Constant_Testing(void);
void AK10_9_StaticTorqueConstantTestingManager(AK10_9Handle* hmotor, float iq_step, float iq_max, float iq_sign, uint32_t sampling_num_per_step);
void AK10_9_StaticTorqueConstantTesting_Init(void);


extern AK10_9Handle hAKMotorLeftHip, hAKMotorLeftKnee, hAKMotorRightHip, hAKMotorRightKnee;
extern AK10_9Handle* hMotorPtrManualControl;
extern uint8_t ifMotorProfilingStarted;
extern uint32_t timeDifference;
extern enum ControlMode controlMode;
extern uint8_t ifManualControlStarted;
extern float impedance_control_spring_constant;
extern float impedance_control_damping_constant;
extern uint8_t ifImpedanceControlStarted;
extern uint8_t ifIMUFeedbackStarted;
extern float manualControlValue_pos;
extern float manualControlValue_vel;
extern float manualControlValue_cur;
extern float tmotorProfilingSinWaveFrequency;
extern TMotorStaticTorqueConstantHandle hStaticTorqueConstantTesting;

#endif
