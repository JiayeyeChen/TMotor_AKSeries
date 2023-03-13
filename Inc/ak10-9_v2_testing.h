#ifndef AK10_9_V2_TESTING_H
#define AK10_9_V2_TESTING_H

#include "system_periphrals.h"
#include "tmotor_ak10-9_v2.h"
#include "my_math.h"
#include <math.h>
#include "exoskeleton.h"
#include "usb.h"

enum ControlModeCubeMarsFW
{
  AK10_9_CUBEMARS_FW_MODE_CURRENT,
  AK10_9_CUBEMARS_FW_MODE_POSITION,
  AK10_9_CUBEMARS_FW_MODE_VELOCITY,
  AK10_9_CUBEMARS_FW_MODE_BRAKE
};

void EXOSKELETON_MotorInit(void);
void MotorInit_CubeMarsFW(void);
void AK10_9_MotorProfiling_Function1_Half_Sin(AK10_9HandleCubaMarsFW* hmotor, float frequency);
void AK10_9_MotorProfiling_Function2_CurrentControlStepResponse(AK10_9HandleCubaMarsFW* hmotor);
void AK10_9_Calculate_velocity_current_AVG(AK10_9HandleCubaMarsFW* hmotor);

void AK10_9_DataLog_Update_Data_Slots_CubeMARS_FW(AK10_9HandleCubaMarsFW* hmotor, BNO055Handle* himu);
void AK10_9_DataLog_Update_Data_Slots_DM_FW(AK10_9HandleDMFW* hmotor, BNO055Handle* himu);
void AK10_9_DataLog_Manager_CubeMARS_FW(AK10_9HandleCubaMarsFW* hmotor, BNO055Handle* himu);
void AK10_9_DataLog_Manager_DM_FW(AK10_9HandleDMFW* hmotor, BNO055Handle* himu);
void AK10_9_Set_DataLog_Label_Acceleration_Observer(void);

extern AK10_9HandleCubaMarsFW hAKMotorRightHip, hAKMotorRightKnee;

extern AK10_9HandleCubaMarsFW* hMotorPtrManualControl;

extern uint8_t ifMotorProfilingStarted;
extern uint32_t timeDifference;
extern enum ControlModeCubeMarsFW controlModeCubeMarsFW;
extern uint8_t ifManualControlStarted;
extern uint8_t ifIMUFeedbackStarted;
extern float manualControlValue_pos;
extern float manualControlValue_vel;
extern float manualControlValue_cur;
extern float manualControlValue_kp, manualControlValue_kd;
extern float tmotorProfilingSinWaveFrequency;

#endif
