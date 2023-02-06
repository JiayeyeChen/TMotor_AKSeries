#include "ak10-9_v2_testing.h"

AK10_9HandleCubaMarsFW hAKMotorRightHip_old, hAKMotorRightKnee, hAKMotorSpare1;
AK10_9HandleDMFW       hAKMotorRightHip;
AK10_9HandleCubaMarsFW* hMotorPtrManualControl;
AK10_9HandleDMFW* hMotorPtrManualControlDMFW;


float motor_profiling_trajectory = 0.0f;
float manualControlValue_pos = 0.0f;
float manualControlValue_vel = 0.0f;
float manualControlValue_cur = 0.0f;
float manualControlValue_kp = 0.0f, manualControlValue_kd = 0.0f;
uint8_t ifMotorProfilingStarted = 0;
uint32_t timeDifference = 0;
uint8_t ifManualControlStarted = 0;
uint8_t ifIMUFeedbackStarted = 0;
enum ControlModeCubeMarsFW controlModeCubeMarsFW = AK10_9_CUBEMARS_FW_MODE_POSITION;

float velocityAVG[1024] = {0};
uint16_t velocityAVGPtr = 1;
float currentAVG[1024] = {0};
uint16_t currentAVGPtr = 1;

float tmotorProfilingSinWaveFrequency = 0.0f;

void EXOSKELETON_MotorInit(void)
{
  hAKMotorRightHip.canID = CAN_ID_TMOTOR_EXOSKELETON_RIGHT_HIP_TX;
  hAKMotorRightHip.hcan = &hcan2;
  hAKMotorRightHip.lastReceivedTime = 0;
  hAKMotorRightHip.status = AK10_9_Offline;
  hAKMotorRightHip.kt = 1.23078f;
  hAKMotorRightHip.posOffsetDeg = -210.0f;
  hAKMotorRightHip.posOffsetRad = hAKMotorRightHip.posOffsetDeg * deg2rad;
  hAKMotorRightHip.posDirectionCorrection = 1.0f;
  hAKMotorRightHip.setPos.f = 0.0f;
  hAKMotorRightHip.setVel.f = 0.0f;
  hAKMotorRightHip.setIq.f = 0.0f;
  hAKMotorRightHip.setKp.f = 0.0f;
  hAKMotorRightHip.setKd.f = 0.0f;
  hAKMotorRightHip.goalPos.f = 0.0f;
  hAKMotorRightHip.goalVel.f = 0.0f;
  hAKMotorRightHip.goalIq.f = 0.0f;
  hAKMotorRightHip.goalKp.f = 0.0f;
  hAKMotorRightHip.goalKd.f = 0.0f;
  hAKMotorRightHip.controlMode = AK10_9_DM_FW_MODE_MIT;
  hAKMotorRightHip.a2Butter = -1.9645;
  hAKMotorRightHip.a3Butter = 0.9651;
  hAKMotorRightHip.b1Butter = 0.0001551;
  hAKMotorRightHip.b2Butter = 0.0003103;
  hAKMotorRightHip.b3Butter = 0.0001551;
  hAKMotorRightHip.realAccelerationFiltered.f = 0.0f;
  hAKMotorRightHip.realAccelerationRaw.f = 0.0f;
  hAKMotorRightHip.ifCustomizedPositionSpeedControlFinished = 1;
  hAKMotorRightHip.enablingStatus = AK10_9_MITMODE_DISABLED;
  hAKMotorRightHip.ifMITModeParameterSmootherWorkFinished = 0;
  
  hAKMotorRightKnee.hcan = &hcan2;
  hAKMotorRightKnee.canID = CAN_ID_TMOTOR_EXOSKELETON_RIGHT_KNEE_TX;
  hAKMotorRightKnee.lastReceivedTime = 0;
  hAKMotorRightKnee.status = AK10_9_Offline;
  hAKMotorRightKnee.kt = 1.04154f;
  hAKMotorRightKnee.accAvgPtr = 0;
  hAKMotorRightKnee.posOffsetDeg = 0.0f;
  hAKMotorRightKnee.posOffsetRad = hAKMotorRightKnee.posOffsetDeg * deg2rad;
  hAKMotorRightKnee.posDirectionCorrection = -1.0f;
  hAKMotorRightKnee.setPos.f = 0.0f;
  hAKMotorRightKnee.setVel.f = 0.0f;
  hAKMotorRightKnee.setIq.f = 0.0f;
  hAKMotorRightKnee.setKp.f = 0.0f;
  hAKMotorRightKnee.setKd.f = 0.0f;
  hAKMotorRightKnee.goalPos.f = 0.0f;
  hAKMotorRightKnee.goalVel.f = 0.0f;
  hAKMotorRightKnee.goalIq.f = 0.0f;
  hAKMotorRightKnee.goalKp.f = 0.0f;
  hAKMotorRightKnee.goalKd.f = 0.0f;
  hAKMotorRightKnee.realAccelerationFiltered.f = 0.0f;
  hAKMotorRightKnee.realAccelerationFilteredPrevious = 0.0f;
  hAKMotorRightKnee.realAccelerationRaw.f = 0.0f;
  hAKMotorRightKnee.cutOffFrequency = 14.043;
  hAKMotorRightKnee.timeDuration = 1.0f / 500.0f;
  hAKMotorRightKnee.alpha = hAKMotorRightKnee.cutOffFrequency * hAKMotorRightKnee.timeDuration / (1.0f + hAKMotorRightKnee.cutOffFrequency * hAKMotorRightKnee.timeDuration);
  hAKMotorRightKnee.ifCustomizedPositionSpeedControlFinished = 1;
  hAKMotorRightKnee.ifMITModeParameterSmootherWorkFinished = 0;
  //cut-off frequency = 4 hz sampling rate 1000hz(no problem)
  hAKMotorRightKnee.a2Butter = -1.9645;
  hAKMotorRightKnee.a3Butter = 0.9651;
  hAKMotorRightKnee.b1Butter = 0.0001551;
  hAKMotorRightKnee.b2Butter = 0.0003103;
  hAKMotorRightKnee.b3Butter = 0.0001551;
//cut-off frequency = 3 hz sampling rate 1000hz(no problem)
//  hAKMotorRightKnee.a2Butter = -1.9733;
//  hAKMotorRightKnee.a3Butter = 0.9737;
//  hAKMotorRightKnee.b1Butter = 0.0000877;
//  hAKMotorRightKnee.b2Butter = 0.0001753;
//  hAKMotorRightKnee.b3Butter = 0.0000877;
  hAKMotorRightKnee.enablingStatus = AK10_9_MITMODE_DISABLED;
}

void MotorInit_CubeMarsFW(void)
{
  hAKMotorSpare1.hcan = &hcan2;
  hAKMotorSpare1.canID = CAN_ID_TMOTOR_SPARE1_SERVOMODE;
  hAKMotorSpare1.lastReceivedTime = 0;
  hAKMotorSpare1.status = AK10_9_Offline;
  hAKMotorSpare1.kt = 1.04154f;
  hAKMotorSpare1.accAvgPtr = 0;
  hAKMotorSpare1.posOffsetDeg = 0.0f;
  hAKMotorSpare1.posOffsetRad = hAKMotorSpare1.posOffsetDeg * deg2rad;
  hAKMotorSpare1.posDirectionCorrection = -1.0f;
  hAKMotorSpare1.setPos.f = 0.0f;
  hAKMotorSpare1.setVel.f = 0.0f;
  hAKMotorSpare1.setIq.f = 0.0f;
  hAKMotorSpare1.setKp.f = 0.0f;
  hAKMotorSpare1.setKd.f = 0.0f;
  hAKMotorSpare1.goalPos.f = 0.0f;
  hAKMotorSpare1.goalVel.f = 0.0f;
  hAKMotorSpare1.goalIq.f = 0.0f;
  hAKMotorSpare1.goalKp.f = 0.0f;
  hAKMotorSpare1.goalKd.f = 0.0f;
  hAKMotorSpare1.realAccelerationFiltered.f = 0.0f;
  hAKMotorSpare1.realAccelerationFilteredPrevious = 0.0f;
  hAKMotorSpare1.realAccelerationRaw.f = 0.0f;
  hAKMotorSpare1.cutOffFrequency = 14.043;
  hAKMotorSpare1.timeDuration = 1.0f / 500.0f;
  hAKMotorSpare1.alpha = hAKMotorSpare1.cutOffFrequency * hAKMotorSpare1.timeDuration / (1.0f + hAKMotorSpare1.cutOffFrequency * hAKMotorSpare1.timeDuration);
  hAKMotorSpare1.ifCustomizedPositionSpeedControlFinished = 1;
  hAKMotorSpare1.ifMITModeParameterSmootherWorkFinished = 0;
  //cut-off frequency = 4 hz sampling rate 1000hz(no problem)
  hAKMotorSpare1.a2Butter = -1.9645;
  hAKMotorSpare1.a3Butter = 0.9651;
  hAKMotorSpare1.b1Butter = 0.0001551;
  hAKMotorSpare1.b2Butter = 0.0003103;
  hAKMotorSpare1.b3Butter = 0.0001551;
  hAKMotorSpare1.enablingStatus = AK10_9_MITMODE_DISABLED;
}

void AK10_9_MotorProfiling_Function1_Half_Sin(AK10_9HandleCubaMarsFW* hmotor, float frequency)
{
  float t = (float)(HAL_GetTick() - timeDifference) / 1000.0f;
  motor_profiling_trajectory = 180.0f * (float)sin(frequency * 2.0f * pi * t);
  
  /* for Servo Mode*/
//  AK10_9_ServoMode_PositionControl(hmotor, motor_profiling_trajectory);
  /* for MIT Mode*/
  AK10_9_MITModeControl_Deg(hmotor, motor_profiling_trajectory, 0.0f, 35.0f, 3.25f, 0.0f);
  hmotor->setAcceleration.f = -180.0f * pow(2.0f * pi * frequency, 2.0f) * sin(frequency * 2.0f * pi * t);
  hmotor->setAcceleration_ByRealPosition.f = -pow(2.0f * pi * frequency, 2.0f) * hmotor->realPosition.f;
}

void AK10_9_MotorProfiling_Function2_CurrentControlStepResponse(AK10_9HandleCubaMarsFW* hmotor)
{
  AK10_9_ServoMode_CurrentControl(hmotor, -1.0f);
}

void AK10_9_Calculate_velocity_current_AVG(AK10_9HandleCubaMarsFW* hmotor)
{
  velocityAVG[velocityAVGPtr++] = hmotor->realVelocityPresent.f;
  if (velocityAVGPtr == 1023)
    velocityAVGPtr = 1;
  velocityAVG[0] = 0.0f;
  for (uint16_t i = 1; i <= 1023; i++)
    velocityAVG[0] += velocityAVG[i];
  velocityAVG[0] /= 1023.0f;
  
  currentAVG[currentAVGPtr++] = hmotor->realCurrent.f;
  if (currentAVGPtr == 1023)
    currentAVGPtr = 1;
  currentAVG[0] = 0.0f;
  for (uint16_t i = 1; i <= 1023; i++)
    currentAVG[0] += currentAVG[i];
  currentAVG[0] /= 1023.0f;
}

void AK10_9_Set_DataLog_Label_Acceleration_Observer(void)
{
  USB_SendDataSlotLabel("12", "P desired (rad)", "P mes (rad)", "V mes (rad/s)", \
                        "Acc desired (deg/s)", "Acc raw (deg/s)", \
                        "Acc desired by real pos (deg/s)", "Acc filtered (deg/s)", \
                        "Acc estimation error", "AccXIMU (m/s2)", "AccYIMU (m/s2)", "AccZIMU (m/s2)", "Iq");
}


void AK10_9_DataLog_Update_Data_Slots_CubeMARS_FW(AK10_9HandleCubaMarsFW* hmotor, BNO055Handle* himu)
{
  uint8_t ptr = 0;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = hmotor->setPos.f;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = hmotor->realPosition.f;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = hmotor->realVelocityPresent.f;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = hmotor->setAcceleration.f;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = hmotor->realAccelerationRaw.f;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = hmotor->setAcceleration_ByRealPosition.f;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = hmotor->realAccelerationFiltered.f;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = hmotor->realAccelerationFiltered.f - hmotor->setAcceleration_ByRealPosition.f;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = himu->parsedData.AccX.f;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = himu->parsedData.AccY.f;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = himu->parsedData.AccZ.f;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = hmotor->realCurrent.f;
  hUSB.ifNewDataLogPiece2Send = 1;
}

void AK10_9_DataLog_Update_Data_Slots_DM_FW(AK10_9HandleDMFW* hmotor, BNO055Handle* himu)
{
  uint8_t ptr = 0;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = hmotor->setPos.f;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = hmotor->realPositionDeg.f;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = hmotor->realVelocityPresent.f;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = hmotor->setAcceleration.f;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = hmotor->realAccelerationRaw.f;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = hmotor->setAcceleration_ByRealPosition.f;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = hmotor->realAccelerationFiltered.f;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = hmotor->realAccelerationFiltered.f - hmotor->setAcceleration_ByRealPosition.f;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = himu->parsedData.AccX.f;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = himu->parsedData.AccY.f;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = himu->parsedData.AccZ.f;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = hmotor->realCurrent.f;
  hUSB.ifNewDataLogPiece2Send = 1;
}

void AK10_9_DataLog_Manager_CubeMARS_FW(AK10_9HandleCubaMarsFW* hmotor, BNO055Handle* himu)
{
  USB_DataLogManager(AK10_9_Set_DataLog_Label_Acceleration_Observer, dataSlots_AK10_9_Acceleration_Observer_Testing);
  AK10_9_DataLog_Update_Data_Slots_CubeMARS_FW(hmotor, himu);
}

void AK10_9_DataLog_Manager_DM_FW(AK10_9HandleDMFW* hmotor, BNO055Handle* himu)
{
  USB_DataLogManager(AK10_9_Set_DataLog_Label_Acceleration_Observer, dataSlots_AK10_9_Acceleration_Observer_Testing);
  AK10_9_DataLog_Update_Data_Slots_DM_FW(hmotor, himu);
}



