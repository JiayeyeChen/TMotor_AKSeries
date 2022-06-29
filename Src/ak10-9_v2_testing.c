#include "ak10-9_v2_testing.h"

AK10_9HandleCubaMarsFW hAKMotorLeftHip, hAKMotorLeftKnee, hAKMotorRightHip, hAKMotorRightKnee;
AK10_9HandleCubaMarsFW* hMotorPtrManualControl;
AK10_9HandleDMFW hAKMotorDMFW1, hAKMotorDMFW2, hAKMotorDMFW3;

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

float impedance_control_spring_constant = 0.0f;
float impedance_control_damping_constant = 0.0f;
uint8_t ifImpedanceControlStarted = 0;

float tmotorProfilingSinWaveFrequency = 0.0f;

void MotorInit(void)
{
  hAKMotorRightHip.hcan = &hcan2;
  hAKMotorRightHip.canID = CAN_ID_TMOTOR_EXOSKELETON_RIGHT_HIP;
  hAKMotorRightHip.lastReceivedTime = 0;
  hAKMotorRightHip.status = AK10_9_Offline;
  hAKMotorRightHip.kt = 1.2138f;
  hAKMotorRightHip.accAvgPtr = 0;
  hAKMotorRightHip.posOffset = 210.0f;
  hAKMotorRightHip.posDirectionCorrection = -1.0f;
  hAKMotorRightHip.realAccelerationFiltered.f = 0.0f;
  hAKMotorRightHip.realAccelerationFilteredPrevious = 0.0f;
  hAKMotorRightHip.realAccelerationRaw.f = 0.0f;
  hAKMotorRightHip.cutOffFrequency = 14.043;
  hAKMotorRightHip.timeDuration = 1.0f / 500.0f;
  hAKMotorRightHip.alpha = 2.0f * pi * hAKMotorRightHip.cutOffFrequency * hAKMotorRightHip.timeDuration / (1.0f + 2.0f * pi * hAKMotorRightHip.cutOffFrequency * hAKMotorRightHip.timeDuration);
  hAKMotorRightHip.ifCustomizedPositionSpeedControlFinished = 1;
  //cut-off frequency = 15 hz
//  hAKMotorRightHip.a2Butter = -1.7347f;
//  hAKMotorRightHip.a3Butter = 0.766f;
//  hAKMotorRightHip.b1Butter = 0.0078f;
//  hAKMotorRightHip.b2Butter = 0.0156f;
//  hAKMotorRightHip.b3Butter = 0.0078f;
//cut-off frequency = 2 hz
//  hAKMotorRightHip.a2Butter = -1.9467f;
//  hAKMotorRightHip.a3Butter = 0.9481f;
//  hAKMotorRightHip.b1Butter = 0.00346f;
//  hAKMotorRightHip.b2Butter = 0.006921f;
//  hAKMotorRightHip.b3Butter = 0.00346f;
//cut-off frequency = 6 hz
//  hAKMotorRightHip.a2Butter = -1.8935f;
//  hAKMotorRightHip.a3Butter = 0.8989;
//  hAKMotorRightHip.b1Butter = 0.0013f;
//  hAKMotorRightHip.b2Butter = 0.0027f;
//  hAKMotorRightHip.b3Butter = 0.0013f;
//cut-off frequency = 4 hz
  hAKMotorRightHip.a2Butter = -1.9289f;
  hAKMotorRightHip.a3Butter = 0.9314;
  hAKMotorRightHip.b1Butter = 0.0006;
  hAKMotorRightHip.b2Butter = 0.0012;
  hAKMotorRightHip.b3Butter = 0.0006;
  
  hAKMotorRightKnee.hcan = &hcan2;
  hAKMotorRightKnee.canID = CAN_ID_TMOTOR_EXOSKELETON_RIGHT_KNEE;
  hAKMotorRightKnee.lastReceivedTime = 0;
  hAKMotorRightKnee.status = AK10_9_Offline;
  hAKMotorRightKnee.kt = 1.2138f;
  hAKMotorRightKnee.accAvgPtr = 0;
  hAKMotorRightKnee.posOffset = 0.0f;
  hAKMotorRightKnee.posDirectionCorrection = -1.0f;
  hAKMotorRightKnee.realAccelerationFiltered.f = 0.0f;
  hAKMotorRightKnee.realAccelerationFilteredPrevious = 0.0f;
  hAKMotorRightKnee.realAccelerationRaw.f = 0.0f;
  hAKMotorRightKnee.cutOffFrequency = 14.043;
  hAKMotorRightKnee.timeDuration = 1.0f / 500.0f;
  hAKMotorRightKnee.alpha = hAKMotorRightKnee.cutOffFrequency * hAKMotorRightKnee.timeDuration / (1.0f + hAKMotorRightKnee.cutOffFrequency * hAKMotorRightKnee.timeDuration);
  hAKMotorRightKnee.ifCustomizedPositionSpeedControlFinished = 1;
  //cut-off frequency = 4 hz
  hAKMotorRightKnee.a2Butter = -1.9289f;
  hAKMotorRightKnee.a3Butter = 0.9314;
  hAKMotorRightKnee.b1Butter = 0.0006;
  hAKMotorRightKnee.b2Butter = 0.0012;
  hAKMotorRightKnee.b3Butter = 0.0006;
  
  
  hAKMotorDMFW1.canID = 0x01;
  hAKMotorDMFW1.hcan = &hcan2;
  hAKMotorDMFW1.lastReceivedTime = 0;
  hAKMotorDMFW1.status = AK10_9_Offline;
  hAKMotorDMFW1.kt = 1.2138f;
  hAKMotorDMFW1.kp.f = 0.0f;
  hAKMotorDMFW1.kd.f = 0.0f;
  hAKMotorDMFW1.controlMode = AK10_9_DM_FW_MODE_MIT;
  
  hAKMotorDMFW2.canID = 0x02;
  hAKMotorDMFW2.hcan = &hcan2;
  hAKMotorDMFW2.lastReceivedTime = 0;
  hAKMotorDMFW2.status = AK10_9_Offline;
  hAKMotorDMFW2.kt = 1.2138f;
  hAKMotorDMFW2.kp.f = 0.0f;
  hAKMotorDMFW2.kd.f = 0.0f;
  hAKMotorDMFW2.controlMode = AK10_9_DM_FW_MODE_MIT;
  
  hAKMotorDMFW3.canID = 0x03;
  hAKMotorDMFW3.hcan = &hcan2;
  hAKMotorDMFW3.lastReceivedTime = 0;
  hAKMotorDMFW3.status = AK10_9_Offline;
  hAKMotorDMFW3.kt = 1.2138f;
  hAKMotorDMFW3.kp.f = 0.0f;
  hAKMotorDMFW3.kd.f = 0.0f;
  hAKMotorDMFW3.controlMode = AK10_9_DM_FW_MODE_MIT;
}

void AK10_9_MotorProfiling_Function1_Half_Sin(AK10_9HandleCubaMarsFW* hmotor, float frequency)
{
  float t = (float)(HAL_GetTick() - timeDifference) / 1000.0f;
  motor_profiling_trajectory = 180.0f * (float)sin(frequency * 2.0f * pi * t);
  
  AK10_9_ServoMode_PositionControl(hmotor, motor_profiling_trajectory);
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

void AK10_9_ImpedanceControl(AK10_9HandleCubaMarsFW* hmotor, float spring_constant, float damping_constant, float center_position)
{
  spring_constant = fabs(spring_constant);
  damping_constant = fabs(damping_constant);
  
  float setCurrent = spring_constant * (hmotor->realPosition.f - center_position) - damping_constant * hmotor->realVelocityPresent.f;
  AK10_9_ServoMode_CurrentControl(hmotor, setCurrent);
}

void AK10_9_Set_DataLog_Label_Acceleration_Observer(void)
{
  USB_SendDataSlotLabel("11", "P desired (rad)", "P mes (rad)", "V mes (rad/s)", \
                        "Acc desired (deg/s)", "Acc raw (deg/s)", \
                        "Acc desired by real pos (deg/s)", "Acc filtered (deg/s)", \
                        "Acc estimation error", "AccXIMU (m/s2)", "AccYIMU (m/s2)", "AccZIMU (m/s2)");
}


void AK10_9_DataLog_Update_Data_Slots(AK10_9HandleCubaMarsFW* hmotor, BNO055Handle* himu)
{
  uint8_t ptr = 0;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = hmotor->setPosition.f;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = hmotor->realPosition.f;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = hmotor->realVelocityPresent.f;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = hmotor->setAcceleration.f;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = hmotor->realAccelerationRaw.f;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = hmotor->setAcceleration_ByRealPosition.f;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = hmotor->realAccelerationFiltered.f;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = hmotor->realAccelerationFiltered.f - hmotor->setAcceleration_ByRealPosition.f;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = himu->parsedData.AccX.f * 200.0f;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = himu->parsedData.AccY.f;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = himu->parsedData.AccZ.f;
  hUSB.ifNewDataLogPiece2Send = 1;
}

void AK10_9_DataLog_Manager(AK10_9HandleCubaMarsFW* hmotor, BNO055Handle* himu)
{
  USB_DataLogManager(AK10_9_Set_DataLog_Label_Acceleration_Observer, dataSlots_AK10_9_Acceleration_Observer_Testing);
  AK10_9_DataLog_Update_Data_Slots(hmotor, himu);
}




