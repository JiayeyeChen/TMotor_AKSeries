#include "ak10-9_v2_testing.h"

AK10_9Handle hAKMotorLeftHip, hAKMotorLeftKnee, hAKMotorRightHip, hAKMotorRightKnee;
AK10_9Handle* hMotorPtrManualControl;

float motor_profiling_trajectory = 0.0f;
uint8_t ifMotorProfilingStarted = 0;
uint32_t timeDifference = 0;
float manualControlValue = 0.0f;
uint8_t ifManualControlStarted = 0;
uint8_t ifIMUFeedbackStarted = 0;
enum ControlMode controlMode = AK10_9_MODE_POSITION;

float velocityAVG[1024] = {0};
uint16_t velocityAVGPtr = 1;
float currentAVG[1024] = {0};
uint16_t currentAVGPtr = 1;

float impedance_control_spring_constant = 0.0f;
float impedance_control_damping_constant = 0.0f;
uint8_t ifImpedanceControlStarted = 0;

void MotorInit(void)
{
  hAKMotorRightHip.hcan = &hcan2;
  hAKMotorRightHip.canID = CAN_ID_TMOTOR_EXOSKELETON_RIGHT_HIP;
  hAKMotorRightHip.lastReceivedTime = 0;
  hAKMotorRightHip.status = AK10_9_Offline;
  hAKMotorRightHip.kt = 1.2138f;
  
  hAKMotorRightKnee.hcan = &hcan2;
  hAKMotorRightKnee.canID = CAN_ID_TMOTOR_EXOSKELETON_RIGHT_KNEE;
  hAKMotorRightKnee.lastReceivedTime = 0;
  hAKMotorRightKnee.status = AK10_9_Offline;
  hAKMotorRightKnee.kt = 1.2138f;
}

void AK10_9_MotorProfiling_Function1(AK10_9Handle* hmotor)
{
  float t = (float)(HAL_GetTick() - timeDifference) / 1000.0f;
  motor_profiling_trajectory = 180.0f * (float)sin((2.0f*pi / 6.0f) * t);
  
//  AK10_9_ImpedanceControl(hmotor, 0.03f, 0.009, motor_profiling_trajectory);
  AK10_9_ServoMode_PositionControl(hmotor, motor_profiling_trajectory);
}

void AK10_9_MotorProfiling_Function2_CurrentControlStepResponse(AK10_9Handle* hmotor)
{
  AK10_9_ServoMode_CurrentControl(hmotor, -1.0f);
}

void AK10_9_Calculate_velocity_current_AVG(AK10_9Handle* hmotor)
{
  velocityAVG[velocityAVGPtr++] = hmotor->realVelocity.f;
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

void AK10_9_ImpedanceControl(AK10_9Handle* hmotor, float spring_constant, float damping_constant, float center_position)
{
  spring_constant = fabs(spring_constant);
  damping_constant = fabs(damping_constant);
  
  float setCurrent = spring_constant * (hmotor->realPosition.f - center_position) - damping_constant * hmotor->realVelocity.f;
  AK10_9_ServoMode_CurrentControl(hmotor, setCurrent);
}

void AK10_9_Set_DataLog_Label(void)
{
  USB_SendDataSlotLabel("13", "P desired (rad)", "P mes (rad)", "V mes (rad/s)", \
                        "A mes (rad/s2)", "LiAccX (m/s2)", "LiAccY (m/s2)", "LiAccZ (m/s2)", \
                        "GyroX (rad/s)", "GyroY (rad/s)", "GyroZ (rad/s)", \
                        "RtAccXGyro (rad/s2)", "RtAccYGyro (rad/s2)", "RtAccZGyro (rad/s2)");
}

void AK10_9_DataLog_Update_Data_Slots(AK10_9Handle* hmotor, BNO055Handle* himu)
{
  uint8_t ptr = 0;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = hmotor->setPosition.f;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = hmotor->realPosition.f;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = hmotor->realVelocity.f;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = 0.0f;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = (float)himu->rawData.AccX.b16;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = (float)himu->rawData.AccY.b16;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = (float)himu->rawData.AccZ.b16;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = (float)himu->rawData.gyroX.b16;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = (float)himu->rawData.gyroY.b16;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = (float)himu->rawData.gyroZ.b16;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = 0.0f;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = 0.0f;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = 0.0f;
  hUSB.ifNewDataLogPiece2Send = 1;
}

void AK10_9_DataLog_Manager(AK10_9Handle* hmotor, BNO055Handle* himu)
{
  USB_DataLogManager(AK10_9_Set_DataLog_Label, dataSlots_AK10_9_Acceleration_Observer_Testing);
  AK10_9_DataLog_Update_Data_Slots(hmotor, himu);
}
