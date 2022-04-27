#include "ak10-9_v2_testing.h"

AK10_9Handle hAKMotorLeftHip, hAKMotorLeftKnee, hAKMotorRightHip, hAKMotorRightKnee;

float motor_profiling_trajectory = 0.0f;
uint8_t ifMotorProfilingStarted = 0;
uint32_t timeDifference = 0;
float manualControlValue = 0.0f;
uint8_t ifManualControlStarted = 0;
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
  hAKMotorLeftHip.hcan = &hcan2;
  hAKMotorLeftHip.canID = CAN_ID_TMOTOR_EXOSKELETON_LEFT_HIP;
  hAKMotorLeftHip.lastReceivedTime = 0;
  hAKMotorLeftHip.status = AK10_9_Offline;
  hAKMotorLeftHip.kt = 1.2138f;
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
