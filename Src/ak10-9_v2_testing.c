#include "ak10-9_v2_testing.h"

AK10_9Handle hAKMotorLeftHip, hAKMotorLeftKnee, hAKMotorRightHip, hAKMotorRightKnee;
AK10_9Handle* hMotorPtrManualControl;
TMotorStaticTorqueConstantHandle hStaticTorqueConstantTesting;


float motor_profiling_trajectory = 0.0f;
float manualControlValue_pos = 0.0f;
float manualControlValue_vel = 0.0f;
float manualControlValue_cur = 0.0f;
uint8_t ifMotorProfilingStarted = 0;
uint32_t timeDifference = 0;
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

float tmotorProfilingSinWaveFrequency = 0.0f;

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

void AK10_9_MotorProfiling_Function1_Half_Sin(AK10_9Handle* hmotor, float frequency)
{
  float t = (float)(HAL_GetTick() - timeDifference) / 1000.0f;
  motor_profiling_trajectory = 180.0f * (float)sin(frequency * 2.0f * pi * t);
  
  AK10_9_ServoMode_PositionControl(hmotor, motor_profiling_trajectory);
  hmotor->setAcceleration.f = -180.0f * pow(2.0f * pi * frequency, 2.0f) * sin(frequency * 2.0f * pi * t);
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

void AK10_9_Set_DataLog_Label_Acceleration_Observer(void)
{
  USB_SendDataSlotLabel("14", "P desired (rad)", "P mes (rad)", "V mes (rad/s)", \
                        "A des (rad/s2)", "A mes (rad/s2)", "LiAccX (m/s2)", "LiAccY (m/s2)", "LiAccZ (m/s2)", \
                        "GyroX (rad/s)", "GyroY (rad/s)", "GyroZ (rad/s)", \
                        "RtAccXGyro (rad/s2)", "RtAccYGyro (rad/s2)", "RtAccZGyro (rad/s2)");
}

void AK10_9_Set_DataLog_Label_Torque_Constant_Testing(void)
{
  USB_SendDataSlotLabel("2", "Torque(Nm)", "Iq(A)");
}

void AK10_9_DataLog_Update_Data_Slots(AK10_9Handle* hmotor, BNO055Handle* himu)
{
  uint8_t ptr = 0;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = hmotor->setPosition.f;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = hmotor->realPosition.f;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = hmotor->realVelocity.f;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = hmotor->setAcceleration.f;
  dataSlots_AK10_9_Acceleration_Observer_Testing[ptr++].f = hmotor->estimateAcceleration.f;
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
  USB_DataLogManager(AK10_9_Set_DataLog_Label_Acceleration_Observer, dataSlots_AK10_9_Acceleration_Observer_Testing);
  AK10_9_DataLog_Update_Data_Slots(hmotor, himu);
}

void AK10_9_StaticTorqueConstantTestingManager(AK10_9Handle* hmotor, float iq_step, float iq_max, float iq_sign, uint32_t sampling_num_per_step)
{
  if (hStaticTorqueConstantTesting.ifTestingStarted)
  {
    
    //Iq tracking is considered stable
    if (hStaticTorqueConstantTesting.ifCurStable)
    {
      //Data sampling count reaches the max limit
      if (hStaticTorqueConstantTesting.curSamplingCount++ >= sampling_num_per_step)
      {
        hStaticTorqueConstantTesting.curSamplingCount = 0;
        hStaticTorqueConstantTesting.curSetIq += (iq_sign * iq_step);
        if (fabs(hStaticTorqueConstantTesting.curSetIq) > iq_max)
        {
          hStaticTorqueConstantTesting.ifTestingFinished = 1;
          hStaticTorqueConstantTesting.ifTestingStarted = 0;
          USB_DataLogEnd();
          return;
        }
        
        hStaticTorqueConstantTesting.curNewSetTimeStamp = HAL_GetTick();
        hStaticTorqueConstantTesting.ifCurStable = 0;
      }
      //Data sampling still within the max limit
      else
      {
        dataSlots_AK10_9_TorqueConstantTesting[0].f = hStaticTorqueConstantTesting.torqueMeasured;
        dataSlots_AK10_9_TorqueConstantTesting[1].f = hmotor->realCurrent.f;
        hUSB.ifNewDataLogPiece2Send = 1;
        USB_DataLogManager(AK10_9_Set_DataLog_Label_Torque_Constant_Testing, dataSlots_AK10_9_TorqueConstantTesting);
        hStaticTorqueConstantTesting.curSamplingCount++;
      }
    }
    //Iq tracking is considered unstable
    else
    {
      if (HAL_GetTick() - hStaticTorqueConstantTesting.curNewSetTimeStamp > 500)
      {
        hStaticTorqueConstantTesting.ifCurStable = 1;
        hStaticTorqueConstantTesting.curSamplingCount = 0;
      }
    }
    AK10_9_ServoMode_CurrentControl(hmotor, hStaticTorqueConstantTesting.curSetIq);
  }
}

void AK10_9_StaticTorqueConstantTesting_Init(void)
{
  hStaticTorqueConstantTesting.ifTestingStarted = 0;
  hStaticTorqueConstantTesting.ifTestingFinished = 0;
  hStaticTorqueConstantTesting.curSamplingCount = 0;
  hStaticTorqueConstantTesting.curSetIq = 0.0f;
  hStaticTorqueConstantTesting.curNewSetTimeStamp = 0;
  hStaticTorqueConstantTesting.ifCurStable = 0;
}
