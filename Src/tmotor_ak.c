/** @file   QDD.c
 *  @brief  Functions for controlling QDD
 *  @author Rehabilitation Research Institute of Singapore / MRTA Team
 */
#include "QDD.h"


#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))


CAN_HandleTypeDef hcan1;
static volatile uint8_t flagCANIdle = 0;


// Waist translation, Waist rotation and Elbow rotation
QDD_MOTOR qddWaistT = {.id = ID_WAIST_T, .currentMode = MODE_RESET, .control = {0}, .feedback = {0}};
QDD_MOTOR qddWaistR = {.id = ID_WAIST_R, .currentMode = MODE_RESET, .control = {0}, .feedback = {0}};
QDD_MOTOR qddElbowR = {.id = ID_ELBOW_R, .currentMode = MODE_RESET, .control = {0}, .feedback = {0}};


static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
	// Converts a float to an unsigned int, given range and number of bits
	float span = x_max - x_min;
	float offset = x_min;
	
	return (uint16_t) ((x-offset)*((float)((1<<bits)-1))/span);
}


static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	// converts unsigned int to float, given range and number of bits
	float span = x_max - x_min;
	float offset = x_min;
	
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}


static void ctrlParameter_to_package(QDD_MOTOR *qdd)
{
	uint16_t p, v, kp, kd, t;
	
	// Set limitation
	LIMIT_MIN_MAX(qdd->control.position, P_MIN, P_MAX);
	LIMIT_MIN_MAX(qdd->control.velocity, V_MIN, V_MAX);
	LIMIT_MIN_MAX(qdd->control.kp, KP_MIN, KP_MAX);
	LIMIT_MIN_MAX(qdd->control.kd, KD_MIN, KD_MAX);
	LIMIT_MIN_MAX(qdd->control.torque, T_MIN, T_MAX);
	
	// Transfer the type of float to unsigned int
	p = float_to_uint(qdd->control.position, P_MIN, P_MAX, 16);            
	v = float_to_uint(qdd->control.velocity, V_MIN, V_MAX, 12);
	kp = float_to_uint(qdd->control.kp, KP_MIN, KP_MAX, 12);
	kd = float_to_uint(qdd->control.kd, KD_MIN, KD_MAX, 12);
	t = float_to_uint(qdd->control.torque, T_MIN, T_MAX, 12);
	
	// Shift the data to the buffer of CAN
	qdd->txCtrlBuf[0] = p>>8;
	qdd->txCtrlBuf[1] = p&0xFF;
	qdd->txCtrlBuf[2] = v>>4;
	qdd->txCtrlBuf[3] = ((v&0xF)<<4)|(kp>>8);
	qdd->txCtrlBuf[4] = kp&0xFF;
	qdd->txCtrlBuf[5] = kd>>4;
	qdd->txCtrlBuf[6] = ((kd&0xF)<<4)|(t>>8);
	qdd->txCtrlBuf[7] = t&0xff;
}


HAL_StatusTypeDef QDD_Init(void)
{
	__HAL_RCC_CAN1_CLK_ENABLE();
	
	/**CAN1 GPIO Configuration    
	PD0     ------> CAN1_RX
	PD1     ------> CAN1_TX 
	*/
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/* CAN1 interrupt Init */
	HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
	
	// The baudrate of CAN BUS is PCLK1 ( 42MHz ) / Prescaler / ( TimeSeg1 + TimeSeg2 + 1 ) = 1Mbit/s
	
	hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if(HAL_CAN_Init(&hcan1) != HAL_OK)	return HAL_ERROR;
	
	// Config the CAN input filter and enable the reception function
	CAN_FilterTypeDef canFilter;
	canFilter.FilterIdHigh = 0;
	canFilter.FilterIdLow = 0;
	canFilter.FilterMaskIdHigh = 0;
	canFilter.FilterMaskIdLow = 0;
	canFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	canFilter.FilterBank = 0;
	canFilter.FilterMode = CAN_FILTERMODE_IDMASK;
	canFilter.FilterScale = CAN_FILTERSCALE_32BIT;
	canFilter.FilterActivation = CAN_FILTER_ENABLE;
	canFilter.SlaveStartFilterBank = 0;
	if(HAL_CAN_ConfigFilter(&hcan1, &canFilter) != HAL_OK )	return HAL_ERROR;
	
	if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK )	return HAL_ERROR;
	
	if(HAL_CAN_Start(&hcan1) != HAL_OK )	return HAL_ERROR;
	
	// Fill the txCtrlBuf with parameter 0
	ctrlParameter_to_package(&qddWaistT);
	ctrlParameter_to_package(&qddWaistR);
	ctrlParameter_to_package(&qddElbowR);
	
	flagCANIdle = 1;
	
	return HAL_OK;
}


HAL_StatusTypeDef QDD_Control(QDD_MOTOR *qdd)
{
	if(qdd->currentMode != MODE_MOTOR)
		QDD_SetMotorMode(qdd);
	
	ctrlParameter_to_package(qdd);
	
	return CAN_TX(qdd->id, qdd->txCtrlBuf, 8);
}


HAL_StatusTypeDef QDD_Read(QDD_MOTOR *qdd)
{
	return CAN_TX(qdd->id, qdd->txCtrlBuf, 8);
}


HAL_StatusTypeDef QDD_Stop(QDD_MOTOR *qdd)
{
	return QDD_SetResetMode(qdd);
}


HAL_StatusTypeDef QDD_SetMotorMode(QDD_MOTOR *qdd)
{
	if(qdd->currentMode == MODE_MOTOR)
		return HAL_OK;
	
	uint8_t txBuf[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, MODE_MOTOR};
	if(CAN_TX(qdd->id, txBuf, 8) != HAL_OK )	return HAL_ERROR;
	qdd->currentMode = MODE_MOTOR;
	return HAL_OK;
}


HAL_StatusTypeDef QDD_SetResetMode(QDD_MOTOR *qdd)
{
	if(qdd->currentMode == MODE_RESET)
		return HAL_OK;
	
	// Clear the current parameter
	qdd->control.kp = 0;
	qdd->control.kd = 0;
	qdd->control.position = 0;
	qdd->control.velocity = 0;
	qdd->control.torque = 0;
	
	if(QDD_Control(qdd) != HAL_OK) return HAL_ERROR;
	
	// Send the motor reset command
	uint8_t txBuf[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, MODE_RESET};
	if(CAN_TX(qdd->id, txBuf, 8) != HAL_OK )	return HAL_ERROR;
	
	qdd->currentMode = MODE_RESET;
	return HAL_OK;
}


HAL_StatusTypeDef QDD_SetZeroPosition(QDD_MOTOR *qdd)
{	
	uint8_t txBuf[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, SET_ZERO};	
	return CAN_TX(qdd->id, txBuf, 8);
}


HAL_StatusTypeDef CAN_TX(uint8_t ID, uint8_t *buf, uint8_t dataSize)
{
	static CAN_TxHeaderTypeDef canTxHeader = {0, 0, CAN_ID_STD, CAN_RTR_DATA, 0, DISABLE};
	static uint32_t txMailBox;
	
	canTxHeader.StdId = ID;
	canTxHeader.DLC = dataSize;
	
	uint32_t tickstart = HAL_GetTick();
	
//	while( flagCANIdle != 1 ){
//		if ((HAL_GetTick() - tickstart) > CAN_TRANSIMIT_TIMEOUT){
//			return HAL_ERROR;
//		}
//	}
	
	flagCANIdle = 0;
	return HAL_CAN_AddTxMessage(&hcan1, &canTxHeader, buf, &txMailBox);
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *canHandler)
{
	CAN_RxHeaderTypeDef canRxHeader;
	uint8_t rxData[8];
	float p, v, c;	
	
	if(HAL_CAN_GetRxMessage(canHandler, CAN_RX_FIFO0, &canRxHeader, rxData) == HAL_OK){
		p = uint_to_float(rxData[1] << 8 | rxData[2], P_MIN, P_MAX, 16);
		v = uint_to_float(rxData[3] << 4 | rxData[4] >> 4, V_MIN, V_MAX, 12);
		c = uint_to_float((rxData[4]&0x0F) << 8 | rxData[5], T_MIN, T_MAX, 12);
		
		switch (rxData[0]){
			case ID_WAIST_T:
				qddWaistT.feedback.position = p;
				qddWaistT.feedback.velocity = v;
				qddWaistT.feedback.current = c;
				break;
			case ID_WAIST_R:
				qddWaistR.feedback.position = p;
				qddWaistR.feedback.velocity = v;
				qddWaistR.feedback.current = c;
				break;
			case ID_ELBOW_R:
				qddElbowR.feedback.position = p;
				qddElbowR.feedback.velocity = v;
				qddElbowR.feedback.current = c;
				break;
			default:
				break;
		}
	}
	
	flagCANIdle = 1;
}
