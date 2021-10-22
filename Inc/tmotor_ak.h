/** @file   QDD.h
 *  @brief  Header file for QDD (Quasi Direct Drive)
 *  @author Rehabilitation Research Institute of Singapore / MRTA Team
 */

#ifndef __QDD_H
#define __QDD_H

#include "stm32f4xx_hal.h"


#define CAN_TRANSIMIT_TIMEOUT		3			// in ms


// Define the ID of each motor
#define ID_WAIST_T			1
#define ID_WAIST_R			7
#define ID_ELBOW_R			5


// The command code of motor
#define MODE_MOTOR			0xFC			// The command code for setting motor into speed mode
#define MODE_RESET			0xFD			// The command code for setting motor into position mode
#define SET_ZERO				0xFE			// The command code for setting current position as 0


// Limitation of the motor
// TMotor AK80-9
#define P_MIN 					-95.5f    // Radians
#define P_MAX 					95.5f        
#define V_MIN 					-30.0f    // Rad/s
#define V_MAX 					30.0f
#define KP_MIN 					0.0f     	// N-m/rad
#define KP_MAX 					500.0f
#define KD_MIN 					0.0f     	// N-m/rad/s
#define KD_MAX 					5.0f
#define T_MIN 					-18.0f		// N-m
#define T_MAX 					18.0f


/// CAN Command Packet Structure ///
/// 12 bit kp, between 0 and 500 N-m/rad
/// 12 bit kd, between 0 and 100 N-m*s/rad
/// 16 bit position command, between -4*pi and 4*pi
/// 12 bit velocity command, between -30 and + 30 rad/s
/// 12 bit feed forward torque, between -18 and 18 N-m
typedef struct{
	float kp;
	float kd;
	float position;
	float velocity;
	float torque;
}QDD_CONTROL_PARAMETER;


/// CAN Reply Packet Structure ///
/// 16 bit position, between -4*pi and 4*pi
/// 12 bit velocity, between -30 and + 30 rad/s
/// 12 bit current, between -40 and 40;
typedef struct{
	float position;
	float velocity;
	float current;
}QDD_REPLY_PARAMETER;


/// Structure for QDD motor
typedef struct{
	uint8_t id;
	uint8_t currentMode;
	uint8_t txCtrlBuf[8];					// Store the current parameters of controlling.
																// Send this buffer to motor for reading parameters of motor.
	QDD_CONTROL_PARAMETER control;
	QDD_REPLY_PARAMETER feedback;
}QDD_MOTOR;


extern QDD_MOTOR qddWaistT, qddWaistR, qddElbowR;


/** @brief QDD Initialization Function. Initialization for the CAN1.
 *  			 CAN1 is responsible for the control of the QDD.
 *  @return HAL_OK or HAL_ERROR.
 */
HAL_StatusTypeDef QDD_Init(void);


/**	@brief Send the parameters of controlling to the motor.
 *  @return HAL_OK or HAL_ERROR.
 */
HAL_StatusTypeDef QDD_Control(QDD_MOTOR *qdd);


/**	@brief Read the parameters from motor.
 *  @return HAL_OK or HAL_ERROR.
 */
HAL_StatusTypeDef QDD_Read(QDD_MOTOR *qdd);


/**	@brief Stop the motor and set the motor into reset mode.
 *  @return HAL_OK or HAL_ERROR.
 */
HAL_StatusTypeDef QDD_Stop(QDD_MOTOR *qdd);


/**	@brief Set the motor into motor mode (for controlling).
 *  @return HAL_OK or HAL_ERROR.
 */
HAL_StatusTypeDef QDD_SetMotorMode(QDD_MOTOR *qdd);


/**	@brief Set the motor into reset mode.
 *  @return HAL_OK or HAL_ERROR.
 */
HAL_StatusTypeDef QDD_SetResetMode(QDD_MOTOR *qdd);


/**	@brief Set the current position of motor to 0.
 *  @return HAL_OK or HAL_ERROR.
 */
HAL_StatusTypeDef QDD_SetZeroPosition(QDD_MOTOR *qdd);


/**	@brief Transmit data via CAN BUS.
 *  @return HAL_OK or HAL_ERROR.
 */
HAL_StatusTypeDef CAN_TX(uint8_t ID, uint8_t *buf, uint8_t dataSize);


#endif
