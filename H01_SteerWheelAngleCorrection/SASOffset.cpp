
#include "SASOffset.h"

float32 f32_ESC_WheelSpeed_FL;
uint8 u8_ESC_WheelSpeed_FL_Valid;
float32 f32_ESC_WheelSpeed_FR;
uint8 u8_ESC_WheelSpeed_FR_Valid;
float32 f32_ESC_WheelSpeed_RL;
uint8 u8_ESC_WheelSpeed_RL_Valid;
float32 f32_ESC_WheelSpeed_RR;
uint8 u8_ESC_WheelSpeed_RR_Valid;

/*!
 * @brief ESC_FD_2, 0x1A1, Cycle 20ms, 32Byte
 */
float32 f32_ESC_VehicleSpeed;
uint8 u8_ESC_VehicleSpeed_Valid;

/*!
 * @brief ACU_FD_2, 0x182, Cycle 20ms, 32Byte
 */
float32 f32_ACU_YawRate;
uint8 u8_ACU_YawRateSensor_St;
float32 f32_ACU_LateralAcc;  //ACU_LateralAcceleration
uint8 u8_ACU_LateralAcc_Valid;   //ACU_LateralAccelerationSensor_St
/*!
 * @brief EPS_FD_1, 0x1A5, Cycle 20ms, 32Byte
 */
float32 f32_EPS_SteerWheelAngle;
uint8  u8_EPS_SteerWheelAngle_Valid;

float32 f32_EPS_SteerWheelTorque;
uint8   u8_EPS_SteerWheelTorque_Valid;

//new 0613
float32 f32_EPS_ActualRackPosition;
uint8 u8_EPS_ActualRackPosition_Valid;
float32 f32_ADCM_EPSRackPositionReq;
uint8 u8_ADCM_EPSRackPositionReq_St;

/*!
 * @brief BDCM_FD_RT_8, 0x136, Cycle 10ms, 32Byte
 */
float32 f32_CSM_SAS_SteeringWheelSpeed;
float32 f32_CSM_SAS_SteeringWheelAngle;
uint8   u8_CSM_SAS_SteeringWheelAngle_Valid;


/*!
 * @brief Output and to save 
 */

float32 f32_SwaOffset;
uint16 u16_SwaDeltaN;
float32  f32_SwaDelta_S0;



float32 f32_RackDelta_S0;
boolean b_SwaOffsetValid;
boolean b_Rack_OffsetValid;



