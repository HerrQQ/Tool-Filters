/*<RBHead>
 *************************************************************************
 *                                                                       *
 *                              U A E S                                  *
 *           A Joint Venture of Robert Bosch GmbH and CNEMS.             *
 *                                                                       *
 *                      (C) All rights reserved                          *
 *                                                                       *
 *************************************************************************

 *************************************************************************
 *
 * $Filename__:SASOffset$ 
 *
 * $Author____:Zhang Shengkui$
 *
 * $Function__:Implement the input/output for SAS offset calculation$
 *
 *************************************************************************
 * $Date______:2023-03-30$
 * $Variant___:0.1$
 * $Revision__:0$
 *************************************************************************
 * List Of Changes
 *
 * $History___:_
 *
 * 0.1      2023-03-30      Zhang Shengkui
 *          First edition
 * $
 *
 *************************************************************************
</RBHead>*/

/******************************************************************************
*  Include Headerfiles
******************************************************************************/
#pragma once 

#include "Platform_Types.h"



/*************************************************************************
*   Global Macros Definition
*************************************************************************/



/*************************************************************************
*   Global Typedef Definition
*************************************************************************/



/*************************************************************************
*   Global Variables Declaration
*************************************************************************/

/*!
 * @brief ESC_FD_1, 0xB0, Cycle 10ms, 32Byte
 */
extern float32 f32_ESC_WheelSpeed_FL;
extern uint8 u8_ESC_WheelSpeed_FL_Valid;
extern float32 f32_ESC_WheelSpeed_FR;
extern uint8 u8_ESC_WheelSpeed_FR_Valid;
extern float32 f32_ESC_WheelSpeed_RL;
extern uint8 u8_ESC_WheelSpeed_RL_Valid;
extern float32 f32_ESC_WheelSpeed_RR;
extern uint8 u8_ESC_WheelSpeed_RR_Valid;

/*!
 * @brief ESC_FD_2, 0x1A1, Cycle 20ms, 32Byte
 */
extern float32 f32_ESC_VehicleSpeed;
extern uint8 u8_ESC_VehicleSpeed_Valid;

/*!
 * @brief ACU_FD_2, 0x182, Cycle 20ms, 32Byte
 */
extern float32 f32_ACU_YawRate;
extern uint8 u8_ACU_YawRateSensor_St;
extern float32 f32_ACU_LateralAcc;  //ACU_LateralAcceleration
extern uint8 u8_ACU_LateralAcc_Valid;   //ACU_LateralAccelerationSensor_St
/*!
 * @brief EPS_FD_1, 0x1A5, Cycle 20ms, 32Byte
 */
extern float32 f32_EPS_SteerWheelAngle;
extern uint8  u8_EPS_SteerWheelAngle_Valid;
extern float32 f32_EPS_SteerWheelTorque;
extern uint8   u8_EPS_SteerWheelTorque_Valid;

//new EPS input 0613
extern float32 f32_EPS_ActualRackPosition;
extern uint8 u8_EPS_ActualRackPosition_Valid;

// ADCM input 0613
extern float32 f32_ADCM_EPSRackPositionReq;
extern uint8 u8_ADCM_EPSRackPositionReq_St;


/*!
 * @brief BDCM_FD_RT_8, 0x136, Cycle 10ms, 32Byte
 */
extern float32 f32_CSM_SAS_SteeringWheelSpeed;
extern float32 f32_CSM_SAS_SteeringWheelAngle;
extern uint8   u8_CSM_SAS_SteeringWheelAngle_Valid;

/*!
 * @brief Output and to save
 */
extern float32 f32_SwaOffset;  // sas intern 
extern uint16 u16_SwaDeltaN;  // ֵsas counter
extern float32  f32_SwaDelta_S0; // sas output
// only output
extern float32 f32_RackDelta_S0; //  rack output
extern boolean b_SwaOffsetValid; // SAS offset validation flag  
extern boolean b_Rack_OffsetValid; // Rack offset validation flag





/*************************************************************************
*  Global Function Declaration
*************************************************************************/

/*!
 * @brief Converts bus raw values to physical values
 * @param 
 * @return 
 * @retval 
 * @context        10ms task
 * @reentrant      FALSE
 * @synchronous    TRUE
 */
extern void SASOffset_PreProcess(void);

/*!
 * @brief Converts calculated  physical values to raw values
 * @param 
 * @return 
 * @retval 
 * @context        10ms task
 * @reentrant      FALSE
 * @synchronous    TRUE
 */
extern void SASOffset_PostProcess(void);
/*!
 * @brief return KL15 status
 * @param
 * @return
 * @retval
 * @context        
 * @reentrant      
 * @synchronous    
 */
extern boolean SASOffset_IsKL15ON(void);



typedef struct Qkfdebug_tag
{
    boolean valid1;
    boolean valid2;
    float32 data1;
    float32 data2;
    float32 data3;
    float32 data4;
    float32 data5;
    float32 data6;
    float32 data7;
    float32 data8;
}Qkfdebug;

