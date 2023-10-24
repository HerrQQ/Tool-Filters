/* ===========================================================================
 *
 *                   Human Horizons Operation Center Shanghai
 *
 *   Human Horizons Operation Center Shanghai (HRYT) owns all rights to this work and
 *   intends to maintain it in confidence to preserve its trade secret status.
 *   HRYT reserves its rights under all copyright laws to protect this work
 *   as a published work, when appropriate.Those having access to this work may
 *   not copy it, use it, modify it, or disclose the information contained in
 *   it without the written authorization of HRYT.
 *
 * ========================================================================= */

/* ===========================================================================
 *
 *  Include Files
 *
 * ========================================================================= */
#include "Swa_OffsetComp.h"
#include <iostream>

/* ========================================================================== =
 *
 *Private Typedefs
 *
 *======================================================================== = */
// SteerWheelAngle ---SWA

/* ===========================================================================
 *
 *  Private Function Prototypes
 *
 * ========================================================================= */
static void fInputFilter(const float32* const value, float32* const lastValue, float32 minBoundary\
	, float32  maxBoundary);
static void fSASRackOutput(void);
static void fSASRackInputFilter(void);
static void fSASRackCalGradientValue(const float32* const value, const float32 * const lastValue, float32 * const gradvalue);
static float32 fSASRackCalDeltaWheelSpeed(const float32* const fl, const float32* const fr, const float32* const rl, const float32* const rr);
static void fSASRackOffsetCompEn(uint16 *enableCntSAS, uint16 *enableCntRack);
static void fSASOffsetCompSaturation(float32 *deltaS0);
void fSASRackOffsetProcess(void);
static void fRackPosOffsetLimitation(float32 *rackPosLimitCal);
static void fSASOffsetCal(float32 filterPercent, uint8 *swaOffsetCompStatus);
static void fRackOffsetCal(float32 filterPercentRack, uint8 *swaOffsetCompStatusRack);
static void fRackOffsetCalCalibrated(float32 filterPercentRack, uint8 *swaOffsetCompStatusRack);

/* ===========================================================================
 *
 *  Public Data Storage
 *
 * ========================================================================= */

/*========================================================================== =
 *
 *Private Data Storage
 *
 *======================================================================== = */
Qkfdebug qkfdebug;
// SAS Data
  uint16 u16_SASRackEnableCnt = 0u;

 // Rack Data
  uint16 u16_RackDeltaN = 0u;
  float32 f32_RackPosOffset = 0.f;
  uint16 u16_RackEnableCnt = 0u;
  boolean b_SwaOffsetValid_Rack = FALSE;
 // Private Data
  float32 f32_ESC_VehicleSpeedLast = 0.f;

  float32 f32_ACU_YawRateLast = 0.f;
  float32 f32_ESC_WheelSpeed_FLLast = 0.f;
  float32 f32_ESC_WheelSpeed_FRLast = 0.f;
  float32 f32_ESC_WheelSpeed_RLLast = 0.f;
  float32 f32_ESC_WheelSpeed_RRLast = 0.f;
  float32 f32_ACU_LateralAccLast = 0.f;
  float32 f32_EPS_SteerWheelAngleLast = 0.f;
  float32 f32_EPS_SteerWheelTorqueLast = 0.f;
  float32 f32_CSM_SAS_SteeringWheelSpeedLast = 0.f;
  float32 f32_CSM_SAS_SteeringWheelAngleLast = 0.f;
  float32 f32_EPS_ActualRackPositionLast = 0.f;
  boolean b_InitFlag = FALSE;
  boolean flag_refresh_rack1 = FALSE;
  boolean flag_refresh_rack2 = FALSE;
  boolean b_NotNewFireCycle = FALSE;
/* ===========================================================================
 *
 *  Private Define
 *
 * ========================================================================= */


#define SASRackInputFilter_lib(value, lastValue, minBoundary, maxBoundary)             \
	{                                                                                  \
		if (((minBoundary) <= (value)) && ((value) <= (maxBoundary)))                          \
		{                                                                              \
			(lastValue) = ((value) * SWA_FilterFactor_08) + ((lastValue) * SWA_FilterFactor_02); \
		}                                                                              \
	}

/* ===========================================================================
 *
 *  Function Define
 *
 * ========================================================================= */

 /// @brief Filtering input values
 /// @param
static void fInputFilter(const float32* const value, float32* const lastValue, float32 minBoundary,float32 maxBoundary) /* parasoft-suppress MISRA2008-7_1_1 "设计如此" */
{
	if ((minBoundary <= *value) && (*value <= maxBoundary))
	{
		*lastValue = (*value * (SWA_FilterFactor_08)) + ((*lastValue) * SWA_FilterFactor_02);
	}
}


/// @brief processing input values
/// @param
static void fSASRackInputFilter(void)
{
	fInputFilter(&f32_ESC_VehicleSpeed, &f32_ESC_VehicleSpeedLast,
						    SWA_VehicleSpeed_MinBoundary,  SWA_VehicleSpeed_MaxBoundary);
	fInputFilter(&f32_ACU_YawRate, &f32_ACU_YawRateLast,
						    SWA_YawRate_MinBoundary,  SWA_YawRate_MaxBoundary);
	fInputFilter(&f32_ESC_WheelSpeed_FL, &f32_ESC_WheelSpeed_FLLast,
						    SWA_VehicleSpeed_MinBoundary,  SWA_VehicleSpeed_MaxBoundary);
	fInputFilter(&f32_ESC_WheelSpeed_FR, &f32_ESC_WheelSpeed_FRLast,
						    SWA_VehicleSpeed_MinBoundary,  SWA_VehicleSpeed_MaxBoundary);
	fInputFilter(&f32_ESC_WheelSpeed_RL, &f32_ESC_WheelSpeed_RLLast,
						    SWA_VehicleSpeed_MinBoundary,  SWA_VehicleSpeed_MaxBoundary);
	fInputFilter(&f32_ESC_WheelSpeed_RR, &f32_ESC_WheelSpeed_RRLast,
						    SWA_VehicleSpeed_MinBoundary,  SWA_VehicleSpeed_MaxBoundary);
	fInputFilter(&f32_ACU_LateralAcc, &f32_ACU_LateralAccLast,
						    SWA_LateralAcceleration_MinBoundary,
						    SWA_LateralAcceleration_MaxBoundary);
	fInputFilter(&f32_EPS_SteerWheelAngle, &f32_EPS_SteerWheelAngleLast,
						    SWA_SteeringWheelAngle_MinBoundary,
						    SWA_SteeringWheelAngle_MaxBoundary);
	fInputFilter(&f32_EPS_SteerWheelTorque, &f32_EPS_SteerWheelTorqueLast,
						    SWA_SteerWheelTorque_MinBoundary,  SWA_SteerWheelTorque_MaxBoundary);
	fInputFilter(&f32_CSM_SAS_SteeringWheelSpeed,
						   &f32_CSM_SAS_SteeringWheelSpeedLast,
						    SWA_SASSteeringWheelSpeed_MinBoundary,
						    SWA_SASSteeringWheelSpeed_MaxBoundary);
	fInputFilter(&f32_CSM_SAS_SteeringWheelAngle,
						   &f32_CSM_SAS_SteeringWheelAngleLast,
						    SWA_SteeringWheelAngle_MinBoundary,
						    SWA_SteeringWheelAngle_MaxBoundary);
	fInputFilter(&f32_EPS_ActualRackPosition,
						   &f32_EPS_ActualRackPositionLast,
						    SWA_ActualRackPosition_MinBoundary,
						    SWA_ActualRackPosition_MaxBoundary);
}
/// @brief Gradient calculation
/// @param value
/// @param lastValue
/// @param gradvalue
/// @param tick
static void fSASRackCalGradientValue(const float32* const value, const float32* const lastValue, float32 * const gradvalue)
{
	float32 temp = 0.f;
	temp = (*value - *lastValue) / (SWA_CycleTime_s);
	*gradvalue =(*gradvalue * (SWA_FilterFactor_08)) + (temp * (SWA_FilterFactor_02));
}
/// @brief
/// @param fl
/// @param fr
/// @param rl
/// @param rr
/// @return DeltaWheelSpeed
static float32 fSASRackCalDeltaWheelSpeed(const float32* const fl, const float32* const fr, const float32* const rl, const float32* const rr)
{
	float32 ans_t = 0.f;
	float32 delta1_t=0.f;
	float32 delta2_t=0.f;
	delta1_t=fabsf(&fl - &fr);
	delta2_t=fabsf(&rl - &rr);
	ans_t = max_hryt(delta1_t,delta2_t);
	return ans_t;
}
/// @brief
/// @param enableCnt enableCntRack
static void fSASRackOffsetCompEn(uint16 * const enableCntSAS, uint16 * const enableCntRack)
{
	static float32 f32_Deltawheelspeed = 0.f;
	static float32 f32_SwaYawRateGradient = 0.f;
	fSASRackCalGradientValue(&f32_ACU_YawRate,
							 &f32_ACU_YawRateLast,
							 &f32_SwaYawRateGradient);
	f32_Deltawheelspeed = fSASRackCalDeltaWheelSpeed(&f32_ESC_WheelSpeed_FLLast,
													 &f32_ESC_WheelSpeed_FRLast,
													 &f32_ESC_WheelSpeed_RLLast,
													 &f32_ESC_WheelSpeed_RRLast);
	// 1 cal sas enable cnt
	if ((f32_ESC_VehicleSpeedLast > P_SWA_VehicleSpeed_kmph) &&						  // 30
		(fabsf_hryt(f32_ACU_YawRateLast) < P_SWA_YawRate_degs) &&						  // 0.3
		(fabsf_hryt(f32_EPS_SteerWheelTorqueLast) < P_SWA_SteerWheelTorque_nm) &&		  // 0.3
		(fabsf_hryt(f32_EPS_SteerWheelAngleLast) < P_SWA_SteeringWheelAngle_deg) &&		  // 5
		(f32_Deltawheelspeed < P_SWA_DeltaWheelSpeed_kmph) &&						  // 1.8
		(fabsf_hryt(f32_ACU_LateralAccLast) < P_SWA_LateralAcc_mps2) &&					  // 0.03
		(fabsf_hryt(f32_SwaYawRateGradient) < P_SWA_YawRateGradient_degps2) &&			  // 1
		(fabsf_hryt(f32_CSM_SAS_SteeringWheelSpeedLast) < P_SWA_SteeringWheelSpeed_degps)) // 80
	{
		

		if ((u8_ESC_VehicleSpeed_Valid == SWA_VALID) &&
			(u8_ESC_WheelSpeed_FR_Valid == SWA_VALID) &&
			(u8_ESC_WheelSpeed_RL_Valid == SWA_VALID) &&
			(u8_ESC_WheelSpeed_RR_Valid == SWA_VALID) &&
			(u8_ESC_WheelSpeed_FL_Valid == SWA_VALID) &&
			(u8_ACU_YawRateSensor_St == SWA_VALID) &&
			(u8_EPS_SteerWheelAngle_Valid == SWA_VALID) &&
			(u8_EPS_SteerWheelTorque_Valid == SWA_VALID) &&
			(u8_ACU_LateralAcc_Valid == SWA_VALID) &&
			(u8_CSM_SAS_SteeringWheelAngle_Valid == SWA_VALID))
		{
			
			if (*enableCntSAS < SWA_MaxUINT16)
			{
				(*enableCntSAS)++;
			}
		}
		else
		{
			*enableCntSAS = (uint16)0;
		}
	}
	else
	{
		*enableCntSAS = (uint16)0;
	}
	// 2 cal rack enable cnt with different preconditions: vehicalSpeed tbd;
	if ((TRUE == b_InitFlag) &&
		(f32_ESC_VehicleSpeedLast > P_SWA_VehicleSpeed_kmph_Rack) &&					   // 10 kph
		(fabsf_hryt(f32_ACU_YawRateLast) < P_SWA_YawRate_degs_Rack) &&						   // 0.3 dps
		(fabsf_hryt(f32_EPS_SteerWheelTorqueLast) < P_SWA_SteerWheelTorque_nm_Rack) &&		   // 0.3nm
		(fabsf_hryt(f32_EPS_SteerWheelAngleLast) < P_SWA_SteeringWheelAngle_deg_Rack) &&		   // 5 degree
		(fabsf_hryt(f32_CSM_SAS_SteeringWheelSpeedLast) < P_SWA_SteeringWheelSpeed_degps_Rack)) // 3dps
	{
		if ((u8_EPS_ActualRackPosition_Valid == SWA_VALID) &&
			(u8_ESC_VehicleSpeed_Valid == SWA_VALID) &&
			(u8_ACU_YawRateSensor_St == SWA_VALID) &&
			(u8_EPS_SteerWheelTorque_Valid == SWA_VALID) &&
			(u8_CSM_SAS_SteeringWheelAngle_Valid == SWA_VALID))
		{
			if (*enableCntRack < SWA_MaxUINT16)
			{
				(*enableCntRack)++;
			}
		}
		else
		{
			*enableCntRack = (uint16)0;
		}
	}
	else
	{
		*enableCntRack = (uint16)0;
	}
}
/// @brief SAS output limitation
/// @param deltaS0
static void fSASOffsetCompSaturation(float32 *const deltaS0)
{
	if (fabsf_hryt(*deltaS0) > P_SWA_MaxDeltaS0_deg)
	{
		if (*deltaS0 >= P_SWA_MaxDeltaS0_deg)
		{
			*deltaS0 = P_SWA_MaxDeltaS0_deg;
		}
		else
		{
			*deltaS0 = P_SWA_MaxDeltaS0_deg * (-1);
		}
	}
}


/// @brief output function
/// @param
static void fSASRackOutput(void)
{
	if (FALSE == b_Rack_OffsetValid)
	{
		b_Rack_OffsetValid = b_SwaOffsetValid_Rack &&
			(!((SWA_ADCMRequestStatue_active == u8_ADCM_EPSRackPositionReq_St) ||
				(SWA_ADCMRequestStatue_overide == u8_ADCM_EPSRackPositionReq_St)));
	}
	else
	{
		// do nothing, keeping valid  
	}
	
	// Rack compensation validation flag
}
/// @brief Rack offset value limitation
/// @param  rackPosLimitCal
static void fRackPosOffsetLimitation(float32 * const rackPosLimitCal)
{

	float32 rackPosAbs = fabsf_hryt(*rackPosLimitCal);

	if (rackPosAbs >= 3 * P_SWA_LengthProRack)
	{
		rackPosAbs = 3 * P_SWA_LengthProRack;
	}
	else if (rackPosAbs >= 2 * P_SWA_LengthProRack)
	{
		rackPosAbs = 2 * P_SWA_LengthProRack;
	}
	else if (rackPosAbs >= P_SWA_LengthProRack)
	{
		rackPosAbs = P_SWA_LengthProRack;
	}
	else
	{
		rackPosAbs = 0.f;
	}
	if (*rackPosLimitCal >= 0.f)
	{
		*rackPosLimitCal = rackPosAbs;
	}
	else
	{
		*rackPosLimitCal = -rackPosAbs;
	}
	*rackPosLimitCal = -(*rackPosLimitCal);// 0724 reverse the sign before output 
}



/// @brief SAS offset value calculation
/// @param  filterPercent
/// @param swaOffsetCompStatus
static void fSASOffsetCal(float32  filterPercent, uint8 * const swaOffsetCompStatus)
{
	static boolean flag_refresh = FALSE;
	if (u16_SASRackEnableCnt < (uint16)SWA_ENABLECNT_STEP1)
	{
		*swaOffsetCompStatus = (uint8)SWA_OffsetCompStatus_Disable;
	}
	switch (*swaOffsetCompStatus)
	{
	case SWA_OffsetCompStatus_Disable:
		if (u16_SASRackEnableCnt > (uint16)SWA_ENABLECNT_STEP1)
		{
			*swaOffsetCompStatus = (uint8)SWA_OffsetCompStatus_Enable;
		}
		break;
	case SWA_OffsetCompStatus_Enable:

		f32_SwaOffset = f32_SwaOffset * (1 - filterPercent) + f32_CSM_SAS_SteeringWheelAngleLast * filterPercent;
		u16_SwaDeltaN++;
		if (u16_SwaDeltaN >= (uint16)P_SWA_DetaN_enum)
		{
			u16_SwaDeltaN = (uint16)P_SWA_DetaN_enum;
			*swaOffsetCompStatus = (uint8)SWA_OffsetCompStatus_Output;
		}

		break;
	case SWA_OffsetCompStatus_Output:
		f32_SwaOffset = f32_SwaOffset * (1 - filterPercent) + f32_CSM_SAS_SteeringWheelAngleLast * filterPercent;
		b_SwaOffsetValid = TRUE;
		break;
	default:
		break;
	}
	if ((b_SwaOffsetValid == TRUE) && fabsf_hryt(f32_SwaOffset - f32_SwaDelta_S0) > P_SWA_MinDeltaS0_deg)
	{
		 	flag_refresh=TRUE;
	}
		 if (fabsf_hryt(f32_SwaOffset - f32_SwaDelta_S0) <P_SWA_MinDeltaS0_close_deg )
		 {
		 	flag_refresh=FALSE;
		 }
		 if (TRUE==flag_refresh)
		 {
		f32_SwaDelta_S0 = f32_SwaDelta_S0 * SWA_FilterFactor_99 + f32_SwaOffset * SWA_FilterFactor_01;
	}
}

/// @brief Rack offset value calculation , while sas-offset was not calculated
/// @param  filterPercentRack
/// @param swaOffsetCompStatusRack
static void fRackOffsetCalCalibrated(float32  filterPercentRack, uint8 * const swaOffsetCompStatusRack)
{
	float32 SASRackPos_mm = 0.f;
	if (u16_RackEnableCnt < (uint16)SWA_ENABLECNT_STEP1)
	{
		*swaOffsetCompStatusRack = (uint8)SWA_OffsetCompStatus_Disable;
	}
	
	SASRackPos_mm = (f32_CSM_SAS_SteeringWheelAngleLast - f32_SwaDelta_S0) / (float32)P_SWA_SteeringWheelAngle2MiniMeter;
	switch (*swaOffsetCompStatusRack)
	{
	case SWA_OffsetCompStatus_Disable:
		if (u16_RackEnableCnt > (uint16)SWA_ENABLECNT_STEP1)
		{
			*swaOffsetCompStatusRack = (uint8)SWA_OffsetCompStatus_Enable;
		}
		break;
	case SWA_OffsetCompStatus_Enable:

		f32_RackPosOffset = f32_RackPosOffset * (1 - filterPercentRack) + (f32_EPS_ActualRackPositionLast - SASRackPos_mm) * filterPercentRack;
		u16_RackDeltaN++;
		if (u16_RackDeltaN >= (uint16)P_SWA_Rack_DetaN_enum)
		{
			u16_RackDeltaN = (uint16)P_SWA_Rack_DetaN_enum;
			*swaOffsetCompStatusRack = (uint8)SWA_OffsetCompStatus_Output;
		}

		break;
	case SWA_OffsetCompStatus_Output:
		f32_RackPosOffset = f32_RackPosOffset * (1 - filterPercentRack) + (f32_EPS_ActualRackPositionLast - SASRackPos_mm) * filterPercentRack;
		b_SwaOffsetValid_Rack = TRUE;
		break;
	default:
		break;
	}
	if ((b_SwaOffsetValid_Rack == TRUE) && fabsf_hryt(f32_RackPosOffset - f32_RackDelta_S0) > P_SWA_MinDeltaS0_degRack)
	{
			flag_refresh_rack1=TRUE;
	}
	if (fabsf_hryt(f32_RackPosOffset - f32_RackDelta_S0) < P_SWA_MinDeltaS0_close_degRack)
	{
		flag_refresh_rack1=FALSE;
	}
		if (TRUE==flag_refresh_rack1)
		{
		f32_RackDelta_S0 = f32_RackDelta_S0 * SWA_FilterFactor_20 + f32_RackPosOffset * SWA_FilterFactor_80;
	}
}
/// @brief Rack offset value calculation
/// @param  filterPercentRack
/// @param swaOffsetCompStatusRack

static void fRackOffsetCal(float32 filterPercentRack, uint8 *swaOffsetCompStatusRack)
{
	if (u16_SASRackEnableCnt < (uint16)SWA_ENABLECNT_STEP1)
	{
		*swaOffsetCompStatusRack = (uint8)SWA_OffsetCompStatus_Disable;
	}
	switch (*swaOffsetCompStatusRack)
	{
	case SWA_OffsetCompStatus_Disable:
		if (u16_SASRackEnableCnt > (uint16) SWA_ENABLECNT_STEP1)
		{
			*swaOffsetCompStatusRack = (uint8)SWA_OffsetCompStatus_Enable;
		}
		break;
	case SWA_OffsetCompStatus_Enable:

		f32_RackPosOffset = f32_RackPosOffset * (1 - filterPercentRack) + f32_EPS_ActualRackPositionLast * filterPercentRack;
		u16_RackDeltaN++;
		if (u16_RackDeltaN >= (uint16)P_SWA_Rack_DetaN_enum)
		{
			u16_RackDeltaN = (uint16)P_SWA_Rack_DetaN_enum;
			*swaOffsetCompStatusRack = (uint8)SWA_OffsetCompStatus_Output;
		}

		break;
	case SWA_OffsetCompStatus_Output:
		f32_RackPosOffset = f32_RackPosOffset * (1 - filterPercentRack) + f32_EPS_ActualRackPositionLast * filterPercentRack;
		b_SwaOffsetValid_Rack = TRUE;
		break;
	default:
		break;
	}
	if ((b_SwaOffsetValid_Rack == TRUE) && fabsf_hryt(f32_RackPosOffset - f32_RackDelta_S0) > P_SWA_MinDeltaS0_degRack)
	{
			flag_refresh_rack2=TRUE;
		}
		if (fabsf_hryt(f32_RackPosOffset - f32_RackDelta_S0) < P_SWA_MinDeltaS0_close_degRack)
		{
			flag_refresh_rack2=FALSE;
		}
		if (TRUE==flag_refresh_rack2)
		{
		f32_RackDelta_S0 = f32_RackDelta_S0 * SWA_FilterFactor_20 + f32_RackPosOffset * SWA_FilterFactor_80;
	}
}
/// @brief main process
/// @param

void fSASRackOffsetProcess(void)
{

	boolean KL15Flag = SASOffset_IsKL15ON();

	float32 filterPercent = 0;
	float32 filterPercent_Rack = 0;
	static uint8 s_SwaOffsetCompStatus = SWA_OffsetCompStatus_Disable;
	static uint8 s_SwaOffsetCompStatus_Rack = SWA_OffsetCompStatus_Disable;
	// 1 preprocess
	// 1-1 new fire cycle and SAS judgement
	if (FALSE == KL15Flag)
	{
		b_Rack_OffsetValid = FALSE;
		f32_RackDelta_S0 = 0.f;
		u16_RackDeltaN = 0u;
		f32_RackPosOffset = 0.f;
		u16_RackEnableCnt = 0u;
		b_SwaOffsetValid_Rack = FALSE;
		b_InitFlag = FALSE;
		b_NotNewFireCycle = FALSE;
		s_SwaOffsetCompStatus_Rack = SWA_OffsetCompStatus_Disable;
		flag_refresh_rack1 = FALSE;
		flag_refresh_rack2 = FALSE;
	}
	else
	{
		if (u16_SwaDeltaN >= P_SWA_DetaN_enum)
		{
			b_SwaOffsetValid = TRUE;
			if (FALSE == b_NotNewFireCycle)
			{
				b_InitFlag = TRUE;
			}
		}

		// 1-2 Filter factor calulation
		if (u16_SwaDeltaN > 0u)
		{
			filterPercent = 1 / (float32)u16_SwaDeltaN;
		}
		else
		{
			filterPercent = 0;
		}
		if (u16_RackDeltaN > 0)
		{
			filterPercent_Rack = 1 / (float32)u16_RackDeltaN;
		}
		else
		{
			filterPercent_Rack = 0;
		}
		// 1-3 raw value precessing
		fSASRackInputFilter();
		// 1-4 enable Cnt calculation
		fSASRackOffsetCompEn(&u16_SASRackEnableCnt, &u16_RackEnableCnt);
		qkfdebug.data7=u16_SASRackEnableCnt;
		// 2 main process
		fSASOffsetCal(filterPercent, &s_SwaOffsetCompStatus);
		if (FALSE == b_InitFlag)
		{
			fRackOffsetCal(filterPercent_Rack, &s_SwaOffsetCompStatus_Rack);
			qkfdebug.data6=u16_SASRackEnableCnt;
		}
		else
		{
			fRackOffsetCalCalibrated(filterPercent_Rack, &s_SwaOffsetCompStatus_Rack);
		}
		qkfdebug.data5=u16_SASRackEnableCnt;
		// 3 post process
		fSASOffsetCompSaturation(&f32_SwaDelta_S0);
		fRackPosOffsetLimitation(&f32_RackDelta_S0);
		fSASRackOutput();
		if (FALSE == b_NotNewFireCycle)
		{
			b_NotNewFireCycle = TRUE;
		}
	}	
}

/* ===========================================================================
 *
 * Project Name:
 * File Name:      SASRackOffsetComp.c
 * Archive:
 * Author:
 * Date:
 * (c) Copyright   2022, Human Horizons Operation Center Shanghai
 *-----------------------------------------------------------------------------
 *              V E R S I O N 	T R A C K I N G
 *-----------------------------------------------------------------------------
 * Version       Name                 Date                      Log
 * --------   -----------       ----------------     --------------------------
 *   v1.1		Kaifeng Qu				0615				Rack Cal added
 *
 * ========================================================================= */
