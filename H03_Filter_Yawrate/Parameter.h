#pragma once

#define CYCLE_TIME 0.02
#define GRAVITY_ACC 9.8
#define CAR_AXISLOAFDISTANCE 3.15 // temp
#define CAR_WEIGHTPUNKT_HIGHT 1.5
#define CAR_WEIGHT 2992 //ok
#define OneTenThousandth 1e-6
#define CAR_FRONTAXISWIDTH 1.701
#define CAR_REARAXISWIDTH 1.701 //fest
#define CAR_STEERTOFRONTWHEELRATIO 14.2
#define RADTODEGREE 57.3
#define FilterFactor_09 0.9
#define FilterFactor_01 0.1
#define FilterFactor_07 0.7
#define FilterFactor_03 0.3


#define LPF19(value, lastValue, minBoundary, maxBoundary)             \
	{                                                                                  \
		if ((minBoundary <= value) && (value <= maxBoundary))                          \
		{                                                                              \
			lastValue = value * FilterFactor_01 + lastValue * FilterFactor_09; \
		}                                                                              \
	}
#define LPF37(value, lastValue, minBoundary, maxBoundary)             \
	{                                                                                  \
		if ((minBoundary <= value) && (value <= maxBoundary))                          \
		{                                                                              \
			lastValue = value * FilterFactor_03 + lastValue * FilterFactor_07; \
		}                                                                              \
	}
//parameter default

typedef struct relevantParameter_tag
{
	/********Inputs*********/
	//float yaw;//set as zero
	float Vx_mps;
	float ay;
	float yawrate_raw_rps;// 总线raw 横摆角 raw值
	float yawrate_raw_dps;
	float frontWheelSpeedLeft;
	float frontWheelSpeedRight;
	float rearWheelSpeedLeft;
	float rearWheelSpeedRight;
	float yawrate_gyeAcc;//总线raw 横摆角加速度
	float steerWheelRatio;//  temp used  will be deleted
	float frontWheelAngle;//  temp used  will be deleted
	float steeringAngle;
	/********Inputs End*********/
	/********Middle Variables*********/
	float a_offset;
	float WLDf;
	float WLDr;
	float yawrate_WLDf;
	float yawrate_WLDr;
	float yawrate_gyeOffset ;
	float slideAngle ;// temp used  will be deleted
	float SSG ; //Self-Steer Gradient
	float curve;
	float curveRate;

	float yawrate_wld;
	float yawrate_gye_filted; // 总线 横摆角求出之后
	float yawrate_acc;
	float yawrate_steering;
	/********Middle Variables End*********/

	/********Finale Results*********/
	float yawrate_final;
	float curve_final;
	/********Finale Results ENd*********/

}RelevantParameter;

typedef struct ParameterWLDf_tag
{

	double WLDf_P[3][3] = { 0.001,0.001,0.,
		0,0.001,0,
	0,0,0.1 };
	double WLDf_Q[3][3] = { 0.0001,0.,0.,
	0,0.0001,0,
	0,0,0.0001 };
	double WLDf_R[2][2] = { 0.02,0,0,0.02 };

}ParameterWLDf;

typedef struct ParameterWLDr_tag
{

	double WLDr_P[3][3] = { 0.001,0.001,0.,
		0,0.001,0,
	0,0,0.1 };
	double WLDr_Q[3][3] = { 0.0001,0.,0.,
	0,0.0001,0,
	0,0,0.0001 };
	double WLDr_R[2][2] = { 0.02,0,0,0.02 };

}ParameterWLDr;


typedef struct ParameterOffset_tag
{

	double Offset_P[3][3] = { 0.01,0.01,0.,
		0,0.01,0,
	0,0,0.01 };
	double Offset_Q[3][3] = { 0.0001,0.,0.,
	0,0.0001,0,
	0,0,0.0001 };
	double Offset_R[3][3] = { 0.02,0.,0.,
	0,0.02,0,
	0,0,0.01 };

}ParameterOffset;

typedef struct ParameterWheel_tag
{

	double wheelLoadBased_P[2][2] = { 0.01,0.01,0.,0.01 };
	double wheelLoadBased_Q[2][2] = { 0.0001,0.,0.,0.0001 };
	double wheelLoadBased_R[2][2] = { 0.02,0,0,0.02 };

}ParameterWheel;


typedef struct ParameterYaw_tag
{


	double yawrateBased_P[2][2] = { 0.01,0.01,0.,0.01 };
	double yawrateBased_Q[2][2] = { 0.01,0.,0.,0.01 };
	double yawrateBased_R = 0.005f;


}ParameterYaw;
typedef struct ParameterAcc_tag
{

	double lateralAccBased_P[4][4] = { 0.01,0.01,0.,0.,
	0,0.01,0,0,
	0,0,0.01,0,
	0,0,0,0.01};
	double lateralAccBased_Q[4][4] = { 0.01,0.,0.,0.,
	0.,0.01,0.,0.,
	0.,0.,0.01,0.,
	0.,0.,0.,0.01
	};
	double lateralAccBased_R[2][2] = { 0.2,0,0,0.2 };

}ParameterAcc;

typedef struct ParameterSSG_tag
{

	double SSG_P[3][3] = { 0.01,0.01,0.,
		0,0.01,0,
	0,0,0.01 };
	double SSG_Q[3][3] = { 0.01,0.,0.,
	0,0.01,0,
	0,0,0.01};
	double SSG_R[2][2] = { 0.005,0,0,0.005 };

}ParameterSSG;

typedef struct ParameterSteer_tag
{


	double steeringAngleBased_P[2][2] = { 0.01,0.01,0.,0.01 };
	double steeringAngleBased_Q[2][2] = { 0.01,0.,0.,0.01 };
	double steeringAngleBased_R=0.01f;

}ParameterSteer;

typedef struct ParameterCurve_tag
{

	double Curve_P[2][2] = { 0.0001,0.0001,0.,0.00001 };

	double Curve_Q[2][2] = { 0.0001,0.,0.,0.00001 };
	double Curve_R = 0.01f;

}ParameterCurve;

typedef struct ParameterFinalYawrate_tag
{
	double finalMergeYawrate_P[2][2] = { 0.01,0.01,0.,0.01 };
	double finalMergeYawrate_Q[2][2] = { 0.00001,0.,0.,0.00001 };
	double finalMergeYawrate_R[4][4] = { 0.05,0,0,0, 
	0,0.001,0,0,
	0,0,0.001,0,
	0,0,0,0.005
	};

}ParameterFinalYawrate;

typedef struct ParameterFinalCurve_tag
{


	double finalMergeCurve_P[2][2] = { 0.0001,0.0001,0.,0.0001 };// 0 0 at begining use bigger p0
	double finalMergeCurve_Q[2][2] = { 0.0001,0.,0.,0.0001 };
	double finalMergeCurve_R[2][2] = { 0.0005,0,0,0.0002 };

}ParameterFinalCurve;

typedef struct Parameters_tag
{
	ParameterWLDf parameterWLDf;
	ParameterWLDr parameterWLDr;
	ParameterOffset parameterOffset;
	ParameterWheel parameterWheel;
	ParameterYaw parameterYaw;
	ParameterAcc parameterAcc;
	ParameterSSG parameterSSG;
	ParameterSteer parameterSteer;
	ParameterCurve parameterCurve;
	ParameterFinalYawrate parameterFinalYawrate;
	ParameterFinalCurve parameterFinalCurve;
}Parameters;


