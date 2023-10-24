#include "FilterRun.h"
#include <iostream>
#include <algorithm>
FilterRun WLDf_estimation;
FilterRun WLDr_estimation;
FilterRun offset_Estimation;



FilterRun wheelLoadBased;
FilterRun yawrateBased;
FilterRun lateralAccBased;
FilterRun SSG_estimation;
FilterRun steeringAngleBased;
FilterRun Curve_estimation;
FilterRun finalMergeYawrate;
FilterRun finalMergeCurve;
Parameters parameter;
RelevantParameter relevantParameter;


int main_test()
{

	/*****************************test code start****************************/

	//int n = 100;
	//int a[10] = { 1,2,3,4,5,6,7,8,9,10 };
	//while (n--)
	//{

	//	static int i = 0;
	//	i++;
	//	i = i % 10;

	/*use for Simulink comment out in the real car */

	static double s_time = 0.f;

	s_time += 0.02f;
	std::cout << "current time :"<<s_time << std::endl;
	std::cout << "yawrate_raw :" << relevantParameter.yawrate_raw_rps << std::endl;



	/*use for Simulink comment out in the real car end*/


	/**********************comment out for simulink , used in real vehical
	* 
	* 
	* 
	* 
		relevantParameter.Vx =					 *Simulink_V;
		relevantParameter.ay =					 *Simulink_A;
		relevantParameter.yawrate_raw =			 *Simulink_Yawrate;
		relevantParameter.steerWheelAngle =		 *Simulink_SteeringAngle;
		relevantParameter.frontWheelSpeedLeft  = *Simulink_frontWheelSpeedLeft;
		relevantParameter.frontWheelSpeedRight = *Simulink_frontWheelSpeedRight;
		relevantParameter.rearWheelSpeedLeft   = *Simulink_rearWheelSpeedLeft;
		relevantParameter.rearWheelSpeedRight  = *Simulink_rearWheelSpeedRight;







		relevantParameter.Vx =						//std::max((double)a[i],0.001f);
		relevantParameter.ay =						// a[i];
		relevantParameter.yawrate_raw =				//a[i];
		relevantParameter.steerWheelAngle =			//((double)a[i]) / 10; //  rad
		relevantParameter.frontWheelSpeedLeft  =	// a[i];
		relevantParameter.frontWheelSpeedRight =	//a[i] - 0.2;
		relevantParameter.rearWheelSpeedLeft   =	//a[i];
		relevantParameter.rearWheelSpeedRight  =	//a[i] - 0.3;

		//std::cout << "Input: " << a[i] << std::endl;
		comment out for simulink , used in real vehical end ******************/


	/*****************************test code end****************************/

	/***************unit conversion ************/
	static double Vx_mps_old = relevantParameter.Vx_mps;
	static double frontWheelAngle_old = relevantParameter.frontWheelAngle;
	static double yawrate_raw_dps_old = relevantParameter.yawrate_raw_dps;
	static double ay_old = relevantParameter.ay;
	static double frontWheelSpeedLeft_old = relevantParameter.frontWheelSpeedLeft;
	static double frontWheelSpeedRight_old = relevantParameter.frontWheelSpeedRight;
	static double rearWheelSpeedLeft_old = relevantParameter.rearWheelSpeedLeft;
	static double rearWheelSpeedRight_old = relevantParameter.rearWheelSpeedRight;

	LPF37(relevantParameter.Vx_mps, Vx_mps_old, -60, +60);
	LPF37(relevantParameter.frontWheelSpeedLeft, frontWheelSpeedLeft_old, -60, +60);
	LPF37(relevantParameter.frontWheelSpeedRight, frontWheelSpeedRight_old, -60, +60);
	LPF37(relevantParameter.rearWheelSpeedLeft, rearWheelSpeedLeft_old, -60, +60);
	LPF37(relevantParameter.rearWheelSpeedRight, rearWheelSpeedRight_old, -60, +60);
	LPF37(relevantParameter.frontWheelAngle, frontWheelAngle_old, -20, +20);
	LPF37(relevantParameter.yawrate_raw_dps, yawrate_raw_dps_old, -20, +20);
	LPF37(relevantParameter.ay, ay_old, -7, +7);
	Vx_mps_old = relevantParameter.Vx_mps;
	frontWheelAngle_old = relevantParameter.frontWheelAngle;
	yawrate_raw_dps_old = relevantParameter.yawrate_raw_rps;
	ay_old = relevantParameter.ay;
	frontWheelSpeedLeft_old = relevantParameter.frontWheelSpeedLeft;
	frontWheelSpeedRight_old = relevantParameter.frontWheelSpeedRight;
	rearWheelSpeedLeft_old = relevantParameter.rearWheelSpeedLeft;
	rearWheelSpeedRight_old = relevantParameter.rearWheelSpeedRight;

	relevantParameter.Vx_mps=					relevantParameter.Vx_mps / 3.6;
	relevantParameter.frontWheelSpeedLeft  =	relevantParameter.frontWheelSpeedLeft /3.6;
	relevantParameter.frontWheelSpeedRight =	relevantParameter.frontWheelSpeedRight/3.6;
	relevantParameter.rearWheelSpeedLeft   =	relevantParameter.rearWheelSpeedLeft  /3.6;
	relevantParameter.rearWheelSpeedRight  =	relevantParameter.rearWheelSpeedRight /3.6;
	relevantParameter.steeringAngle =			relevantParameter.steeringAngle/RADTODEGREE;
	relevantParameter.frontWheelAngle =			relevantParameter.steeringAngle / CAR_STEERTOFRONTWHEELRATIO;
	relevantParameter.steerWheelRatio =			static_cast<float>(CAR_STEERTOFRONTWHEELRATIO);
	relevantParameter.yawrate_raw_rps = relevantParameter.yawrate_raw_dps / RADTODEGREE;




	/***************unit conversion end **********/

	//// 1. wheelLoadBased Process
	// 
	// 1-1 WLDf cal
	// 
		static bool sflag_WLDf = false;
	if (false == sflag_WLDf) // use signal valid status in real vehical 
	{
		WLDf_estimation.fInitializeFilter(parameter.parameterWLDf, relevantParameter);
		sflag_WLDf = true;
	}
	std::cout << "Initial state sflag_WLDf: " << sflag_WLDf << std::endl;
	double WLDf_t = WLDf_estimation.kalmanFilter.getStates()[2];
	double yawrateWLDf_t = WLDf_estimation.kalmanFilter.getStates()[1];

	double obsWLDfH_t1 = 1 + ((OneTenThousandth * WLDf_t * CAR_WEIGHTPUNKT_HIGHT * CAR_WEIGHT * CAR_AXISLOAFDISTANCE * relevantParameter.Vx_mps * relevantParameter.Vx_mps) / (CAR_FRONTAXISWIDTH * CAR_FRONTAXISWIDTH));
	double obsWLDfH_t2 = (OneTenThousandth * yawrateWLDf_t * CAR_WEIGHTPUNKT_HIGHT * CAR_WEIGHT * CAR_AXISLOAFDISTANCE * relevantParameter.Vx_mps * relevantParameter.Vx_mps) / (CAR_FRONTAXISWIDTH * CAR_FRONTAXISWIDTH);
	WLDf_estimation.kalmanFilter.observations[0].transformMatrix(0, 1) = obsWLDfH_t1;
	WLDf_estimation.kalmanFilter.observations[0].transformMatrix(0, 2) = obsWLDfH_t2;

	//yawrate front axis 
	double obsWLDf_t1 = (relevantParameter.frontWheelSpeedRight - relevantParameter.frontWheelSpeedLeft) * cos(relevantParameter.frontWheelAngle) / CAR_FRONTAXISWIDTH;
	//yawrate rear axis 
	double obsWLDf_t2 =  (relevantParameter.rearWheelSpeedRight - relevantParameter.rearWheelSpeedLeft) / CAR_REARAXISWIDTH;
	double obsWLD_mean = 0.f;
	static double  obsWLDf_t1_old = obsWLDf_t1;
	static double  obsWLDf_t2_old = obsWLDf_t2;
	LPF19(obsWLDf_t1, obsWLDf_t1_old, -7, +7);
	LPF19(obsWLDf_t2, obsWLDf_t2_old, -7, +7);
	obsWLD_mean = (obsWLDf_t1 + obsWLDf_t2) / 2;

	obsWLDf_t1_old= obsWLDf_t1;
	obsWLDf_t2_old= obsWLDf_t2;




	WLDf_estimation.kalmanFilter.observations[0].observations << obsWLDf_t1, obsWLDf_t2;
	Eigen::VectorXd res_WLDf_estimation;
	res_WLDf_estimation = WLDf_estimation.fKalmanProcess();
	relevantParameter.WLDf = res_WLDf_estimation[2];
	if (relevantParameter.WLDf >= 0.f)
	{
		relevantParameter.WLDf = std::max(relevantParameter.WLDf, 0.001f);
	}
	else
	{
		relevantParameter.WLDf = std::min(relevantParameter.WLDf, -0.001f);
	}
	std::cout << "res  WLDf:" << res_WLDf_estimation[2] << std::endl;
	relevantParameter.yawrate_WLDf = obsWLDf_t1 / relevantParameter.WLDf;

	//1-2 WLDr cal
	static bool sflag_WLDr = false;
	if (false == sflag_WLDr) // use signal valid status in real vehical 
	{
		WLDr_estimation.fInitializeFilter(parameter.parameterWLDr, relevantParameter);
		sflag_WLDr = true;
	}
	std::cout << "Initial state sflag_WLDr: " << sflag_WLDr << std::endl;
	double WLDr_t = WLDr_estimation.kalmanFilter.getStates()[2];
	double yawrateWLDr_t = WLDr_estimation.kalmanFilter.getStates()[1];

	double obsWLDrH_t1 = 1 + ((OneTenThousandth * WLDr_t * CAR_WEIGHTPUNKT_HIGHT * CAR_WEIGHT * CAR_AXISLOAFDISTANCE * relevantParameter.Vx_mps * relevantParameter.Vx_mps) / (CAR_REARAXISWIDTH * CAR_REARAXISWIDTH));
	double obsWLDrH_t2 = ((OneTenThousandth * yawrateWLDr_t * CAR_WEIGHTPUNKT_HIGHT * CAR_WEIGHT * CAR_AXISLOAFDISTANCE * relevantParameter.Vx_mps * relevantParameter.Vx_mps) / (CAR_REARAXISWIDTH * CAR_REARAXISWIDTH));
	WLDr_estimation.kalmanFilter.observations[0].transformMatrix(0, 1) = obsWLDrH_t1;
	WLDr_estimation.kalmanFilter.observations[0].transformMatrix(0, 2) = obsWLDrH_t2;

	WLDr_estimation.kalmanFilter.observations[0].observations << obsWLDf_t1, obsWLDf_t2;
	Eigen::VectorXd res_WLDr_estimation;
	res_WLDr_estimation = WLDr_estimation.fKalmanProcess();
	relevantParameter.WLDr = res_WLDr_estimation[2];

	if (relevantParameter.WLDr >= 0.f)
	{
		relevantParameter.WLDr = std::max(relevantParameter.WLDr, 0.001f);
	}
	else
	{
		relevantParameter.WLDr = std::min(relevantParameter.WLDr, -0.001f);
	}

	std::cout << "res  WLDr:" << res_WLDr_estimation[2] << std::endl;
	relevantParameter.yawrate_WLDr = obsWLDf_t2 / relevantParameter.WLDr;

	//1-3 yawRate Offset Estimation

	static bool sflag_offset = false;
	if (false == sflag_offset) // use signal valid status in real vehical 
	{
		offset_Estimation.fInitializeFilter(parameter.parameterOffset, relevantParameter);
		sflag_offset = true;
	}
	std::cout << "Initial state sflag_offset: " << sflag_offset << std::endl;
	offset_Estimation.kalmanFilter.observations[0].observations << obsWLDf_t1, obsWLDf_t2, relevantParameter.yawrate_raw_rps;// relevantParameter.yawrate_WLDf,
		//relevantParameter.yawrate_WLDr, relevantParameter.yawrate_raw_dps;
	Eigen::VectorXd res_offset_Estimation;
	res_offset_Estimation = offset_Estimation.fKalmanProcess();
	relevantParameter.yawrate_gyeOffset = res_offset_Estimation[2];
	std::cout << "res  yawrate_gyeOffset:" << res_offset_Estimation[2] << std::endl;

	//1-4 WheelAngleBased
	static bool sflag_wheel = false;
	if (false == sflag_wheel) // use signal valid status in real vehical 
	{
		wheelLoadBased.fInitializeFilter(parameter.parameterWheel, relevantParameter);
		sflag_wheel = true;
	}
	std::cout << "Initial state sflag_wheel: " << sflag_wheel << std::endl;
	wheelLoadBased.kalmanFilter.observations[0].observations << obsWLDf_t1, obsWLDf_t2;// relevantParameter.yawrate_WLDf,relevantParameter.yawrate_WLDr;
	Eigen::VectorXd res_wheelLoadBased;
	res_wheelLoadBased = wheelLoadBased.fKalmanProcess();
	relevantParameter.yawrate_wld = res_wheelLoadBased[0];
	std::cout << "!!!!!!!!!!!res  yawrate_wld:" << res_wheelLoadBased[0] << std::endl;



	// 2. yawrateBased Process
	static bool sflag_yawrate = false;
	if (false == sflag_yawrate) // use signal valid status in real vehical 
	{
		yawrateBased.fInitializeFilter(parameter.parameterYaw, relevantParameter);
		sflag_yawrate = true;
	}
	std::cout << "Initial state yawrate: " << sflag_yawrate << std::endl;
	yawrateBased.kalmanFilter.observations[0].observations << 
		(relevantParameter.yawrate_raw_rps - 0);// relevantParameter.yawrate_gyeOffset);
	Eigen::VectorXd res_yawrateBased;
	res_yawrateBased = yawrateBased.fKalmanProcess();
	relevantParameter.yawrate_gye_filted = res_yawrateBased[0];
	std::cout << "!!!!!!!!!!!res  yawrate_gye_filted:" << res_yawrateBased[0] << std::endl;

	// 3. lateralAccBased Process

	static bool sflag_Acc = false;
	if (false == sflag_Acc) // use signal valid status in real vehical 
	{
		lateralAccBased.fInitializeFilter(parameter.parameterAcc, relevantParameter);
		sflag_Acc = true;
	}
	std::cout << "Initial state ACC: " << sflag_Acc << std::endl;

	relevantParameter.slideAngle = lateralAccBased.kalmanFilter.getStates()[2];
	lateralAccBased.kalmanFilter.observations[0].transformMatrix(0, 1) = relevantParameter.Vx_mps;
	lateralAccBased.kalmanFilter.observations[0].transformMatrix(0, 2) = -GRAVITY_ACC * cos(relevantParameter.slideAngle);

	lateralAccBased.kalmanFilter.observations[0].observations << relevantParameter.ay, relevantParameter.yawrate_raw_rps;
	Eigen::VectorXd res_accBased;
	res_accBased = lateralAccBased.fKalmanProcess();
	relevantParameter.yawrate_acc = res_accBased[1];
	std::cout <<"!!!!!!!!!!!res  yawrate_acc : "<< res_accBased[1] << std::endl;

	// 3. steeringAngleBased Process
		
	//3-1 SSG Estimation

	static bool sflag_SSG = false;
	if (false == sflag_SSG) // use signal valid status in real vehical 
	{
		SSG_estimation.fInitializeFilter(parameter.parameterSSG, relevantParameter);
		sflag_SSG = true;
	}
	std::cout << "Initial state SSG: " << sflag_SSG << std::endl;
	double yawrate_t = SSG_estimation.kalmanFilter.getStates()[1];
	SSG_estimation.kalmanFilter.observations[0].observations << yawrate_t, relevantParameter.steeringAngle;

	double obsSSGH_t1 = relevantParameter.steerWheelRatio * CAR_AXISLOAFDISTANCE / relevantParameter.Vx_mps;
	double obsSSGH_t2 = ((double)relevantParameter.steerWheelRatio * relevantParameter.Vx_mps * yawrate_t);
	SSG_estimation.kalmanFilter.observations[0].transformMatrix(1,1) = obsSSGH_t1;
	SSG_estimation.kalmanFilter.observations[0].transformMatrix(1,2)= obsSSGH_t2;
	Eigen::VectorXd res_SSG_estimation;
	res_SSG_estimation = SSG_estimation.fKalmanProcess();

	std::cout << "res  SSG_estimation:" << res_SSG_estimation[2] << std::endl;

	relevantParameter.SSG = res_SSG_estimation[2];

	// 3-2 Yawrate Estimation

	static bool sflag_steering = false;
	if (false == sflag_steering) // use signal valid status in real vehical 
	{
		steeringAngleBased.fInitializeFilter(parameter.parameterSteer, relevantParameter);
		sflag_steering = true;
	}
	std::cout << "Initial state Wheel: " << sflag_steering << std::endl;
	double obs_t = relevantParameter.steeringAngle / (relevantParameter.steerWheelRatio *
		(CAR_AXISLOAFDISTANCE / relevantParameter.Vx_mps + relevantParameter.Vx_mps * relevantParameter.SSG));
	steeringAngleBased.kalmanFilter.observations[0].observations << obs_t;
	Eigen::VectorXd res_Steer_estimation;
	res_Steer_estimation = steeringAngleBased.fKalmanProcess();
	relevantParameter.yawrate_steering = res_Steer_estimation[1];
	std::cout << "!!!!!!!!!!!res  yawrate_steering:" << res_Steer_estimation[1] << std::endl;


	//3-3 Curve Estimation

	static bool sflag_Curve = false;
	if (false == sflag_Curve) // use signal valid status in real vehical 
	{
		Curve_estimation.fInitializeFilter(parameter.parameterCurve, relevantParameter);
		sflag_Curve = true;
	}
	std::cout << "Initial state Curve: " << sflag_Curve << std::endl;
	Curve_estimation.kalmanFilter.observations[0].observations << relevantParameter.steeringAngle;
	double obsCurveH_t1 = (CAR_AXISLOAFDISTANCE + (double)relevantParameter.SSG * relevantParameter.Vx_mps * relevantParameter.Vx_mps) * relevantParameter.steerWheelRatio;
	Curve_estimation.kalmanFilter.observations[0].transformMatrix(0.0) = obsSSGH_t1;
	Eigen::VectorXd res_Curve_estimation;
	res_Curve_estimation = Curve_estimation.fKalmanProcess();
	relevantParameter.curve = res_Curve_estimation[0];
	std::cout << "res  Curve_estimation:" << res_Curve_estimation[0] << std::endl;

	// 4 final 

//4-1 final yawrate estimation
	static bool sflag_yawrate_final = false;
	if (false == sflag_yawrate_final) // use signal valid status in real vehical 
	{
		finalMergeYawrate.fInitializeFilter(parameter.parameterFinalYawrate, relevantParameter);
		sflag_yawrate_final = true;
	}
	std::cout << "Initial state yawrate final: " << sflag_yawrate_final << std::endl;
	finalMergeYawrate.kalmanFilter.observations[0].observations << obsWLD_mean,
		relevantParameter.yawrate_gye_filted, relevantParameter.yawrate_acc, relevantParameter.yawrate_steering;
	Eigen::VectorXd res_finalMergeYawrate;
	res_finalMergeYawrate = finalMergeYawrate.fKalmanProcess();
	relevantParameter.yawrate_final = res_finalMergeYawrate[1];
	std::cout << "!!!!!!!!!!!res  FinalYawrate:" << res_finalMergeYawrate[1] << std::endl;

	//4-2 final curve estimation
	static bool sflag_curve_final = false;
	if (false == sflag_curve_final) // use signal valid status in real vehical 
	{
		finalMergeCurve.fInitializeFilter(parameter.parameterFinalCurve, relevantParameter);
		sflag_curve_final = true;
	}
	std::cout << "Initial state curve final: " << sflag_curve_final << std::endl;
	finalMergeCurve.kalmanFilter.observations[0].observations << relevantParameter.yawrate_final/ relevantParameter.Vx_mps, relevantParameter.curve;
	Eigen::VectorXd res_finalMergeCurve;
	res_finalMergeCurve = finalMergeCurve.fKalmanProcess();
	relevantParameter.curve_final = res_finalMergeCurve[0]; 
	std::cout << "res  FinalCurve:" << res_finalMergeCurve[0] << std::endl;

	return 0;
	}

//
//}