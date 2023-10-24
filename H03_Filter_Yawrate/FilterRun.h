#pragma once
#include "KalmanFilter.h"
#include "Parameter.h"







extern double Actualspeed_mps;
extern double Acceleration_mpss;
//input 
extern double ESC_VehicleSpeed;
extern double Acceleration;



//parameter
extern double a_P[2][2];
extern double a_Q[2][2];
extern double a_R[2][2];
//output
extern double Filtered_speed;
extern double Filtered_acceleration;



class FilterRun
{
public :
	void fInitializeFilter(const ParameterWLDf& parameter, const RelevantParameter& relevantParameter);
	void fInitializeFilter(const ParameterWLDr& parameter, const RelevantParameter& relevantParameter);
	void fInitializeFilter(const ParameterOffset& parameter, const RelevantParameter& relevantParameter);
	void fInitializeFilter(const ParameterWheel& parameter, const RelevantParameter& relevantParameter);
	void fInitializeFilter(const ParameterYaw& parameter, const RelevantParameter& relevantParameter);
	void fInitializeFilter(const ParameterAcc& parameter, const RelevantParameter& relevantParameter);
	void fInitializeFilter(const ParameterSSG& parameter, const RelevantParameter& relevantParameter);
	void fInitializeFilter(const ParameterSteer& parameter, const RelevantParameter& relevantParameter);
	void fInitializeFilter(const ParameterCurve& parameter, const RelevantParameter& relevantParameter);
	void fInitializeFilter(const ParameterFinalYawrate& parameter, const RelevantParameter& relevantParameter);
	void fInitializeFilter(const ParameterFinalCurve& parameter, const RelevantParameter& relevantParameter);

	Eigen::VectorXd fKalmanProcess();
	FilterRun() = default;
	~FilterRun();
public:
	KalmanFilter kalmanFilter;
};



