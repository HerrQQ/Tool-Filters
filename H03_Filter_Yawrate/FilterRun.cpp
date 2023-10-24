#include <stddef.h>
#include <stdio.h>
#include"KalmanFilter.h"
#include "FilterRun.h"
#include <iostream>
//#include "test.h"

// Global Variables
//KalmanFilter Filter;

//FilterRun wheelLoadBased;
//FilterRun yawrateBased;
//FilterRun lateralAccBased;
//FilterRun steeringAngleBased;
//FilterRun finalMerge;


void FilterRun:: fInitializeFilter(const ParameterWLDf& parameter, const RelevantParameter& relevantParameter)
{
    // 1. timeStep
    this->kalmanFilter.setTimeStep(0.02);

    // 2. states
    Eigen::VectorXd init_states = Eigen::VectorXd::Zero(3);



    init_states[0] = 0.f;// heading init;
    init_states[1] = relevantParameter.yawrate_raw_rps;
    init_states[2] = 0.f;//relevantParameter.WLDf; // need to check whether it could be zero


    Eigen::MatrixXd transition_matrix(3, 3);
    transition_matrix << 1, CYCLE_TIME, 0, 0, 1, 0, 0, 0, 1;
    // std::cout << transition_matrix(0, 0) << transition_matrix(1, 1) << std::endl;

    Eigen::MatrixXd init_states_covariance(3, 3);//p_init


    Eigen::MatrixXd predict_noise(3, 3);

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; ++j)
        {
            init_states_covariance(i, j) = parameter.WLDf_P[i][j];
            predict_noise(i, j) = parameter.WLDf_Q[i][j];

        }
    }
    this->kalmanFilter.initialize_states(&init_states,
        &transition_matrix,
        &init_states_covariance,
        &predict_noise);
    // 3. initilize observations
    Observation obs_;
    obs_.observations = Eigen::VectorXd::Zero(2);
    //printf("I am here \n");
    obs_.transformMatrix.resize(2, 3);
    //temp obs_t1
    double obsH_t1 = 1+((OneTenThousandth * relevantParameter.WLDf * CAR_WEIGHTPUNKT_HIGHT * CAR_WEIGHT * CAR_AXISLOAFDISTANCE* relevantParameter.Vx_mps *relevantParameter.Vx_mps)/( CAR_FRONTAXISWIDTH* CAR_FRONTAXISWIDTH));
    double obsH_t2 = (OneTenThousandth * relevantParameter.yawrate_raw_rps * CAR_WEIGHTPUNKT_HIGHT * CAR_WEIGHT * CAR_AXISLOAFDISTANCE * relevantParameter.Vx_mps * relevantParameter.Vx_mps) / (CAR_FRONTAXISWIDTH * CAR_FRONTAXISWIDTH);
    obs_.transformMatrix << 0, obsH_t1, obsH_t2,
        0, 1, 0;
    obs_.observationNoise.resize(2, 2);
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; ++j)
        {
            obs_.observationNoise(i, j) = parameter.WLDf_R[i][j];
        }
    }
    //0.2, 0.1,
    //0.0, 0.2;
    this->kalmanFilter.observations_increment(obs_);




}
void FilterRun:: fInitializeFilter(const ParameterWLDr& parameter, const RelevantParameter& relevantParameter)
{
    // 1. timeStep
    this->kalmanFilter.setTimeStep(0.02);

    // 2. states
    Eigen::VectorXd init_states = Eigen::VectorXd::Zero(3);



    init_states[0] = 0.f;// heading init;
    init_states[1] = relevantParameter.yawrate_raw_rps;
    init_states[2] = 0;// relevantParameter.WLDr; // need to check whether it could be zero


    Eigen::MatrixXd transition_matrix(3, 3);
    transition_matrix << 1, CYCLE_TIME, 0, 0, 1, 0, 0, 0, 1;
    // std::cout << transition_matrix(0, 0) << transition_matrix(1, 1) << std::endl;

    Eigen::MatrixXd init_states_covariance(3, 3);//p_init


    Eigen::MatrixXd predict_noise(3, 3);

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; ++j)
        {
            init_states_covariance(i, j) = parameter.WLDr_P[i][j];
            predict_noise(i, j) = parameter.WLDr_Q[i][j];

        }
    }
    this->kalmanFilter.initialize_states(&init_states,
        &transition_matrix,
        &init_states_covariance,
        &predict_noise);
    // 3. initilize observations
    Observation obs_;
    obs_.observations = Eigen::VectorXd::Zero(2);
    //printf("I am here \n");
    obs_.transformMatrix.resize(2, 3);
    //temp obs_t1
    double obsH_t1 = 1 + ((OneTenThousandth * relevantParameter.WLDr * CAR_WEIGHTPUNKT_HIGHT * CAR_WEIGHT * CAR_AXISLOAFDISTANCE * relevantParameter.Vx_mps * relevantParameter.Vx_mps) / (CAR_REARAXISWIDTH * CAR_REARAXISWIDTH));
    double obsH_t2 = (OneTenThousandth * relevantParameter.yawrate_raw_rps * CAR_WEIGHTPUNKT_HIGHT * CAR_WEIGHT * CAR_AXISLOAFDISTANCE * relevantParameter.Vx_mps * relevantParameter.Vx_mps) / (CAR_REARAXISWIDTH * CAR_REARAXISWIDTH);
    obs_.transformMatrix << 0, obsH_t1, obsH_t2,
        0, 1, 0;
    obs_.observationNoise.resize(2, 2);
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; ++j)
        {
            obs_.observationNoise(i, j) = parameter.WLDr_R[i][j];
        }
    }
    //0.2, 0.1,
    //0.0, 0.2;
    this->kalmanFilter.observations_increment(obs_);
}


void FilterRun:: fInitializeFilter(const ParameterOffset& parameter, const RelevantParameter& relevantParameter) {
    // 1. timeStep
    this->kalmanFilter.setTimeStep(0.02);

    // 2. states
    Eigen::VectorXd init_states = Eigen::VectorXd::Zero(3);



    init_states[0] = 0;// heading
    init_states[1] = relevantParameter.yawrate_raw_rps;
    init_states[2] = 0;//offset


    Eigen::MatrixXd transition_matrix(3, 3);
    transition_matrix << 1, CYCLE_TIME, 0, 0,1,0,0,0,1;
    // std::cout << transition_matrix(0, 0) << transition_matrix(1, 1) << std::endl;

    Eigen::MatrixXd init_states_covariance(3, 3);


    Eigen::MatrixXd predict_noise(3, 3);

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; ++j)
        {
            init_states_covariance(i, j) = parameter.Offset_P[i][j];
            predict_noise(i, j) = parameter.Offset_Q[i][j];

        }
    }
    this->kalmanFilter.initialize_states(&init_states,
        &transition_matrix,
        &init_states_covariance,
        &predict_noise);
    // 3. initilize observations
    Observation obs_;
    obs_.observations = Eigen::VectorXd::Zero(3);
    //printf("I am here \n");
    obs_.transformMatrix.resize(3, 3);
    obs_.transformMatrix << 0,1, 0,0,1,0,0,1,1;
    obs_.observationNoise.resize(3, 3);
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; ++j)
        {
            obs_.observationNoise(i, j) = parameter.Offset_R[i][j];

        }
    }
    //0.2, 0.1,
    //0.0, 0.2;
    this->kalmanFilter.observations_increment(obs_);
}


void FilterRun:: fInitializeFilter(const ParameterWheel& parameter,const RelevantParameter& relevantParameter) {
    // 1. timeStep
    this->kalmanFilter.setTimeStep(0.02);

    // 2. states
    Eigen::VectorXd init_states = Eigen::VectorXd::Zero(2);

    init_states[0] = relevantParameter.yawrate_raw_rps;
    init_states[1] = 0;//yawrate acc


    Eigen::MatrixXd transition_matrix(2, 2);
    transition_matrix << 1., CYCLE_TIME, 0., 1.;
   // std::cout << transition_matrix(0, 0) << transition_matrix(1, 1) << std::endl;
        //1, 0.02,
        //0, 1;
    Eigen::MatrixXd init_states_covariance(2, 2);
    Eigen::MatrixXd predict_noise(2, 2);

    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; ++j)
        {
            init_states_covariance(i, j) = parameter.wheelLoadBased_P[i][j];
            predict_noise(i, j) = parameter.wheelLoadBased_Q[i][j];

        }
    }

    this->kalmanFilter.initialize_states(&init_states,
        &transition_matrix,
        &init_states_covariance,
        &predict_noise);
    // 3. initilize observations
    Observation obs_;
    obs_.observations = Eigen::VectorXd::Zero(2);
    //printf("I am here \n");
    obs_.transformMatrix.resize(2, 2);
    obs_.transformMatrix << 1,0,1,0;
    obs_.observationNoise.resize(2, 2);
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; ++j)
        {
            obs_.observationNoise(i, j) = parameter.wheelLoadBased_R[i][j];

        }
    }
    this->kalmanFilter.observations_increment(obs_);
}

void FilterRun:: fInitializeFilter(const ParameterYaw& parameter, const RelevantParameter& relevantParameter) {
    // 1. timeStep
    this->kalmanFilter.setTimeStep(0.02);

    // 2. states
    Eigen::VectorXd init_states = Eigen::VectorXd::Zero(2);

 

    init_states[0] = relevantParameter.yawrate_raw_rps;
    init_states[1] = relevantParameter.yawrate_gyeAcc;


    Eigen::MatrixXd transition_matrix(2, 2);
    transition_matrix << 1, CYCLE_TIME, 0, 1;
    // std::cout << transition_matrix(0, 0) << transition_matrix(1, 1) << std::endl;

    Eigen::MatrixXd init_states_covariance(2, 2);


    Eigen::MatrixXd predict_noise(2, 2);

    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; ++j)
        {
            init_states_covariance(i, j) = parameter.yawrateBased_P[i][j];
            predict_noise(i, j) = parameter.yawrateBased_Q[i][j];

        }
    }
    this->kalmanFilter.initialize_states(&init_states,
        &transition_matrix,
        &init_states_covariance,
        &predict_noise);
    // 3. initilize observations
    Observation obs_;
    obs_.observations = Eigen::VectorXd::Zero(1);
    //printf("I am here \n");
    obs_.transformMatrix.resize(1, 2);
    obs_.transformMatrix << 1, 0;
    obs_.observationNoise.resize(1, 1);
    obs_.observationNoise << parameter.yawrateBased_R;
    //0.2, 0.1,
    //0.0, 0.2;
    this->kalmanFilter.observations_increment(obs_);
}

void FilterRun:: fInitializeFilter(const ParameterAcc& parameter, const RelevantParameter& relevantParameter) {
    // 1. timeStep
    this->kalmanFilter.setTimeStep(0.02);

    // 2. states
    Eigen::VectorXd init_states = Eigen::VectorXd::Zero(4);

    init_states[0] = 0;
    init_states[1] = relevantParameter.yawrate_raw_rps;
    init_states[2] = relevantParameter.slideAngle;
    init_states[3] = relevantParameter.a_offset;
    

    Eigen::MatrixXd transition_matrix(4, 4);
    transition_matrix << 1., CYCLE_TIME, 0., 0.,
        0., 1., 0., 0.,
        0., 0., 1., 0.,
        0., 0., 0., 1.;
    // std::cout << transition_matrix(0, 0) << transition_matrix(1, 1) << std::endl;
         //1, 0.02,
         //0, 1;
    Eigen::MatrixXd init_states_covariance(4, 4);
    /*0.5, 0.1,
    0, 0.5;*/
    Eigen::MatrixXd predict_noise(4, 4);

    for (int i = 0; i<4;i++)
    {
        for (int j = 0; j < 4;++j)
        {
            init_states_covariance(i, j) = parameter.lateralAccBased_P[i][j];
            predict_noise(i, j) = parameter.lateralAccBased_Q[i][j];

        }
    }
    /*0.01, 0.01,
    0, 0.01;*/
    this->kalmanFilter.initialize_states(&init_states,
        &transition_matrix,
        &init_states_covariance,
        &predict_noise);
    // 3. initilize observations
    Observation obs_;
    obs_.observations = Eigen::VectorXd::Zero(2);
    //printf("I am here \n");
    obs_.transformMatrix.resize(2, 4);
    //temp obs
    obs_.transformMatrix << 0, relevantParameter.Vx_mps, -GRAVITY_ACC * cos(relevantParameter.slideAngle), 1,
        0, 1, 0, 0;//slide angle need to cal ;cos rad 
    obs_.observationNoise.resize(2, 2);

    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; ++j)
        {
            obs_.observationNoise(i, j) = parameter.lateralAccBased_R[i][j];
        }
    }
    //0.2, 0.1,
    //0.0, 0.2;
    this->kalmanFilter.observations_increment(obs_);
}


void FilterRun:: fInitializeFilter(const ParameterSSG& parameter, const RelevantParameter& relevantParameter) {
    // 1. timeStep
    this->kalmanFilter.setTimeStep(0.02);

    // 2. states
    Eigen::VectorXd init_states = Eigen::VectorXd::Zero(3);



    init_states[0] = 0.f;// heading 
    init_states[1] = relevantParameter.yawrate_raw_rps;
    init_states[2] = 0;// relevantParameter.SSG;


    Eigen::MatrixXd transition_matrix(3, 3);
    transition_matrix << 1, CYCLE_TIME, 0,0,1,0,0,0,1;
    // std::cout << transition_matrix(0, 0) << transition_matrix(1, 1) << std::endl;

    Eigen::MatrixXd init_states_covariance(3, 3);//p_init


    Eigen::MatrixXd predict_noise(3, 3);

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; ++j)
        {
            init_states_covariance(i, j) = parameter.SSG_P[i][j];
            predict_noise(i, j) = parameter.SSG_Q[i][j];

        }
    }
    this->kalmanFilter.initialize_states(&init_states,
        &transition_matrix,
        &init_states_covariance,
        &predict_noise);
    // 3. initilize observations
    Observation obs_;
    obs_.observations = Eigen::VectorXd::Zero(2);
    //printf("I am here \n");
    obs_.transformMatrix.resize(2, 3);
    double obsH_t1 = CAR_AXISLOAFDISTANCE * relevantParameter.steerWheelRatio / relevantParameter.Vx_mps;
    // temp
    double obsH_t2 = ((double)relevantParameter.steerWheelRatio * (double)relevantParameter.Vx_mps * (double)relevantParameter.yawrate_raw_rps);
    obs_.transformMatrix << 0,1,0,
        0, obsH_t1, obsH_t2;
    obs_.observationNoise.resize(2, 2);
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; ++j)
        {
            obs_.observationNoise(i, j) = parameter.SSG_R[i][j];
        }
    }
    //0.2, 0.1,
    //0.0, 0.2;
    this->kalmanFilter.observations_increment(obs_);
}




void FilterRun:: fInitializeFilter(const ParameterSteer& parameter, const RelevantParameter& RelevantParameter) {
    // 1. timeStep
    this->kalmanFilter.setTimeStep(0.02);

    // 2. states
    Eigen::VectorXd init_states = Eigen::VectorXd::Zero(2);

    init_states[0] = 0;
    init_states[1] = RelevantParameter.yawrate_raw_rps;


    Eigen::MatrixXd transition_matrix(2, 2);
    transition_matrix << 1,CYCLE_TIME,0,1;
    // std::cout << transition_matrix(0, 0) << transition_matrix(1, 1) << std::endl;
         //1, 0.02,
         //0, 1;
    Eigen::MatrixXd init_states_covariance(2, 2);

    Eigen::MatrixXd predict_noise(2, 2);

    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; ++j)
        {
            init_states_covariance(i, j) = parameter.steeringAngleBased_P[i][j];
            predict_noise(i, j) = parameter.steeringAngleBased_Q[i][j];

        }
    }
    this->kalmanFilter.initialize_states(&init_states,
        &transition_matrix,
        &init_states_covariance,
        &predict_noise);
    // 3. initilize observations
    Observation obs_;
    obs_.observations = Eigen::VectorXd::Zero(1);
    //printf("I am here \n");
    obs_.transformMatrix.resize(1, 2);
    obs_.transformMatrix << 0, 1;
    obs_.observationNoise.resize(1,1);
    obs_.observationNoise<< parameter.steeringAngleBased_R;
    //0.2, 0.1,
    //0.0, 0.2;
    this->kalmanFilter.observations_increment(obs_);
}

void FilterRun:: fInitializeFilter(const ParameterCurve& parameter, const RelevantParameter& relevantParameter) {
    // 1. timeStep
    this->kalmanFilter.setTimeStep(0.02);

    // 2. states
    Eigen::VectorXd init_states = Eigen::VectorXd::Zero(2);

    init_states[0] = relevantParameter.curve;
    init_states[1] = relevantParameter.curveRate;


    Eigen::MatrixXd transition_matrix(2, 2);
    transition_matrix << 1, CYCLE_TIME, 0, 1;
    // std::cout << transition_matrix(0, 0) << transition_matrix(1, 1) << std::endl;

    Eigen::MatrixXd init_states_covariance(2, 2);//p_init


    Eigen::MatrixXd predict_noise(2, 2);

    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; ++j)
        {
            init_states_covariance(i, j) = parameter.Curve_P[i][j];
            predict_noise(i, j) = parameter.Curve_Q[i][j];

        }
    }
    this->kalmanFilter.initialize_states(&init_states,
        &transition_matrix,
        &init_states_covariance,
        &predict_noise);
    // 3. initilize observations
    Observation obs_;
    obs_.observations = Eigen::VectorXd::Zero(1);
    //printf("I am here \n");
    obs_.transformMatrix.resize(1, 2);
    double obs_t1 = (CAR_AXISLOAFDISTANCE+ (double)relevantParameter.SSG* relevantParameter.Vx_mps* relevantParameter.Vx_mps) * relevantParameter.steerWheelRatio;
    
    obs_.transformMatrix << obs_t1, 0;
        
    obs_.observationNoise.resize(1,1);

            obs_.observationNoise<< parameter.Curve_R;

    //0.2, 0.1,
    //0.0, 0.2;
    this->kalmanFilter.observations_increment(obs_);
}

void  FilterRun:: fInitializeFilter(const ParameterFinalYawrate& parameter, const RelevantParameter& RelevantParameter){
    // 1. timeStep
    this->kalmanFilter.setTimeStep(0.02);

    // 2. states
    Eigen::VectorXd init_states = Eigen::VectorXd::Zero(2);

    init_states[0] = 0;
    init_states[1] = RelevantParameter.yawrate_raw_rps;// temp usage


    Eigen::MatrixXd transition_matrix(2, 2);
    transition_matrix << 1,CYCLE_TIME,0,1;
    // std::cout << transition_matrix(0, 0) << transition_matrix(1, 1) << std::endl;

    Eigen::MatrixXd init_states_covariance(2, 2);

    Eigen::MatrixXd predict_noise(2, 2);

    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; ++j)
        {
            init_states_covariance(i, j) = parameter.finalMergeYawrate_P[i][j];
            predict_noise(i, j) = parameter.finalMergeYawrate_Q[i][j];

        }
    }
    this->kalmanFilter.initialize_states(&init_states,
        &transition_matrix,
        &init_states_covariance,
        &predict_noise);
    // 3. initilize observations
    Observation obs_;
    obs_.observations = Eigen::VectorXd::Zero(4);
    //printf("I am here \n");
    obs_.transformMatrix.resize(4, 2);
    obs_.transformMatrix << 0,1,0,1,0,1,0,1;
    obs_.observationNoise.resize(4,4);
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; ++j)
        {
            obs_.observationNoise(i, j) = parameter.finalMergeYawrate_R[i][j];
        }
    }
    this->kalmanFilter.observations_increment(obs_);
}


void  FilterRun:: fInitializeFilter(const ParameterFinalCurve& parameter, const RelevantParameter& RelevantParameter)
{
    // 1. timeStep
    this->kalmanFilter.setTimeStep(0.02);

    // 2. states
    Eigen::VectorXd init_states = Eigen::VectorXd::Zero(2);

    init_states[0] = 0;
    init_states[1] = 0;// temp usage


    Eigen::MatrixXd transition_matrix(2, 2);
    transition_matrix << 1, CYCLE_TIME, 0, 1;
    // std::cout << transition_matrix(0, 0) << transition_matrix(1, 1) << std::endl;

    Eigen::MatrixXd init_states_covariance(2, 2);

    Eigen::MatrixXd predict_noise(2, 2);

    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; ++j)
        {
            init_states_covariance(i, j) = parameter.finalMergeCurve_P[i][j];
            predict_noise(i, j) = parameter.finalMergeCurve_Q[i][j];
        }
    }
    this->kalmanFilter.initialize_states(&init_states,
        &transition_matrix,
        &init_states_covariance,
        &predict_noise);
    // 3. initilize observations
    Observation obs_;
    obs_.observations = Eigen::VectorXd::Zero(2);
    //printf("I am here \n");
    obs_.transformMatrix.resize(2, 2);
    obs_.transformMatrix <<  1, 0, 1, 0;
    obs_.observationNoise.resize(2, 2);
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; ++j)
        {
            obs_.observationNoise(i, j) = parameter.finalMergeCurve_R[i][j];
        }
    }
    this->kalmanFilter.observations_increment(obs_);
}

Eigen::VectorXd FilterRun:: fKalmanProcess()
{

   


    // 2. filter
    this->kalmanFilter.filter();
    // 3. output
    Eigen::VectorXd states;
    states = this->kalmanFilter.getStates();
    return states;
   

    
    //system("pause");
}

FilterRun :: ~FilterRun()
{
    std::cout << "FilterRun End" << std::endl;
}



