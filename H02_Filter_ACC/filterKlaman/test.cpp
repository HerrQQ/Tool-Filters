#include <stddef.h>
#include <stdio.h>
#include"KalmanFilter.h"
//#include "test.h"
#include <iostream>
//#include "test.h"


KalmanFilter Filter;
 double Actualspeed_mps = 0.f;
 double Acceleration_mpss = 0.f;

 
//input 
double ESC_VehicleSpeed=0.f;
double Acceleration=0.f;
//paramter
double a_A[2][2] = {1.,0.02,0.,1.};
double a_P[2][2] = { 0.5,0.1,0.,0.5 };
double a_Q[2][2] = { 0.1,0.,0.,0.1 };
double a_R[2][2] = { 0.2,0,0,0.2 };
int a_H1[2][2] = { 1,0,0,1 };

// output
double Filtered_speed;
double Filtered_acceleration;



void initialize_filter() {
    // 1. timeStep
    Filter.setTimeStep(0.02);

    // 2. states
    Eigen::VectorXd init_states = Eigen::VectorXd::Zero(2);

    init_states[0] =  0;
    init_states[1] = 0;


    Eigen::MatrixXd transition_matrix(2, 2);
    transition_matrix <<  a_A[0][0], a_A[0][1], a_A[1][0], a_A[1][1];
   // std::cout << transition_matrix(0, 0) << transition_matrix(1, 1) << std::endl;
        //1, 0.02,
        //0, 1;
    Eigen::MatrixXd init_states_covariance(2, 2);
    init_states_covariance << a_P[0][0], a_P[0][1], a_P[1][0], a_P[1][1];
        /*0.5, 0.1,
        0, 0.5;*/
    Eigen::MatrixXd predict_noise(2, 2);
    predict_noise << a_Q[0][0], a_Q[0][1], a_Q[1][0], a_Q[1][1];
        /*0.01, 0.01,
        0, 0.01;*/
    Filter.initialize_states(&init_states,
        &transition_matrix,
        &init_states_covariance,
        &predict_noise);
    // 3. initilize observations
Observation obs_;
    obs_.observations = Eigen::VectorXd::Zero(2);
    //printf("I am here \n");
    obs_.transformMatrix.resize(2, 2);
    obs_.transformMatrix << a_H1[0][0], a_H1[0][1], a_H1[1][0], a_H1[1][1];
    obs_.observationNoise.resize(2, 2);
    obs_.observationNoise << a_R[0][0], a_R[0][1], a_R[1][0], a_R[1][1];
        //0.2, 0.1,
        //0.0, 0.2;
    Filter.observations_increment(obs_);
}




void KalmanProcess()
{

    Actualspeed_mps =  ESC_VehicleSpeed / 3.6; // 接收实时车速，并将单位转化为m/s;


    Acceleration_mpss = Acceleration;

static bool sflag = false;
    // 1. initial
if (false == sflag)
{
    initialize_filter();
    sflag = true;
}

    Filter.observations[0].observations << Actualspeed_mps, Acceleration_mpss;


    // 2. filter
    Filter.filter();
    // 3. output
    Eigen::VectorXd states;
    states = Filter.getStates();
    Filtered_speed = states[0];
    Filtered_acceleration = states[1];
    printf("run one step, Filter speed & Acceleration is: %f, %f. \n", Filtered_speed, Filtered_acceleration);
    //system("pause");
}




