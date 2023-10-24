#pragma once
void initialize_filter();
void KalmanProcess();


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

