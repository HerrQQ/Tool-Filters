#ifndef Swa_OffsetPara_h
#define Swa_OffsetPara_h

#define SWA_INVALID (0u)
#define SWA_VALID (1u)
#define SWA_MaxUINT16   (65535u)
#define SWA_CycleTime_s (0.01f)
//Swa_InputFilter
#define SWA_FilterFactor_05      (0.5f)
#define SWA_FilterFactor_02      (0.2f)
#define SWA_FilterFactor_08      (0.8f)
#define SWA_FilterFactor_99      (0.99f)
#define SWA_FilterFactor_01      (0.01f)
#define SWA_FilterFactor_20      (0.20f)
#define SWA_FilterFactor_80      (0.80f)
#define SWA_VehicleSpeed_MinBoundary (0.f)
#define SWA_VehicleSpeed_MaxBoundary (300.f)
#define SWA_YawRate_MinBoundary (-300.f)
#define SWA_YawRate_MaxBoundary (300.f)
#define SWA_LateralAcceleration_MinBoundary (-6.f)
#define SWA_LateralAcceleration_MaxBoundary (6.f)
#define SWA_SteeringWheelAngle_MinBoundary (-780.f)
#define SWA_SteeringWheelAngle_MaxBoundary (780.f)
#define SWA_SteerWheelTorque_MinBoundary (-10.24f)
#define SWA_SteerWheelTorque_MaxBoundary (10.23f)
#define SWA_SASSteeringWheelSpeed_MinBoundary (0.f)
#define SWA_SASSteeringWheelSpeed_MaxBoundary (1016.f)
#define SWA_ActualRackPosition_MinBoundary (-87.96f)
#define SWA_ActualRackPosition_MaxBoundary (87.96f)
//Swa_OffsetCompEn
#define P_SWA_VehicleSpeed_kmph      (30.f)//60
#define P_SWA_VehicleSpeed_kmph_Rack      (10.f)
#define P_SWA_YawRate_degs      (0.3f)//0.3
#define P_SWA_YawRate_degs_Rack (0.3f)
#define P_SWA_SteerWheelTorque_nm      (0.5f)//0.3
#define P_SWA_SteerWheelTorque_nm_Rack      (0.5f)//0.3
#define P_SWA_SteeringWheelAngle_deg      (5.f)//3
#define P_SWA_SteeringWheelAngle_deg_Rack      (5.f)//3
#define P_SWA_DeltaWheelSpeed_kmph      (1.8f)
#define P_SWA_LateralAcc_mps2      (0.03f)//0.03
#define P_SWA_YawRateGradient_degps2      (1u)
#define P_SWA_SteeringWheelSpeed_degps      (80.f)  //this threshold value need check
#define P_SWA_SteeringWheelSpeed_degps_Rack (3.f)
#define P_SWA_SteeringWheelAngle2MiniMeter (360/58.64)
//Swa_OffsetCompSaturation
#define P_SWA_MaxDeltaS0_deg      (5.f)//3
#define P_SWA_LengthProRack      (0.0625f)//in mm
#define P_SWA_MinDeltaS0_deg      (0.3f)
#define P_SWA_MinDeltaS0_close_deg (0.001f)
#define P_SWA_MinDeltaS0_degRack (0.05f)
#define P_SWA_MinDeltaS0_close_degRack (0.001f)
#define P_SWA_DetaN_enum          (2500u)
#define P_SWA_Rack_DetaN_enum          (1000u)//temp 2000
#define SWA_ENABLECNT_STEP1         (10u)    //enable
#define SWA_OffsetCompStatus_Disable     (0u)
#define SWA_OffsetCompStatus_Enable      (1u)
#define SWA_OffsetCompStatus_Output      (2u)
#define SWA_ADCMRequestStatue_active     (2u)
#define SWA_ADCMRequestStatue_overide     (3u)
#endif
