//#include <chrono>
#pragma once

#include "Eigen/Core"
#include <Eigen/Dense>
#include <memory>
#include <vector>



struct Observation{
  // 1. 观测量->Zk
  Eigen::VectorXd observations;
  // 2. 转换矩阵 Transformation ->H
  Eigen::MatrixXd transformMatrix;
  // 3. 观测协方差->Rk
  Eigen::MatrixXd observationNoise;
};

class KalmanFilter{
private:
  // 1. 状态量->Xk
  Eigen::VectorXd states;
  // 2. 状态转移矩阵->A
  Eigen::MatrixXd transitionMatrix;
public:
  // 3. 输入向量
  Eigen::VectorXd inputs;
private:
  // 4. 输入矩阵->B
  Eigen::MatrixXd inputMatrix;
  // 5. 状态量协方差Pk
  Eigen::MatrixXd statesCovariance;
  // 6. 预测协方差->Qk
  Eigen::MatrixXd predictNoise;
public:
  // 7. 观测量向量
  std::vector<Observation> observations ;
private:
  // 8. 观测量个数
  size_t observation_num = 0;
  // 9. 离散时间步长(default,0.02)
  double timeStep = 0.02; 
  // 10. 是否存在控制输入
  bool if_use_input = false;

public:  
  // 初始化状态量x，状态转移矩阵A，输入向量u，输入矩阵B，初始状态量协方差Pk，预测噪声Qk
  void initialize_states(const Eigen::VectorXd* init_states, 
                         const Eigen::MatrixXd* transition_matrix, 
                         const Eigen::MatrixXd* input_vector,
                         const Eigen::MatrixXd* input_matrix,
                         const Eigen::MatrixXd* init_states_covariance,
                         const Eigen::MatrixXd* predict_noise);
  void initialize_states(const Eigen::VectorXd* init_states, 
                         const Eigen::MatrixXd* transition_matrix, 
                         const Eigen::MatrixXd* init_states_covariance,
                         const Eigen::MatrixXd* predict_noise);
  // 设置预测协方差
  void setPredictNoise(Eigen::MatrixXd& covariance);
  // 初始化观测量Zk, 转换矩阵Hk, 观测噪声Rk
  void initialize_observations(const std::vector<Observation>* observations_ptr);
  // 更新观测量Zk
  void update_observations();
  // 增加观测量
  void observations_increment(Observation& observation);
  // 滤波
  void filter();
  // 获取状态量Xk
  Eigen::VectorXd getStates()const;
  // 设置时间步长
  void setTimeStep(double step);
};

