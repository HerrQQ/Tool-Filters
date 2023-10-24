#include"KalmanFilter.h"
#include <iostream>



// 初始化状态量
void KalmanFilter::initialize_states(const Eigen::VectorXd* init_states, 
                                     const Eigen::MatrixXd* transition_matrix, 
                                     const Eigen::MatrixXd* input_vector,
                                     const Eigen::MatrixXd* input_matrix,
                                     const Eigen::MatrixXd* init_states_covariance,
                                     const Eigen::MatrixXd* predict_noise){
    this->states = *init_states;
    this->transitionMatrix = *transition_matrix;
    this->inputs = *input_vector;
    this->inputMatrix = *input_matrix;
    this->statesCovariance = *init_states_covariance;
    this->predictNoise = * predict_noise;
    if_use_input = true;
    return;
}

void KalmanFilter::initialize_states(const Eigen::VectorXd* init_states, 
                                     const Eigen::MatrixXd* transition_matrix, 
                                     const Eigen::MatrixXd* init_states_covariance,
                                     const Eigen::MatrixXd* predict_noise){
    this->states = *init_states;
    this->transitionMatrix = *transition_matrix;
    this->statesCovariance = *init_states_covariance;
    this->predictNoise = * predict_noise;
    if_use_input = false;
    return;
}

// 初始化观测量
void KalmanFilter::initialize_observations(const std::vector<Observation>* observations_ptr){
    observation_num = observations_ptr->size();
    for(size_t i=0; i<observation_num; i++){
        Observation obser_ = observations_ptr->at(i);
        observations.push_back(obser_);
    }
    return;
}

// 增加观测量
void KalmanFilter::observations_increment(Observation& observation){
    observation_num++;
    observations.push_back(observation);
    return;
}

// 更新观测量
void KalmanFilter::update_observations(){
    return;
}

// 滤波
void KalmanFilter::filter(){
    // 1.1 预测
    if(if_use_input){
        // x(k)=A*x(k-1)+B*u(k);
        states = transitionMatrix*states + inputMatrix*inputs;
    }else{
        // x(k)=A*x(k-1);
        states = transitionMatrix*states;
    }
    //std::cout << "P012:" << statesCovariance(0, 0) << statesCovariance(1,1)<< std::endl;
    //std::cout << "A12:" << transitionMatrix(0, 0) << transitionMatrix(1, 1) << std::endl;
    //std::cout << "A12:" << transitionMatrix.transpose()(0, 0) << transitionMatrix.transpose()(1, 1) << std::endl;
    //std::cout << "Q12:" << predictNoise(0, 0) << predictNoise(1, 1) << std::endl;
    // 1.2 预测方差更新P(k)=A*P(k-1)*At+Q(k)
    Eigen::MatrixXd P_ = transitionMatrix*statesCovariance*transitionMatrix.transpose() + predictNoise;
    std::cout <<"P12:"<< P_(0, 0) << P_(1, 1) << std::endl;
    // 2.0 遍历观测量，并融合
    for(size_t i=0; i<observation_num; i++){
        // 2.1 计算Kalmen增益K_=P(k)*Ht*inverse(H*P(k)*Ht+R(k))
        Observation obs_ = observations[i];
        Eigen::MatrixXd tmp_ = obs_.transformMatrix*P_*obs_.transformMatrix.transpose();
        tmp_+=obs_.observationNoise;
        // if(tmp_.isInvertible()){
        tmp_ = tmp_.inverse();
        // }
        Eigen::MatrixXd K_ = P_*obs_.transformMatrix.transpose()*tmp_;
        std::cout << "K12:" << K_(0, 0) << K_(1, 1) << std::endl;
        // 3. 更新状态量方差 P(k)_=P(k) - K_*H(k)*P(k)
        statesCovariance = P_ - K_*obs_.transformMatrix*P_;
        // 4. 更新状态量 x_(k) = x(k) + K_*(z(k) - H(k)*x(k))
        states = states + K_*(obs_.observations - obs_.transformMatrix*states);

        std::cout << "Xout:" << states[0] << states[1] << std::endl;
    }
    return;
}

Eigen::VectorXd KalmanFilter::getStates()const{
    return states;
}

void KalmanFilter::setTimeStep(double step){
    this->timeStep = step;
    return;
}

// 设置预测协方差
void KalmanFilter::setPredictNoise(Eigen::MatrixXd& covariance){
    this->predictNoise = covariance;
}

