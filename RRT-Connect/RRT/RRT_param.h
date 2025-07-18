#include <Eigen/Dense>

#pragma once
struct MPPIParam {
    float dt;
    int T;
    int N;
    double gamma_u;
    Eigen::MatrixXd sigma_u;

    Eigen::VectorXd x_init;
    Eigen::VectorXd x_target;
};

struct SmoothMPPIParam{
    double dt;
    double lambda;
    Eigen::MatrixXd w;
};

struct BiMPPIParam {
    float dt;
    int Tf;
    int Tb;
    Eigen::VectorXd x_init;
    Eigen::VectorXd x_target;
    int Nf;
    int Nb;
    int Nr;
    double gamma_u;
    Eigen::MatrixXd sigma_u;
    double deviation_mu;
    double cost_mu;
    int minpts;
    double epsilon;
    double psi;
};

struct RRTConParam {
    double      step_size      = 0.1;       // 확장 길이
    int         max_iter       = 5000;      // 최대 반복
    double      goal_thresh    = 0.15;      // 두 트리가 연결됐다고 판단할 거리
    double      goal_bias      = 0.05;      // goal-biased sampling 확률
    Eigen::VectorXd  min_bounds;            // 공간 최소 좌표
    Eigen::VectorXd  max_bounds;            // 공간 최대 좌표
    Eigen::VectorXd  q_start;               // 시작 상태
    Eigen::VectorXd  q_goal;                // 목표 상태



    float dt;
    int T;
    Eigen::VectorXd x_init;
    Eigen::VectorXd x_target;
    int N;
    double gamma_u;
    Eigen::MatrixXd sigma_u;
};