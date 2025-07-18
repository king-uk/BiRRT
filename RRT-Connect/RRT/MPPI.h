#pragma once

#include <vector>
#include <functional>
#include <random>
#include <chrono>
#include <omp.h>

#include <Eigen/Dense>
#include <EigenRand/EigenRand>

#include "RRT_param.h"           // MPPIParam 정의
#include "collision_checker.h"
#include "model_base.h"

class MPPI {
public:
    template<typename ModelClass>
    MPPI(const ModelClass& model);
    ~MPPI() = default;

    void init(const MPPIParam& param);
    void setReferencePath(const std::vector<Eigen::VectorXd>& path);
    void setCollisionChecker(CollisionChecker* checker);
    void guideMPPI();
    void move();
    void step(); // Combines one MPPI iteration and a move step

    // Outputs
    Eigen::MatrixXd Uo;        // optimal control (dim_u × T)
    Eigen::MatrixXd Xo;        // state trajectory (dim_x × (T+1))
    Eigen::VectorXd u0;        // current input (dim_u)
    double cost    = 0.0;      // cost of best rollout
    double elapsed = 0.0;      // move() execution time

    // timing
    std::chrono::time_point<std::chrono::high_resolution_clock> start, finish;

protected:
    // dimensions & functions
    int dim_x = 0;
    int dim_u = 0;
    float dt = 0.0f;
    int T = 0;      // horizon length
    int N = 0;      // rollout count
    double gamma_u  = 1.0;
    Eigen::MatrixXd sigma_u;

    std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)> f;
    std::function<double(const Eigen::VectorXd&, const Eigen::VectorXd&)>            p;
    std::function<void(Eigen::Ref<Eigen::MatrixXd>)>                                h;

    // reference path
    std::vector<Eigen::VectorXd>  ref_path;
    Eigen::MatrixXd               ref_mat;  // dim_x × (T+1)

    // rollout storage
    Eigen::MatrixXd  Ur;
    Eigen::MatrixXd  Xr;
    double           Cr;

    // collision checker
    CollisionChecker*             collision_checker = nullptr;

    // noise generator
    Eigen::Rand::NormalGen<double> norm_gen{0.0, 1.0};
};

// --- Definitions below ---

template<typename ModelClass>
MPPI::MPPI(const ModelClass& model)
: dim_x(model.dim_x),
  dim_u(model.dim_u),
  f(model.f),
  p(model.p),
  h(model.h)
{}

inline void MPPI::init(const MPPIParam& param) {
    dt      = param.dt;
    T       = param.T;
    N       = param.N;
    gamma_u = param.gamma_u;
    sigma_u = param.sigma_u;

    Uo = Eigen::MatrixXd::Zero(dim_u, T);
    Xo = Eigen::MatrixXd::Zero(dim_x, T + 1);
    if (!ref_path.empty()) {
        Xo.col(0) = ref_path.front();
    }
    u0 = Eigen::VectorXd::Zero(dim_u);

    Ur = Eigen::MatrixXd::Zero(dim_u, T);
}

inline void MPPI::setReferencePath(const std::vector<Eigen::VectorXd>& path) {
    ref_path = path;
    int K = static_cast<int>(path.size());
    ref_mat.resize(dim_x, K);
    for (int i = 0; i < K; ++i) {
        ref_mat.col(i) = path[i];
    }
}

inline void MPPI::setCollisionChecker(CollisionChecker* checker) {
    collision_checker = checker;
}

inline void MPPI::guideMPPI() {
    int Tr = T;
    Eigen::MatrixXd Ui = Eigen::MatrixXd::Zero(dim_u * N, Tr);
    Eigen::VectorXd costs = Eigen::VectorXd::Zero(N);
    Eigen::VectorXd weights(N);

    #pragma omp parallel
    {
        std::mt19937_64 rng(std::random_device{}());
        #pragma omp for
        for (int i = 0; i < N; ++i) {
            // 노이즈 추가 및 초기 guess 반영
            Eigen::MatrixXd noise = sigma_u * norm_gen.generate<Eigen::MatrixXd>(dim_u, Tr, rng);
            Ui.middleRows(i * dim_u, dim_u) += noise + Ur;
            h(Ui.middleRows(i * dim_u, dim_u));

            // 순방향 시뮬레이션
            Eigen::MatrixXd Xi(dim_x, Tr + 1);
            Xi.col(0) = ref_mat.col(0);
            double cost_roll = 0.0;
            for (int t = 0; t < Tr; ++t) {
                cost_roll += p(Xi.col(t), ref_mat.rightCols(1));
                Xi.col(t+1) = Xi.col(t) + dt * f(Xi.col(t), Ui.block(i * dim_u, t, dim_u, 1));
            }
            cost_roll += p(Xi.col(Tr), ref_mat.rightCols(1));

            // 궤적 추종 페널티
            cost_roll += (Xi - ref_mat).colwise().norm().sum();

            // 충돌 페널티
            for (int t = 0; t <= Tr; ++t) {
                if (collision_checker && collision_checker->getCollisionGrid(Xi.col(t))) {
                    cost_roll = 1e8;
                    break;
                }
            }
            costs(i) = cost_roll;
        }
    }

    // 소프트맥스 가중치
    double cmin = costs.minCoeff();
    weights = (-(costs.array() - cmin) * gamma_u).exp();
    weights /= weights.sum();

    // 제어 업데이트
    Eigen::MatrixXd Ures = Eigen::MatrixXd::Zero(dim_u, Tr);
    for (int i = 0; i < N; ++i) {
        Ures += Ui.middleRows(i * dim_u, dim_u) * weights(i);
    }
    h(Ures);

    // 최종 롤아웃 및 저장
    Eigen::MatrixXd Xi(dim_x, Tr + 1);
    Xi.col(0) = ref_mat.col(0);
    double finalCost = 0.0;
    for (int t = 0; t < Tr; ++t) {
        finalCost += p(Xi.col(t), ref_mat.rightCols(1));
        Xi.col(t+1) = Xi.col(t) + dt * f(Xi.col(t), Ures.col(t));
    }
    finalCost += p(Xi.col(Tr), ref_mat.rightCols(1));
    for (int t = 0; t <= Tr; ++t) {
        if (collision_checker && collision_checker->getCollisionGrid(Xi.col(t))) {
            finalCost = 1e8;
            break;
        }
    }

    // 저장 및 public output 업데이트
    Xr = Xi;
    Cr = finalCost;
    Ur = Ures;
    u0 = Ures.col(0);

    Uo = Ur;
    Xo = Xr;
    cost = Cr;
}

inline void MPPI::move() {
    start = std::chrono::high_resolution_clock::now();
    Eigen::VectorXd x_cur  = Xo.col(0);
    Eigen::VectorXd x_next = x_cur + dt * f(x_cur, u0);

    // 제어 시퀀스 shift
    Uo.leftCols(T-1) = Uo.rightCols(T-1);
    Uo.col(T-1).setZero();

    // 상태 궤적 shift
    Xo.leftCols(T) = Xo.rightCols(T);
    Xo.col(T) = x_next;

    // 다음 루프의 초기 guess 로 Ur 역시 shift된 Uo 사용
    Ur = Uo;

    finish = std::chrono::high_resolution_clock::now();
    elapsed = std::chrono::duration<double>(finish - start).count();
}

inline void MPPI::step() {
    guideMPPI();
    move();
}
