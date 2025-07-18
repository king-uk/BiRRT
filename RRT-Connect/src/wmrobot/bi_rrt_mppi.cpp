#include "wmrobot_map.h"        // 모델 정의
#include "RRT_connect.h"        // BiRRT
#include "collision_checker.h"
#include "MPPI.h"               // MPPI 클래스 :contentReference[oaicite:0]{index=0}

#include <Eigen/Dense>
#include <vector>               // std::vector 사용
#include <array>
#include <chrono>
#include <iostream>
#include <string>

int main()
{
    // 1. RRT 파라미터 설정
    RRTConParam prm;
    prm.q_goal      = Eigen::Vector2d(1.5, 5.0);
    prm.min_bounds  = Eigen::Vector2d(0.0, 0.0);
    prm.max_bounds  = Eigen::Vector2d(3.0, 5.0);
    prm.step_size   = 0.05;
    prm.goal_thresh = 0.05;
    prm.max_iter    = 5000;

    WMRobotMap model;

    // MPPI 기본 파라미터 설정 (dt, N, gamma_u, sigma_u)
    MPPIParam mppi_param;
    mppi_param.dt      = 0.1f;      // 샘플 타임
    mppi_param.N       = 3000;      // 롤아웃 개수
    mppi_param.gamma_u = 10.0;      // 가중치 스케일
    Eigen::VectorXd sigma_u(model.dim_u);
    sigma_u << 0.25, 0.25;      // 모델에 맞게 dim_u(2) 길이 지정
    mppi_param.sigma_u = sigma_u.asDiagonal();

    std::array<double,3> start_x = {0.5, 2.5, 1.5};

    // 2. 시작 위치 루프
    for (int s = 0; s < 3; ++s) {
        prm.q_start = Eigen::Vector2d(start_x[s], 0.0);

        // 3. BARN map 루프
        for (int map = 299; map >= 0; --map) {
            CollisionChecker cc;
            cc.loadMap(
              "../BARN_dataset/txt_files/output_" + std::to_string(map) + ".txt",
              0.1);

            // --- RRT 실행 ---
            BiRRT planner;
            planner.init(prm);
            planner.setCollisionChecker(&cc);
            auto t0 = std::chrono::high_resolution_clock::now();
            bool solved = planner.solve();
            auto t1 = std::chrono::high_resolution_clock::now();
            double elapsed_rrt = std::chrono::duration<double>(t1-t0).count();

            bool is_success = false, is_collision = false;
            std::vector<Eigen::VectorXd> path;

            if (solved) {
                path = planner.getPath();  // RRT가 찾은 경로
                for (auto& q : path) {
                    if (cc.getCollisionGrid(q)) {
                        is_collision = true;
                        break;
                    }
                }
                is_success = !is_collision &&
                             ((path.back() - prm.q_goal).norm() < prm.goal_thresh);
            }

            std::cout << s << '\t' << map << '\t'
                      << is_success << '\t'
                      << planner.iteration << '\t'
                      << elapsed_rrt;

            // --- MPPI 반복 실행 (경로 찾기에 성공했을 경우) ---
            if (is_success) {
                // MPPIParam에 초기/목표 상태와 horizon(T) 설정
                mppi_param.T = static_cast<int>(path.size())+1;

                MPPI mppi(model);
                mppi.setReferencePath(path);
                mppi.init(mppi_param);
                mppi.setCollisionChecker(&cc);

                const int max_mppi_iter = 200;
                bool mppi_failed = true;
                double total_elapsed_mppi = 0.0;
                double pos_err = 0.0;
                int it = 0;

                for (; it < max_mppi_iter; ++it) {
                    mppi.step();
                    total_elapsed_mppi += mppi.elapsed;

                    Eigen::VectorXd curr = mppi.Xo.col(0);
                    if (cc.getCollisionGrid(curr)) {
                        // 충돌 발생 시 실패
                        mppi_failed = true;
                        break;
                    }
                    // 위치 오차 계산
                    // pos_err = (curr.head<2>() - path.back().head<2>()).norm();
                    // if (pos_err < prm.goal_thresh) {
                    //     // 충분히 목표에 근접하면 성공
                    //     mppi_failed = false;
                    //     break;
                    // }
                }

                std::cout << '\t'
                          << "MPPI_fail=" << mppi_failed << '\t'
                          << "iter="     << it          << '\t'
                          << "time="     << total_elapsed_mppi << '\t';
                        //   << "err="      << pos_err;
            }

            std::cout << '\n';
        }
    }
    return 0;
}
