#include "wmrobot_map.h"      // 모델 정의
#include "RRT_connect.h"
#include "collision_checker.h"

#include <Eigen/Dense>
#include <chrono>
#include <array>              // ← std::array 사용

#include <Path.h>

int main()
{
    /*-- 1. 파라미터 공통 설정 --*/
    RRTConParam prm;
    prm.q_goal      = Eigen::Vector2d(1.5, 5.0);
    prm.min_bounds  = Eigen::Vector2d(0.0, 0.0);
    prm.max_bounds  = Eigen::Vector2d(3.0, 5.0);
    prm.step_size   = 0.01;
    prm.goal_thresh = 0.01;

    prm.max_iter    = 5000;

    prm.q_start = Eigen::Vector2d(0.2, 0.0);

    /*-- 3. BARN map 루프 --*/
    int map = 269;


    std::array<double,3> start_x = {0.5, 2.5, 1.5};

    for(int s = 0; s < 3; ++s)
    {
        prm.q_start = Eigen::Vector2d(start_x[s], 0.0);

        CollisionChecker cc;
        cc.loadMap("../BARN_dataset/txt_files/output_" + std::to_string(map) + ".txt", 0.1);

        /*---------- 핵심: 템플릿 인자 전달 ----------*/
        BiRRT planner;   // ✔
        planner.init(prm);
        planner.setCollisionChecker(&cc);

        auto t0 = std::chrono::high_resolution_clock::now();
        bool solved = planner.solve();
        auto t1 = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration<double>(t1-t0).count();

        bool is_success = false, is_collision = false;
        if (solved)
        {
            const auto& P = planner.getPath();
            for (const auto& q : P)
                if (cc.getCollisionGrid(q)) { is_collision = true; break; }

            if (!is_collision &&
                (P.back() - prm.q_goal).norm() < prm.goal_thresh)
                is_success = true;
        }

        std::cout  << map << '\t'
                << is_success << '\t' 
                << planner.iteration << '\t'
                << elapsed << '\n';

        std::vector<Eigen::VectorXd> path = planner.getPath();
    
        planner.showTraj();  // 경로 시각화
    }

    return 0;
}