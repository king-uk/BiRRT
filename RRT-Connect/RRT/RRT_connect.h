#pragma once

#include <EigenRand/EigenRand>
#include "matplotlibcpp.h"
#include "RRT_param.h"          // RRTConParam
#include "collision_checker.h"
#include "model_base.h"

#include <random>
#include <vector>
#include <chrono>
#include <iostream>
#include <cmath>

#include <omp.h>

class BiRRT {
public:
    BiRRT() = default;
    ~BiRRT() = default;

    void init(const RRTConParam& param);
    bool solve();          // plan (re‑plan if path finished)
    void setCollisionChecker(CollisionChecker* checker) { collision_checker = checker; }
    void showTraj() const;

    // Return the found path (sequence of states)
    std::vector<Eigen::VectorXd> getPath();

    // Timing and iteration stats
    double elapsed = 0.0;   // total elapsed time (seconds)
    int iteration = 0;      // number of iterations performed

private:
    struct Node {
        Eigen::VectorXd state;
        int parent;
        Node(const Eigen::VectorXd& q, int p = -1) : state(q), parent(p) {}
    };

    using Tree = std::vector<Node>;
    enum class ExtendStatus { Trapped, Advanced, Reached };

    // Core RRT operations
    Eigen::VectorXd sampleRandom();
    int nearest(const Tree& T, const Eigen::VectorXd& q) const;
    Eigen::VectorXd steer(const Eigen::VectorXd& from, const Eigen::VectorXd& to) const;
    bool collisionFree(const Eigen::VectorXd& from, const Eigen::VectorXd& to) const;
    ExtendStatus extend(Tree& T, const Eigen::VectorXd& target);
    ExtendStatus connect(Tree& T, const Eigen::VectorXd& target);
    std::vector<Eigen::VectorXd> tracePath(const Tree& Ta, int idxA, const Tree& Tb, int idxB) const;

    // Problem data
    int dim;
    RRTConParam param;
    Tree tree_start, tree_goal;
    std::vector<Eigen::VectorXd> path;

    // RNG
    std::mt19937_64 rng{static_cast<std::uint_fast64_t>(std::time(nullptr))};
    std::uniform_real_distribution<double> unif{0.0, 1.0};
    CollisionChecker* collision_checker = nullptr;

    // Parameters
    double step_size = 0.0;
    int max_iter = 0;
    double goal_thresh = 0.0;
};

// Definitions

inline void BiRRT::init(const RRTConParam& p) {
    param = p;
    dim = static_cast<int>(p.q_start.size());   
    tree_start = { Node(p.q_start) };
    tree_goal  = { Node(p.q_goal)  };
    path.clear();

    step_size  = p.step_size;
    max_iter   = p.max_iter;
    goal_thresh = p.goal_thresh;

    elapsed = 0.0;
    iteration = 0;
}

inline std::vector<Eigen::VectorXd> BiRRT::getPath() {
    return path;
}

inline Eigen::VectorXd BiRRT::sampleRandom() {
    Eigen::VectorXd span = param.max_bounds - param.min_bounds;
    Eigen::VectorXd rnd  = Eigen::Rand::uniformReal<Eigen::VectorXd>(dim, 1, rng);
    return param.min_bounds + span.cwiseProduct(rnd);
}

inline int BiRRT::nearest(const Tree& T, const Eigen::VectorXd& q) const {
    double best = std::numeric_limits<double>::infinity();
    int idx = -1;
    for (int i = 0; i < (int)T.size(); ++i) {
        double d = (T[i].state - q).squaredNorm();
        if (d < best) { best = d; idx = i; }
    }
    return idx;
}

inline Eigen::VectorXd BiRRT::steer(const Eigen::VectorXd& from, const Eigen::VectorXd& to) const {
    Eigen::VectorXd dir = to - from;
    double dist = dir.norm();
    return (dist <= step_size) ? to : (from + step_size * dir / dist);
}

inline bool BiRRT::collisionFree(const Eigen::VectorXd& from, const Eigen::VectorXd& to) const {
    int K = static_cast<int>(std::ceil((to - from).norm() / (step_size * 0.5)));
    for (int k = 0; k <= K; ++k) {
        double a = double(k) / K;
        Eigen::VectorXd q = (1 - a) * from + a * to;
        if (collision_checker && collision_checker->getCollisionGrid(q))
            return false;
    }
    return true;
}

inline BiRRT::ExtendStatus BiRRT::extend(Tree& T, const Eigen::VectorXd& target) {
    int idx = nearest(T, target);
    Eigen::VectorXd q_new = steer(T[idx].state, target);
    if (!collisionFree(T[idx].state, q_new)) return ExtendStatus::Trapped;
    T.emplace_back(q_new, idx);
    return (q_new - target).norm() < goal_thresh ? ExtendStatus::Reached
                                                 : ExtendStatus::Advanced;
}

inline BiRRT::ExtendStatus BiRRT::connect(Tree& T, const Eigen::VectorXd& target) {
    return extend(T, target);
}

// ------------------- connect primitive ---------------
// inline BiRRT::ExtendStatus BiRRT::connect(Tree& T, const Eigen::VectorXd& q_target) {
//     ExtendStatus s;
//     do { 
//         s = extend(T, q_target);
//         // ++iteration;   // RRT-connect는 iteration을 세지 않았다..... 계속 확장함
//     } while (s == ExtendStatus::Advanced);
//     return s;
// }




inline std::vector<Eigen::VectorXd> BiRRT::tracePath(
    const Tree& Ta, int idxA, const Tree& Tb, int idxB) const {
    std::vector<Eigen::VectorXd> pa, pb;
    for (int i = idxA; i != -1; i = Ta[i].parent)
        pa.push_back(Ta[i].state);
    for (int i = idxB; i != -1; i = Tb[i].parent)
        pb.push_back(Tb[i].state);
    std::reverse(pa.begin(), pa.end());
    pa.insert(pa.end(), pb.begin() + 1, pb.end());
    return pa;
}

inline bool BiRRT::solve() {
    auto t0 = std::chrono::high_resolution_clock::now();
    path.clear();
    iteration = 0;
    Tree *Ta = &tree_start, *Tb = &tree_goal;
    bool solved = false;

    for (int iter = 0; iter < max_iter && !solved; ++iter) {
        Eigen::VectorXd q_rand = sampleRandom();
        if (extend(*Ta, q_rand) != ExtendStatus::Trapped) {
            Eigen::VectorXd q_new = Ta->back().state;
            if (connect(*Tb, q_new) == ExtendStatus::Reached) {
                path = tracePath(
                    (Ta == &tree_start ? *Ta : *Tb),
                    (Ta == &tree_start ? (int)Ta->size() - 1 : (int)Tb->size() - 1),
                    (Tb == &tree_goal  ? *Tb : *Ta),
                    (Tb == &tree_goal  ? (int)Tb->size() - 1 : (int)Ta->size() - 1)
                );
                solved = true;
            }
        }
        std::swap(Ta, Tb);
        iteration = iter + 1;
    }

    elapsed = std::chrono::duration<double>(
        std::chrono::high_resolution_clock::now() - t0).count();

    if (!solved) {
        path.clear();
        path.push_back(param.q_start);
    }
    return solved;
}

inline void BiRRT::showTraj() const {
    namespace plt = matplotlibcpp;
    double res = 0.1, hl = res / 2.0;
    // Draw obstacles
    for (int i = 0; i < collision_checker->map.size(); ++i) {
        for (int j = 0; j < collision_checker->map[0].size(); ++j) {
            if (collision_checker->map[i][j] == 10) {
                double mx = i * res, my = j * res;
                std::vector<double> oX = {mx-hl, mx+hl, mx+hl, mx-hl, mx-hl};
                std::vector<double> oY = {my-hl, my-hl, my+hl, my+hl, my-hl};
                plt::plot(oX, oY, "k-");
            }
        }
    }
    // Draw trees
    auto drawTree = [&](const Tree& T, const std::string& color) {
        for (size_t idx = 1; idx < T.size(); ++idx) {
            auto& c = T[idx].state; auto& p = T[T[idx].parent].state;
            plt::plot(std::vector<double>{p(0), c(0)}, std::vector<double>{p(1), c(1)}, color);
        }
    };
    drawTree(tree_start, "r-");
    drawTree(tree_goal,  "g-");
    // Draw path
    if (!path.empty()) {
        std::vector<double> xs, ys;
        for (auto& q : path) { xs.push_back(q(0)); ys.push_back(q(1)); }
        plt::plot(xs, ys, { {"color","blue"}, {"linewidth","1.0"} });
    }
    plt::xlim(0.0, collision_checker->map[0].size() * res);
    plt::ylim(0.0, collision_checker->map.size()    * res);
    plt::axis("equal"); plt::grid(true); plt::show();
}
