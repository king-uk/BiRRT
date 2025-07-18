#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <vector>            // std::vector
#include <Eigen/Dense>      // Eigen::VectorXd

bool savePathTxt(const std::vector<Eigen::VectorXd>& path,
                 const std::string& filename)
{
    std::ofstream ofs(filename);
    if (!ofs.is_open()) {                 // 파일 열기 실패
        std::cerr << "Cannot open " << filename << '\n';
        return false;
    }

    ofs << std::fixed << std::setprecision(6);    // 소수 6자리 고정

    for (const auto& p : path) {                  // 한 스텝씩
        for (int i = 0; i < p.size(); ++i) {
            ofs << p[i];
            if (i + 1 < p.size()) ofs << ' ';     // 요소 구분 공백
        }
        ofs << '\n';                              // 다음 스텝은 새 줄
    }
    return true;
}