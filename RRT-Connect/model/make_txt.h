#pragma once
#include <Eigen/Dense>
#include <fstream>
#include <iomanip>
#include <string>

/*------------------------------------------------------------
 * saveVectorTxt
 *   vec   : 저장할 Eigen::VectorXd
 *   path  : 출력 파일 경로 (예: "traj/x_init.txt")
 *   delim : 요소 사이 구분자(기본 = ' ')
 *   prec  : 소수점 자릿수(기본 = 8)
 *-----------------------------------------------------------*/
inline bool saveVectorTxt(const Eigen::VectorXd& vec,
                          const std::string&    path,
                          char                  delim = ' ',
                          int                   prec  = 8)
{
    std::ofstream fout(path, std::ios::out);
    if (!fout.is_open()) return false;

    fout << std::fixed << std::setprecision(prec);
    for (int i = 0; i < vec.size(); ++i) {
        fout << vec(i);
        if (i + 1 != vec.size()) fout << delim;
    }
    fout << '\n';
    return true;                 // 성공 시 true, 실패 시 false
}
