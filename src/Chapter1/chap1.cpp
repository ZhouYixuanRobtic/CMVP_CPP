#include "chap1.h"
#include <Eigen/Core>

void linear_calibrate() {
    Eigen::Vector3d test_vector{1, 2, 3};
    std::cout << "check I'm in chap1" << test_vector << std::endl;
}

void non_linear_calibrate() {
    std::cout << "check I'm in chap1 a" << std::endl;
}