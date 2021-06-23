#include "Chapter1/chap1.h"
#include <gtest/gtest.h>

TEST(Chap1Test, linear_test
) {
    Eigen::Matrix2Xd pixel_coordinates;
    pixel_coordinates.resize(2, 7);
    pixel_coordinates << 1, 2, 3, 4, 5, 6, 7, 7, 6, 5, 4, 3, 2, 1;
    Eigen::Matrix3Xd real_coordinates;
    real_coordinates.resize(3, 7);
    real_coordinates << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7,
            0.3, 0.4, 0.7, 0.9, 0.10, 0.34, 0.32,
            0.1, 0.234, 0.345, 0.534, 0.634, 0.657, 0.574;
    auto a = linear_calibrate(pixel_coordinates, real_coordinates);
    std::cout << "K " << a.getIntrinsicMatrix().matrix() << std::endl;

    std::cout << "T" << a.getExtrinsicMatrix().matrix() << std::endl;
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}