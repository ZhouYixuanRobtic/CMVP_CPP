#include "Chapter1/chap1.h"
#include <gtest/gtest.h>

TEST(Chap1Test, linear_test
){
linear_calibrate();

}

TEST(Chap1Test, non_linear_test
){
non_linear_calibrate();

}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}