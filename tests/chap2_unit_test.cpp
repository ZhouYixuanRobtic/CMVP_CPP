#include "Chapter2/chap2.h"
#include <gtest/gtest.h>

TEST(Chap2Test, photometric_test) {
    const int NUM_IMGS{12};
    const std::string CALIBRATION{"../../dataset/Chapter2/chrome/chrome."};
    const std::vector<std::string> MODEL_LIST{"buddha", "cat", "gray", "horse", "owl", "rock"};
    const std::string MODEL{"../../dataset/Chapter2/rock/rock."};

    PHOTOMETRIC_STEREO instance{CALIBRATION};

    instance.addModel(MODEL, NUM_IMGS);

    auto normal_map = instance.getNormalMap();
    auto mesh = instance.getMesh();

    cv::imshow("Normal Map", normal_map);
    cv::waitKey(0);
    instance.displayMesh(mesh);

}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
