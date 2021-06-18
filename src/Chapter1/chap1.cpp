#include "chap1.h"

CameraParameters linear_calibrate(const Eigen::Matrix2Xd &pixel_coordinates, const Eigen::Matrix3Xd &real_coordinates) {
    int min_pairs = (int)std::min(pixel_coordinates.cols(), real_coordinates.cols());
    assert(min_pairs > 6);

    /*Estimation Project Matrix*/
    Eigen::MatrixXd P;
    P.resize(2*min_pairs,2*min_pairs);
    for(int i=0;i<min_pairs;++i){
        P.row(i+0)<<real_coordinates.col(i),1,0,0,0,0,-pixel_coordinates.col(i)[0]*real_coordinates.col(i),-pixel_coordinates.col(i)[0];
        P.row(i+1)<<0,0,0,0,real_coordinates.col(i),1,-pixel_coordinates.col(i)[1]*real_coordinates.col(i),-pixel_coordinates.col(i)[1];
    }
    Eigen::VectorXd b;
    b.resize(2*min_pairs,1);
    b.setConstant(0);
    Eigen::MatrixXd m{P.jacobiSvd(Eigen::ComputeThinU|Eigen::ComputeThinV).solve(b)};
    m.conservativeResize(3,4);


}

void non_linear_calibrate() {
    std::cout << "check I'm in chap1 a" << std::endl;
}