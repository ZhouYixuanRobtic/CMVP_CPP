#include "chap1.h"

CameraParameters linear_calibrate(const Eigen::Matrix2Xd &pixel_coordinates, const Eigen::Matrix3Xd &real_coordinates) {
    int min_pairs = (int) std::min(pixel_coordinates.cols(), real_coordinates.cols());
    assert(!(min_pairs < 6));

    /*Estimation Project Matrix*/
    Eigen::MatrixXd P;
    P.resize(2 * min_pairs, 12);
    for (int i = 0; i < min_pairs; ++i) {
        P.row(i + 0) << real_coordinates.col(i).transpose(), 1, 0, 0, 0, 0, -pixel_coordinates.col(i)[0] *
                                                                            real_coordinates.col(
                                                                                    i).transpose(), -pixel_coordinates.col(
                i)[0];
        P.row(i + 1) << 0, 0, 0, 0, real_coordinates.col(i).transpose(), 1, -pixel_coordinates.col(i)[1] *
                                                                            real_coordinates.col(
                                                                                    i).transpose(), -pixel_coordinates.col(
                i)[1];
    }
    Eigen::VectorXd b;
    b.resize(2 * min_pairs, 1);
    b.setConstant(0);
    Eigen::MatrixXd m{P.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b)};
    m.conservativeResize(3, 4);
    /* Estimation  Intrinsic and Extrinsic parameters*/
    Eigen::RowVector3d a1 = m.block<1, 3>(0, 0);
    Eigen::RowVector3d a2 = m.block<1, 3>(1, 0);
    Eigen::RowVector3d a3 = m.block<1, 3>(2, 0);
    //suppose epsilon=-1.0, the world origin is behind the camera
    double rho = -1.0 / a3.norm();
    Eigen::Matrix3d R, K;
    double theta = acos(-(a1.cross(a3).normalized()).dot(a2.cross(a3).normalized()));
    //intrinsic parameter
    K << rho * rho * a1.cross(a3).norm() * sin(theta), -rho * rho * a1.cross(a3).norm() * cos(theta), rho * rho *
                                                                                                      a1.dot(a3),
            0, rho * rho * a2.cross(a3).norm(), rho * rho * a2.dot(a3),
            0, 0, 1;
    //Extrinsic parameter
    R.row(2) = rho * a3;
    R.row(0) = a2.cross(a3).normalized();
    R.row(1) = R.row(2).cross(R.row(0));
    Eigen::Vector3d t;
    t = rho * K.inverse() * (m.block<3, 1>(0, 3));
    return {K, R, t};
}