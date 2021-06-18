/**
 * Chapter 1 contains two applications: linear calibrate and non-linear calibrate
 */
#ifndef CV_MP_CHAP1_H
#define CV_MP_CHAP1_H

#include <iostream>
#include <Eigen/Dense>

struct CameraParameters {
public:
    Eigen::Matrix3d K;
    Eigen::Matrix3d R;
    Eigen::Vector3d t;

    CameraParameters()
            : K(Eigen::Matrix3d::Zero()),
              R(Eigen::Matrix3d::Identity()),
              t(Eigen::Vector3d::Zero()) {
    };
    Eigen::Matrix<double,3,4> getProjectMatrix(){
        Eigen::Matrix<double,3,4> extrinsic_temp_matrix;
        extrinsic_temp_matrix<<R,t;
        return Eigen::Matrix<double,3,4>{K*extrinsic_temp_matrix};
    }
    Eigen::Matrix4d getExtrinsicMatrix(){
        Eigen::Matrix4d results;
        results<<R,t,0,0,0,1;
        return results;
    }
};

/**
 * @brief estimation of camera intrinsic and extrinsic parameters with linear method,
 *        at least need 6 non-coplane correspondences
 * @source Section 1.3.1
 * @param pixel_coordinates: 2 X n pixel coordinates, n must bigger than 6
 * @param real_coordinates: 3 X n real coordinates. Each column is a point [X,Y,Z];
 */
CameraParameters linear_calibrate(const Eigen::Matrix2Xd &pixel_coordinates, const Eigen::Matrix3Xd &real_coordinates);

CameraParameters non_linear_calibrate(const Eigen::Matrix2Xd &pixel_coordinates, const Eigen::Matrix3Xd &real_coordinates);

#endif //CV_MP_CHAP1_H
