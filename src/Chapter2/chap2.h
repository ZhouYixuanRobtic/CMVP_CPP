/**
 * @brief Chapter 2 contains application: photometric stereo
 * @author YX.E.Z
 */
#ifndef CV_MP_CHAP2_H
#define CV_MP_CHAP2_H

#include <iostream>
#include <vector>
#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyData.h>
#include <vtkPLYWriter.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkImageViewer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleImage.h>
#include <vtkLight.h>
#include <vtkLightCollection.h>
#include <vtkRenderer.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkFloatArray.h>
#include <vtkTriangle.h>
#include <vtkAutoInit.h>

VTK_MODULE_INIT(vtkRenderingOpenGL);

VTK_MODULE_INIT(vtkInteractionStyle);

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

class PHOTOMETRIC_STEREO {
private:
    const int THRESH = 254;

    int HEIGHT{}, WIDTH{};
    int NUM_IMAGES{};
    const std::string CALIBRATION;

    std::vector<cv::Mat> modelImages;

    cv::Mat LightsInv;

    cv::Mat Normals, Pgrads, Qgrads;

    cv::Vec3f getLightDirecFromSphere(const cv::Mat &calib, const cv::Rect &bb) const;


    static Eigen::MatrixXf
    pseudoInverse(const Eigen::MatrixXf &a, double epsilon = std::numeric_limits<double>::epsilon()) {

        Eigen::JacobiSVD<Eigen::MatrixXf> svd(a, Eigen::ComputeFullU | Eigen::ComputeFullV);
        double tolerance = epsilon * std::max(a.cols(), a.rows()) * svd.singularValues().array().abs()(0);
        return svd.matrixV() *
               (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(),
                                                                       0).matrix().asDiagonal() *
               svd.matrixU().adjoint();
    }

public:
    explicit PHOTOMETRIC_STEREO(std::string calibration_base_path)
            : CALIBRATION(std::move(calibration_base_path)) {

    }

    ~PHOTOMETRIC_STEREO() = default;

    void addModel(const std::string &model_path, int num_images);

    cv::Mat getNormalMap();

    cv::Mat getMesh();

    void displayMesh(const cv::Mat &Z) const;
};

#endif //CV_MP_CHAP2_H
