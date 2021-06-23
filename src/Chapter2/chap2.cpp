
#include "chap2.h"

Eigen::Vector3f PHOTOMETRIC_STEREO::getLightDirecFromSphere(const cv::Mat &Image, const cv::Rect &boundingbox) const {

    const float radius = boundingbox.width / 2.0f;

    cv::Mat Binary;
    threshold(Image, Binary, THRESH, 255, CV_THRESH_BINARY);
    cv::Mat SubImage(Binary, boundingbox);

    /* calculate center of pixels */
    cv::Moments m = moments(SubImage, false);
    cv::Point center(m.m10 / m.m00, m.m01 / m.m00);

    /* x,y are swapped here */
    float x = (center.y - radius) / radius;
    float y = (center.x - radius) / radius;
    float z = sqrt(1.0 - pow(x, 2.0) - pow(y, 2.0));

    return {x, y, z};
}

void PHOTOMETRIC_STEREO::addModel(const std::string &model_path, int num_images) {
    NUM_IMAGES = num_images;
    cv::Mat Mask = cv::imread(CALIBRATION + "mask.png", CV_LOAD_IMAGE_GRAYSCALE);
    cv::Mat ModelMask = cv::imread(model_path + "mask.png", CV_LOAD_IMAGE_GRAYSCALE);
    std::vector<std::vector<cv::Point>> v;
    cv::findContours(Mask.clone(), v, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
    assert(!v.empty());
    cv::Rect bb = cv::boundingRect(v[0]);
    Eigen::MatrixXf lights_tmp(NUM_IMAGES, 3);
    for (int i = 0; i < num_images; ++i) {
        cv::Mat Calib = cv::imread(CALIBRATION + std::to_string(i) + ".png", CV_LOAD_IMAGE_GRAYSCALE);
        cv::Mat tmp = cv::imread(model_path + std::to_string(i) + ".png", CV_LOAD_IMAGE_GRAYSCALE);

        cv::Mat Model;
        tmp.copyTo(Model, ModelMask);
        auto light = getLightDirecFromSphere(Calib, bb);
        lights_tmp.row(i) = light;

        HEIGHT = Calib.rows;
        WIDTH = Calib.cols;
        modelImages.emplace_back(Model);
    }
    LightsInv = pseudoInverse(lights_tmp);
    Normals = cv::Mat(HEIGHT, WIDTH, CV_32FC3, cv::Scalar::all(0));
    Pgrads = cv::Mat(HEIGHT, WIDTH, CV_32F, cv::Scalar::all(0));
    Qgrads = cv::Mat(HEIGHT, WIDTH, CV_32F, cv::Scalar::all(0));
}

cv::Mat PHOTOMETRIC_STEREO::getNormalMap() {
    for (int x = 0; x < WIDTH; ++x) {
        for (int y = 0; y < HEIGHT; ++y) {

            Eigen::VectorXf I;
            I.resize(NUM_IMAGES);
            for (int i = 0; i < NUM_IMAGES; i++) {
                I[i] = modelImages[i].at<uchar>(cv::Point(x, y));
            }
            Eigen::VectorXf n = LightsInv * I;
            n = n.norm() > 0 ? n.normalized() : n;
            n[2] = n[2] == 0 ? 1.0f : n[2];
            if (I.maxCoeff() < 0) {
                cv::Vec3f nullvec(0.0f, 0.0f, 1.0f);
                Normals.at<cv::Vec3f>(cv::Point(x, y)) = nullvec;
                Pgrads.at<float>(cv::Point(x, y)) = 0.0f;
                Qgrads.at<float>(cv::Point(x, y)) = 0.0f;
            } else {
                Normals.at<cv::Vec3f>(cv::Point(x, y)) = cv::Vec3f{n[0], n[1], n[2]};
                Pgrads.at<float>(cv::Point(x, y)) = n[0] / n[2];
                Qgrads.at<float>(cv::Point(x, y)) = n[1] / n[2];
            }
        }
    }
    cv::Mat normal_map;
    cv::cvtColor(Normals, normal_map, CV_BGR2RGB);
    return normal_map;
}

cv::Mat PHOTOMETRIC_STEREO::getMesh() {
    cv::Mat P(Pgrads.rows, Pgrads.cols, CV_32FC2, cv::Scalar::all(0));
    cv::Mat Q(Pgrads.rows, Pgrads.cols, CV_32FC2, cv::Scalar::all(0));
    cv::Mat Z(Pgrads.rows, Pgrads.cols, CV_32FC2, cv::Scalar::all(0));

    float lambda = 1.0f;
    float mu = 1.0f;

    cv::dft(Pgrads, P, cv::DFT_COMPLEX_OUTPUT);
    cv::dft(Qgrads, Q, cv::DFT_COMPLEX_OUTPUT);
    for (int i = 0; i < Pgrads.rows; i++) {
        for (int j = 0; j < Pgrads.cols; j++) {
            if (i != 0 || j != 0) {
                float u = sin((float) (i * 2 * CV_PI / Pgrads.rows));
                float v = sin((float) (j * 2 * CV_PI / Pgrads.cols));

                float uv = pow(u, 2) + pow(v, 2);
                float d = (1.0f + lambda) * uv + mu * pow(uv, 2);
                Z.at<cv::Vec2f>(i, j)[0] = (u * P.at<cv::Vec2f>(i, j)[1] + v * Q.at<cv::Vec2f>(i, j)[1]) / d;
                Z.at<cv::Vec2f>(i, j)[1] = (-u * P.at<cv::Vec2f>(i, j)[0] - v * Q.at<cv::Vec2f>(i, j)[0]) / d;
            }
        }
    }

    /* setting unknown average HEIGHT to zero */
    Z.at<cv::Vec2f>(0, 0)[0] = 0.0f;
    Z.at<cv::Vec2f>(0, 0)[1] = 0.0f;

    cv::dft(Z, Z, cv::DFT_INVERSE | cv::DFT_SCALE | cv::DFT_REAL_OUTPUT);

    return Z;
}

void PHOTOMETRIC_STEREO::displayMesh(const cv::Mat &Z) const {
    /* creating visualization pipeline which basically looks like this:
     vtkPoints -> vtkPolyData -> vtkPolyDataMapper -> vtkActor -> vtkRenderer */
    vtkSmartPointer <vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer <vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer <vtkPolyDataMapper> modelMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    vtkSmartPointer <vtkActor> modelActor = vtkSmartPointer<vtkActor>::New();
    vtkSmartPointer <vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer <vtkCellArray> vtkTriangles = vtkSmartPointer<vtkCellArray>::New();

    /* insert x,y,z coords */
    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            points->InsertNextPoint(x, y, Z.at<float>(y, x));
        }
    }

    /* setup the connectivity between grid points */
    vtkSmartPointer <vtkTriangle> triangle = vtkSmartPointer<vtkTriangle>::New();
    triangle->GetPointIds()->SetNumberOfIds(3);
    for (int i = 0; i < HEIGHT - 1; i++) {
        for (int j = 0; j < WIDTH - 1; j++) {
            triangle->GetPointIds()->SetId(0, j + (i * WIDTH));
            triangle->GetPointIds()->SetId(1, (i + 1) * WIDTH + j);
            triangle->GetPointIds()->SetId(2, j + (i * WIDTH) + 1);
            vtkTriangles->InsertNextCell(triangle);
            triangle->GetPointIds()->SetId(0, (i + 1) * WIDTH + j);
            triangle->GetPointIds()->SetId(1, (i + 1) * WIDTH + j + 1);
            triangle->GetPointIds()->SetId(2, j + (i * WIDTH) + 1);
            vtkTriangles->InsertNextCell(triangle);
        }
    }
    polyData->SetPoints(points);
    polyData->SetPolys(vtkTriangles);

    /* create two lights */
    vtkSmartPointer <vtkLight> light1 = vtkSmartPointer<vtkLight>::New();
    light1->SetPosition(-1, 1, 1);
    renderer->AddLight(light1);
    vtkSmartPointer <vtkLight> light2 = vtkSmartPointer<vtkLight>::New();
    light2->SetPosition(1, -1, -1);
    renderer->AddLight(light2);

    /* meshlab-ish background */
    modelMapper->SetInputData(polyData);
    renderer->SetBackground(.45, .45, .9);
    renderer->SetBackground2(.0, .0, .0);
    renderer->GradientBackgroundOn();
    vtkSmartPointer <vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    modelActor->SetMapper(modelMapper);

    /* setting some properties to make it look just right */
    modelActor->GetProperty()->SetSpecularColor(1, 1, 1);
    modelActor->GetProperty()->SetAmbient(0.2);
    modelActor->GetProperty()->SetDiffuse(0.2);
    modelActor->GetProperty()->SetInterpolationToPhong();
    modelActor->GetProperty()->SetSpecular(0.8);
    modelActor->GetProperty()->SetSpecularPower(8.0);

    renderer->AddActor(modelActor);
    vtkSmartPointer <vtkRenderWindowInteractor> interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    interactor->SetRenderWindow(renderWindow);

    /* export mesh */
    vtkSmartPointer <vtkPLYWriter> plyExporter = vtkSmartPointer<vtkPLYWriter>::New();
    plyExporter->SetInputData(polyData);
    plyExporter->SetFileName("export.ply");
    plyExporter->SetColorModeToDefault();
    plyExporter->SetArrayName("Colors");
    plyExporter->Update();
    plyExporter->Write();

    /* render mesh */
    renderWindow->Render();
    interactor->Start();
}