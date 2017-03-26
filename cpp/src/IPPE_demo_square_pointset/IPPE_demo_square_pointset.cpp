#include <IPPE/ippe.h>
#include <opencv2/imgproc.hpp>

#include <iostream>

using namespace cv;

double meanSceneDepth(InputArray objectPts, InputArray rvec, InputArray tvec);

int main(int argc, char** argv)
{
    std::cout << "IPPE_demo_square_pointset started." << std::endl
              << std::endl;
    std::cout << "This is a simple demonstration to show how to use IPPE for fast plane-based pose estimation with a square planar pointset, using IPPE::PoseSolver::solveSquareObjectPose. "
                 "This is typically used for marker-based camera pose estimation using AR markers such as ARUCO. "
                 " The object points are defined by 4 corners as follows:"
              << std::endl
              << std::endl
              << " point 0: [-squareLength / 2.0, squareLength / 2.0, 0]" << std::endl
              << " point 1: [squareLength / 2.0, squareLength / 2.0, 0]" << std::endl
              << " point 2: [squareLength / 2.0, -squareLength / 2.0, 0]" << std::endl
              << " point 3: [-squareLength / 2.0, -squareLength / 2.0, 0]" << std::endl
              << " where squareLength denotes the length of the square. Therefore, the square is defined in object coordinates on the plane z=0 and centred at the origin." << std::endl
              << std::endl;

    std::cout << "The demo works similarly to IPPE_Demo_generic_pointset. We use IPPE::PoseSolver::solveSquareObjectPose instead of the generic solver because it exploits"
                 " the structure of the object's shape, which gives a massive speedup. Specifically, the object-to-image homography can be computed with a simple analytic expression. "
                 "This is discussed in the IPPE paper."
              << std::endl
              << std::endl;

    std::cout << "A virtual camera is used to simulate their correspondences in an image. "
                 "Then IPPE is run to estimate two candidate poses. The first pose returned gives the correct pose with zero reprojection error (because there is no noise in this simple demo). "
                 "The second pose gives the alternative pose hypothesis. "
                 "In this demo, we generate object poses at increasing distances to the camera. This causes the reprojection error of the second pose to approach zero, "
                 "because the object projects to smaller and smaller image regions, causing the projection to become more affine. This leads to a two-fold solution ambiguity. "
                 "When the reprojection error of the two poses are similar up to noise uncertainty, the problem is ambiguous and there are two valid pose solutions. "
                 "Please refer to the IPPE paper for a further discussion on this two-fold ambiguity."
              << std::endl;

    //set the length of the square object (in mm, m or whatever units you chose).
    double l = 10;

    cv::Mat objectPoints, imagePoints;
    IPPE::PoseSolver::generateSquareObjectCorners3D(l, objectPoints);

    //setup camera matrix:
    Mat cameraMatrix(3, 3, CV_64FC1);
    cameraMatrix.setTo(0);
    cameraMatrix.at<double>(0, 0) = 500;
    cameraMatrix.at<double>(1, 1) = 500;
    cameraMatrix.at<double>(0, 2) = 320;
    cameraMatrix.at<double>(1, 2) = 240;

    //setup distortion vector:
    cv::Mat distCoeffs(5, 1, CV_64FC1);
    distCoeffs.setTo(0); //There's no distortion in this demo.

    //a ground truth pose:
    cv::Mat rvecGT(3, 1, CV_64FC1);
    cv::Mat tvecGT(3, 1, CV_64FC1);

    rvecGT.at<double>(0) = 0.1;
    rvecGT.at<double>(1) = -0.3;
    rvecGT.at<double>(2) = 0.6;

    tvecGT.at<double>(0) = 5.0;
    tvecGT.at<double>(1) = 10.0;
    tvecGT.at<double>(2) = l * 2;

    //now solve the object's pose at increasing depths:
    bool term = false;
    size_t itLim = 30;
    size_t itCount = 0;

    std::cout << std::endl
              << "Running the demo... " << std::endl;
    cv::Mat rvec1, rvec2, tvec1, tvec2;
    float err1, err2;
    while (term == false) {
        //increase the z-component of translation:
        tvecGT.at<double>(2) = tvecGT.at<double>(2) * 1.2;

        //make image correspondences:
        cv::projectPoints(objectPoints, rvecGT, tvecGT, cameraMatrix, distCoeffs, imagePoints);

        //there are two ways you can call IPPE:
        //   (1) do it directly:
        //   IPPE::PoseSolver::solveSquare(l, imagePoints,cameraMatrix,distCoeffs,rvec1,tvec1,err1,rvec2,tvec2, err2);

        //(2) first undistort the image points, then call solveGeneric. The time taken to run solveGeneric will be less because cv::undistortPoints is not required:
        //The returned RMS errors will be different because for (1) it is computed in pixels and for (2) it is computed in normalized pixels:
        cv::Mat imagePointsud;
        cv::undistortPoints(imagePoints, imagePointsud, cameraMatrix, distCoeffs);
        IPPE::PoseSolver::solveSquare(l, imagePointsud, cv::noArray(), cv::noArray(), rvec1, tvec1, err1, rvec2, tvec2, err2);

        //display results:
        double zBar = IPPE::PoseSolver::meanSceneDepth(objectPoints, rvecGT, tvecGT);
        std::cout << "Mean object depth: " << zBar << ", RMSE reprojection error of returned poses: (" << err1 << ", " << err2 << ")" << std::endl;
        itCount++;
        if (itCount == itLim) {
            term = true;
        }
    }
    std::cout << "Finished" << std::endl;
}
