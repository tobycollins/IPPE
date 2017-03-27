#include <IPPE/ippe.h>
#include <opencv2/imgproc.hpp>

#include <iostream>

using namespace cv;

double meanSceneDepth(InputArray objectPts, InputArray rvec, InputArray tvec);

int main(int argc, char** argv)
{
    std::cout << "IPPE_demo_generic_pointset started." << std::endl;
    std::cout << "This is a simple demonstration to show how to use IPPE for fast plane-based pose estimation with a generic planar pointset, using IPPE::PoseSolver::solve"
                 "An object is setup consisting of a random set of n=10 object points."
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

    //////////////////////////////////////////////////////
    ///setup the object points:
    //number of points:
    size_t n = 10;

    Mat objectPoints(1, n, CV_64FC3); //points in 3D object coordinates
    Mat imagePoints; //points in image coordinates (pixels)

    //set the scale of the object points to 10 units.
    float X = 10;

    //define the object points on a 3D plane given by a rotation vector rObj. Set this to (0,0,0) if you want the object points to be defined on the plane z=0 in object coordinates.
    Mat rObj(3, 1, CV_64FC1);
    rObj.at<double>(0) = 0.2;
    rObj.at<double>(1) = -0.3;
    rObj.at<double>(2) = 0.5;
    Mat Robj;
    Rodrigues(rObj, Robj);

    for (size_t i = 0; i < n; i++) {
        //first define a random point on the plane z=0
        cv::Mat p(3, 1, CV_64FC1);
        p.at<double>(0) = X / 2 - static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / X));
        p.at<double>(1) = X / 2 - static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / X));
        p.at<double>(2) = 0;

        //now define it in object coordinates by rotating with Robj:
        p = Robj * p;
        objectPoints.at<Vec3d>(i)[0] = p.at<double>(0);
        objectPoints.at<Vec3d>(i)[1] = p.at<double>(1);
        objectPoints.at<Vec3d>(i)[2] = p.at<double>(2);
    }
    ///////////////////////////////////////////////////////

    //a ground truth pose:
    cv::Mat rvecGT(3, 1, CV_64FC1);
    cv::Mat tvecGT(3, 1, CV_64FC1);

    rvecGT.at<double>(0) = 0.1;
    rvecGT.at<double>(1) = -0.3;
    rvecGT.at<double>(2) = 0.6;

    tvecGT.at<double>(0) = 5.0;
    tvecGT.at<double>(1) = 10.0;
    tvecGT.at<double>(2) = X;

    //now solve the object's pose at increasing depths:
    bool term = false;
    size_t itLim = 30;
    size_t itCount = 0;

    std::cout << std::endl
              << "Running the demo... " << std::endl;

    IPPE::PoseSolver planePoseSolver;
    while (term == false) {
        //increase the z-component of translation:
        tvecGT.at<double>(2) = tvecGT.at<double>(2) * 1.2;

        //compute image correspondences:
        cv::projectPoints(objectPoints, rvecGT, tvecGT, cameraMatrix, distCoeffs, imagePoints);

        //call IPPE:
        cv::Mat rvec1, tvec1; //first pose
        cv::Mat rvec2, tvec2; //second pose
        float err1, err2; //RMS reprojection errors of first and second poses

        //there are two ways you can call IPPE.
        //(1) do it directly:
        //IPPE::PoseSolver::solveGeneric(objectPoints, imagePoints,cameraMatrix,distCoeffs,rvec1,tvec1,err1,rvec2,tvec2, err2);

        //(2) first undistort the image points, then call solveGeneric. The time taken to run solveGeneric will be less because cv::undistortPoints is not required:
        //The returned RMS errors will be different because (1) it is computed in pixels, whereas in (2) it is computed in normalized pixels:
        cv::Mat imagePointsud;
        cv::undistortPoints(imagePoints, imagePointsud, cameraMatrix, distCoeffs);
        planePoseSolver.solveGeneric(objectPoints, imagePointsud, cv::noArray(), cv::noArray(), rvec1, tvec1, err1, rvec2, tvec2, err2);

        //display results:
        double zBar = planePoseSolver.meanSceneDepth(objectPoints, rvecGT, tvecGT);
        std::cout << "Mean object depth: " << zBar << ", RMSE reprojection error of returned poses: (" << err1 << ", " << err2 << ")" << std::endl;
        itCount++;
        if (itCount == itLim) {
            term = true;
        }
    }
    std::cout << "Finished" << std::endl;
}
