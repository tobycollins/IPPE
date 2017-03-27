# IPPE
## Overview
This is the c++ and Matlab implementations of Infinitesimal Plane-based Pose Estimation (IPPE): A very fast and accurate way to compute a planar object's 3D pose from a single image from 4 or more point correspondences. It works for perspective, weak-perspective and para-perspective cameras and is used in several applications, including augmented reality, 3D tracking and pose estimation with planar markers, and 3D scene understanding.

IPPE has the significant benefit of being able to handle ambiguous cases. Specifically, it always returns *two* candidate pose solutions and their respective reprojection errors. These are sorted with the first pose having the lowest reprojection error. It is possible to reject the second pose if its reprojection error is significantly worse than the first. This can be done with a likelihood ratio test. Some good information on this can be found at https://en.wikipedia.org/wiki/Likelihood-ratio_test.

The second pose is needed if the problem is ambiguous, which means there are two valid pose solutions. The problem is ambiguous when the projection of the object is close to affine, which in practice happens if it is small or viewed from a large distance. In these cases there are generally two pose solutions that can correctly align the correspondences (up to noise), so it is impossible to select the right pose using the reprojection error. IPPE gives you both the solutions, rather than just a single solution (which in ambiguous cases would be wrong 50% of the time). This problem is suffered by OpenCV's solvePnP default iterative solver because it only returns one solution. Geometrically, the two poses roughly correspond to a flip of the object about a plane whose normal passes through the line-of-sight from the camera centre to the object's centre. For more details about the ambiguity, please refer to the IPPE paper.

We hope you find IPPE useful and if so please cite our paper:

@article{
year={2014},
issn={0920-5691},
journal={International Journal of Computer Vision},
volume={109},
number={3},
doi={10.1007/s11263-014-0725-5},
title={Infinitesimal Plane-Based Pose Estimation},
url={http://dx.doi.org/10.1007/s11263-014-0725-5},
publisher={Springer US},
keywords={Plane; Pose; SfM; PnP; Homography},
author={Collins, Toby and Bartoli, Adrien},
pages={252-286},
language={English}
}

## c++ implementation
The c++ implementation requires OpenCV v3.0 or above and CMake 2.8 or above. There are no other dependencies. It is a small library with one header and one library file. These are located in the build path's include and lib subdirectories respectively.

There are two ways to use IPPE: (1) IPPE::PoseSolver::solveGeneric and (2) IPPE::PoseSolver::solveSquare.

### solveGeneric
 This has a very similar calling syntax to OpenCV's solvePnP, but with the important difference that it returns *two*
 pose solutions and their respective reprojection errors. These are sorted so that the first one is the one with the lowest reprojection error.

### solveSquare
 solveSquare is used to solve the pose of a square object defined by its 4 corners. It is mainly used for getting the pose of square targets such as AR markers (e.g. it can be used with aruco markers). In this special case, solveSquare is faster than solveGeneric because we compute the object-to-image homography with an analytic expression (discussed in the IPPE paper). This makes it exceptionally fast because pose is solved *completely* analytically.
 
The plane's 4 corners are defined as follows in object coordinates:

point 0: [-squareLength / 2.0, squareLength / 2.0, 0.0]

point 1: [squareLength / 2.0, squareLength / 2.0, 0.0]

point 2: [squareLength / 2.0, -squareLength / 2.0, 0.0]

point 3: [-squareLength / 2.0, -squareLength / 2.0, 0.0]

 where squareLength denotes the length of the square. Therefore, the square is defined in object coordinates on the plane z=0 and centred at the origin.
 Just like solveGeneric, solveSquare also returns the two possible pose solutions and their respective reprojection errors.

### Demos
We show how to use IPPE with two demo executables (located in the build path's bin subdirectory). The first: IPPE_demo_generic_pointset shows how to solve the general problem (4 or more point correspondences) using solveGeneric. The second: IPPE_demo_square_pointset shows how to solve the special problem with a square object using solveSquare. In both these demos we show how the problem can be ambiguous. Specifically, we increase the distance of the object to the camera, and show how the reprojection error of the second pose solution tends to zero as the depth tends to infinity.

## Matlab implementation
The function perspectiveIPPE isolves the generic problem. You can use addpath(genpath('./IPPE/matlab')) to setup necessary dependencies. To run all demos you need the Affine SIFT (ASIFT) library for affine-invariant SIFT matching: http://www.ipol.im/pub/art/2011/my-asift/ This is easy to compile with cmake, and you should put the executable demo_ASIFT in the demo_ASIFT_src folder.

### Demos
We give two demo functions for running IPPE.

The first demo: IPPE_demo1, shows how to use IPPE to solve a perspective camera's pose with a plane and a single image using ransac-based feature matching. Two feature methods are supported, which are affine-sift (ASIFT) and SURF. Other can be easily added. This demo loops through a set of 5 images and computes the camera's pose relative to the plane for each image.

The second demo: IPPE_demo2, shows an example of using IPPE to solve a camera's pose with a plane and simulated point correspondences.
