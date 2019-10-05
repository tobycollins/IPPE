Note: IPPE is now integrated and maintained in OpenCV. This repository may be discontinued.

# IPPE
## Overview
This is the C++ and Matlab implementations of Infinitesimal Plane-based Pose Estimation (IPPE): A very fast way to compute a planar object's 3D pose from a single image from 4 or more point correspondences. It works for perspective, weak-perspective and para-perspective cameras and is used in several applications, including Augmented Reality (AR), 3D tracking and pose estimation with planar markers, and 3D scene understanding. It is highly accurate for models that have points distributed in a regular way, such as square models used in Augmented Reality (AR) markers. For other models, the most accurate results are generally found by using an iterative solver such as Levenberg Marquardt. IPPE can be used as a very fast way to initialize these iterative solvers.   

IPPE has the significant benefit of being able to handle ambiguous cases. Specifically, it always returns *two* candidate pose solutions and their respective reprojection errors. These are sorted with the first pose having the lowest reprojection error. It is possible to reject the second pose if its reprojection error is significantly worse than the first. This can be done with a likelihood ratio test. The second pose is needed if the problem is ambiguous, which means there are two valid pose solutions. The problem is ambiguous when the projection of the object is close to affine, which in practice happens if it is small or viewed from a large distance. In these cases there are generally two pose solutions that can correctly align the correspondences (up to noise), so it is impossible to select the right pose using the reprojection error. IPPE gives you both the solutions, rather than just a single solution (which in ambiguous cases would be wrong 50% of the time). This problem is suffered by OpenCV's solvePnP default iterative solver because it only returns one solution. Geometrically, the two poses roughly correspond to a flip of the object about a plane whose normal passes through the line-of-sight from the camera centre to the object's centre. For more details about the ambiguity, please refer to the IPPE paper.

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
 solveSquare is used to solve the pose of a square object defined by its 4 corners. It is mainly used for getting the pose of square targets such as AR markers (e.g. it can be used with aruco markers). In this special case, solveSquare is faster than solveGeneric because we compute the object-to-image homography with an analytic expression (discussed in the IPPE paper). This makes it exceptionally fast because pose is solved *completely* analytically. It is approximately 50 to 80 times faster than OpenCV's default PnP solver.
 
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



## Resolving the flip ambiguity
So how to resolve the ambiguity? You can't do it using pnp, and indeed no pnp algorithm will be able to resolve it, because it is a property of the problem. More information is required to constrain the marker's pose.

There are four main options:

1. To use the effect of shading to determine the correct pose. Basically, the brightness of the marker is influenced by its orientation relative to the scene's illumination. It is possible to determine the correct pose using this information. This is nicely shown in the optical illusion posted above by @catree. However this can be difficult in practice because it requires determining the illumination of the scene.

2. To use temporal filtering, as suggested by @catree. But in practice this is very hard to use, and it cannot solve the problem reliably. It might reduce random flipping between the two solutions, but we cannot guarantee that the smoothed trajectory of poses is correct.

3. To use additional non-coplanar markers.
If you have one or more additional markers that are not co-planar to the original marker, then you can resolve the ambiguity. This is because the pnp problem no longer involves a planar model, and we can compute pose uniquely.

4. To use additional coplanar markers.
If you have a set of markers that all lie on the same plane, then is it possible to resolve the ambiguity, but there is a condition. The set of markers must span a region of space that is 'sufficiently large'. You need this condition to prevent an ambiguity of the entire set of markers being flipped. There are two ways to solve this in practice. The first is to solve the pose of each marker independently and then find the correct pose using a consensus. The correct pose should be revealed as a tight cluster within the set of all poses (2 poses per marker). If you don't know the relative positions of the markers on the plane, you can do a consensus with the estimated normals of each marker (with two normals per marker). Alternatively if you know the relative positions of the markers on the plane, things are simpler. You can simply detect all the markers in the image, then solve a single pnp problem with 4n point correspondences, where n is the number of detected markers.


