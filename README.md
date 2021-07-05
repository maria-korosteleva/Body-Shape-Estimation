# Body shape and pose estimation on 3D scans of people in clothing using Ceres Solver

This project is based on the paper of Zhang et al. "Detailed, accurate, human shape estimation from clothed 3D scan sequences" (https://arxiv.org/abs/1703.04454), but adapted to work on static 3D scans and with a better pose estimation. The project surves the research purposes only. 

The work heavily uses the SMPL body model (refer to http://smpl.is.tue.mpg.de/). 

## Disclaimer & Licensing

* This project was created as personal and is not perfectly polished for external use. Please, raise issues if you ran into any problems.
   * VS solution is organized for the convenience of the developer and might take some time to adapt for running on the other computer;
   * the code style and some small architectural decisions might not be consistent throughout the project code (the developer tries out new tricks from time to time). 

* We heavily rely on SMPL model, so please refer to SMPL Licence agreements https://smpl.is.tue.mpg.de/modellicense, https://smpl.is.tue.mpg.de/bodylicense before use.
   * The shape blenshapes are not included in the resources as they are not allowed for redistribution ( https://smpl.is.tue.mpg.de/modellicense, https://smpl.is.tue.mpg.de/bodylicense), but they are needed for the successful running of the present module. You could get them from SMPL website (non-commertial use only).
   * Pose blendshapes are not included as they are too large. You could get them from SMPL website, or run the tool without them (when creating SMPLWrapper object)

## Inputs requirements
* obj or ply format
* the input is expected to be given on a metric scale (m, cm, dm, mm..), corresponding to the measurements of the person in a real world.
* gender should be explicitly set

### Example Input & output
In the `./Example Output` we provide an example 3D model and the full logs of the system output when estimating body shape for this model. 
* The provided sample is only given for the purpose of documenting the system output, and not to fully demonstrate the quality of estimations.
    * For example, the estimated body is noticeably thinner that the input.This happens because the system assumes a clothed body and not the body model itself (as in the example).
    * Unfortunately, we cannot provide an example with the body model with clothing due to the licensing terms.
* The example input is a female SMPL body posed in A pose. We are using it under the [SMPL-Body license](https://smpl.is.tue.mpg.de/bodylicense). Please, comply with the license terms when working with this example.
* We provide the SMPL model parameters in `.log` file for a reference. It's not used by the system in any way.


## System requirements
* The project is developed under Windows 10, using Visual Studio 2017 x64, and have never been tested in other environments.
* Uses some C++11 features

## Dependencies

#### External modules:
1. GeneralMesh (https://github.com/maria-korosteleva/GeneralMesh)
1. Photographer (https://github.com/maria-korosteleva/Photographer)

#### Libraries with the versions used during development
1. Eigen v3.3.7 (http://eigen.tuxfamily.org/index.php?title=Main_Page)
1. libigl v2.0.0 or 2.1.0 (https://libigl.github.io/)
*NOTE*: if compilation produces POSIX errors, add the following before including igl headersflags
'''
#ifdef _MSC_VER
#pragma warning(disable:4996)
#endif
'''
1. Ceres v1.14.0 (http://ceres-solver.org/index.html) compiled to be used on x64 platform (same goes for the Ceres dependencies).
1. OpenPose v1.4.0 (https://github.com/CMU-Perceptual-Computing-Lab/openpose/). 
    1. *Important* The path to the OpenPose models should be set inside the program
1. Boost.Filesystem module of Boost library (https://www.boost.org/), v1.68.0.

## Tips for installing Ceres
1. Make sure to use x64 compiler for all installations.
1. Ceres dependencies need to be compiled as DLLs. For that turn on an option "BUILD SHARES LIBS" when configuring the installation with CMAKE each time a new library is installed. 
1. Add the DLLs to the PATH environment variable. This will eliminate the need to copy DLLs to the folder containing the .exe file of the project.
1. Ceres required dependencies are glog and gflags. Glog depends on gflags, so gflags should be installed before glog.
1. Use EIGEN as a library for the sparse linear solver. For this, check the EIGENSPARSE flag in CMAKE GUI when configuring ceres. This will allow the use of sparse solvers like SPARSE_NORMAL_CHOLESKY without a need to install additional libraries (CXSparse, SuiteSparse, BLAS, LAPACK). It's probably not the fastest solver though. 
