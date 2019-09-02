# Body shape and pose estimation on 3D scans of people in clothing using Ceres Solver

This project is based on the paper of Zhang et al. "Detailed, accurate, human shape estimation from clothed 3D scan sequences" (https://arxiv.org/abs/1703.04454), but adapted to work on static 3D scans and with a better pose estimation. 

## Disclaimer 

This is a work and progress which means that
* the master branch is not guaranteed to work properly at any given moment;
* VS solution is organized for the convenience of the developer and might take some time to adapt for running on the other computer;
* the code style and some small architectural decisions might not be consistent throughout the project code (the developer tries out new tricks from time to time). 

With any troubles of running the code, contact me here on GitHub or email me on mariako@kaist.ac.kr

## Inputs requirements
* obj or ply format
* the input is expected to be given on a metric scale (m, cm, dm, mm..), corresponding to the measurements of the person in a real world.
* gender should be explicitly set

## System requirements
* The project is developed under Windows 10, using Visual Studio 2017 x64, and have never been tested in other environments.
* Uses some C++11 features
* Uses Windows-specific functionality to work with the filesystem. If you are using other OS, you'd need to modify the CustomLogger

## Dependencies

#### External modules:
1. GeneralMesh (https://github.com/maria-korosteleva/GeneralMesh)
1. Photographer (https://github.com/maria-korosteleva/Photographer)

#### Libraries    
1. Eigen (http://eigen.tuxfamily.org/index.php?title=Main_Page)
1. libigl (https://libigl.github.io/)
*NOTE*: if compilation produces POSIX errors, add the following before including igl headersflags
'''
#ifdef _MSC_VER
#pragma warning(disable:4996)
#endif
'''
1. Ceres (http://ceres-solver.org/index.html) compiled to be used on x64 platform (same goes for the Ceres dependencies).
1. OpenPose (https://github.com/CMU-Perceptual-Computing-Lab/openpose/). 
    1. *Important* The path to the OpenPose models should be set inside the program

## Tips for installing Ceres
1. Make sure to use x64 compiler for all installations.
1. Ceres dependencies need to be compiled as DLLs. For that turn on an option "BUILD SHARES LIBS" when configuring the installation with CMAKE each time a new library is installed. 
1. Add the DLLs to the PATH environment variable. This will eliminate the need to copy DLLs to the folder containing the .exe file of the project.
1. Ceres required dependencies are glog and gflags. Glog depends on gflags, so gflags should be installed before glog.
1. Use EIGEN as a library for the sparse linear solver. For this, check the EIGENSPARSE flag in CMAKE GUI when configuring ceres. This will allow the use of sparse solvers like SPARSE_NORMAL_CHOLESKY without a need to install additional libraries (CXSparse, SuiteSparse, BLAS, LAPACK). It's probably not the fastest solver though. 