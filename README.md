# Shape estimation under clothing using Ceres Solver
Part of the GigaKorea project. 
Ancessor of https://motionlab.kaist.ac.kr/git/mariako/Giga-Korea-Undressing-People
Reimplementation of the Zhang et al"Detailed, accurate, human shape estimation from clothed 3D scan sequences" (https://arxiv.org/abs/1703.04454).

## System requirements
The project is developed under Windows 10, using Visual Studio 2017 x64, and have never been tested in other environment.

## Dependecies
1. Eigen (http://eigen.tuxfamily.org/index.php?title=Main_Page)
1. libigl (https://libigl.github.io/)
1. Ceres (http://ceres-solver.org/index.html) compiled to be used on x64 platform (same goes for the Ceres dependencies).

VS property sheets to work with the latter two libraries are provided for your reference.

## Tips for installing Ceres
1. Make sure to use x64 compiler for all installations.
1. Ceres its dependencies need to be compiled as dlls. For that check the option "BUILD SHARES LIBS" when configuring the installation with CMAKE each time a new library in installed. 
1. Add the paths to the dlls generated to the PATH environment variable. This will eliminate the need to copy dlls to the folder containg the .exe file of the project.
1. Ceres required dependencies are glog and gflags. Glog depends on gflags, so it's needed to install the glags before glog.
1. Use EIGEN as a library for the sparse linear solver. For this, check EIGENSPARSE flag in CMAKE GUI when configuring ceres. This will allow to use sparse solvers like SPARSE_NORMAL_CHOLESKY without a need to install additional libraries (CXSparse, SuiteSparse, BLAS, LAPACK). It's probably not the fastest solver though. 
