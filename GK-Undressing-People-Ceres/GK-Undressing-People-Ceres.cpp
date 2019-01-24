// GK-Undressing-People-Ceres.cpp : This file contains the 'main' function. Program execution begins and ends there.
// It shows the example of how to use the code developed for the Undressing the input scan
//

#define DEBUG
// WARNING! Uses win-specific features to create log directory 
#include <Windows.h>
#include "pch.h"
#include <iostream>
#include <ctime>

#include <assert.h>

#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>

#include "GeneralMesh.h"
#include "SMPLWrapper.h"
#include "ShapeUnderClothOptimizer.h"

/*
    TODO
    + libigl installation
    + Input mesh reader and storage
    + Smpl wrapper - Shape
    + the simpliest optimizer possible (no translation and pose, shape only, with known correspondance)
    + move to simple types (double*) 
    + Declare constexpressions for model sizes
    + utils: Fill with zeros, print array, etc.  (only needed incide optimizer
    + glog output to file
    + log result params & objects
    + log result objects inside SMPL
    + SMPL wrapper Pose
    + Pose estimation using ceres
    + ceres rotations vs non-ceres
    + simplify function hierarchy 
    + regularization
    - LBS optimization (sparse weight matrix etc)
    - !! point-to-surface distance
    - add translation
    - move (important) parameters outside
    - ? directional pose estimation
    - Idea: allow start optimization from the last results
    - Idea: could keep some python scripts?
    - Idea: will the optimizer work for different types of the input blocks (so that optimization separation won't be needed)?
    + libigl menu
    - libigl as static library
    - SMPL wrapper avalible for everyone
    - Readme with installation notes
    - learning curve visualization
*/

std::string getNewLogFolder(const char * tag = "test")
{
    std::string logName("C:/Users/Maria/MyDocs/GigaKorea/GK-Undressing-People-Ceres/Outputs/");
    logName += tag;
    logName += "_";
    
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(buffer, sizeof(buffer), "%y%m%d_%H_%M", timeinfo);
    
    logName += buffer;
    logName += "/";
    
    CreateDirectory(logName.c_str(), NULL);

    return logName;
}


void logSMPLParams(double* pose, double* shape, std::string logFolderName)
{
    std::ofstream out(logFolderName + "smpl_params.txt");

    out << "Pose params \n[ ";
    if (pose != nullptr)
        for (int i = 0; i < SMPLWrapper::POSE_SIZE; i++)
            out << pose[i] << " , ";
    else
        for (int i = 0; i < SMPLWrapper::POSE_SIZE; i++)
            out << "0." << " , ";

    out << "]" << std::endl;

    out << "Shape (betas) params \n[ ";
    if (shape != nullptr)
        for (int i = 0; i < SMPLWrapper::SHAPE_SIZE; i++)
            out << shape[i] << " , ";
    else
        for (int i = 0; i < SMPLWrapper::SHAPE_SIZE; i++)
            out << "0." << " , ";
    out << "]" << std::endl;

    out.close();
}


int main()
{
    std::string logFolderName = getNewLogFolder("sp_reg_50");
    
    GeneralMesh input("D:/Data/smpl_outs/pose_50004_knees_270_dyna_thin_custom_smpl.obj");  // _custom_smpl
    // For convenience
    igl::writeOBJ(logFolderName + "input.obj", *input.getVertices(), *input.getFaces());
    std::cout << "Input mesh loaded!\n";
    SMPLWrapper smpl('f', "C:/Users/Maria/MyDocs/GigaKorea/GK-Undressing-People-Ceres/Resources");
    std::cout << "SMPL model loaded\n";

    // Run optimization
    ShapeUnderClothOptimizer optimizer(&smpl, input.getVertices(), "C:/Users/Maria/MyDocs/GigaKorea/GK-Undressing-People-Ceres/Resources");
    std::cout << "Optimizer loaded\n";
    
    // Redirect optimizer output to file
    std::ofstream out(logFolderName + "optimization.txt");
    std::streambuf *coutbuf = std::cout.rdbuf();    //save old buf
    std::cout.rdbuf(out.rdbuf());                   //redirect std::cout to file!
    std::cout << logFolderName + "optimization.txt" << std::endl;

    optimizer.findOptimalParameters();

    std::cout.rdbuf(coutbuf);   //  reset cout to standard output again
    out.close();
    std::cout << "Optimization finished!\n";

    // Save the results
    double* shape_res = optimizer.getEstimatesShapeParams();
    double* pose_res = optimizer.getEstimatesPoseParams();
    logSMPLParams(pose_res, shape_res, logFolderName);
    smpl.saveToObj(pose_res, shape_res, (logFolderName + "posed_shaped.obj"));
    smpl.saveToObj(nullptr, shape_res, (logFolderName + "unposed_shaped.obj"));
    smpl.saveToObj(pose_res, nullptr, (logFolderName + "posed_unshaped.obj"));
    

 /*   double* pose_res = new double[SMPLWrapper::POSE_SIZE];
    double* shape_res = new double[SMPLWrapper::SHAPE_SIZE];
    for (int i = 0; i < SMPLWrapper::POSE_SIZE; i++)
    {
        pose_res[i] = 0.;
        if (i < SMPLWrapper::SHAPE_SIZE)
            shape_res[i] = 0.;
    }
    pose_res[69] = 1.;
    pose_res[51] = 1.;
    pose_res[52] =0.5;
    pose_res[53] = -1.;
    pose_res[58] = 1;
    pose_res[5] = 2.;
    pose_res[0] = -1.;
    shape_res[0] = -0.5;

    std::vector<double> pose_v{ 3.08410123e-01, - 1.05524458e-01, - 1.96436839e-01, - 4.20388675e-01,
   7.06017900e-02, - 9.78927189e-04,   1.78518676e-01,   1.32230268e-01,
  - 6.92474425e-03,   1.84399515e-01,   6.64744222e-02,   1.52742055e-02,
   9.97696737e-02,   4.55078255e-02,   1.27937498e-01,   1.64509601e+00,
   7.28455572e-02,   1.23536599e+00, - 5.77539802e-02,   4.75015259e-02,
   4.50203230e-02,   4.74740569e-02, - 2.45507880e-02,   5.12806066e-03,
   1.39208061e-01,   1.95541421e-01,   1.43355251e-01, - 2.89011585e-01,
  - 9.62822705e-03,   7.29079129e-02,   3.71509152e-02, - 7.63992865e-02,
   4.95293677e-02,   6.46608949e-02, - 2.04391369e-01, - 3.49676681e-02,
   5.45909513e-03,   1.05900844e-03, - 1.02948656e-03,   8.71519738e-03,
   6.72209469e-03, - 5.06902203e-01, - 5.19515559e-02,   2.55365939e-02,
   4.87420214e-01, - 6.62440702e-02, - 1.57825244e-02,   3.73497664e-03,
   7.11666455e-02,   6.75781579e-02, - 4.59377296e-01, - 1.96738438e-02,
  - 2.21771815e-02,   3.45383667e-01,   7.08987074e-02, - 1.11668877e-02,
   8.93912764e-03, - 1.21386240e-01, - 9.85613379e-02,   8.40101409e-03,
  - 5.14314053e-03,   1.30868354e-01,   2.16632748e-02,   4.14801475e-03,
  - 2.59121185e-01,   1.47873327e-01,   9.92928214e-02, - 2.83824139e-01,
  - 1.43411428e-01, - 1.33443054e-01,   2.76190242e-02, - 1.88618938e-01 };

    std::vector<double> shape_v { 0.36360461,  1.40706639,  2.20286688,  0.21138089,  1.18583149,  1.78600698,
  1.50903028,  0.30836872,  2.25329121,  1.56860682 };

    smpl.saveToObj(&pose_v[0], &shape_v[0], (logFolderName + "pose_50004_knees_270_dyna_thin_custom_smpl.obj"));*/

    // Visualize the output
    // TODO: add the input too. Meekyong knows something about two meshes  
    Eigen::MatrixXd verts = smpl.calcModel<double>(pose_res, shape_res);
    Eigen::MatrixXi faces = smpl.getFaces();
    igl::opengl::glfw::Viewer viewer;
    igl::opengl::glfw::imgui::ImGuiMenu menu;
    viewer.plugins.push_back(&menu);
    viewer.data().set_mesh(verts, faces);
    //viewer.data().set_points(Eigen::RowVector3d(1., 1., 0.), Eigen::RowVector3d(1., 1., 0.));
    //viewer.data().set_points(joints, Eigen::RowVector3d(1., 1., 0.));
    viewer.launch();

    // Cleaning
    delete[] shape_res;
    delete[] pose_res;
}
