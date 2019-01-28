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
    + LBS optimization (sparse weight matrix etc)
    - point-to-surface distance
    + add translation
    + Log input name

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


void logSMPLParams(double* translation, double* pose, double* shape, std::string logFolderName)
{
    std::ofstream out(logFolderName + "smpl_params.txt");

    out << "Translation \n[ ";
    if (translation != nullptr)
        for (int i = 0; i < SMPLWrapper::SPACE_DIM; i++)
            out << translation[i] << " , ";
    else
        for (int i = 0; i < SMPLWrapper::SPACE_DIM; i++)
            out << "0." << " , ";

    out << "]" << std::endl;

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
    const char* input_name = "D:/Data/smpl_outs/pose_50004_knees_270_dyna_fat.obj";

    std::string logFolderName = getNewLogFolder("sprt_opt_sparse_50");

    GeneralMesh input(input_name);  // _custom_smpl
    // For convenience
    igl::writeOBJ(logFolderName + "input.obj", input.getVertices(), input.getFaces());
    std::cout << "Input mesh loaded!\n";
    SMPLWrapper smpl('f', "C:/Users/Maria/MyDocs/GigaKorea/GK-Undressing-People-Ceres/Resources");
    std::cout << "SMPL model loaded\n";

    // Run optimization
    ShapeUnderClothOptimizer optimizer(&smpl, &input, "C:/Users/Maria/MyDocs/GigaKorea/GK-Undressing-People-Ceres/Resources");
    std::cout << "Optimizer loaded\n";
    
    // Redirect optimizer output to file
    std::ofstream out(logFolderName + "optimization.txt");
    std::streambuf *coutbuf = std::cout.rdbuf();    //save old buf
    std::cout.rdbuf(out.rdbuf());                   //redirect std::cout to file!
    std::cout << "Input file: " << input_name << std::endl;
    std::cout << logFolderName + "optimization.txt" << std::endl;

    optimizer.findOptimalParameters();

    std::cout.rdbuf(coutbuf);   //  reset cout to standard output again
    out.close();
    std::cout << "Optimization finished!\n";

    // Save the results
    double* shape_res = optimizer.getEstimatesShapeParams();
    double* pose_res = optimizer.getEstimatesPoseParams();
    double* translation_res = optimizer.getEstimatesTranslationParams();
    logSMPLParams(translation_res, pose_res, shape_res, logFolderName);
    smpl.saveToObj(translation_res, pose_res, shape_res, (logFolderName + "posed_shaped.obj"));
    smpl.saveToObj(translation_res, nullptr, shape_res, (logFolderName + "unposed_shaped.obj"));
    smpl.saveToObj(translation_res, pose_res, nullptr, (logFolderName + "posed_unshaped.obj"));

    //double* pose_res = new double[SMPLWrapper::POSE_SIZE];
    //double* shape_res = new double[SMPLWrapper::SHAPE_SIZE];
    //for (int i = 0; i < SMPLWrapper::POSE_SIZE; i++)
    //{
    //    pose_res[i] = 0.;
    //    if (i < SMPLWrapper::SHAPE_SIZE)
    //        shape_res[i] = 0.;
    //}
    //pose_res[69] = 1.;
    //pose_res[51] = 1.;
    //pose_res[52] =0.5;
    //pose_res[53] = -1.;
    //pose_res[58] = 1;
    //pose_res[5] = 2.;
    //pose_res[0] = -1.;
    //shape_res[0] = -0.5;

    //smpl.saveToObj(&pose_v[0], &shape_v[0], (logFolderName + "pose_50004_knees_270_dyna_thin_custom_smpl.obj"));

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
