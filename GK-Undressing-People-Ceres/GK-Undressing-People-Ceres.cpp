// GK-Undressing-People-Ceres.cpp : This file contains the 'main' function. Program execution begins and ends there.
// It shows the example of how to use the code developed for the Undressing the input scan
//

// WARNING! Uses win-specific features to create log directory 
//#include <Windows.h>
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
    - SMPL wrapper Pose
    - clean-up and delete all the things
    - point-to-surface distance
    - regularization
    - directional pose estimation
    - Idea: aloow start optimization from the last results
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
    
    //CreateDirectory(logName.c_str(), NULL);

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
    GeneralMesh input("D:/Data/smpl_outs/smpl_1.obj");
    std::cout << "Input mesh loaded!\n";
    SMPLWrapper smpl('f', "C:/Users/Maria/MyDocs/GigaKorea/GK-Undressing-People-Ceres/Resources");
    std::cout << "SMPL model loaded\n";
    
    //std::string logFolderName = getNewLogFolder("dev");
    
    /*
    // Run optimization
    ShapeUnderClothOptimizer optimizer(&smpl, input.getVertices());
    
    //// Redirect optimizer output to file
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
    smpl.saveToObj(nullptr, shape_res, (logFolderName + "unposed_shaped.obj"));
    logSMPLParams(nullptr, shape_res, logFolderName);
    */

    double* pose = new double[SMPLWrapper::POSE_SIZE];
    for (int i = 0; i < SMPLWrapper::POSE_SIZE; i++)
        pose[i] = 0.;
    pose[51] = 10.;
    //smpl.saveToObj(pose, nullptr, (logFolderName + "posed_unshaped.obj"));

    // Visualize the output
    // TODO: add the input too. Meekyong knows something about two meshes  
    Eigen::MatrixXd verts = smpl.calcModel<double>(pose, nullptr);
    Eigen::MatrixXi faces = smpl.getFaces();
    igl::opengl::glfw::Viewer viewer;
    igl::opengl::glfw::imgui::ImGuiMenu menu;
    viewer.plugins.push_back(&menu);
    viewer.data().set_mesh(verts, faces);
    viewer.launch();

    // Cleaning
    //delete[] shape_res;
    delete[] pose;
}
