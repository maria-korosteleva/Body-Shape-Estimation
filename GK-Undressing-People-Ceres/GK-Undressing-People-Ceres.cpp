// GK-Undressing-People-Ceres.cpp : This file contains the 'main' function. Program execution begins and ends there.
// It shows the example of how to use the code developed for the Undressing the input scan
//

#include "pch.h"
#include <iostream>

#include <igl/readOBJ.h>
#include <igl/writeOBJ.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>

#include "SMPLWrapper.h"

/*
    TODO
    + libigl installation
    x Input mesh reader and storage
    - Smpl wrapper
    - the simpliest optimizer possible (no translation and pose, shape only, with known correspondance)
    - Do I need a separate logger? 
    - Idea: aloow start optimization from the last results
    - Idea: could keep some python scripts?
    - Idea: will the optimizer work for different types of the input blocks (so that optimization separation won't be needed)?
    + libigl menu
    - libigl as static library
    - SMPL wrapper avalible for everyone
*/


int main()
{
    // Get the input
    // TODO move the routine outside
    Eigen::MatrixXd verts;
    Eigen::MatrixXi faces;

    // Load a mesh
    // TODO: make it type-independent
    const std::string INPUT("D:/Data/DYNA/50004_chicken_wings/00100.obj");
    igl::readOBJ(INPUT, verts, faces);
    // TODO: Calculate a tree

    std::cout << "Input mesh loaded!\n";

    // Init SMPL wrapper
    SMPLWrapper smpl('f', "C:/Users/Maria/MyDocs/GigaKorea/GK-Undressing-People-Ceres/Resources");
    Eigen::VectorXd pose;
    Eigen::VectorXd shape = Eigen::VectorXd::Zero(smpl.getShapeSize());
    shape[0] = -5.;
    shape[5] = -2;
    smpl.calcModel(pose, shape);

    // init optimizer
    // set the options
    // run optimization
    // get the results
    
    // Log the results
    // TODO: move outside (utils, optimizer, logger) 
    igl::writeOBJ("C:/Users/Maria/MyDocs/GigaKorea/GK-Undressing-People-Ceres/tmp.obj", *smpl.getLastVertices(), *smpl.getFaces());

    // Visualize the output
    // TODO: add the input too. Meekyong knows something about two meshes

    igl::opengl::glfw::Viewer viewer;
    igl::opengl::glfw::imgui::ImGuiMenu menu;
    viewer.plugins.push_back(&menu);
    viewer.data().set_mesh(*smpl.getLastVertices(), *smpl.getFaces());
    viewer.launch();
}
