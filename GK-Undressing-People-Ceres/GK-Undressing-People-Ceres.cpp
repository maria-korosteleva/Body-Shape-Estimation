// GK-Undressing-People-Ceres.cpp : This file contains the 'main' function. Program execution begins and ends there.
// It shows the example of how to use the code developed for the Undressing the input scan
//

#include "pch.h"
#include <iostream>

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
    - utils: Fill with zeros, print array, etc.
    - glog output to file
    - log result params & objects
    - log result objects inside SMPL
    - SMPL wrapper POse
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
    GeneralMesh input("D:/Data/smpl_outs/smpl_1.obj");
    std::cout << "Input mesh loaded!\n";

    SMPLWrapper smpl('f', "C:/Users/Maria/MyDocs/GigaKorea/GK-Undressing-People-Ceres/Resources");
    std::cout << "SMPL model loaded\n";

    ShapeUnderClothOptimizer optimizer(&smpl, input.getVertices());
    optimizer.findOptimalParameters();
    double* shape_res = optimizer.getEstimatesShapeParams();

    // Log the results
    // TODO: move outside (utils, optimizer, logger) 
    //igl::writeOBJ("C:/Users/Maria/MyDocs/GigaKorea/GK-Undressing-People-Ceres/tmp.obj", *smpl.getLastVertices(), *smpl.getFaces());

    // Visualize the output
    // TODO: add the input too. Meekyong knows something about two meshes
    
    igl::opengl::glfw::Viewer viewer;
    igl::opengl::glfw::imgui::ImGuiMenu menu;
    viewer.plugins.push_back(&menu);
    viewer.data().set_mesh(smpl.calcModel<double>(nullptr, shape_res), *smpl.getFaces());
    viewer.launch();

    delete[] shape_res;
}
