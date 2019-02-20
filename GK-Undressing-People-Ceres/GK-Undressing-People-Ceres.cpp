// GK-Undressing-People-Ceres.cpp : This file contains the 'main' function. Program execution begins and ends there.
// It shows the example of how to use the code developed for the Undressing the input scan
//

//#define DEBUG
//#define EIGEN_STACK_ALLOCATION_LIMIT 0
// WARNING! Uses win-specific features to create log directory 
#include <Windows.h>
#include "pch.h"
#include <iostream>
#include <ctime>

#include <assert.h>

#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/point_mesh_squared_distance.h>

#include "GeneralMesh.h"
#include "SMPLWrapper.h"
#include "ShapeUnderClothOptimizer.h"

//#include "AbsoluteVertsToMeshDistance.h"

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
    + point-to-surface distance
        + Idea: try numerical derivative for point-to-surface distance
    + add translation
    + Log input name
    + optimization process visualization on-the-fly
    - directional pose estimation -- idea: add it to the main objective as additional resudual 
    - Acuurate shape and pose estimation (iterative?)
    - Shape regularization 

    - move (important) parameters outside
    - Idea: allow start optimization from the last results
    - libigl as static library
    - SMPL wrapper avalible for everyone
    + Idea: could keep some python scripts?
    + Idea: will the optimizer work for different types of the input blocks (so that optimization separation won't be needed)?
    + libigl menu
    + Readme with installation notes
    x learning curve visualization
*/

// global vars are needed for visualization purposes only
bool progress_visualization = true;
//bool progress_visualization = false;
std::vector<Eigen::MatrixXd> iteration_outputs;
int counter = 0;
SMPLWrapper* smpl;
GeneralMesh* input;

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


bool visulaze_progress_pre_draw(igl::opengl::glfw::Viewer & viewer) {
    if (viewer.core.is_animating && counter < iteration_outputs.size())
    {
        viewer.data().clear();
        Eigen::MatrixXi faces = smpl->getFaces();

        viewer.data().set_mesh(iteration_outputs[counter], faces);
        viewer.core.align_camera_center(iteration_outputs[counter], faces);

        //Eigen::VectorXd sqrD;
        //Eigen::MatrixXd closest_points;
        //Eigen::VectorXi closest_face_ids;
        //igl::point_mesh_squared_distance(iteration_outputs[counter], input->getVertices(), input->getFaces(), sqrD, closest_face_ids, closest_points);

        //viewer.data().add_points(closest_points, Eigen::RowVector3d(1., 1., 0.));
        //viewer.data().add_edges(iteration_outputs[counter], closest_points, Eigen::RowVector3d(1., 0., 0.));

        Eigen::MatrixXd input_key_points(input->getKeyPoints().size(), 3);
        Eigen::MatrixXd smpl_key_points(input->getKeyPoints().size(), 3);

        CoordsDictionary inputKeyVerts = input->getKeyPoints();
        Dictionary smplKeyVerts = smpl->getKeyVertices();
        int res_id = 0;
        for (auto const& keyIterator : inputKeyVerts)
        {
            input_key_points.block(res_id, 0, 1, 3) = keyIterator.second;
            smpl_key_points.block(res_id, 0, 1, 3) = iteration_outputs[counter].row(smplKeyVerts[keyIterator.first]);
            res_id++;
        }
        viewer.data().add_points(input_key_points, Eigen::RowVector3d(1., 1., 0.));
        viewer.data().add_edges(smpl_key_points, input_key_points, Eigen::RowVector3d(1., 0., 0.));

        counter++;
    }
    else if (viewer.core.is_animating && counter >= iteration_outputs.size())
    {
        viewer.core.is_animating = false;
        counter = 0;
        std::cout << "You can start the animation again by pressing [space]" << std::endl;
    }
    return false;
}


bool visulaze_progress_key_down(igl::opengl::glfw::Viewer& viewer, unsigned char key, int modifier)
{
    if (key == ' ' )
    {
        viewer.core.is_animating = !viewer.core.is_animating;
    }
    return false;
}


int main()
{
    //const char* input_name = "D:/Data/smpl_outs/pose_50004_knees_270_dyna_thin.obj";
    //const char* input_name = "D:/Data/smpl_outs/pose_50004_knees_270_dyna_thin_custom_smpl.obj";
    // const char* input_name = "D:/Data/smpl_outs/pose_50004_knees_270_dyna_fat.obj";
    //const char* input_name = "D:/Data/smpl_outs/smpl_2.obj";
    //const char* input_name = "D:/Data/DYNA/50004_jumping_jacks/00000.obj";
    //const char* input_name = "D:/Data/DYNA/50004_chicken_wings/00091.obj";
    //const char* input_name = "D:/Data/smpl_outs/pose_hand_up.obj";
    //const char* input_name = "D:/Data/smpl_outs/pose_hand_up_down.obj";
    //const char* input_name = "D:/Data/smpl_outs/pose_leg_up_up.obj";
    const char* input_name = "D:/Data/smpl_outs/pose_leg_up_knee_up.obj";

    // for SMPL/DYNA inputs
    // expected to contain the subset of the keys defined for the model 
    const char* input_key_vertices_name = "D:/Data/smpl_outs/smpl_key_vertices.txt";

    std::string logFolderName = getNewLogFolder("key_dirs_pose_jac_check_grads_150");

    input = new GeneralMesh(input_name, input_key_vertices_name);
    //// For convenience
    igl::writeOBJ(logFolderName + "input.obj", input->getVertices(), input->getFaces());
    std::cout << "Input mesh loaded!\n";
    smpl = new SMPLWrapper('f', "C:/Users/Maria/MyDocs/GigaKorea/GK-Undressing-People-Ceres/Resources");
    std::cout << "SMPL model loaded\n";

    ShapeUnderClothOptimizer optimizer(smpl, input, "C:/Users/Maria/MyDocs/GigaKorea/GK-Undressing-People-Ceres/Resources");
    std::cout << "Optimizer loaded\n";
    
    // Redirect optimizer output to file
    std::ofstream out(logFolderName + "optimization.txt");
    std::streambuf *coutbuf = std::cout.rdbuf();    //save old buf
    std::cout.rdbuf(out.rdbuf());                   //redirect std::cout to file!
    std::cout << "Input file: " << input_name << std::endl;
    std::cout << logFolderName + "optimization.txt" << std::endl;
   
    if (progress_visualization)
    {
        // collect the meshes from each iteration
        iteration_outputs.clear();
        optimizer.findOptimalParameters(&iteration_outputs);
    }
    else
        optimizer.findOptimalParameters();

    std::cout.rdbuf(coutbuf);   //  reset cout to standard output again
    out.close();
    std::cout << "Optimization finished!\n";

    // Save the results
    double* shape_res = optimizer.getEstimatesShapeParams();
    double* pose_res = optimizer.getEstimatesPoseParams();
    double* translation_res = optimizer.getEstimatesTranslationParams();
    logSMPLParams(translation_res, pose_res, shape_res, logFolderName);
    smpl->saveToObj(translation_res, pose_res, shape_res, (logFolderName + "posed_shaped.obj"));
    smpl->saveToObj(translation_res, nullptr, shape_res, (logFolderName + "unposed_shaped.obj"));
    smpl->saveToObj(translation_res, pose_res, nullptr, (logFolderName + "posed_unshaped.obj"));

    // FOR TESTING 
    //double* pose_res = new double[SMPLWrapper::POSE_SIZE];
    //double* shape_res = new double[SMPLWrapper::SHAPE_SIZE];
    //for (int i = 0; i < SMPLWrapper::POSE_SIZE; i++)
    //{
    //    pose_res[i] = 0.;
    //    if (i < SMPLWrapper::SHAPE_SIZE)
    //        shape_res[i] = 0.;
    //}
    ////pose_res[69] = 1.;
    ////pose_res[50] = 0.5;
    ////pose_res[56] = 1.;
    ////pose_res[52] =0.5;
    ////pose_res[53] = -1.;
    ////pose_res[58] = 1;
    ////pose_res[5] = 2.;
    ////pose_res[0] = -1.;
    //pose_res[6] = 0.5;
    //pose_res[15] = 1.;
    ////shape_res[0] = -0.5;

    //smpl->saveToObj(nullptr, pose_res, nullptr, logFolderName + "pose_leg_up_up.obj");

    // Visualize the output
    igl::opengl::glfw::Viewer viewer;
    igl::opengl::glfw::imgui::ImGuiMenu menu;
    viewer.plugins.push_back(&menu);

    if (progress_visualization)
    {
        counter = 0;
        viewer.callback_key_down = &visulaze_progress_key_down;
        viewer.callback_pre_draw = &visulaze_progress_pre_draw;
        viewer.core.is_animating = false;
        viewer.core.animation_max_fps = 24.;
        std::cout << "Press [space] to toggle animation." << std::endl;
    }
    else // visualizing the result only
    {
        Eigen::MatrixXd verts = smpl->calcModel(pose_res, shape_res);
        // translate
        for (int i = 0; i < SMPLWrapper::VERTICES_NUM; i++)
            for (int j = 0; j < SMPLWrapper::SPACE_DIM; j++)
                verts(i, j) += translation_res[j];
        Eigen::MatrixXi faces = smpl->getFaces();

        Eigen::VectorXd sqrD;
        Eigen::MatrixXd closest_points;
        Eigen::VectorXi closest_face_ids;
        igl::point_mesh_squared_distance(verts, input->getVertices(), input->getFaces(), sqrD, closest_face_ids, closest_points);

        Eigen::MatrixXd input_key_points (input->getKeyPoints().size(), 3);
        Eigen::MatrixXd smpl_key_points (input->getKeyPoints().size(), 3);

        CoordsDictionary inputKeyPoints = input->getKeyPoints();
        Dictionary smplKeyVerts = smpl->getKeyVertices();
        int res_id = 0;
        for (auto const& keyIterator : inputKeyPoints)
        {
            input_key_points.block(res_id, 0, 1, 3) = keyIterator.second;
            smpl_key_points.block(res_id, 0, 1, 3) = verts.row(smplKeyVerts[keyIterator.first]);
            res_id++;
        }

        viewer.data().set_mesh(verts, faces);
        viewer.data().add_points(input_key_points, Eigen::RowVector3d(1., 1., 0.));
        viewer.data().add_edges(smpl_key_points, input_key_points, Eigen::RowVector3d(1., 0., 0.));
        //viewer.data().add_points(closest_points, Eigen::RowVector3d(1., 1., 0.));
        //viewer.data().add_edges(verts, closest_points, Eigen::RowVector3d(1., 0., 0.));
        //viewer.data().set_points(Eigen::RowVector3d(1., 1., 0.), Eigen::RowVector3d(1., 1., 0.));
    }
    viewer.launch();

    // Cleaning
    delete input;
    delete smpl;
    delete[] shape_res;
    delete[] pose_res;
}

