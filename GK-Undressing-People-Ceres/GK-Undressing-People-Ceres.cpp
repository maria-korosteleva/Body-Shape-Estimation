// GK-Undressing-People-Ceres.cpp : This file contains the 'main' function. Program execution begins and ends there.
// It shows the example of how to use the code developed for the Undressing the input scan
//

//#define DEBUG
//#define EIGEN_STACK_ALLOCATION_LIMIT 0
// WARNING! Uses win-specific features to create log directory 
#define NOMINMAX
#include <Windows.h>
#undef THIS

#include <iostream>
#include <ctime>

#include <assert.h>

#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/point_mesh_squared_distance.h>

#include <GeneralMesh/GeneralMesh.h>
#include <Photographer/Photographer.h>
#include "SMPLWrapper.h"
#include "ShapeUnderClothOptimizer.h"
#include "OpenPoseWrapper.h"

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
    + Acuurate shape and pose estimation (iterative?)

    - initial pose estimation with openpose

    - use normalized General Mesh vertices for optimization 

    - Shape regularization 
    x directional pose estimation -- idea: add it to the main objective as additional resudual
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
std::vector<Eigen::MatrixXd> iteration_outputs;
int counter = 0;
SMPLWrapper* smpl;
GeneralMesh* input;
double* shape_res;
double* pose_res;
double* translation_res;

std::string getNewLogFolder(const std::string tag = "test")
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


void logSMPLParams(double* translation, double* pose, double* shape, Eigen::MatrixXd& jointsLoc, std::string logFolderName)
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

    out << "Pose params [ \n";
    if (pose != nullptr)
    {
        for (int i = 0; i < SMPLWrapper::JOINTS_NUM; i++)
        {
            for (int j = 0; j < SMPLWrapper::SPACE_DIM; j++)
            {
                out << pose[i * SMPLWrapper::SPACE_DIM + j] << " , ";
            }
            out << std::endl;
        }
    }
    else
    {
        for (int i = 0; i < SMPLWrapper::JOINTS_NUM; i++)
        {
            for (int j = 0; j < SMPLWrapper::SPACE_DIM; j++)
            {
                out << "0." << " , ";
            }
            out << std::endl;
        }
    }

    out << "]" << std::endl;

    out << "Shape (betas) params [ \n";
    if (shape != nullptr)
        for (int i = 0; i < SMPLWrapper::SHAPE_SIZE; i++)
            out << shape[i] << " , ";
    else
        for (int i = 0; i < SMPLWrapper::SHAPE_SIZE; i++)
            out << "0." << " , ";
    out << std::endl << "]" << std::endl;

    out << "Joints locations for posed and shaped model [\n";
    // translate
    Eigen::MatrixXd translatedJointLoc(jointsLoc);
    for (int i = 0; i < translatedJointLoc.rows(); ++i)
    {
        for (int j = 0; j < SMPLWrapper::SPACE_DIM; ++j)
        {
            translatedJointLoc(i, j) += translation[j];
        }
    }
    if (translatedJointLoc.size() > 0)
    {
        out << translatedJointLoc << std::endl;
    }

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

        // calculating point-to-surface closest points is too slow
        // add key points for the reference
        if (input->getKeyPoints().size() > 0)
        {
            Eigen::MatrixXd input_key_points(input->getKeyPoints().size(), 3);
            Eigen::MatrixXd smpl_key_points(input->getKeyPoints().size(), 3);

            CoordsDictionary inputKeyPoints = input->getKeyPoints();
            Dictionary smplKeyVerts = smpl->getKeyVertices();
            int res_id = 0;
            for (auto const& keyIterator : inputKeyPoints)
            {
                input_key_points.block(res_id, 0, 1, 3) = keyIterator.second;
                smpl_key_points.block(res_id, 0, 1, 3) = iteration_outputs[counter].row(smplKeyVerts[keyIterator.first]);
                res_id++;
            }
            viewer.data().add_points(input_key_points, Eigen::RowVector3d(1., 1., 0.));
            viewer.data().add_edges(smpl_key_points, input_key_points, Eigen::RowVector3d(1., 0., 0.));
        }

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
    else if (key == 'F')
    {
        std::cout << "[Shift+F] pressed: Showing the final result. Press [space] to go back to animation mode." << std::endl;

        viewer.core.is_animating = false;
        
        // visualizing the final result only
        viewer.data().clear();
        Eigen::MatrixXi faces = smpl->getFaces();
        Eigen::MatrixXd verts = iteration_outputs[iteration_outputs.size()-1];

        Eigen::VectorXd sqrD;
        Eigen::MatrixXd closest_points;
        Eigen::VectorXi closest_face_ids;
        igl::point_mesh_squared_distance(verts, input->getVertices(), input->getFaces(), sqrD, closest_face_ids, closest_points);

        viewer.data().set_mesh(verts, faces);
        viewer.data().add_edges(verts, closest_points, Eigen::RowVector3d(1., 0., 0.));

        // visualize joint locations
        Eigen::MatrixXd finJointLocations = smpl->calcJointLocations(shape_res, pose_res);
        for (int i = 0; i < finJointLocations.rows(); ++i)
        {
            for (int j = 0; j < SMPLWrapper::SPACE_DIM; ++j)
            {
                finJointLocations(i, j) += translation_res[j];
            }
        }
        viewer.data().add_points(finJointLocations, Eigen::RowVector3d(1., 1., 0.));
    }
    return false;
}


int main()
{
    try {
        // Females
        char gender = 'f';
        const char* input_name = "D:/Data/DYNA/50004_jumping_jacks/00000.obj";  // A-pose
        //const char* input_name = "D:/Data/smpl_outs/pose_hand_up.obj";
        //const char* input_name = "D:/Data/smpl_outs/smpl_2.obj";
        //const char* input_name = "D:/Data/Clo/clo_model.obj";
        //Males
        //gender = 'm';
        //const char* input_name = "D:/Data/SketchFab/Web.obj";

        // for SMPL/DYNA inputs
        //const char* input_key_vertices_name = "D:/Data/smpl_outs/smpl_key_vertices.txt";
        //input = new GeneralMesh(input_name, input_key_vertices_name);

        input = new GeneralMesh(input_name);
        std::cout << "Input mesh loaded!\n";
        smpl = new SMPLWrapper(gender, "C:/Users/Maria/MyDocs/GigaKorea/GK-Undressing-People-Ceres/Resources");
        std::cout << "SMPL model loaded\n";
        ShapeUnderClothOptimizer optimizer(smpl, input, "C:/Users/Maria/MyDocs/GigaKorea/GK-Undressing-People-Ceres/Resources");
        std::cout << "Optimizer loaded\n";

        //////// NEW CODE: 
        std::string logFolderName = getNewLogFolder("OP_images_" + input->getName());
        ////// Photographer
        Photographer photographer(input);

        photographer.addCameraToPosition(0.0f, 1.0f, 3.0f, 4.0f);
        photographer.addCameraToPosition(1.0f, -0.5f, 2.0f, 4.0f);
        photographer.addCameraToPosition(-1.0f, 0.0f, 1.0f, 4.0f);

        CreateDirectory((logFolderName + "/images/").c_str(), NULL);
        photographer.renderToImages(logFolderName + "/images/");
        photographer.saveImageCamerasParamsCV(logFolderName + "/images/");

        //photographer.viewScene();

        /////// OpenPose
        OpenPoseWrapper openpose(input, logFolderName);
        openpose.runPoseEstimation();

        ////////

        // for experiments
        //int gm_params[] = { 0, 10, 50 };

        //for (int i = 0; i < 5; i++)
        //{
        //    // Logging For convenience
        //    //std::string logFolderName = getNewLogFolder("3cyc_ptrsA_in_scale_" + std::to_string(inside_sclaing_param) + input->getName());
        //    std::string logFolderName = getNewLogFolder("in_shape_gem_mc_" + std::to_string(gm_params[i]) + input->getName());
        //    igl::writeOBJ(logFolderName + input->getName() + ".obj", input->getVertices(), input->getFaces());

        //    // Redirect optimizer output to file
        //    std::ofstream out(logFolderName + "optimization.txt");
        //    std::streambuf *coutbuf = std::cout.rdbuf();    //save old buf
        //    std::cout.rdbuf(out.rdbuf());                   //redirect std::cout to file!
        //    std::cout << "Input file: " << input_name << std::endl;
        //    std::cout << logFolderName + "optimization.txt" << std::endl;

        //    // collect the meshes from each iteration
        //    iteration_outputs.clear();
        //    //optimizer.findOptimalParameters(&iteration_outputs, outside_shape_param);
        //    optimizer.findOptimalParameters(nullptr, gm_params[i]);

        //    std::cout.rdbuf(coutbuf);   //  reset cout to standard output again
        //    out.close();
        //    std::cout << "Optimization finished!\n";

        //    // Save the results
        //    shape_res = optimizer.getEstimatesShapeParams();
        //    pose_res = optimizer.getEstimatesPoseParams();
        //    translation_res = optimizer.getEstimatesTranslationParams();
        //    Eigen::MatrixXd finJointLocations = smpl->calcJointLocations(shape_res, pose_res);

        //    logSMPLParams(translation_res, pose_res, shape_res, finJointLocations, logFolderName);
        //    smpl->saveToObj(translation_res, pose_res, shape_res, (logFolderName + "posed_shaped.obj"));
        //    smpl->saveToObj(translation_res, nullptr, shape_res, (logFolderName + "unposed_shaped.obj"));
        //    smpl->saveToObj(translation_res, pose_res, nullptr, (logFolderName + "posed_unshaped.obj"));

        //    delete[] shape_res;
        //    delete[] pose_res;
        //}
        

        // FOR TESTING 
        //double* pose_res = new double[SMPLWrapper::POSE_SIZE];
        //double* shape_res = new double[SMPLWrapper::SHAPE_SIZE];
        //double translation_res[3] = { 0, 0, 0 };
        //for (int i = 0; i < SMPLWrapper::POSE_SIZE; i++)
        //{
        //    pose_res[i] = 0.;
        //    if (i < SMPLWrapper::SHAPE_SIZE)
        //        shape_res[i] = 0.;
        //}
        //pose_res[50] = -0.7854; // pi/4
        //pose_res[53] = 0.7854;
        ////shape_res[0] = -0.5;

        //smpl->saveToObj(nullptr, pose_res, nullptr, logFolderName + "pose_A.obj");
        //logSMPLParams(translation_res, pose_res, shape_res, logFolderName);

        // Visualize the output
        if (iteration_outputs.size() > 0)
        {
            igl::opengl::glfw::Viewer viewer;
            igl::opengl::glfw::imgui::ImGuiMenu menu;
            viewer.plugins.push_back(&menu);

            counter = 0;
            viewer.callback_key_down = &visulaze_progress_key_down;
            viewer.callback_pre_draw = &visulaze_progress_pre_draw;
            viewer.core.is_animating = false;
            viewer.core.animation_max_fps = 24.;
            std::cout << "Press [space] to toggle animation or [Shift+F] to see the final result." << std::endl;
            viewer.launch();
        }
        else
        {
            std::cout << "I skipped visualization since iteration results were not collected." << std::endl;
        }

        // Cleaning
        delete input;
        delete smpl;
    }
    catch (std::exception& e)
    {
        std::cout << "Exception encountered: " << e.what() << std::endl
            << "Terminating." << std::endl;
    }
}

