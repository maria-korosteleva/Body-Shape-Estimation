// GK-Undressing-People-Ceres.cpp : This file contains the 'main' function. Program execution begins and ends there.
// It shows the example of how to use the code developed for the Undressing the input scan
//

//#define DEBUG
//#define EIGEN_STACK_ALLOCATION_LIMIT 0

// need to include first, because it uses Windows.h
#include "CustomLogger.h"

#include <iostream>

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
    TODO initial pose estimation with openpose
    TODO use normalized General Mesh vertices for optimization 
    TODO Shape regularization 
    TODO directional pose estimation -- idea: add it to the main objective as additional resudual
    TODO move (important) parameters outside
    TODO Idea: allow start optimization from the last results
    TODO libigl as static library
    TODO SMPL wrapper avalible for everyone
*/


static constexpr char output_path[] = "C:/Users/Maria/MyDocs/GigaKorea/GK-Undressing-People-Ceres/Outputs/";
static constexpr char smpl_model_path[] = "C:/Users/Maria/MyDocs/GigaKorea/GK-Undressing-People-Ceres/Resources";

// global vars are needed for visualization purposes only
std::vector<Eigen::MatrixXd> iteration_outputs;
int counter = 0;
SMPLWrapper* smpl;
GeneralMesh* input;
ShapeUnderClothOptimizer* optimizer;

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
            DictionaryInt smplKeyVerts = smpl->getKeyVertices();
            int res_id = 0;
            for (auto const& keyIterator : inputKeyPoints)
            {
                input_key_points.block(res_id, 0, 1, 3) = keyIterator.second;
                smpl_key_points.block(res_id, 0, 1, 3) = 
                    iteration_outputs[counter].row(smplKeyVerts[keyIterator.first]);
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
        std::cout << "[Shift+F] pressed: Showing the final result."
            << "Press [space] to go back to animation mode." << std::endl;

        viewer.core.is_animating = false;
        
        // visualizing the final result only
        viewer.data().clear();
        Eigen::MatrixXi faces = smpl->getFaces();
        Eigen::MatrixXd verts = iteration_outputs[iteration_outputs.size()-1];

        Eigen::VectorXd sqrD;
        Eigen::MatrixXd closest_points;
        Eigen::VectorXi closest_face_ids;
        igl::point_mesh_squared_distance(verts, 
            input->getVertices(), input->getFaces(), sqrD, 
            closest_face_ids, closest_points);

        viewer.data().set_mesh(verts, faces);
        viewer.data().add_edges(verts, closest_points, Eigen::RowVector3d(1., 0., 0.));

        // visualize joint locations
        Eigen::MatrixXd finJointLocations = smpl->calcJointLocations();
        for (int i = 0; i < finJointLocations.rows(); ++i)
        {
            for (int j = 0; j < SMPLWrapper::SPACE_DIM; ++j)
            {
                finJointLocations(i, j) += optimizer->getEstimatesTranslationParams()[j];
            }
        }
        viewer.data().add_points(finJointLocations, Eigen::RowVector3d(1., 1., 0.));
    }
    return false;
}

void generateSMPLoutput()
{
    SMPLWrapper new_smpl('f', smpl_model_path);
    SMPLWrapper::State smpl_state_ptrs = new_smpl.getStatePointers();

    smpl_state_ptrs.pose[50] = -0.7854; // pi/4
    smpl_state_ptrs.pose[53] = 0.7854;
    //smpl_state_ptrs.shape[0] = -0.5;

    CustomLogger logger(output_path, "smpl_output");

    logger.saveFinalModel(new_smpl);
}

int main()
{
    try {
        char gender = 'f';
        const char* input_name = "D:/Data/DYNA/50004_jumping_jacks/00000.obj";  // A-pose
        //const char* input_name = "D:/Data/SketchFab/Sexy Girl.obj";  // A-pose
        //gender = 'm';
        //const char* input_name = "D:/Data/SketchFab/Web.obj";

        // for SMPL/DYNA inputs
        //const char* input_key_vertices_name = "D:/Data/smpl_outs/smpl_key_vertices.txt";
        //input = new GeneralMesh(input_name, input_key_vertices_name);

        ///// 1. Load and Initialize stuff /////
        input = new GeneralMesh(input_name);
        std::cout << "Input mesh loaded!" << std::endl;
        smpl = new SMPLWrapper(gender, smpl_model_path);
        std::cout << "SMPL model loaded" << std::endl;
        optimizer = new ShapeUnderClothOptimizer(smpl, input, 
            "C:/Users/Maria/MyDocs/GigaKorea/GK-Undressing-People-Ceres/Resources");
        std::cout << "Optimizer loaded" << std::endl;
        Photographer photographer(input);
        std::cout << "Photographer loaded" << std::endl;

        CustomLogger logger("C:/Users/Maria/MyDocs/GigaKorea/GK-Undressing-People-Ceres/Outputs/", 
            "map_op_most_body" + input->getName());

        ///// 2. Initial pose estimation /////
        ////// 2.1 Prepare pictures for OpenPose: Photographer /////
        photographer.addCameraToPosition(0.0f, 1.0f, 3.0f, 4.0f);
        photographer.addCameraToPosition(1.0f, -0.5f, 2.0f, 4.0f);
        photographer.addCameraToPosition(-1.0f, 0.0f, 1.0f, 4.0f);

        photographer.renderToImages(logger.getPhotosFolderPath());
        photographer.saveImageCamerasParamsCV(logger.getPhotosFolderPath());

        ////// 2.2 Run OpenPose /////
        OpenPoseWrapper openpose(logger.getPhotosFolderPath(),
            logger.getPhotosFolderPath(), 3,
            logger.getOpenPoseGuessesPath(), 
            "C:/Users/Maria/MyDocs/libs/Installed_libs/ml_models/openpose");
        openpose.runPoseEstimation();

        ////// TODO 2.3 Map OpenPose pose to SMPL /////
        openpose.mapToSmpl(*smpl);

        logger.saveFinalModel(*smpl);

        igl::opengl::glfw::Viewer viewer;
        //igl::opengl::glfw::imgui::ImGuiMenu menu;
        //viewer.plugins.push_back(&menu);
        viewer.data().set_mesh(smpl->calcModel(), smpl->getFaces());
        Eigen::MatrixXd op_keypoints = openpose.getKeypoints();
        op_keypoints = op_keypoints.block(0, 0, op_keypoints.rows(), 3);

        viewer.data().set_points(op_keypoints, Eigen::RowVector3d(1., 1., 0.));
        viewer.launch();

        ///// 3. Run shape&pose optimization ////
        /// TODO Update with the SMPLWrapper changes
        // for experiments
        //int gm_params[] = { 0, 10, 50 };

        //for (int i = 0; i < 5; i++)
        //{
        //    CustomLogger gm_logger(output_path, "in_shape_gem_mc_" + std::to_string(gm_params[i]) + input->getName());
        //    // save input for convenience
        //    igl::writeOBJ(gm_logger.getLogFolderPath() + input->getName() + ".obj", 
        //      input->getVertices(), input->getFaces());

        //    gm_logger.startRedirectCoutToFile("optimization.txt");
        //    std::cout << "Input file: " << input_name << std::endl;

        //    // collect the meshes from each iteration
        //    iteration_outputs.clear();
        //    //optimizer.findOptimalParameters(&iteration_outputs, outside_shape_param);
        //    optimizer->findOptimalParameters(nullptr, gm_params[i]);

        //    gm_logger.endRedirectCoutToFile();
        //    std::cout << "Optimization finished!\n";

        //    // Save the results
        //    gm_logger.logSMPLParams(*smpl, *optimizer);
        //    gm_logger.saveFinalSMPLObject(*smpl, *optimizer);
        //}
        

        ///// 4. Visualize the optimization progress ////
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

