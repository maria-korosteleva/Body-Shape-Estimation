#include "PoseShapeExtractor.h"


PoseShapeExtractor::PoseShapeExtractor(SMPLWrapper * smpl,
    const std::string & open_pose_path, const std::string & pose_prior_path,
    const std::string& logging_path)
    : smpl_(smpl),
    openpose_model_path_(open_pose_path), 
    pose_prior_path_(pose_prior_path), 
    logging_base_path_(logging_path)
{
    smpl_owner_ = false;
    logger_ = nullptr;
    optimizer_ = nullptr;
    openpose_ = nullptr;

    // as glog is used by class members
    google::InitGoogleLogging("PoseShapeExtractor");
}

PoseShapeExtractor::PoseShapeExtractor(const std::string& smpl_model_path, char gender,
    const std::string& open_pose_path,
    const std::string& pose_prior_path,
    const std::string& logging_path)
    : openpose_model_path_(open_pose_path), pose_prior_path_(pose_prior_path),
    logging_base_path_(logging_path)
{
    smpl_ = new SMPLWrapper(gender, smpl_model_path);
    smpl_owner_ = true;

    optimizer_ = nullptr;
    input_ = nullptr;
    logger_ = nullptr;
    openpose_ = nullptr;

    // as glog is used by class members
    google::InitGoogleLogging("PoseShapeExtractor");
}

PoseShapeExtractor::~PoseShapeExtractor()
{
    if (smpl_owner_)
        delete smpl_;
    if (optimizer_ != nullptr)
        delete optimizer_;
    if (openpose_ != nullptr)
        delete openpose_;
    if (logger_ != nullptr)
        delete logger_;
}

void PoseShapeExtractor::setupNewExperiment(GeneralMesh * input, const std::string experiment_name)
{
    input_ = input;
    if (logger_ != nullptr)
        delete logger_;

    logger_ = new CustomLogger(logging_base_path_, experiment_name + "_" + input_->getName());

    // for convenience
    input->saveNormalizedMesh(logger_->getLogFolderPath());

    // update tools
    if (openpose_ != nullptr)
    {
        delete openpose_;
        openpose_ = nullptr;
    }
}

SMPLWrapper* PoseShapeExtractor::runExtraction()
{
    if (input_ == nullptr)
        throw std::exception("PoseShapeExtractor: ERROR: You asked to run extraction before setting up the experiment.");
    // 1.
    int num_cameras = takePhotos_();
    
    // 2.
    estimateInitialPoseWithOP_(num_cameras);

    // 3.
    runPoseShapeOptimization_();

    // 4.
    logger_->saveFinalModel(*smpl_);
    if (save_iteration_results_)
        logger_->saveIterationsSMPLObjects(*smpl_, iteration_outputs);

    return smpl_;
}

void PoseShapeExtractor::viewFinalResult(bool withOpenPoseKeypoints)
{
    igl::opengl::glfw::Viewer viewer;
    // TODO fix imgui bug
    //igl::opengl::glfw::imgui::ImGuiMenu menu;
    //viewer.plugins.push_back(&menu);

    viewer.data().set_mesh(smpl_->calcModel(), smpl_->getFaces());
    
    if (withOpenPoseKeypoints)
    {
        if (openpose_ == nullptr)
        {
            throw std::exception("PoseShapeExtractor: openpose keypoints are "
                " unavalible for Visualization. Run extraction first.");
        }
        Eigen::MatrixXd op_keypoints = openpose_->getKeypoints();
        op_keypoints = op_keypoints.block(0, 0, op_keypoints.rows(), 3);

        viewer.data().set_points(op_keypoints, Eigen::RowVector3d(1., 1., 0.));
    }
    
    viewer.launch();
}

void PoseShapeExtractor::viewIteratoinProcess()
{
    if (iteration_outputs.size() > 0)
    {
        igl::opengl::glfw::Viewer viewer;
        igl::opengl::glfw::imgui::ImGuiMenu menu;
        viewer.plugins.push_back(&menu);

        iteration_viewer_counter_ = 0;
        /*viewer.callback_key_down = &visualizeIterationKeyDown_;
        viewer.callback_pre_draw = &visualizeIterationPreDraw_;*/
        viewer.core.is_animating = false;
        viewer.core.animation_max_fps = 24.;
        std::cout << "Press [space] to toggle animation or [Shift+F] to see the final result." << std::endl;
        viewer.launch();
    }
    else
    {
        std::cout << "PoseShapeExtractor: "
            << "I skipped visualization since iteration results were not collected." << std::endl;
    }
}

int PoseShapeExtractor::takePhotos_()
{
    std::cout << "PoseShapeExtractor: I'm taking photos of the input!" << std::endl;

    // TODO shared photographer?
    Photographer photographer(input_);

    photographer.addCameraToPosition(0.0f, 1.0f, 3.0f, 4.0f);
    photographer.addCameraToPosition(1.0f, -0.5f, 2.0f, 4.0f);
    photographer.addCameraToPosition(-1.0f, 0.0f, 1.0f, 4.0f);

    photographer.renderToImages(logger_->getPhotosFolderPath());
    photographer.saveImageCamerasParamsCV(logger_->getPhotosFolderPath());

    return 3;
}

void PoseShapeExtractor::estimateInitialPoseWithOP_(int num_pictures)
{
    if (openpose_ == nullptr)
    {
        openpose_ = new OpenPoseWrapper(logger_->getPhotosFolderPath(),
            logger_->getPhotosFolderPath(), num_pictures,
            logger_->getOpenPoseGuessesPath(),
            openpose_model_path_);
    }
    
    openpose_->runPoseEstimation();

    // Map OpenPose pose to SMPL /////
    logger_->startRedirectCoutToFile("mapping_process_info.txt");
    openpose_->mapToSmpl(*smpl_);
    logger_->endRedirectCoutToFile();
}

void PoseShapeExtractor::runPoseShapeOptimization_()
{
    // TODO Update with the SMPLWrapper changes
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
}

//bool PoseShapeExtractor::visualizeIterationPreDraw_(igl::opengl::glfw::Viewer & viewer)
//{
//    if (viewer.core.is_animating && iteration_viewer_counter_ < iteration_outputs.size())
//    {
//        viewer.data().clear();
//        Eigen::MatrixXi faces = smpl_->getFaces();
//
//        viewer.data().set_mesh(iteration_outputs[iteration_viewer_counter_], faces);
//        viewer.core.align_camera_center(iteration_outputs[iteration_viewer_counter_], faces);
//
//        // calculating point-to-surface closest points is too slow
//        // add key points for the reference
//        if (input_->getKeyPoints().size() > 0)
//        {
//            Eigen::MatrixXd input_key_points(input_->getKeyPoints().size(), 3);
//            Eigen::MatrixXd smpl_key_points(input_->getKeyPoints().size(), 3);
//
//            CoordsDictionary inputKeyPoints = input_->getKeyPoints();
//            DictionaryInt smplKeyVerts = smpl_->getKeyVertices();
//            int res_id = 0;
//            for (auto const& keyIterator : inputKeyPoints)
//            {
//                input_key_points.block(res_id, 0, 1, 3) = keyIterator.second;
//                smpl_key_points.block(res_id, 0, 1, 3) =
//                    iteration_outputs[iteration_viewer_counter_].row(smplKeyVerts[keyIterator.first]);
//                res_id++;
//            }
//            viewer.data().add_points(input_key_points, Eigen::RowVector3d(1., 1., 0.));
//            viewer.data().add_edges(smpl_key_points, input_key_points, Eigen::RowVector3d(1., 0., 0.));
//        }
//
//        iteration_viewer_counter_++;
//    }
//    else if (viewer.core.is_animating && iteration_viewer_counter_ >= iteration_outputs.size())
//    {
//        viewer.core.is_animating = false;
//        iteration_viewer_counter_ = 0;
//        std::cout << "You can start the animation again by pressing [space]" << std::endl;
//    }
//    return false;
//}

//bool PoseShapeExtractor::visualizeIterationKeyDown_(igl::opengl::glfw::Viewer & viewer, unsigned char key, int modifier)
//{
//    if (key == ' ')
//    {
//        viewer.core.is_animating = !viewer.core.is_animating;
//    }
//    else if (key == 'F')
//    {
//        std::cout << "[Shift+F] pressed: Showing the final result."
//            << "Press [space] to go back to animation mode." << std::endl;
//
//        viewer.core.is_animating = false;
//
//        // visualizing the final result only
//        viewer.data().clear();
//        Eigen::MatrixXi faces = smpl_->getFaces();
//        Eigen::MatrixXd verts = iteration_outputs[iteration_outputs.size() - 1];
//
//        Eigen::VectorXd sqrD;
//        Eigen::MatrixXd closest_points;
//        Eigen::VectorXi closest_face_ids;
//        igl::point_mesh_squared_distance(verts,
//            input_->getVertices(), input_->getFaces(), sqrD,
//            closest_face_ids, closest_points);
//
//        viewer.data().set_mesh(verts, faces);
//        viewer.data().add_edges(verts, closest_points, Eigen::RowVector3d(1., 0., 0.));
//
//        // visualize joint locations
//        Eigen::MatrixXd finJointLocations = smpl_->calcJointLocations();
//        for (int i = 0; i < finJointLocations.rows(); ++i)
//        {
//            for (int j = 0; j < SMPLWrapper::SPACE_DIM; ++j)
//            {
//                finJointLocations(i, j) += optimizer_->getEstimatesTranslationParams()[j];
//            }
//        }
//        viewer.data().add_points(finJointLocations, Eigen::RowVector3d(1., 1., 0.));
//    }
//    return false;
//}
