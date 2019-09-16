#include "OpenPoseWrapper.h"

OpenPoseWrapper::OpenPoseWrapper(const std::string images_path,
    const std::string camera_parameters_path,
    const int number_of_images,
    const std::string outpath,
    const std::string model_path)
    : images_path_(images_path), 
    cameras_path_(camera_parameters_path),
    number_of_cameras_(number_of_images),
    models_path_(model_path),
    out_path_(outpath)
{
}

OpenPoseWrapper::~OpenPoseWrapper()
{}

void OpenPoseWrapper::runPoseEstimation()
{
    try
    {
        op::log("Starting OpenPose 3DPose estimation", op::Priority::High);
        const auto opTimer = op::getTimerInit();

        // Configure OpenPose
        op::log("Configuring OpenPose...", op::Priority::High);
        // AsynchronousOut for collecting the results
        op::Wrapper opWrapper(op::ThreadManagerMode::AsynchronousOut);
        openPoseConfiguration_(opWrapper);

        // Run!!!
        op::log("Starting thread(s)...", op::Priority::High);
        opWrapper.start();

        // Save the last result
        bool success = opWrapper.waitAndPop(last_pose_datum_);
        if (!success)
            throw std::runtime_error("Processed datum could not be emplaced.");

        op::log("Stopping thread(s)", op::Priority::High);
        opWrapper.stop();

        if (!checkCorrect3DDetection_(last_pose_datum_))
        {
            throw std::runtime_error("OpenPoseWrapper::ERROR::No keypoints detected!!");
        }

        log3DKeypoints_(last_pose_datum_);
        last_pose_ = convertKeypointsToEigen_(last_pose_datum_);
        last_pose_ = normalizeKeypoints_(last_pose_);

        // Measuring total time
        op::printTime(opTimer, "OpenPose 3D pose estimation successfully finished. Total time: ", " seconds.", op::Priority::High);
    }
    catch (const std::exception& e)
    {
        op::log("OpenPose failed...", op::Priority::High);
        throw e;
    }
}

void OpenPoseWrapper::mapToSmpl(SMPLWrapper& smpl)
{
    if (last_pose_.size() == 0)
    {
        op::log("OpenPose::Error::request to match detected pose to smpl made before any pose was detected",
            op::Priority::High);
        throw std::invalid_argument("OpenPose::Error::Request to match detected pose to smpl made before any pose was detected");
    }

    sendRootRotationToSMPL_(smpl);
    sendTwistToSMPL_(smpl);
    sendLimbsRotationToSMPL_(smpl);
}

void OpenPoseWrapper::openPoseConfiguration_(op::Wrapper& opWrapper)
{
    try
    {
        // Pose configuration
        // use default and recommended configuration: BODY_25 model and Gpu mode
        op::WrapperStructPose wrapperStructPose{};
        wrapperStructPose.modelFolder = models_path_;
        wrapperStructPose.enableGoogleLogging = false; // !!!!! Conflicts when run with ceres. Not very much needed anyway
        wrapperStructPose.numberPeopleMax = 1;  // required for 3D estimation
        opWrapper.configure(wrapperStructPose);

        // Disable Face and Hand configuration 
        const op::WrapperStructFace wrapperStructFace{};
        const op::WrapperStructHand wrapperStructHand{};
        opWrapper.configure(wrapperStructFace);
        opWrapper.configure(wrapperStructHand);

        // Extra functionality configuration: going 3D!!
        op::WrapperStructExtra wrapperStructExtra{};
        wrapperStructExtra.reconstruct3d = true;
        wrapperStructExtra.minViews3d = 2;      // min views requred for successful reconstruction of the keypoint
        opWrapper.configure(wrapperStructExtra);

        // Input
        op::WrapperStructInput wrapperStructInput{};
        wrapperStructInput.cameraParameterPath = cameras_path_;
        wrapperStructInput.producerString = images_path_;
        wrapperStructInput.producerType = op::ProducerType::ImageDirectory;
        wrapperStructInput.numberViews = number_of_cameras_;
        opWrapper.configure(wrapperStructInput);

        // Output 
        op::WrapperStructOutput wrapperStructOutput{};
        wrapperStructOutput.writeJson = out_path_;
        wrapperStructOutput.writeImages = out_path_;
        opWrapper.configure(wrapperStructOutput);

        // GUI (disable any visual output)
        const op::WrapperStructGui wrapperStructGui{};
        opWrapper.configure(wrapperStructGui);

#ifdef OPENPOSE_WRAPPER_DISABLE_MULTITHREAD 
        opWrapper.disableMultiThreading();
#endif
    }
    catch (const std::exception& e)
    {
        op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
    }
}

bool OpenPoseWrapper::checkCorrect3DDetection_(PtrToDatum & datumsPtr)
{
    if (datumsPtr == nullptr || datumsPtr->empty())
        return false;

    const auto& poseKeypoints3D = datumsPtr->at(0)->poseKeypoints3D;
    if (poseKeypoints3D.getSize(1) == 0)
        return false;

    return true;
}

void OpenPoseWrapper::log3DKeypoints_(PtrToDatum & datumsPtr)
{
    if (datumsPtr != nullptr && !datumsPtr->empty())
    {
        std::ofstream out(out_path_ + pose_filename);
        // 3D
        const auto& poseKeypoints3D = datumsPtr->at(0)->poseKeypoints3D;
        int person = 0;
        for (auto bodyPart = 0; bodyPart < poseKeypoints3D.getSize(1); bodyPart++)
        {
            std::string valueToPrint;
            for (auto xyscore = 0; xyscore < poseKeypoints3D.getSize(2); xyscore++)
            {
                valueToPrint += std::to_string(poseKeypoints3D[{person, bodyPart, xyscore}]) + ", ";
            }
            out << valueToPrint << std::endl;
        }
        out.close();
    }
    else
        op::log("Nullptr or empty datumsPtr found.", op::Priority::High);
}

Eigen::MatrixXd OpenPoseWrapper::convertKeypointsToEigen_(PtrToDatum & datumsPtr)
{
    Eigen::MatrixXd keypoints;

    if (datumsPtr != nullptr && !datumsPtr->empty())
    {
        const auto& poseKeypoints3D = datumsPtr->at(0)->poseKeypoints3D;
        keypoints.resize(poseKeypoints3D.getSize(1), poseKeypoints3D.getSize(2));

        int person = 0;
        for (auto bodyPart = 0; bodyPart < poseKeypoints3D.getSize(1); bodyPart++)
        {
            for (auto xyscore = 0; xyscore < poseKeypoints3D.getSize(2); xyscore++)
            {
                keypoints(bodyPart, xyscore) = poseKeypoints3D[{person, bodyPart, xyscore}];
            }
        }
    }
    else
        op::log("ConvertPoseToEigen: Nullptr or empty datumsPtr found.", op::Priority::High);

    return keypoints;
}

Eigen::MatrixXd OpenPoseWrapper::normalizeKeypoints_(const Eigen::MatrixXd& keypoints)
{
    Eigen::MatrixXd normalized(keypoints);

    Eigen::Matrix<double, -1, 3> coords_block = normalized.block(0, 0, keypoints.rows(), 3);
    Eigen::Vector3d mean_point = coords_block.colwise().mean();
    normalized.block(0, 0, normalized.rows(), 3) = (coords_block.rowwise() - mean_point.transpose());

    return normalized;
}

bool OpenPoseWrapper::isDetected_(const int keypoint)
{
    return last_pose_(keypoint, 3) > 0.0;
}

void OpenPoseWrapper::sendRootRotationToSMPL_(SMPLWrapper & smpl)
{
    int root_id = 8, neck_id = 1, Lhip_id = 12, Rhip_id = 9;
    
    if (isDetected_(Lhip_id) && isDetected_(Rhip_id) && isDetected_(root_id) && isDetected_(neck_id))
    {
        Eigen::Vector3d up =
            (last_pose_.row(neck_id) - last_pose_.row(root_id)).transpose();
        Eigen::Vector3d right_to_left =
            (last_pose_.row(Lhip_id) - last_pose_.row(Rhip_id)).transpose();
        smpl.rotateRoot(up, right_to_left);
    } // or fail
    else
    {
        std::cout << "OpenPoseWrapper::Warning::Root rotation setup is skipped."
            << "Some of the involved keypoints were not detected." << std::endl;
    }
}

void OpenPoseWrapper::sendTwistToSMPL_(SMPLWrapper & smpl)
{
    int Rshoulder_id = 2, Lshouder_id = 5;

    if (isDetected_(Rshoulder_id) && isDetected_(Lshouder_id))
    {
        Eigen::Vector3d shoulder_dir =
            (last_pose_.row(Lshouder_id) - last_pose_.row(Rshoulder_id)).transpose();
        smpl.twistBack(shoulder_dir);
    } // or fail
    else
    {
        std::cout << "OpenPoseWrapper::Warning::Back Twist setup is skipped."
            << "Some of the involved keypoints were not detected." << std::endl;
    }
}

void OpenPoseWrapper::sendLimbsRotationToSMPL_(SMPLWrapper & smpl)
{
    const std::map<unsigned int, std::string>& keypoints_names = op::getPoseBodyPartMapping(op::PoseModel::BODY_25);

    static constexpr int n_dirs = 11;
    Eigen::Matrix<double, n_dirs, 2> directions_to_match;
    // Important: directions should follow the hierarchy -- from root to end effectors
    directions_to_match <<
        1, 0, 2, 3, 3, 4, 5, 6, 6, 7, 9, 10, 10, 11, 11, 22, 12, 13, 13, 14, 14, 19;

    for (int i = 0; i < n_dirs; i++)
    {
        int keypoint = directions_to_match(i, 0);
        int child = directions_to_match(i, 1);

        std::cout << "OP Keypoint pair " << keypoint << " -> " << child << std::endl;

        if (!isDetected_(keypoint) || !isDetected_(child))
        {
            std::cout << "OpenPoseWrapper::Warning::Keypoint pair "
                << keypoint << " -> " << child
                << " is skipped. One or both ends were not detected." << std::endl;
            continue;
        }

        // implicitly cut the 4th coordinate
        Eigen::Vector3d dir = (last_pose_.row(child) - last_pose_.row(keypoint)).transpose();

        // keypoint names are the same as smpl joints names
        smpl.rotateLimbToDirection(keypoints_names.at(keypoint), dir);
    }
}
