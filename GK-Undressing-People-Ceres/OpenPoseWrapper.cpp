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
        op::log("Starting OpenPose 3D pose estimation...", op::Priority::High);
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
            throw std::exception("Processed datum could not be emplaced.");

        op::log("Stopping thread(s)", op::Priority::High);
        opWrapper.stop();

        log3DKeypoints_(last_pose_datum_);
        last_pose_ = convertKeypointsToEigen_(last_pose_datum_);

        // Measuring total time
        op::printTime(opTimer, "OpenPose 3D pose estimation successfully finished. Total time: ", " seconds.", op::Priority::High);
    }
    catch (const std::exception& e)
    {
        op::log("OpenPose failed...", op::Priority::High);
    }
}

double * OpenPoseWrapper::mapToSmpl(SMPLWrapper * smpl)
{
    return nullptr;
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
