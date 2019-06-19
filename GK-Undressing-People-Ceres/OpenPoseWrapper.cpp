#include "OpenPoseWrapper.h"

OpenPoseWrapper::OpenPoseWrapper(const std::string images_path, 
    const std::string camera_parameters_path, 
    const std::string outpath,
    const std::string model_path)
    : images_path_(images_path), 
    cameras_path_(camera_parameters_path),
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
        op::log("Starting OpenPose demo...", op::Priority::High);
        const auto opTimer = op::getTimerInit();

        // Configure OpenPose
        op::log("Configuring OpenPose...", op::Priority::High);
        op::Wrapper opWrapper;
        openPoseConfiguration_(opWrapper);

        // Start, run, and stop processing - exec() blocks this thread until OpenPose wrapper has finished
        op::log("Starting thread(s)...", op::Priority::High);
        opWrapper.exec();

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
        wrapperStructInput.numberViews = 3;     // TODO check for need
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

void OpenPoseWrapper::get3DPoseFromFile_()
{
}
