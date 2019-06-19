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
{
}

void OpenPoseWrapper::runPoseEstimation()
{

    // Setup Openpose
    openPoseSetup_();

    // Run Openpose

    // Get Results
}

double * OpenPoseWrapper::mapToSmpl(SMPLWrapper * smpl)
{
    return nullptr;
}

void OpenPoseWrapper::openPoseSetup_()
{
}

void OpenPoseWrapper::get3DPoseFromFile_()
{
}
