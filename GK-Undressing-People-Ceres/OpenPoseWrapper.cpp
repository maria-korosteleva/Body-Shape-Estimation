#include "pch.h"
#include "OpenPoseWrapper.h"


OpenPoseWrapper::OpenPoseWrapper(GeneralMesh * input, const char * outpath)
    : input_(input), out_path_(outpath)
{

}

OpenPoseWrapper::~OpenPoseWrapper()
{
}

void OpenPoseWrapper::setNewInput(GeneralMesh * input)
{
    this->input_ = input;
}

void OpenPoseWrapper::setNewOutPath(const char * outpath)
{
    this->out_path_ = outpath;
}

void OpenPoseWrapper::runPoseEstimation()
{
    // Render images for the input
    this->photoshoot_();

    // Setup Openpose
    this->openPoseSetup_();

    // Run Openpose

    // Get Results
}

double * OpenPoseWrapper::mapToSmpl(SMPLWrapper * smpl)
{
    return nullptr;
}

void OpenPoseWrapper::photoshoot_()
{
    // hmm?
}

void OpenPoseWrapper::openPoseSetup_()
{
}
