#pragma once
//Runs the OpenPose 3D pose estimation for 3D input scan

#include <openpose/headers.hpp>

#include <GeneralMesh/GeneralMesh.h>
#include "SMPLWrapper.h"

class OpenPoseWrapper
{
public:
    OpenPoseWrapper(const std::string images_path, 
        const std::string camera_parameters_path, 
        const std::string out_path = "./tmp/",
        const std::string models_path = "./models/");
    ~OpenPoseWrapper();

    // Runs the 3D pose estimation for the input scan set before
    // all artefacts are saved to the out_path_ folder
    void runPoseEstimation();

    // Maps the found BODY25 3D pose to the SMPL skeleton
    double* mapToSmpl(SMPLWrapper * smpl);

private: 
    std::string images_path_;
    std::string cameras_path_;
    std::string out_path_;

    std::string models_path_;

    void openPoseSetup_();

    void get3DPoseFromFile_();
};

