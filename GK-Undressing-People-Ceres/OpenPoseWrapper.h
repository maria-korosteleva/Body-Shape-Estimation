#pragma once
//Runs the OpenPose 3D pose estimation for 3D input scan

#include <fstream>
#include <openpose/headers.hpp>

#include <GeneralMesh/GeneralMesh.h>
#include "SMPLWrapper.h"

// Set to single-thread (for sequential processing and/or debugging and/or reducing latency)
//#define OPENPOSE_WRAPPER_DISABLE_MULTITHREAD 

using PtrToDatum = std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>;

class OpenPoseWrapper
{
public:
    OpenPoseWrapper(const std::string images_path,
        const std::string camera_parameters_path,
        const int number_of_cameras,
        const std::string out_path = "./tmp/",
        const std::string models_path = "./models/");
    ~OpenPoseWrapper();

    // Each keypoint is in 4D, indicating if it was estimated
    Eigen::MatrixXd getKeypoints() { return last_pose_; }

    // Runs the 3D pose estimation for the input scan set before
    // all artefacts are saved to the out_path_ folder
    // the code is mostly borrowed from the OpenPoseDemo: 
    // https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/examples/openpose/openpose.cpp
    void runPoseEstimation();

    // Maps the found BODY25 3D pose to the SMPL skeleton
    void mapToSmpl(SMPLWrapper& smpl);

private: 
    static constexpr char pose_filename[] = "3D_keypoints.txt";

    // the code is mostly borrowed from the OpenPoseDemo :
    // https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/examples/openpose/openpose.cpp
    // Plus https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/examples/tutorial_api_cpp/11_asynchronous_custom_output.cpp
    void openPoseConfiguration_(op::Wrapper& opWrapper);

    // see https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/examples/tutorial_api_cpp/11_asynchronous_custom_output.cpp
    void log3DKeypoints_(PtrToDatum& datumsPtr);

    Eigen::MatrixXd convertKeypointsToEigen_(PtrToDatum& datumsPtr);
    Eigen::MatrixXd normalizeKeypoints_(const Eigen::MatrixXd& keypoints);

    bool isDetected_(const int keypoint);
    
    std::string images_path_;
    std::string cameras_path_;
    int number_of_cameras_;
    std::string out_path_;

    std::string models_path_;

    PtrToDatum last_pose_datum_;
    Eigen::MatrixXd last_pose_;

};

