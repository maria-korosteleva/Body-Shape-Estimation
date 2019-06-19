#pragma once
//Runs the OpenPose 3D pose estimation for 3D input scan

#include <igl/opengl/glfw/Viewer.h>

#include <GeneralMesh/GeneralMesh.h>
#include "SMPLWrapper.h"

class OpenPoseWrapper
{
public:
    OpenPoseWrapper(GeneralMesh* input, const char* outpath = "./tmp/");
    OpenPoseWrapper(GeneralMesh* input, const std::string outpath = "./tmp/");
    ~OpenPoseWrapper();

    // setters-getters
    void setNewInput(GeneralMesh* input);
    void setNewOutPath(const char* outpath = "./tmp/");

    // Runs the 3D pose estimation for the input scan set before
    // all artefacts are saved to the out_path_ folder
    void runPoseEstimation();

    // Maps the found BODY25 3D pose to the SMPL skeleton
    double* mapToSmpl(SMPLWrapper * smpl);

private: 
    std::string out_path_;

    static constexpr char IMG_FOLDER[] = "images/";
    static constexpr char CAMERA_FOLDER[] = "cameras_parameters/";
    static constexpr char GUESS_FOLDER[] = "openpose_guesses/";

    GeneralMesh * input_ = nullptr;

    void photoshoot_();

    void openPoseSetup_();

};

