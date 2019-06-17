#include "pch.h"
#include "OpenPoseWrapper.h"


OpenPoseWrapper::OpenPoseWrapper(GeneralMesh * input, const char * outpath)
    : input_(input), out_path_(outpath)
{

}

OpenPoseWrapper::OpenPoseWrapper(GeneralMesh * input, const std::string outpath)
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
    // load normalized mesh into the scene
    //igl::opengl::glfw::Viewer viewer;
    //viewer.data().set_mesh(this->input_->getNormalizedVertices(), this->input_->getFaces());

    //// set camera
    //// set lights
    //// render
    //// save as image

    //// From https://github.com/libigl/libigl/blob/master/tutorial/607_ScreenCapture/main.cpp#L12
    //// Allocate temporary buffers
    //Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R(1280, 800);
    //Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> G(1280, 800);
    //Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> B(1280, 800);
    //Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> A(1280, 800);

    //// Draw the scene in the buffers
    //viewer.core.draw_buffer(
    //    viewer.data(), false, R, G, B, A);

    // Save it to a PNG
    //igl::png::writePNG(R, G, B, A, this->out_path_ + this->IMG_FOLDER + "1.png");
}

void OpenPoseWrapper::openPoseSetup_()
{
}
