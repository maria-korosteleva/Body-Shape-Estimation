#pragma once
// Class incapsulates the process of the shape estimation under clothing
// Uses libigl for visualization

// need to include first, because it uses Windows.h
#include "CustomLogger.h"

#include <iostream>
#include <string>

#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/point_mesh_squared_distance.h>

#include <GeneralMesh/GeneralMesh.h>
#include <Photographer/Photographer.h>
#include "SMPLWrapper.h"
#include "ShapeUnderClothOptimizer.h"
#include "OpenPoseWrapper.h"


class PoseShapeExtractor
{
public:
    PoseShapeExtractor(SMPLWrapper* smpl,
        const std::string& open_pose_path, 
        const std::string& pose_prior_path,
        const std::string& logging_path = "");

    PoseShapeExtractor(const std::string& smpl_model_path, char gender,
        const std::string& open_pose_path,
        const std::string& pose_prior_path,
        const std::string& logging_path = "");
    ~PoseShapeExtractor();

    void setupNewExperiment(GeneralMesh* input, const std::string experiment_name = "");

    SMPLWrapper* getEstimatedModel() { return smpl_; }
    SMPLWrapper* runExtraction();

    void setSaveIntermediateResults(bool save) { save_iteration_results_ = save; };

    // TODO visualize
    // TODO void viewCameraSetupForPhotos();
    void viewFinalResult(bool withOpenPoseKeypoints = false);
    void viewIteratoinProcess();

private:
    // returns number of pictures taken
    int takePhotos_();
    void estimateInitialPoseWithOP_(int num_pictures);
    void runPoseShapeOptimization_();

    // visulization
    //static bool visualizeIterationPreDraw_(igl::opengl::glfw::Viewer & viewer);
    //static bool visualizeIterationKeyDown_(igl::opengl::glfw::Viewer& viewer, unsigned char key, int modifier);
    int iteration_viewer_counter_;

    GeneralMesh * input_;
    //static GeneralMesh * s_input_;
    SMPLWrapper* smpl_;
    bool smpl_owner_;

    CustomLogger* logger_;
    const std::string logging_base_path_;

    // utils
    std::vector<Eigen::MatrixXd> iteration_outputs;
    bool save_iteration_results_ = false;
    
    // tools
    OpenPoseWrapper* openpose_;
    const std::string openpose_model_path_;
    ShapeUnderClothOptimizer* optimizer_;
    const std::string pose_prior_path_;
};

