// GK-Undressing-People-Ceres.cpp : This file contains the 'main' function. Program execution begins and ends there.
// It shows the example of how to use the code developed for the Undressing the input scan
//

//#define DEBUG
//#define EIGEN_STACK_ALLOCATION_LIMIT 0

// need to include first, because it uses Windows.h

#include "PoseShapeExtractor.h"
#include <GeneralMesh/GeneralMesh.h>
#include "SMPLWrapper.h"

/*
    TODO SMPL wrapper avalible for everyone
*/


static constexpr char output_path[] = "C:/Users/Maria/MyDocs/GigaKorea/GK-Undressing-People-Ceres/Outputs/";
static constexpr char smpl_model_path[] = "C:/Users/Maria/MyDocs/GigaKorea/GK-Undressing-People-Ceres/Resources";

void generateSMPLoutput()
{
    SMPLWrapper new_smpl('f', smpl_model_path);
    SMPLWrapper::State smpl_state_ptrs = new_smpl.getStatePointers();

    smpl_state_ptrs.pose[50] = -0.7854; // pi/4
    smpl_state_ptrs.pose[53] = 0.7854;
    //smpl_state_ptrs.shape[0] = -0.5;

    CustomLogger logger(output_path, "smpl_output");

    logger.saveFinalModel(new_smpl);
}

int main()
{
    try
    {
        std::vector<std::shared_ptr<GeneralMesh>> inputs;
        //inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/DYNA/50004_jumping_jacks/00000.obj", GeneralMesh::FEMALE));
        //inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/smpl_outs/pose_00048_270_dyna_thin.obj", GeneralMesh::FEMALE));
        inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/smpl_outs/pose_50004_knees_270_dyna_thin.obj", GeneralMesh::FEMALE));
        //inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/smpl_outs/pose_50004_knees_270_dyna_fat.obj", GeneralMesh::FEMALE));
        //inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/smpl_outs/pose_leg_up_knee_up.obj", GeneralMesh::FEMALE));
        //inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/smpl_outs/pose_mean.obj", GeneralMesh::FEMALE));
        //inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/DYNA/50004_jiggle_on_toes/00048.obj", GeneralMesh::FEMALE));
        //inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/DYNA/50004_chicken_wings/00091.obj", GeneralMesh::FEMALE));
        //inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/DYNA/50004_knees/00070.obj", GeneralMesh::FEMALE));
        //inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/DYNA/50004_knees/00130.obj", GeneralMesh::FEMALE));
        //inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/DYNA/50004_knees/00270.obj", GeneralMesh::FEMALE));
        //inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/DYNA/50004_punching/00053.obj", GeneralMesh::FEMALE));
        //inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/SketchFab/Sexy Girl.obj", GeneralMesh::FEMALE));
        //inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/SketchFab/casual-woman-walking.obj", GeneralMesh::FEMALE));
        //inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/SketchFab/girl_nasi_pants.obj", GeneralMesh::FEMALE));
        //inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/SketchFab/jenya4.obj", GeneralMesh::FEMALE));
        //inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/SketchFab/shan.obj", GeneralMesh::FEMALE));
        //inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/SketchFab/Web.obj", GeneralMesh::MALE));
        //inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/SketchFab/reilly.obj", GeneralMesh::MALE));
        //inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/SketchFab/JonOBJ.obj", GeneralMesh::MALE));
        //inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/SketchFab/Ivan Komarov.obj", GeneralMesh::MALE));
        //inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/SketchFab/casual-man.obj", GeneralMesh::MALE));

        std::cout << "Inputs are loaded! " << std::endl;

        PoseShapeExtractor extractor(smpl_model_path,
            "C:/Users/Maria/MyDocs/libs/Installed_libs/ml_models/openpose",
            "C:/Users/Maria/MyDocs/GigaKorea/GK-Undressing-People-Ceres/Resources",
            output_path);

        extractor.setupNewExperiment(inputs[0], "ceres_opt");
        extractor.setSaveIntermediateResults(true);
        extractor.runExtraction();
        extractor.viewIteratoinProcess();

        //extractor.viewCameraSetupForPhotos();
        //extractor.viewFinalResult(true);

        //for (auto&& input : inputs)
        //{
        //    for (const double& rate : { 0.})
        //    {
        //        try
        //        {
        //            extractor.setupNewShapeRegExperiment(input, rate, "ps_shape_reg_zero");
        //            extractor.runExtraction();
        //        }
        //        catch (std::exception& e)
        //        {
        //            std::cout << "Exception encountered: " << e.what() << std::endl;
        //        }
        //    }
        //}
    }
    catch (std::exception& e)
    {
        std::cout << "Exception encountered: " << e.what() << std::endl
            << "Terminating" << std::endl;
    }
}

