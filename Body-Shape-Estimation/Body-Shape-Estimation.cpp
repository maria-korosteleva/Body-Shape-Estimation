// GK-Undressing-People-Ceres.cpp : This file contains the 'main' function. Program execution begins and ends there.
// It shows the example of how to use the code developed for the Undressing the input scan
//

//#define DEBUG
//#define EIGEN_STACK_ALLOCATION_LIMIT 0

// need to include first, because it uses Windows.h

#include "PoseShapeExtractor.h"
#include <GeneralMesh/GeneralMesh.h>
#include "SMPLWrapper.h"
#include "GeneralUtility.h"

/*
    TODO SMPL wrapper avalible for everyone
*/


static constexpr char output_path[] = "D:/GK-Undressing-Experiments/";
static constexpr char smpl_model_path[] = "D:/MyDocs/GigaKorea/Body-Shape-Estimation/Resources";

void generateSMPLoutput()
{
    SMPLWrapper new_smpl('f', smpl_model_path);
    SMPLWrapper::State& smpl_state_ptrs = new_smpl.getStatePointers();

    SMPLWrapper::ERMatrixXd pose(SMPLWrapper::JOINTS_NUM, SMPLWrapper::SPACE_DIM);
    pose << 0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        -0.0563716, 0.193721, -0.598922,
        0.0188691, 0.720704, -0.700351,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0;
        /*-0.0848798, 0.315784, 0.00665184,
        0.110782, 0.0117129, -0.129827,
        -0.140488, 0.0135771, 0.00507199,
        0.00119627, 0.00500833, -0.0532766,
        -0.0434787, -0.0115501, 0.11333,
        0.350265, 0.0119452, 0.00790079,
        0.00119627, 0.00500833, -0.0532766,
        0.447399, -0.110852, -0.144257,
        0.448445, -0.0578055, 0.0610899,
        0.00119627, 0.00500833, -0.0532766,
        0, 0, 0,
        0, 0, 0,
        0.00984134, 0.00632115, -0.00853605,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        -0.0563716, 0.193721, -0.598922,
        0.0188691, 0.720704, -0.700351,
        0.0259416, -0.677862, -1.58839,
        0.0484256, 0.50971, -2.11744,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0;*/

    Eigen::VectorXd shape(SMPLWrapper::SHAPE_SIZE);
    shape << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    Eigen::VectorXd translation(SMPLWrapper::SPACE_DIM);
    translation << 0, 0, 0;

    smpl_state_ptrs.pose = pose;
    smpl_state_ptrs.shape = shape;
    smpl_state_ptrs.translation = translation;

    CustomLogger logger(output_path, "smpl_pose_blendshapes_sexy_arms");

    logger.saveFinalModel(new_smpl);
}

std::vector<std::shared_ptr<GeneralMesh>> setInputs(std::string root_path) {
    std::vector<std::shared_ptr<GeneralMesh>> ret;
    std::vector<std::string> leaf_list = mg::getSubFolderDir(root_path);
    for (auto leaf : leaf_list) {
        std::vector<std::string> name_list = mg::getSubFileDir(leaf, "obj");
        for (auto i = 0; i < name_list.size(); ++i) {
            std::cout << name_list[i] << std::endl;
            if (name_list[i].find("fix_") == std::string::npos) {
                if (name_list[i].find("_Male") != std::string::npos) {
                    ret.push_back(std::make_shared<GeneralMesh>(name_list[i].c_str(), GeneralMesh::MALE));
                }
                else if (name_list[i].find("_Female") != std::string::npos) {
                    ret.push_back(std::make_shared<GeneralMesh>(name_list[i].c_str(), GeneralMesh::FEMALE));
                }
            }
        }
    }
    return ret;
}

int main()
{
    try
    {
        // std::vector<std::shared_ptr<GeneralMesh>> inputs = setInputs("E:/HumanData/OBJ_Gender/");

        std::vector<std::shared_ptr<GeneralMesh>> inputs;
        //inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/SketchFab/Sexy Girl.obj", GeneralMesh::FEMALE));
        //inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/SketchFab/Sexy_girl_90.obj", GeneralMesh::FEMALE));
        //inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/SketchFab/casual-woman-walking.obj", GeneralMesh::FEMALE));
        //inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/SketchFab/girl_nasi_pants.obj", GeneralMesh::FEMALE));
        //inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/SketchFab/jenya4.obj", GeneralMesh::FEMALE));
        //inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/SketchFab/shan.obj", GeneralMesh::FEMALE));
        //inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/SketchFab/Web.obj", GeneralMesh::MALE));
        //inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/SketchFab/reilly.obj", GeneralMesh::MALE));
        //inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/SketchFab/Ivan Komarov.obj", GeneralMesh::MALE));
        //inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/SketchFab/casual-man.obj", GeneralMesh::MALE));
        //inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/SketchFab/isaac_processed_scaled.obj", GeneralMesh::MALE));
        //inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/SketchFab/Tony_triangles_scaled.obj", GeneralMesh::MALE));
        //inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/SketchFab/JonOBJ_triangles.obj", GeneralMesh::MALE));

        // Seoungbae files
        //inputs.push_back(std::make_shared<GeneralMesh>(
        //    "C:/Users/Maria/MyDocs/GigaKorea/Seoungbae projects/cloth_data/isaac_low/isaac_low_scan.obj", 
        //    GeneralMesh::MALE, 
        //    "C:/Users/Maria/MyDocs/GigaKorea/Seoungbae projects/cloth_data/isaac_low/isaac_low_scan_scalar.txt"));
        //inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/scan_data_for_paper/new/Yoongwoon_Male.obj", GeneralMesh::MALE));

        std::cout << "Inputs are loaded! " << std::endl;
        //inputs = setInputs("D:/Data/scan_data_for_paper/");

        PoseShapeExtractor extractor(smpl_model_path, output_path);
        //extractor.setupInitialization(PoseShapeExtractor::FILE,
        //    "D:/GK-Undressing-Experiments/fit_new-sexy_girl_Female_200122_12_19/OP_guesses/smpl_op_posed_params.txt");
        extractor.setupInitialization(PoseShapeExtractor::OPENPOSE, "D:/MyDocs/libs/Installed_libs/ml_models/openpose");

#if 0   // ---- single test ----
        //extractor.setupNewExperiment(inputs[0], "refactor");
        extractor.setupNewInnerVertsParamsExperiment(inputs[0], 1, 0.05, 10., "in");
        extractor.setSaveIntermediateResults(true);
        extractor.runExtraction();
        extractor.viewIteratoinProcess();

#else   // ---- batch experiment -----
        std::ofstream fails;
        fails.open("D:/GK-Undressing-Experiments/fails.txt");
        for (auto&& input : inputs)
        {
//#define PARAM_TESTING
#ifdef PARAM_TESTING
            for (const double& in_weight : { 0.1 })
            {
                for (const double& prune_threshold : { 0.05 })  // 0.05 is the default
                {
                    for (const double& gm_threshold : { 0.7, 1., 2., 100. })
                    {
#endif
                        try
                        {
                            extractor.setupNewExperiment(input, "fit");
                            //extractor.setupNewInnerVertsParamsExperiment(input, in_weight, prune_threshold, gm_threshold, "in");
                            std::shared_ptr<SMPLWrapper> smpl_estimated = std::move(extractor.runExtraction());
#if 1 // ---- save results in the original folder ----
                            smpl_estimated->logParameters(input->getPath() + "/" + input->getName() + "_smpl_params.txt");
                            smpl_estimated->saveToObj(input->getPath() + "/" + input->getName() + "_smpl.obj");
                            input->saveNormalizedMesh(input->getPath() + "/");
#endif
                        }
                        catch (std::exception& e)
                        {
                            std::cout << "exception encountered: " << e.what() << std::endl;
                            fails << input->getPath() + "/" + input->getName() << std::endl;
                        }
#ifdef PARAM_TESTING
                    }
                }
            }
#endif
        }
#endif
    }
    catch (std::exception& e)
    {
        std::cout << "Exception encountered: " << e.what() << std::endl
            << "Terminating" << std::endl;
    }
}

