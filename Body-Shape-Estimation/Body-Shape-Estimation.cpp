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
static constexpr char smpl_model_path[] = "C:/Users/Maria/MyDocs/GigaKorea/GK-Undressing-People-Ceres/Resources";

void generateSMPLoutput()
{
    SMPLWrapper new_smpl('f', smpl_model_path);
    SMPLWrapper::State& smpl_state_ptrs = new_smpl.getStatePointers();

    SMPLWrapper::ERMatrixXd pose(SMPLWrapper::JOINTS_NUM, SMPLWrapper::SPACE_DIM);
    pose << -0.0848798, 0.315784, 0.00665184,
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
        0, 0, 0;

    Eigen::VectorXd shape(SMPLWrapper::SHAPE_SIZE);
    shape << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    Eigen::VectorXd translation(SMPLWrapper::SPACE_DIM);
    translation << 0, 0, 0;

    smpl_state_ptrs.pose = pose;
    smpl_state_ptrs.shape = shape;
    smpl_state_ptrs.translation = translation;

    CustomLogger logger(output_path, "smpl_op_guess_saved");

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
        std::vector<std::shared_ptr<GeneralMesh>> inputs = setInputs("E:/HumanData/OBJ_Gender/");
        // std::vector<pair<string, int>> ret = myfunc(, );
        //inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/smpl_outs/smpl_3.obj", GeneralMesh::FEMALE));
        //inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/DYNA/50004_jumping_jacks/00000.obj", GeneralMesh::FEMALE));
        //inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/smpl_outs/pose_00048_270_dyna_thin.obj", GeneralMesh::FEMALE));
        //inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/smpl_outs/pose_50004_knees_270_dyna_thin.obj", GeneralMesh::FEMALE));
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

        // Tests for Seoungbae
        //inputs.push_back(std::make_shared<GeneralMesh>(
        //    "D:/Data/From Mingi's purchase/rp_corey_posed_010_MAYA/scenes/rp_corey_posed_010_30k.obj", 
        //    GeneralMesh::MALE));
        //inputs.push_back(std::make_shared<GeneralMesh>(
        //    "D:/Data/From Mingi's purchase/rp_ethan_posed_007_MAYA/scenes/rp_ethan_posed_007_30k.obj",
        //    GeneralMesh::MALE));
        //inputs.push_back(std::make_shared<GeneralMesh>(
        //    "D:/Data/From Mingi's purchase/rp_helen_posed_012_MAYA/scenes/rp_helen_posed_012_30k.obj",
        //    GeneralMesh::FEMALE));
        //inputs.push_back(std::make_shared<GeneralMesh>(
        //    "D:/Data/From Mingi's purchase/rp_julia_posed_001_MAYA/scenes/rp_julia_posed_001_30k.obj",
        //    GeneralMesh::FEMALE));
        //inputs.push_back(std::make_shared<GeneralMesh>(
        //    "D:/Data/From Mingi's purchase/rp_philip_posed_015_MAYA/scenes/rp_philip_posed_015_30k.obj",
        //    GeneralMesh::MALE));
        ////inputs.push_back(std::make_shared<GeneralMesh>(
        ////    "D:/Data/From Mingi's purchase/rp_scott_posed_022_MAYA/scenes/rp_scott_posed_022_30k.obj",
        ////    GeneralMesh::MALE));
        //// Hi-res
        //inputs.push_back(std::make_shared<GeneralMesh>(
        //    "D:/Data/From Mingi's purchase/Female Vis Models/Female 3/OBJ/Female_03_100k.obj",
        //    GeneralMesh::FEMALE));
        //inputs.push_back(std::make_shared<GeneralMesh>(
        //    "D:/Data/From Mingi's purchase/Female Vis Models/Female 11/OBJ/Female_011_100k.obj",
        //    GeneralMesh::FEMALE));
        //inputs.push_back(std::make_shared<GeneralMesh>(
        //    "D:/Data/From Mingi's purchase/Female Vis Models/Female 8/OBJ/Female_08_100k.obj",
        //    GeneralMesh::FEMALE));


        std::cout << "Inputs are loaded! " << std::endl;

        PoseShapeExtractor extractor(smpl_model_path, output_path);
        //extractor.setupInitialization(PoseShapeExtractor::FILE,
        //    "D:/GK-Undressing-Experiments/file_format_SketchFab-Sexy Girl_190916_10_45/OP_guesses/smpl_op_posed_params.txt");
        extractor.setupInitialization(PoseShapeExtractor::OPENPOSE, "C:/Users/Maria/MyDocs/libs/Installed_libs/ml_models/openpose");

        //extractor.setupNewExperiment(inputs[0], "fit");
        //extractor.setupNewDistplacementRegExperiment(inputs[0], 0.001, 0.5, "d_reg");
        //extractor.setSaveIntermediateResults(true);
        // save a copy of parameters to the input folder
        
        //extractor.viewIteratoinProcess();

        //extractor.viewCameraSetupForPhotos();
        //extractor.viewFinalResult(true);

        std::ofstream failures_list;
        failures_list.open("E:/HumanData/OBJ_Gender/fails.txt");

        for (auto&& input : inputs)
        {
            //for (const double& l2_weight : { 0.1, 0.01, 0.001 })
            //{
            //    for (const double& smooth_weight : { 0.5, 1., 0.2 })
            //    {
                    try
                    {
                        extractor.setupNewExperiment(input, "mingi");
                        //extractor.setupNewDistplacementRegExperiment(input, l2_weight, smooth_weight, "displ_reg");
                        //extractor.runExtraction();
                        std::shared_ptr<SMPLWrapper> smpl_estimated = std::move(extractor.runExtraction());
                        smpl_estimated->logParameters(input->getPath() + "/" + input->getName() + "_smpl_params.txt");
                    }
                    catch (std::exception& e)
                    {
                        std::cout << "exception encountered: " << e.what() << std::endl;
                        failures_list << input->getPath() + "/" + input->getName() + ".obj" << std::endl;
                    }
        //        }
        //    }
        }

        failures_list.close();
    }
    catch (std::exception& e)
    {
        std::cout << "Exception encountered: " << e.what() << std::endl
            << "Terminating" << std::endl;
    }
}

