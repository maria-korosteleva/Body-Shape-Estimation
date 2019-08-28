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
    SMPLWrapper::State& smpl_state_ptrs = new_smpl.getStatePointers();

    std::vector<double> pose{
        0.0651227 , 0.147299 , 0.0710667 ,
        -0.21262 , 0.145289 , -0.23906 ,
        0.309962 , 0.096978 , 0.138881 ,
        -0.026791 , -0.0519696 , 0.0412594 ,
        0.165091 , 0.353616 , 0.138207 ,
        -0.241133 , 0.0853866 , -0.180021 ,
        -0.156246 , -0.121236 , -0.0586891 ,
        0.41172 , -0.655098 , -0.501557 ,
        0.814726 , -0.370373 , 0.309268 ,
        -0.0362064 , -0.154798 , -0.0802434 ,
        0.0402412 , 0.13361 , 0.507384 ,
        -0.21919 , -0.158187 , -0.629973 ,
        0.0580983 , -0.262403 , 0.160539 ,
        0.174705 , -0.0242574 , 0.0141313 ,
        -0.0116324 , -0.0182904 , 0.0181343 ,
        0.291841 , -0.184634 , -0.209186 ,
        0.0936878 , 0.0989654 , -1.35397 ,
        -0.00311836 , -0.0915279 , 1.3157 ,
        0.155893 , -0.107155 , 0.0415009 ,
        0.109932 , 1.01562 , 0.406998 ,
        0.433999 , -0.313739 , -0.197886 ,
        0.172167 , 0.255817 , -0.100458 ,
        -0.114067 , 0.170656 , 0.193996 ,
        -0.218555 , -0.262078 , -0.326749 ,
    };

    std::vector<double>  shape
        { -1.35579 , 1.271845 , 0.262945 , 0.723132 , 0.0851556 , 1.80277 , 1.06419 , -0.67788 , 0.112532 , 0.670397 , };

    std::vector<double> translation{ 0.00786991 , 0.22358 , 0.0107836 , };

    smpl_state_ptrs.pose = &pose[0];
    smpl_state_ptrs.shape = &shape[0];
    smpl_state_ptrs.translation = &translation[0];

    CustomLogger logger(output_path, "cas_wom_modified");

    logger.saveFinalModel(new_smpl);
}

int main()
{
    try
    {
        std::vector<std::shared_ptr<GeneralMesh>> inputs;
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
        inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/SketchFab/Sexy Girl.obj", GeneralMesh::FEMALE));
        inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/SketchFab/casual-woman-walking.obj", GeneralMesh::FEMALE));
        inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/SketchFab/girl_nasi_pants.obj", GeneralMesh::FEMALE));
        inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/SketchFab/jenya4.obj", GeneralMesh::FEMALE));
        //inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/SketchFab/shan.obj", GeneralMesh::FEMALE));
        //inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/SketchFab/Web.obj", GeneralMesh::MALE));
        inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/SketchFab/reilly.obj", GeneralMesh::MALE));
        //inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/SketchFab/Ivan Komarov.obj", GeneralMesh::MALE));
        //inputs.push_back(std::make_shared<GeneralMesh>("D:/Data/SketchFab/casual-man.obj", GeneralMesh::MALE));

        std::cout << "Inputs are loaded! " << std::endl;

        PoseShapeExtractor extractor(smpl_model_path,
            "C:/Users/Maria/MyDocs/libs/Installed_libs/ml_models/openpose",
            "C:/Users/Maria/MyDocs/GigaKorea/GK-Undressing-People-Ceres/Resources",
            output_path);

        //extractor.setupNewExperiment(inputs[0], "pre_calc_pose_for_shape");
        ////extractor.setupNewDistplacementRegExperiment(inputs[0], 0.001, "d_dbg");
        //extractor.setSaveIntermediateResults(true);
        //extractor.runExtraction();
        //extractor.viewIteratoinProcess();

        //extractor.viewCameraSetupForPhotos();
        //extractor.viewFinalResult(true);

        for (auto&& input : inputs)
        {
            for (const double& reg_weight : { 0.005, 0.01 })
            {
                try
                {
                    extractor.setupNewDistplacementRegExperiment(input, reg_weight, "displ_reg");
                    extractor.runExtraction();
                }
                catch (std::exception& e)
                {
                    std::cout << "exception encountered: " << e.what() << std::endl;
                }
            }
        }
    }
    catch (std::exception& e)
    {
        std::cout << "Exception encountered: " << e.what() << std::endl
            << "Terminating" << std::endl;
    }
}

