#include "CustomLogger.h"


CustomLogger::CustomLogger(std::string base_path, std::string experiment_name)
    : base_path_(base_path), experiment_name_(experiment_name)
{
    createNewLogFolder_();
}

CustomLogger::~CustomLogger()
{
}

void CustomLogger::logSMPLParams(const SMPLWrapper & smpl, const ShapeUnderClothOptimizer & optimizer) const
{
    double *shape_res, *pose_res, *translation_res;
    
    shape_res = optimizer.getEstimatesShapeParams();
    pose_res = optimizer.getEstimatesPoseParams();
    translation_res = optimizer.getEstimatesTranslationParams();
    Eigen::MatrixXd finJointLocations = smpl.calcJointLocations(shape_res, pose_res);
    
    logSMPLParams(shape_res, pose_res, translation_res, &finJointLocations);
}

void CustomLogger::logSMPLParams(
    const double * shape_res, const double * pose_res, const double * translation_res, 
    const Eigen::MatrixXd *jointPositions) const
{
    std::ofstream out(log_folder_name_ + smpl_param_filename_);

    out << "Translation \n[ ";
    if (translation_res != nullptr)
        for (int i = 0; i < SMPLWrapper::SPACE_DIM; i++)
            out << translation_res[i] << " , ";
    else
        for (int i = 0; i < SMPLWrapper::SPACE_DIM; i++)
            out << "0." << " , ";
    out << "]" << std::endl;

    out << std::endl << "Pose params [ \n";
    if (pose_res != nullptr)
    {
        for (int i = 0; i < SMPLWrapper::JOINTS_NUM; i++)
        {
            for (int j = 0; j < SMPLWrapper::SPACE_DIM; j++)
            {
                out << pose_res[i * SMPLWrapper::SPACE_DIM + j] << " , ";
            }
            out << std::endl;
        }
    }
    else
    {
        for (int i = 0; i < SMPLWrapper::JOINTS_NUM; i++)
        {
            for (int j = 0; j < SMPLWrapper::SPACE_DIM; j++)
            {
                out << "0." << " , ";
            }
            out << std::endl;
        }
    }
    out << "]" << std::endl;

    out << std::endl << "Shape (betas) params [ \n";
    if (shape_res != nullptr)
        for (int i = 0; i < SMPLWrapper::SHAPE_SIZE; i++)
            out << shape_res[i] << " , ";
    else
        for (int i = 0; i < SMPLWrapper::SHAPE_SIZE; i++)
            out << "0." << " , ";
    out << std::endl << "]" << std::endl;

    out << std::endl << "Joints locations for posed and shaped model [\n";
    if (jointPositions != nullptr)
    {
        Eigen::MatrixXd translatedJointLoc(*jointPositions);
        // translate
        if (translation_res != nullptr)
        {
            for (int i = 0; i < translatedJointLoc.rows(); ++i)
            {
                for (int j = 0; j < SMPLWrapper::SPACE_DIM; ++j)
                {
                    translatedJointLoc(i, j) += translation_res[j];
                }
            }
        }

        out << translatedJointLoc << std::endl;
    }
    else
    {
        out << "\tNo joint locations supplied" << std::endl;
    }
    out << "]" << std::endl;

    out.close();
}

void CustomLogger::saveFinalSMPLObject(const SMPLWrapper & smpl, const ShapeUnderClothOptimizer & optimizer) const
{
    CreateDirectory((log_folder_name_ + final_3D_subfolder_).c_str(), NULL);

    double *shape_res, *pose_res, *translation_res;

    shape_res = optimizer.getEstimatesShapeParams();
    pose_res = optimizer.getEstimatesPoseParams();
    translation_res = optimizer.getEstimatesTranslationParams();

    smpl.saveToObj(translation_res, pose_res,   shape_res,  (log_folder_name_ + final_3D_subfolder_ + "posed_shaped.obj"));
    smpl.saveToObj(translation_res, nullptr,    shape_res,  (log_folder_name_ + final_3D_subfolder_ + "unposed_shaped.obj"));
    smpl.saveToObj(translation_res, pose_res,   nullptr,    (log_folder_name_ + final_3D_subfolder_ + "posed_unshaped.obj"));
}

void CustomLogger::saveIterationsSMPLObjects(const SMPLWrapper & smpl, const std::vector<Eigen::MatrixXd>& vertices_vector) const
{
    CreateDirectory((log_folder_name_ + iterations_3D_subfolder_).c_str(), NULL);

    std::string subfilename = log_folder_name_ + iterations_3D_subfolder_ + "/it_";
    for (int i = 0; i < vertices_vector.size(); ++i)
    {
        igl::writeOBJ(subfilename + std::to_string(i), vertices_vector[i], smpl.getFaces());
    }

}

void CustomLogger::startRedirectCoutToFile(const std::string filename)
{
    redirecting_file_stream_ = std::ofstream(log_folder_name_ + filename);
    coutbuf_ = std::cout.rdbuf();   //save old buf
    std::cout.rdbuf(redirecting_file_stream_.rdbuf());  //redirect std::cout to file!

    std::cout << log_folder_name_ + filename << std::endl;

    redirection_started_ = true;
}

void CustomLogger::endRedirectCoutToFile()
{
    if (!redirection_started_)
    {
        std::cout << "CustomLogger WARNING : " 
            << "you called to end redirection cout to file without starting redirection." 
            << "Forgot to call startRedirectCoutToFile()?"
            << std::endl;
        return;
    }

    std::cout.rdbuf(coutbuf_);   //  reset cout to standard output again
    redirecting_file_stream_.close();

    redirection_started_ = false;
}

void CustomLogger::createNewLogFolder_()
{
    log_folder_name_ = base_path_ + "/" + experiment_name_ + "_";

    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(buffer, sizeof(buffer), "%y%m%d_%H_%M", timeinfo);

    log_folder_name_ += buffer;
    log_folder_name_ += "/";

    CreateDirectory(log_folder_name_.c_str(), NULL);
    CreateDirectory((log_folder_name_ + photo_subfolder_).c_str(), NULL);
    
}
