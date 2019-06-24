#include "CustomLogger.h"


CustomLogger::CustomLogger(std::string base_path, std::string experiment_name)
    : base_path_(base_path), experiment_name_(experiment_name)
{
    createNewLogFolder_();
}

CustomLogger::~CustomLogger()
{
}

void CustomLogger::saveFinalModel(SMPLWrapper & smpl)
{
    smpl.logParameters(log_folder_name_ + smpl_param_filename_);

    smpl.saveToObj(log_folder_name_ + final_3D_subfolder_ + "posed_shaped.obj");
    smpl.saveShapedOnlyToObj(log_folder_name_ + final_3D_subfolder_ + "unposed_shaped.obj");
    smpl.savePosedOnlyToObj(log_folder_name_ + final_3D_subfolder_ + "posed_unshaped.obj");

}

void CustomLogger::saveIterationsSMPLObjects(const SMPLWrapper & smpl, const std::vector<Eigen::MatrixXd>& vertices_vector)
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
    CreateDirectory((log_folder_name_ + op_guesses_subfolder_).c_str(), NULL);
    CreateDirectory((log_folder_name_ + final_3D_subfolder_).c_str(), NULL);
}
