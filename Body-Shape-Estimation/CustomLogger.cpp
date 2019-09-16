#include "CustomLogger.h"


CustomLogger::CustomLogger(std::string base_path, std::string experiment_name)
    : base_path_(base_path), experiment_name_(experiment_name)
{
    createNewLogFolder_();
}

CustomLogger::~CustomLogger()
{
}

std::string CustomLogger::getPhotosFolderPath()
{
    if (!photos_dir_exist_)
    {
        boost::filesystem::path log_dir((log_folder_name_ + photo_subfolder_).c_str());
        if (!boost::filesystem::create_directory(log_dir))
            throw std::runtime_error("CustomLogger:ERROR:failed to create logging directory");
        photos_dir_exist_ = true;
    }
    
    return log_folder_name_ + photo_subfolder_;
}

std::string CustomLogger::getOpenPoseGuessesPath()
{
    if (!op_guesses_dir_exist_)
    {        
        boost::filesystem::path log_dir((log_folder_name_ + op_guesses_subfolder_).c_str());
        if (!boost::filesystem::create_directory(log_dir))
            throw std::runtime_error("CustomLogger:ERROR:failed to create logging directory");

        op_guesses_dir_exist_ = true;
    }
    
    return log_folder_name_ + op_guesses_subfolder_;
}

void CustomLogger::saveFinalModel(SMPLWrapper & smpl)
{
    smpl.logParameters(log_folder_name_ + smpl_param_filename_);

    if (!final_obj_dir_exist_)
    { 
        boost::filesystem::path log_dir((log_folder_name_ + final_3D_subfolder_).c_str());
        if (!boost::filesystem::create_directory(log_dir))
            throw std::runtime_error("CustomLogger:ERROR:failed to create logging directory");
        final_obj_dir_exist_ = true;
    }

    smpl.saveToObj(log_folder_name_ + final_3D_subfolder_ + "posed_shaped.obj");
    smpl.saveShapedOnlyToObj(log_folder_name_ + final_3D_subfolder_ + "unposed_shaped.obj");
    smpl.savePosedOnlyToObj(log_folder_name_ + final_3D_subfolder_ + "posed_unshaped.obj");
    smpl.saveWithDisplacementToObj(log_folder_name_ + final_3D_subfolder_ + "posed_shaped_displaced.obj");
    smpl.saveShapedWithDisplacementToObj(log_folder_name_ + final_3D_subfolder_ + "unposed_shaped_displaced.obj");
}

void CustomLogger::saveIterationsSMPLObjects(const SMPLWrapper & smpl, const std::vector<Eigen::MatrixXd>& vertices_vector)
{
    if (!iteration_obj_dir_exist_)
    {
        boost::filesystem::path log_dir((log_folder_name_ + iterations_3D_subfolder_).c_str());
        if (!boost::filesystem::create_directory(log_dir))
            throw std::runtime_error("CustomLogger:ERROR:failed to create logging directory");

        iteration_obj_dir_exist_ = true;
    }
    
    std::string subfilename = log_folder_name_ + iterations_3D_subfolder_ + "/it_";
    for (int i = 0; i < vertices_vector.size(); ++i)
    {
        igl::writeOBJ(subfilename + std::to_string(i) + ".obj", vertices_vector[i], smpl.getFaces());
    }

}

void CustomLogger::startRedirectCoutToFile(const std::string filename)
{
    redirecting_file_stream_ = std::ofstream(log_folder_name_ + filename);
    coutbuf_ = std::cout.rdbuf();   //save old buf
    std::cout.rdbuf(redirecting_file_stream_.rdbuf());  //redirect std::cout to file!

    // redirect cerr to the same file
    stderrbuf_ = std::cerr.rdbuf();
    std::cerr.rdbuf(redirecting_file_stream_.rdbuf());

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
    std::cerr.rdbuf(stderrbuf_);
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

    boost::filesystem::path log_dir(log_folder_name_.c_str());
    if (!boost::filesystem::create_directory(log_dir))
        throw std::runtime_error("CustomLogger:ERROR:failed to create logging directory");
}
