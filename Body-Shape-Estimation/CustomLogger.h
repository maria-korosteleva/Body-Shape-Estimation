#pragma once

#include <iostream>
#include <ctime>
#include <boost/filesystem.hpp>
#include <igl/writeOBJ.h>

#include "SMPLWrapper.h"
#include "ShapeUnderClothOptimizer.h"

class CustomLogger
{
public:
    CustomLogger(std::string base_path, std::string experiment_name = "");
    ~CustomLogger();

    std::string getLogFolderPath() const { return log_folder_name_; }
    std::string getPhotosFolderPath();
    std::string getOpenPoseGuessesPath();

    void saveFinalModel(SMPLWrapper& smpl);
    void saveIterationsSMPLObjects(const SMPLWrapper& smpl, const std::vector<Eigen::MatrixXd>& vertices_vector);

    void startRedirectCoutToFile(const std::string filename = "cout.txt");
    void endRedirectCoutToFile();

private:
    static constexpr char photo_subfolder_[] = "/photos/";
    static constexpr char iterations_3D_subfolder_[] = "/iterations_objects/";
    static constexpr char final_3D_subfolder_[] = "/final_objects/";
    static constexpr char op_guesses_subfolder_[] = "/OP_guesses/";
    static constexpr char smpl_param_filename_[] = "smpl_params.txt";

    void createNewLogFolder_();

    // file management state
    std::string base_path_;
    std::string experiment_name_;
    std::string log_folder_name_;

    // for redirection of cout to file -- easy logging feature
    bool redirection_started_ = false;
    std::string redirecting_filename_;
    std::ofstream redirecting_file_stream_;
    std::streambuf *coutbuf_;
    std::streambuf *stderrbuf_;

    // subfolders creation flags
    bool iteration_obj_dir_exist_ = false;
    bool final_obj_dir_exist_ = false;
    bool op_guesses_dir_exist_ = false;
    bool photos_dir_exist_ = false;
};

