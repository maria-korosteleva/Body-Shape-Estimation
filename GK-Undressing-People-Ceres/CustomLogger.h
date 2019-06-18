#pragma once

// WARNING! Uses win-specific features to create log directory 
#define NOMINMAX
#include <Windows.h>
#undef THIS

#include <iostream>
#include <ctime>

#include "SMPLWrapper.h"
#include "ShapeUnderClothOptimizer.h"

class CustomLogger
{
public:
    CustomLogger(std::string base_path, std::string experiment_name = "");
    ~CustomLogger();

    std::string getLogFolderPath() const { return log_folder_name_; }
    std::string getPhotosFolderPath() const { return log_folder_name_ + photo_subfolder_; }

    void logSMPLParams(const SMPLWrapper& smpl, const ShapeUnderClothOptimizer& optimizer) const;
    void saveFinalSMPLObject(const SMPLWrapper& smpl, const ShapeUnderClothOptimizer& optimizer) const;

    void startRedirectCoutToFile(const std::string filename = "cout.txt");
    void endRedirectCoutToFile();

private:
    static constexpr char photo_subfolder_[] = "/photos/";
    static constexpr char iterations_3D_subfolder_[] = "/iterations_objects/";
    static constexpr char final_3D_subfolder_[] = "/final_objects/";
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
};

