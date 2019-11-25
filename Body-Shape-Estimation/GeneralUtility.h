#pragma once


#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <algorithm>

#include <cstring>
#include <string>
#include <array>
#include <vector>
#include <array>

#include <direct.h>//WINDOWS HEADER

#include <filesystem>

#include <stb/stb_image.h>
#include <stb/stb_image_write.h>


namespace mg {
    //can work on VS2015
    namespace fs = std::experimental::filesystem;

    enum ImageExtension {
        BMP,
        JPG,
        PNG,
        UNKNOWN
    };

    struct ImageInfo {
        //image to array: row first
        unsigned char *before_data;
        int before_width;
        int before_height;
        int before_n_channels;

        unsigned char *after_data;
        int after_width;
        int after_height;
        int after_n_channels;
    };
    //if full_path extension is included in ImageExtension,
    // output will be ImageExtension::BMP, ImageExtension::JPG or ImageExtension::PNG.
    //if not, output is ImageExtension::UNKNOWN
    ImageExtension getType(const std::string& full_path);
    //get tokenized output based on criteria(ex. space, colon etc.)
    std::vector<std::string> ssplit(std::string input, char criteria);
    //read from the first line
    std::vector< std::vector<std::string> > readCSV(const std::string& full_path);
    std::vector <std::string> readTxt(std::string full_path);
    bool saveTexData(std::shared_ptr<ImageInfo> image_info, const std::string& full_path);
    bool writeTxt(std::vector<std::string> cache_list, std::string full_path);
    //change only extension name, not change inner construction
    std::string convertExtension(std::string filename, std::string extension);


    //set texture of the BoundingBox
    //true: can read and set 'imagename'
    //false: cannot read 'imagename'
    bool checkFileExist(const char *filename);

    //connect with ' _ '
    std::string nameModifer(const std::string &name, const std::string &prefix = "", const std::string &postfix = "");
    std::string toUpper(const std::string &str);
    std::string toLower(const std::string &str);
    bool checkExtensionExist(const std::string &str, const std::string &extension = "");
    //recognize file using extension info.
    //ex)input: C:/a/b/c/mytextfile.txt
    //   output: C:/a/b/c/
    std::string getDir(const std::string &full_path);
    //can work on Windows version only
    void mkDir(const std::string &full_path);

    //can work on VS2015
    std::vector<std::string> getSubFileDir(const std::string &path, const std::string &extension = "");
    std::vector<std::string> getSubFolderDir(const std::string &path);
    std::vector<std::string> getSubDir(const std::string &path);
    std::vector<std::string> getSubFile(const std::string &path, const std::string &extension = ""); // get specific extension
    std::vector<std::string> getSubFolder(const std::string &path);
    std::vector<std::string> getSubContentName(const std::string &path);

    std::string getContentName(const std::string &path);

}