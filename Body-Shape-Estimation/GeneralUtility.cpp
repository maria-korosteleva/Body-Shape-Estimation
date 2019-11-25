#include "GeneralUtility.h"

mg::ImageExtension mg::getType(const std::string & full_path)
{
    std::vector<std::string> token_list = ssplit(full_path, '.');
    std::string type = token_list.back();
    ImageExtension extension;

    for_each(type.begin(), type.end(), [](char &chr) {chr = tolower(chr); });
    if (!type.compare("bmp")) {
        extension = ImageExtension::BMP;
    }
    else if (!type.compare("jpg") || !type.compare("jpeg")) {
        extension = ImageExtension::JPG;
    }
    else if (!type.compare("png")) {
        extension = ImageExtension::PNG;
    }
    else {
        extension = ImageExtension::UNKNOWN;
    }

    return extension;
}

std::vector<std::string> mg::ssplit(std::string input, char criteria)
{
    std::vector<std::string> ret;
    std::istringstream ss(input);
    std::string token;

    if ((criteria == '\\') || (criteria == '/')) {
        if (input.find('\\') != std::string::npos) {
            criteria = '\\';
        }
        else if (input.find('/') != std::string::npos) {
            criteria = '/';
        }
    }

    while (std::getline(ss, token, criteria)) {
        if (token == "") {
            continue;
        }
        ret.push_back(token);
    }

    return ret;
}

std::vector<std::vector<std::string>> mg::readCSV(const std::string & full_path)
{
    std::ifstream file;
    std::string in_line;
    std::vector<std::vector<std::string> > csv_contents;

    file.open(full_path);

    try {
        if (!file.is_open()) {
            throw std::runtime_error("Error opening file: " + full_path);
        }
    }
    catch (std::runtime_error &e) {
        std::cout << "std::vector<std::string> mg::readCSV(...): " << std::endl;
        std::cout << e.what() << std::endl;
    }

    while (getline(file, in_line)) {
        csv_contents.push_back(ssplit(in_line, ','));
    }

    return csv_contents;
}

std::vector<std::string> mg::readTxt(std::string full_path)
{
    std::ifstream file;
    std::string in_line;
    std::vector<std::string> cache;

    if (full_path.find(".txt") == std::string::npos) {
        full_path = full_path + ".txt";
    }

    file.open(full_path);

    try {
        if (!file.is_open()) {
            throw std::runtime_error(full_path);
        }
    }
    catch (std::runtime_error &e){
        std::cout << "mg::readTxt(...): " << std::endl;
        std::cout << "Error opening file: " << e.what() << std::endl;
    }

    while (getline(file, in_line)) {
        cache.push_back(in_line);
    }

    return cache;
}

bool mg::saveTexData(std::shared_ptr<ImageInfo> image_info, const std::string & full_path)
{
    if (checkFileExist(full_path.c_str())) {
        std::cout << "mg::saveTexData(,): " + full_path + "  file exist" << std::endl;

        return 1;
    }

    bool isFail;
    mg::mkDir(full_path);
    ImageExtension extension = getType(full_path);
    switch (extension)
    {
    case ImageExtension::BMP:
        isFail = (bool)stbi_write_bmp(full_path.c_str(), image_info->after_width, image_info->after_height, image_info->after_n_channels, image_info->after_data);
        break;
    case ImageExtension::JPG:
        isFail = (bool)stbi_write_jpg(full_path.c_str(), image_info->after_width, image_info->after_height, image_info->after_n_channels, image_info->after_data, 0);
        break;
    case ImageExtension::PNG:
        isFail = (bool)stbi_write_png(full_path.c_str(), image_info->after_width, image_info->after_height, image_info->after_n_channels, image_info->after_data, 0);
        break;
    default:
        break;
    }
    return isFail;
}

bool mg::writeTxt(std::vector<std::string> cache_list, std::string full_path)
{
    if (full_path.find(".txt") == std::string::npos) {
        full_path = full_path + ".txt";
    }

    mg::mkDir(full_path);

    std::ofstream file(full_path);
    for (auto &iter : cache_list) {
        file << iter << std::endl;
    }

    return true;
}

std::string mg::convertExtension(std::string filename, std::string extension)
{
    std::string converted;

    std::vector<std::string> token_list = ssplit(filename, '.');
    int extension_size = token_list.back().size();
    if (extension.size() == 0) {
        converted = filename.substr(0, filename.size() - (extension_size + 1));
    }
    else {
        converted = filename.substr(0, filename.size() - extension_size);
        converted += extension;
    }

    return converted;
}

bool mg::checkFileExist(const char *filename)
{
    std::ifstream infile(filename);
    return infile.good();
}

std::string mg::nameModifer(const std::string & name, const std::string & prefix, const std::string & postfix)
{
    std::vector<std::string> output = mg::ssplit(name,'.');

    std::string ret = name;

    if (output.size() == 2) {
        if (prefix != "") {
            ret = prefix + "_" + output[0];
        }
        else {
            ret = output[0];
        }
        if (postfix != "") {
            ret = ret + "_" + postfix + "." + output[1];
        }
        else {
            ret = ret + "." + output[1];
        }
    }
    else {
        if (prefix != "") {
            ret = prefix + "_" + ret;
        }
        if (postfix != "") {
            ret = ret + "_" + postfix;
        }
    }

    return ret;
}

std::string mg::toUpper(const std::string &str)
{
    std::string upper_str = str;
    for (char &ele : upper_str) {
        ele = toupper(ele);
    }
    return upper_str;
}

std::string mg::toLower(const std::string & str)
{
    std::string lower_str = str;
    for (char &ele : lower_str) {
        ele = tolower(ele);
    }
    return lower_str;
}

bool mg::checkExtensionExist(const std::string &str, const std::string &extension){
    std::array<std::string, 2> extension_list = { ("." + extension), ("." + extension) };

    extension_list[0] = mg::toUpper(extension_list[0]);
    extension_list[1] = mg::toLower(extension_list[1]);

    for (std::string extension_ : extension_list) {
        if (str.find(extension_) != std::string::npos) {
            return true;
        }
    }
    
    return false;
}

std::string mg::getDir(const std::string &full_path) {
    std::vector<std::string> sp_list = ssplit(full_path, '\\');
    std::string path = "";

    for (auto sp : sp_list) {
        if (sp.find(".") != std::string::npos) {
            break;
        }

        path += sp + "\\";
    }

    return path;
}

void mg::mkDir(const std::string &full_path)
{
    std::vector<std::string> sp_list = ssplit(full_path, '\\');
    std::string path = "";

    for (auto sp : sp_list) {
        if (sp.find(".") != std::string::npos) {
            break;
        }

        path += sp + "\\";
        _mkdir(path.c_str());
    }
}

std::vector<std::string> mg::getSubFileDir(const std::string &path, const std::string &extension)
{
    std::string low_extension, high_extension;
    std::vector<std::string> sub_name_list;
    std::vector<std::string> sub_specific_name_list;
    sub_name_list = mg::getSubDir(path);

    for (const std::string &sub_name : sub_name_list) {
        if (checkExtensionExist(sub_name, extension)) {
            sub_specific_name_list.push_back(sub_name);
        }
    }

    return sub_specific_name_list;
}

std::vector<std::string> mg::getSubFolderDir(const std::string & path)
{
    std::string low_extension, high_extension;
    std::vector<std::string> sub_name_list;
    std::vector<std::string> sub_specific_name_list;
    sub_name_list = mg::getSubDir(path);

    for (const std::string &sub_name : sub_name_list) {
        if (!checkExtensionExist(sub_name)) {
            sub_specific_name_list.push_back(sub_name);
        }
    }

    return sub_specific_name_list;
}

std::vector<std::string> mg::getSubDir(const std::string &path)
{
    try {
        if (checkExtensionExist(path)) {
            throw std::runtime_error("ERROR: Param. path can only have a folder path.");
        }
    }
    catch (std::runtime_error &e) {
        std::cout << "std::vector<std::string> mg::getSubDir(...)\n" << e.what() << std::endl;
    }

    std::vector<std::string> sub_path_list;
    for (auto& p : mg::fs::directory_iterator(path))
    {
        sub_path_list.push_back(p.path().string());
    }

    return sub_path_list;
}

std::vector<std::string> mg::getSubFile(const std::string & path, const std::string &extension)
{
    std::string low_extension, high_extension;
    std::vector<std::string> sub_name_list;
    std::vector<std::string> sub_specific_name_list;
    sub_name_list = mg::getSubContentName(path);

    for (const std::string &sub_name : sub_name_list) {
        if (checkExtensionExist(sub_name, extension)) {
            sub_specific_name_list.push_back(sub_name);
        }
    }

    return sub_specific_name_list;
}

std::vector<std::string> mg::getSubFolder(const std::string & path)
{
    std::string low_extension, high_extension;
    std::vector<std::string> sub_name_list;
    std::vector<std::string> sub_specific_name_list;
    sub_name_list = mg::getSubContentName(path);

    for (const std::string &sub_name : sub_name_list) {
        if (!checkExtensionExist(sub_name)) {
            sub_specific_name_list.push_back(sub_name);
        }
    }

    return sub_specific_name_list;
}

std::vector<std::string> mg::getSubContentName(const std::string & path)
{
    try {
        if (checkExtensionExist(path)) {
            throw std::runtime_error("ERROR: Param. path can only have a folder path.");
        }
    }
    catch (std::runtime_error &e) {
        std::cout << "std::vector<std::string> mg::getSubContentName(...)\n" << e.what() << std::endl;
    }
    std::vector<std::string> sub_name_list;
    std::string file_name;
    for (auto& p : mg::fs::directory_iterator(path))
    {
        file_name = getContentName(p.path().string());
        sub_name_list.push_back(file_name);
    }

    return sub_name_list;
}

std::string mg::getContentName(const std::string & path)
{
    return ssplit(path, '\\').back();
}
