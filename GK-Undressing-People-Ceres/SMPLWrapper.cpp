#include "pch.h"
#include "SMPLWrapper.h"

#include <igl/readOBJ.h>


SMPLWrapper::SMPLWrapper(char gender, const char* path)
{
    // set the info
    if (gender != 'f' && gender != 'm') 
    {
        std::string message("Wrong gender supplied: ");
        message += gender;
        throw std::exception(message.c_str());
    }
    this->gender_ = gender;

    // !!!! expects a pre-defined file structure
    // TODO remove specific structure expectation
    this->path_ = path;
    this->path_ += '/';
    this->path_ += gender;
    this->path_ += "_smpl/";

    this->readTemplate_();
    this->readJointMat_();
    this->readShapes_();

    std::cout << "SMPL model loaded\n";
}


SMPLWrapper::~SMPLWrapper()
{
}


Eigen::MatrixXd* SMPLWrapper::calcModel(const Eigen::VectorXd pose, const Eigen::VectorXd shape)
{
    // init parameters
    this->pose_ = pose;
    this->shape_ = shape;
    this->verts_ = this->verts_template_;

    std::cout << "Calculating SMPL...\n";

    this->shapeSMPL_();

    this->poseSMPL_();

    return &this->verts_;
}

void SMPLWrapper::readTemplate_()
{
    std::string file_name(this->path_);
    file_name += this->gender_;
    file_name += "_shapeAv.obj";
    igl::readOBJ(file_name, this->verts_template_, this->faces_);

    std::cout << "-Template loaded\n";
}

void SMPLWrapper::readJointMat_()
{
    std::string file_name(this->path_);
    file_name += this->gender_;
    file_name += "_joints_mat.txt";

    // copy from Meekyong code example
    std::fstream inFile;
    inFile.open(file_name, std::ios_base::in);
    int joints_n, verts_n;
    inFile >> joints_n;
    inFile >> verts_n;
    this->jointMat_.resize(joints_n, verts_n);
    for (int i = 0; i < joints_n; i++)
    {
        for (int j = 0; j < verts_n; j++)
        {
            inFile >> this->jointMat_(i, j);
        }
    }
    inFile.close();

    std::cout << "-Joint matrix loaded\n";
}

void SMPLWrapper::readShapes_()
{
    std::string file_path(this->path_);
    file_path += this->gender_;
    file_path += "_blendshape/shape";

    Eigen::MatrixXi fakeFaces;
    for (int i = 0; i < this->SHAPE_SIZE_; i++)
    {
        std::string file_name(file_path);
        file_name += std::to_string(i);
        file_name += ".obj";

        igl::readOBJ(file_name, this->shape_diffs_[i], fakeFaces);

        this->shape_diffs_[i] -= this->verts_template_;
    }
    std::cout << "-Shapes loaded\n";
}

void SMPLWrapper::shapeSMPL_()
{
    for (int i = 0; i < this->SHAPE_SIZE_; i++)
    {
        this->verts_ += this->shape_[i] * this->shape_diffs_[i];
        std::cout << i << " " << this->shape_[i] << " ";
    }

    std::cout << "SMPL shaped...\n";
}

void SMPLWrapper::poseSMPL_()
{

}
