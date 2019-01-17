#include "pch.h"
#include "SMPLWrapper.h"


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
    this->general_path_ = path;
    this->general_path_ += '/';

    this->gender_path_ = path;
    this->gender_path_ += '/';
    this->gender_path_ += gender;
    this->gender_path_ += "_smpl/";

    this->readTemplate_();
    this->readJointMat_();
    this->readShapes_();
    this->readWeights_();
    this->readHierarchy_();

    //this->joints_default_ = this->jointRegressorMat_.cast<double>() * this->verts_template_;
}


SMPLWrapper::~SMPLWrapper()
{
}


E::MatrixXd SMPLWrapper::calcJointLocations(const double * shape)
{
    //E::MatrixXd verts = this->calcModel<double>(nullptr, shape);

    std::cout << "joint locations calculation " << this->jointRegressorMat_.rows() <<
        " x " << this->jointRegressorMat_.cols() << "; " 
        << this->verts_template_.rows() <<
        " x " << this->verts_template_.cols() << std::endl;
    E::MatrixXd joints(24, 3);
    E::Matrix<double, Eigen::Dynamic, 3> tmp(6890, 3);
    tmp.setRandom();
    joints = this->jointRegressorMat_ * tmp;
    std::cout << "After joints calculation" << std::endl;
    //joints = 1. * this->verts_template_;
    
    return joints;
}

void SMPLWrapper::saveToObj(const double* pose, const double* shape, const std::string path) const
{
    MatrixXt<double> verts = this->calcModel<double>(pose, shape);
    igl::writeOBJ(path, verts, this->faces_);
}


void SMPLWrapper::readTemplate_()
{
    std::string file_name(this->gender_path_);
    file_name += this->gender_;
    file_name += "_shapeAv.obj";

    bool success = igl::readOBJ(file_name, this->verts_template_, this->faces_);
    if (!success)
    {
        std::string message("Abort: Could not read SMPL template at ");
        message += file_name;
        throw std::exception(message.c_str());
    }
}


void SMPLWrapper::readJointMat_()
{
    std::string file_name(this->gender_path_);
    file_name += this->gender_;
    file_name += "_joints_mat.txt";

    // copy from Meekyong code example
    std::fstream inFile;
    inFile.open(file_name, std::ios_base::in);
    int joints_n, verts_n;
    inFile >> joints_n;
    inFile >> verts_n;
    // Sanity check
    if (joints_n != SMPLWrapper::JOINTS_NUM || verts_n != SMPLWrapper::VERTICES_NUM)
        throw std::exception("Joint matrix info (number of joints and vertices) is incompatible with the model");

    this->jointRegressorMat_.resize(joints_n, verts_n);
    for (int i = 0; i < joints_n; i++)
        for (int j = 0; j < verts_n; j++)
            inFile >> this->jointRegressorMat_(i, j);

    inFile.close();
}


void SMPLWrapper::readShapes_()
{
    std::string file_path(this->gender_path_);
    file_path += this->gender_;
    file_path += "_blendshape/shape";

    Eigen::MatrixXi fakeFaces(SMPLWrapper::VERTICES_NUM, SMPLWrapper::SPACE_DIM);

    for (int i = 0; i < SMPLWrapper::SHAPE_SIZE; i++)
    {
        std::string file_name(file_path);
        file_name += std::to_string(i);
        file_name += ".obj";

        igl::readOBJ(file_name, this->shape_diffs_[i], fakeFaces);

        this->shape_diffs_[i] -= this->verts_template_;
    }
}


void SMPLWrapper::readWeights_()
{
    std::string file_name(this->gender_path_);
    file_name += this->gender_;
    file_name += "_weight.txt";

    std::fstream inFile;
    inFile.open(file_name, std::ios_base::in);
    int joints_n, verts_n;
    inFile >> joints_n;
    inFile >> verts_n;
    // Sanity check
    if (joints_n != SMPLWrapper::JOINTS_NUM || verts_n != SMPLWrapper::VERTICES_NUM)
        throw std::exception("Weights info (number of joints and vertices) is incompatible with the model");

    this->weights_.resize(verts_n, joints_n);

    for (int i = 0; i < verts_n; i++)
        for (int j = 0; j < joints_n; j++)
            inFile >> this->weights_(i, j);

    inFile.close();
}


void SMPLWrapper::readHierarchy_()
{
    std::string file_name(this->general_path_);
    file_name += "jointsHierarchy.txt";

    std::fstream inFile;
    inFile.open(file_name, std::ios_base::in);
    int joints_n;
    inFile >> joints_n;
    // Sanity check
    if (joints_n != SMPLWrapper::JOINTS_NUM)
    {
        throw std::exception("Number of joints in joints hierarchy info is incompatible with the model");
    }
    
    int tmpId;
    for (int j = 0; j < joints_n; j++)
    {
        inFile >> tmpId;
        inFile >> this->joints_parents_[tmpId];
    }

    inFile.close();
}

