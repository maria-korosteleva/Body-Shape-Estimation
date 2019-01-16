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
    this->path_ = path;
    this->path_ += '/';
    this->path_ += gender;
    this->path_ += "_smpl/";

    this->readTemplate_();
    this->readJointMat_();
    this->readShapes_();
    this->readWeights_();
}


SMPLWrapper::~SMPLWrapper()
{
}


void SMPLWrapper::saveToObj(const double* pose, const double* shape, const std::string path) const
{
    MatrixXt<double> verts = this->calcModel<double>(pose, shape);
    igl::writeOBJ(path, verts, this->faces_);
}


void SMPLWrapper::readTemplate_()
{
    std::string file_name(this->path_);
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
    std::string file_name(this->path_);
    file_name += this->gender_;
    file_name += "_joints_mat.txt";

    // copy from Meekyong code example
    std::fstream inFile;
    inFile.open(file_name, std::ios_base::in);
    int joints_n, verts_n;
    inFile >> joints_n;
    inFile >> verts_n;
    // Sanity check
    if (joints_n != SMPLWrapper::POSE_SIZE / 3 || verts_n != SMPLWrapper::NUM_VERTICES)
        throw std::exception("Joint matrix info (number of joints and vertices) is incompatible with the model");

    this->jointMat_.resize(joints_n, verts_n);
    for (int i = 0; i < joints_n; i++)
        for (int j = 0; j < verts_n; j++)
            inFile >> this->jointMat_(i, j);

    inFile.close();
}


void SMPLWrapper::readShapes_()
{
    std::string file_path(this->path_);
    file_path += this->gender_;
    file_path += "_blendshape/shape";

    Eigen::MatrixXi fakeFaces(SMPLWrapper::NUM_VERTICES, SMPLWrapper::SPACE_DIM);

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
    std::string file_name(this->path_);
    file_name += this->gender_;
    file_name += "_weight.txt";

    std::fstream inFile;
    inFile.open(file_name, std::ios_base::in);
    int joints_n, verts_n;
    inFile >> joints_n;
    inFile >> verts_n;
    // Sanity check
    if (joints_n != SMPLWrapper::POSE_SIZE / 3 || verts_n != SMPLWrapper::NUM_VERTICES)
        throw std::exception("Joint matrix info (number of joints and vertices) is incompatible with the model");

    this->weights_.resize(verts_n, joints_n);

    for (int i = 0; i < verts_n; i++)
        for (int j = 0; j < joints_n; j++)
            inFile >> this->weights_(i, j);

    inFile.close();
}

