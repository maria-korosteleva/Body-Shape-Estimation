#pragma once
/*
The class is a wrapper around SMPL model. In should be initialized with the gender of the model to use and with the path to the model folder, 
that contains the files in a pre-defined structure. 
The class is able to calculate the SMPL model output based on pose and shape parameters.

TODO
    Think about other ways to set gender
*/

#include <Eigen/Eigen/Dense>

class SMPLWrapper
{
public:
    SMPLWrapper(char, const char*);
    ~SMPLWrapper();

    inline int getPoseSize()                       { return POSE_SIZE_; };
    inline int getShapeSize()                      { return SHAPE_SIZE_; };
    inline char getGender()                        { return gender_};
    inline Eigen::MatrixXi* getFaces()             { return &faces_; };
    inline Eigen::MatrixXd* getTemplateVertices()  { return &template_; };
    inline Eigen::MatrixXd* getLastVertices()      { return &verts_; };
    inline Eigen::VectorXd* getLastPose()          { return &pose_; };
    inline Eigen::VectorXd* getLastShape()         { return &shape_; };

    Eigen::MatrixXd* calcModel(const Eigen::VectorXd, const Eigen::VectorXd);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    // const sizes
    const int POSE_SIZE_ = 72;  // root rotation is included
    const int TRANSLATION_SIZE_ = 3;
    const int SHAPE_SIZE_ = 10;
    
    // initial info
    char gender_;
    char* path_;

    // model info
    Eigen::MatrixXi faces_;
    Eigen::MatrixXd template_;
    Eigen::MatrixXd verts_;
    Eigen::VectorXd pose_;
    Eigen::VectorXd shape_;
};

