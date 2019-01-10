#pragma once
/*
The class is a wrapper around SMPL model. In 
The class is able to calculate the SMPL model output based on pose and shape parameters.

TODO
    - Think about other ways to set gender
    - Check the folder structure
*/

#include <Eigen/Eigen/Dense>

class SMPLWrapper
{
public:
    /*
    Class should be initialized with the gender of the model to use and with the path to the model folder,
    that contains the files in a pre-defined structure.
    gender is either "f" or "m"
    */
    SMPLWrapper(char, const char*);
    ~SMPLWrapper();

    inline int getPoseSize()                       { return POSE_SIZE_; };
    inline int getShapeSize()                      { return SHAPE_SIZE_; };
    inline char getGender()                        { return gender_; };
    inline Eigen::MatrixXi* getFaces()             { return &faces_; };
    inline Eigen::MatrixXd* getTemplateVertices()  { return &verts_template_; };
    inline Eigen::MatrixXd* getLastVertices()      { return &verts_; };
    inline Eigen::VectorXd* getLastPose()          { return &pose_; };
    inline Eigen::VectorXd* getLastShape()         { return &shape_; };

    Eigen::MatrixXd* calcModel(const Eigen::VectorXd, const Eigen::VectorXd);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    // const sizes
    static const int POSE_SIZE_ = 72;  // root rotation is included
    static const int TRANSLATION_SIZE_ = 3;
    static const int SHAPE_SIZE_ = 10;
    
    // initial info
    char gender_;
    std::string path_;

    // model info
    Eigen::MatrixXi faces_;
    Eigen::MatrixXd verts_template_;
    Eigen::MatrixXd jointMat_;
    Eigen::MatrixXd shape_diffs_[SHAPE_SIZE_];  // store only differences between blendshapes and template

    // last calculation info
    Eigen::MatrixXd verts_;
    Eigen::VectorXd pose_;
    Eigen::VectorXd shape_;

    // private functions
    void readTemplate_();
    void readJointMat_();
    void readShapes_();

    void shapeSMPL_();
    void poseSMPL_();
};

