#pragma once
/*
The class is a wrapper around SMPL model. In 
The class is able to calculate the SMPL model output based on pose and shape parameters.

TODO
    - Think about other ways to set gender
    - Check the folder structure
    - move geeters and setters to the cpp
    - Non-static num of vertices
*/

#include <Eigen/Eigen/Dense>
#include <igl/readOBJ.h>
#include <igl/writeOBJ.h>

namespace E = Eigen;

template <typename T>
using  MatrixXt = E::Matrix<T, E::Dynamic, E::Dynamic>;

class SMPLWrapper
{
public:
    static constexpr std::size_t SHAPE_SIZE = 10;
    static constexpr std::size_t SPACE_DIM = 3;
    static constexpr std::size_t POSE_SIZE = 72;
    static constexpr std::size_t NUM_VERTICES = 6890;

    E::MatrixXd shape_diffs_[10];  // store only differences between blendshapes and template

    /*
    Class should be initialized with the gender of the model to use and with the path to the model folder,
    that contains the files in a pre-defined structure.
    gender is either "f" or "m"
    */
    SMPLWrapper(char, const char*);
    ~SMPLWrapper();

    char getGender()                    { return gender_; };
    E::MatrixXi* getFaces()             { return &faces_; };
    E::MatrixXd* getTemplateVertices()  { return &verts_template_; };

    template <typename T>
    MatrixXt<T> calcModel(const T*, const T*) const;

    /*Allows for pose/shape be nullptr. Allows to get template/pose without shaping/shaping of the T-pose */
    void saveToObj(const double*, const double*, const std::string) const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    // initial info
    char gender_;
    std::string path_;

    // model info
    E::MatrixXi faces_;
    E::MatrixXd verts_template_;
    E::MatrixXd jointMat_;

    // private functions
    void readTemplate_();
    void readJointMat_();
    void readShapes_();

    template <typename T>
    void shapeSMPL_(const T*, MatrixXt<T>*) const;

    void poseSMPL_() const;
};


template<typename T>
inline MatrixXt<T> SMPLWrapper::calcModel(const T* pose, const T* shape) const
{
    MatrixXt<T> verts = this->verts_template_.cast<T>();

    if (shape != nullptr)
    {
        this->shapeSMPL_(shape, &verts);
    }

    if (pose != nullptr)
    {
        this->poseSMPL_();
    }

    return verts;
}


template<typename T>
inline void SMPLWrapper::shapeSMPL_(const T* shape, MatrixXt<T>* verts) const
{
    for (int i = 0; i < this->SHAPE_SIZE; i++)
        for (int j = 0; j < this->NUM_VERTICES; j++)
            for (int k = 0; k < this->SPACE_DIM; k++)
                (*verts)(j, k) += shape[i] * this->shape_diffs_[i](j, k);
}
