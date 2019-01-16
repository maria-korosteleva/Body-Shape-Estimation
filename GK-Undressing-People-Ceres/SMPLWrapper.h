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

#include <assert.h>

#include <Eigen/Eigen/Dense>
#include <igl/readOBJ.h>
#include <igl/writeOBJ.h>
//#include <igl/lbs_matrix.h>

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

    /*
    Class should be initialized with the gender of the model to use and with the path to the model folder,
    that contains the files in a pre-defined structure.
    gender is either "f" or "m"
    */
    SMPLWrapper(char, const char*);
    ~SMPLWrapper();

    char getGender()                    { return gender_; };
    E::MatrixXi getFaces()             { return this->faces_; };
    E::MatrixXd getTemplateVertices()  { return this->verts_template_; };

    // Pose/shape parameters can be nullptr: allows to get template/pose without shaping/shaping of the T-pose
    template <typename T>
    MatrixXt<T> calcModel(const T*, const T*) const;

    // Pose/shape parameters can be nullptr: allows to get template/pose without shaping/shaping of the T-pose
    void saveToObj(const double*, const double*, const std::string) const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    // initial info
    char gender_;
    std::string gender_path_;
    std::string general_path_;

    // model info
    E::MatrixXi faces_;
    E::MatrixXd verts_template_;
    E::MatrixXd shape_diffs_[10];  // store only differences between blendshapes and template
    E::MatrixXd jointRegressorMat_;
    // The joints hierarchy is expectes to be so that the 
    int parents[POSE_SIZE / SPACE_DIM];
    E::MatrixXd weights_;

    // private functions
    void readTemplate_();
    void readJointMat_();
    void readShapes_();
    void readWeights_();
    void readHierarchy_();

    template <typename T>
    void shapeSMPL_(const T*, MatrixXt<T>*) const;
    template <typename T>
    void poseSMPL_(const T*, MatrixXt<T>*) const;

    // Used with SMPL python module as a reference 
    template <typename T>
    MatrixXt<T> getJointsGlobalTransformation_(const T*) const;
    
    // Composes weights (object local) and given vertices in the rest pose into LBSMatrix. 
    // Both are expected to be filled and alive at the moment of invocation.
    // Adapted copy of igl::lbs_matrix(..)
    template <typename T>
    MatrixXt<T> getLBSMatrix_(MatrixXt<T>*) const;
};


template<typename T>
inline MatrixXt<T> SMPLWrapper::calcModel(const T* pose, const T* shape) const
{
    MatrixXt<T> verts = this->verts_template_.cast<T>();
    std::cout << "Calc model" << std::endl;

    if (shape != nullptr)
    {
        this->shapeSMPL_(shape, &verts);
    }

    if (pose != nullptr)
    {
        this->poseSMPL_(pose, &verts);
    }

    std::cout << "Fin calculating" << std::endl;

    return verts;
}


template<typename T>
inline void SMPLWrapper::shapeSMPL_(const T* shape, MatrixXt<T>* verts) const
{
    // TODO Might be possible to make this faster https://groups.google.com/forum/#!topic/ceres-solver/7ZH21XX6HWU
    for (int i = 0; i < this->SHAPE_SIZE; i++)
        for (int j = 0; j < this->NUM_VERTICES; j++)
            for (int k = 0; k < this->SPACE_DIM; k++)
                (*verts)(j, k) += shape[i] * this->shape_diffs_[i](j, k);
}


template<typename T>
inline void SMPLWrapper::poseSMPL_(const T* pose, MatrixXt<T>* verts) const
{
    std::cout << "pose" << std::endl;

    MatrixXt<T> jointLocations = this->jointRegressorMat_.cast<T>() * (*verts);
    //std::cout << jointLocations.rows() << ";  " << jointLocations.cols() << std::endl;

    MatrixXt<T> jointsTransformation = this->getJointsGlobalTransformation_(pose).transpose();
    // remove extra column that indicates homogenious coordinates
    //MatrixXt<T> nonHomoTransformation = jointsTransformation.leftCols(SMPLWrapper::SPACE_DIM);

    // Get the LBS matrix
    MatrixXt<T> LBSMat = this->getLBSMatrix_<T>(verts);
    
    // Get the final vertices
    //(*verts) = LBSMat * nonHomoTransformation;
}


template<typename T>
inline MatrixXt<T> SMPLWrapper::getJointsGlobalTransformation_(const T *) const
{
    // Form the world transformation for joints (exclude thansformation due to the rest pose -- check if that's identical matrix)
    // calculate individual matrices with only one parent involved
    // combine into transformations for each joint 
    // stack the matrices
    return MatrixXt<T>(2, 2);
}


template<typename T>
inline MatrixXt<T> SMPLWrapper::getLBSMatrix_(MatrixXt<T>* verts) const
{
    MatrixXt<T> weights = this->weights_.cast<T>();
    MatrixXt<T> LBSMat;
    const int dim = SMPLWrapper::SPACE_DIM;
    const int nVerts = SMPLWrapper::NUM_VERTICES;
    const int nJoints = this->weights_.cols();  // Number of joints

    // +1 goes for homogenious coordinates
    LBSMat.resize(nVerts, (dim + 1) * nJoints);
    for (int j = 0; j < nJoints; j++)
    {
        E::Matrix<T, E::Dynamic, 1> Wj = weights.block(0, j, nVerts, 1);
        for (int i = 0; i < (dim + 1); i++)
        {
            if (i < dim)
            {
                LBSMat.col(i + j * (dim + 1)) = Wj.cwiseProduct((*verts).col(i));
            }
            else
            {
                LBSMat.col(i + j * (dim + 1)).array() = weights.block(0, j, nVerts, 1).array();
            }
        }
    }
    return LBSMat;
}
