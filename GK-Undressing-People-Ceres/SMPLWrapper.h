#pragma once
/*
The class is a wrapper around SMPL model. In 
The class is able to calculate the SMPL model output based on pose and shape parameters.

TODO: 
    - Add pose blendshape 
*/

//#define DEBUG

#include <assert.h>

#include <Eigen/Eigen/Dense>
#include <igl/readOBJ.h>
#include <igl/writeOBJ.h>

#define USE_CERES
#ifdef USE_CERES
// !!! Need for the SMPL posing to work with ceres 
#include <ceres/rotation.h>
#endif // USE_CERES


namespace E = Eigen;

template <typename T>
using  MatrixXt = E::Matrix<T, E::Dynamic, E::Dynamic>;
template <typename T>
using  constMatrixXt = E::Matrix<const T, E::Dynamic, E::Dynamic>;

class SMPLWrapper
{
public:
    static constexpr std::size_t SHAPE_SIZE = 10;
    static constexpr std::size_t SPACE_DIM = 3;
    static constexpr std::size_t POSE_SIZE = 72;
    static constexpr std::size_t VERTICES_NUM = 6890;
    static constexpr std::size_t JOINTS_NUM = POSE_SIZE / SPACE_DIM;

    /*
    Class should be initialized with the gender of the model to use and with the path to the model folder,
    that contains the files in a pre-defined structure.
    gender is either "f" or "m"
    The joints hierarchy is expectes to be so that the parent's id is always less than the child's
    */
    SMPLWrapper(char, const char*);
    ~SMPLWrapper();

    char getGender()                   { return gender_; };
    E::MatrixXi getFaces()             { return this->faces_; };
    E::MatrixXd getTemplateVertices()  { return this->verts_template_; };

    // Pose/shape parameters can be nullptr: allows to get template/pose without shaping/shaping of the T-pose
    template <typename T>
    MatrixXt<T> calcModel(const T * const, const T * const) const;

    // Warning! Function doesn't work properly "sometimes". See issue #364 on Quire
    E::MatrixXd calcJointLocations(const double*);

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
    int joints_parents_[JOINTS_NUM];
    E::MatrixXd weights_;
    E::MatrixXd joints_default_;

    // private functions
    void readTemplate_();
    void readJointMat_();
    void readShapes_();
    void readWeights_();
    void readHierarchy_();

    // for evaluation uses vertex info from the last parameter and uses last parameter for output
    template <typename T>
    void shapeSMPL_(const T * const, MatrixXt<T>&) const;
    // for evaluation uses vertex info from the last parameter and uses last parameter for output
    template <typename T>
    void poseSMPL_(const T * const, MatrixXt<T>&) const;

    // Used with SMPL python module as a reference
    // Assumes that SPACE_DIM == 3
    template <typename T>
    MatrixXt<T> getJointsTransposedGlobalTransformation_(const T * const, MatrixXt<T>&) const;
    
    // Assumes that SPACE_DIM == 3
    template <typename T, typename Derived2>
    MatrixXt<T> get3DLocalTransformMat_(const T * const jointAxisAngleRotation, const E::MatrixBase<Derived2>&) const;
    
    // Assumes that SPACE_DIM == 3
    template <typename T, typename Derived>
    MatrixXt<T> get3DTranslationMat_(const E::MatrixBase<Derived>&) const;
    
    // Composes weights (object local) and given vertices in the rest pose into LBSMatrix. 
    // Both are expected to be filled and alive at the moment of invocation.
    // Adapted copy of igl::lbs_matrix(..)
    template <typename T>
    MatrixXt<T> getLBSMatrix_(MatrixXt<T>&) const;
};


template<typename T>
inline MatrixXt<T> SMPLWrapper::calcModel(const T * const pose, const T *  const shape) const
{
    MatrixXt<T> verts = this->verts_template_.cast<T>();
#ifdef DEBUG
    std::cout << "Calc model" << std::endl;
#endif // DEBUG

    if (shape != nullptr)
    {
        this->shapeSMPL_(shape, verts);
    }

    if (pose != nullptr)
    {
        this->poseSMPL_(pose, verts);
#ifdef DEBUG
        std::cout << "Fin posing " << verts.rows() << " x " << verts.cols() << std::endl;
#endif // DEBUG
    }

#ifdef DEBUG
    std::cout << "Fin calculating" << std::endl;
    //if (std::is_same_v<T, double>)
    //{
    //    for (int i = 0; i < SMPLWrapper::VERTICES_NUM; i++)
    //    {
    //        for (int j = 0; j < SMPLWrapper::SPACE_DIM; j++)
    //        {
    //            std::cout << verts(i, j) << " ";
    //        }
    //        std::cout << std::endl;
    //    }
    //}
#endif // DEBUG

    return verts;
}


template<typename T>
inline void SMPLWrapper::shapeSMPL_(const T *  const shape, MatrixXt<T>& verts) const
{
    // TODO Might be possible to make this faster https://groups.google.com/forum/#!topic/ceres-solver/7ZH21XX6HWU
    for (int i = 0; i < this->SHAPE_SIZE; i++)
        for (int j = 0; j < this->VERTICES_NUM; j++)
            for (int k = 0; k < this->SPACE_DIM; k++)
                verts(j, k) += shape[i] * this->shape_diffs_[i](j, k);
}


template<typename T>
inline void SMPLWrapper::poseSMPL_(const T *  const pose, MatrixXt<T>& verts) const
{
#ifdef DEBUG
    std::cout << "pose" << std::endl;
#endif // DEBUG

    MatrixXt<T> jointLocations = this->jointRegressorMat_.cast<T>() * verts;
    MatrixXt<T> jointsTransformation = this->getJointsTransposedGlobalTransformation_(pose, jointLocations); // .transpose();

    // TODO Use sparce matrices for LBS
    MatrixXt<T> LBSMat = this->getLBSMatrix_<T>(verts);

    MatrixXt<T> homoVerts = LBSMat * jointsTransformation;
    verts = homoVerts.leftCols(SMPLWrapper::SPACE_DIM);
}


template<typename T>
inline MatrixXt<T> SMPLWrapper::getJointsTransposedGlobalTransformation_(const T * const pose, MatrixXt<T>& jointLocations) const
{
#ifdef DEBUG
    std::cout << "global transform" << std::endl;
#endif // DEBUG

    const int homo_size = (SMPLWrapper::SPACE_DIM + 1);
    E::Matrix<T, SMPLWrapper::SPACE_DIM + 1, SMPLWrapper::SPACE_DIM + 1> jointGlobalMats[SMPLWrapper::JOINTS_NUM];
    // Stacked (transposed) global transformation matrices for points
    MatrixXt<T> pointTransformTotal(homo_size * SMPLWrapper::JOINTS_NUM, homo_size);
    
    // uses of function that assume input in 3D. 
    assert(SMPLWrapper::SPACE_DIM == 3);

    jointGlobalMats[0] = this->get3DLocalTransformMat_<T>(pose, jointLocations.row(0));
    MatrixXt<T> tmpPointGlobalTransform = jointGlobalMats[0] * this->get3DTranslationMat_<T>(- jointLocations.row(0));
    pointTransformTotal.block(0, 0, homo_size, homo_size) = tmpPointGlobalTransform.transpose();

    for (int i = 1; i < SMPLWrapper::JOINTS_NUM; i++)
    {
        // Forward Kinematics Formula
        jointGlobalMats[i] = jointGlobalMats[this->joints_parents_[i]] 
            * this->get3DLocalTransformMat_<T>((pose + i*3),
                jointLocations.row(i) - jointLocations.row(this->joints_parents_[i]));
        
        tmpPointGlobalTransform = jointGlobalMats[i] * this->get3DTranslationMat_<T>(-jointLocations.row(i));
        
        pointTransformTotal.block(i * homo_size, 0, homo_size, homo_size) = tmpPointGlobalTransform.transpose();
    }

    return pointTransformTotal;
}


template<typename T, typename Derived2>
inline MatrixXt<T> SMPLWrapper::get3DLocalTransformMat_(const T * const jointAxisAngleRotation,
    const E::MatrixBase<Derived2>& jointLocation) const //E::EigenBase<const T> const E::MatrixBase<Derived1>& 
{    
    MatrixXt<T> localTransform;
    localTransform.setIdentity(4, 4);   // in homogenious coordinates
    localTransform.block(0, 3, 3, 1) = jointLocation.transpose();

#ifdef USE_CERES

    T* rotationMat = new T[9];
    ceres::AngleAxisToRotationMatrix(jointAxisAngleRotation, rotationMat);
    localTransform.block(0, 0, 3, 3) = Eigen::Map<MatrixXt<T>>(rotationMat, 3, 3);

    delete[] rotationMat;

#else // USE_CERES

    T norm = sqrt(jointAxisAngleRotation[0] * jointAxisAngleRotation[0] 
        + jointAxisAngleRotation[1] * jointAxisAngleRotation[1]
        + jointAxisAngleRotation[2] * jointAxisAngleRotation[2]);
    if (norm > 0.0001)  // don't waste computations on zero joint movement
    {
        // apply Rodrigues formula
        MatrixXt<T> exponent;
        exponent.setIdentity(3, 3);

        MatrixXt<T> skew;
        skew.setZero(3, 3);
        skew(0, 1) = - jointAxisAngleRotation[2] / norm;
        skew(0, 2) = jointAxisAngleRotation[1] / norm;
        skew(1, 0) = jointAxisAngleRotation[2] / norm;
        skew(1, 2) = - jointAxisAngleRotation[0] / norm;
        skew(2, 0) = - jointAxisAngleRotation[1] / norm;
        skew(2, 1) = jointAxisAngleRotation[0] / norm;

        exponent += skew * sin(norm) + skew * skew * ((T)1. - cos(norm));
        localTransform.block(0, 0, 3, 3) = exponent;
    }

#endif // USE_CERES

    return localTransform;
}


template<typename T, typename Derived>
inline MatrixXt<T> SMPLWrapper::get3DTranslationMat_(const E::MatrixBase<Derived>& translationVector) const
{
    MatrixXt<T> translation;
    translation.setIdentity(4, 4);  // in homogenious coordinates
    translation.block(0, 3, 3, 1) = translationVector.transpose();

    return translation;
}


template<typename T>
inline MatrixXt<T> SMPLWrapper::getLBSMatrix_(MatrixXt<T>& verts) const
{
    MatrixXt<T> weights = this->weights_.cast<T>();
    MatrixXt<T> LBSMat;
    const int dim = SMPLWrapper::SPACE_DIM;
    const int nVerts = SMPLWrapper::VERTICES_NUM;
    const int nJoints = SMPLWrapper::JOINTS_NUM;  // Number of joints

    // +1 goes for homogenious coordinates
    LBSMat.resize(nVerts, (dim + 1) * nJoints);
    for (int j = 0; j < nJoints; j++)
    {
        E::Matrix<T, E::Dynamic, 1> Wj = weights.block(0, j, nVerts, 1);
        for (int i = 0; i < (dim + 1); i++)
        {
            if (i < dim)
            {
                LBSMat.col(i + j * (dim + 1)) = Wj.cwiseProduct(verts.col(i));
            }
            else
            {
                LBSMat.col(i + j * (dim + 1)).array() = weights.block(0, j, nVerts, 1).array();
            }
        }
    }

    return LBSMat;
}
