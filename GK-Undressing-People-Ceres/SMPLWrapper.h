#pragma once
/*
The class is a wrapper around SMPL model. In 
The class is able to calculate the SMPL model output based on pose and shape parameters.

TODO: 
    - Add pose blendshape 
*/

//#define DEBUG

#include <assert.h>

#include <Eigen/Dense>
#include <Eigen/SparseCore>
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
    static constexpr std::size_t JOINTS_NUM = POSE_SIZE / SPACE_DIM;
    static constexpr std::size_t VERTICES_NUM = 6890;
    static constexpr std::size_t WEIGHTS_BY_VERTEX = 4;     // number of joints each vertex depend on

    /*
    Class should be initialized with the gender of the model to use and with the path to the model folder,
    that contains the files in a pre-defined structure.
    gender is either "f" or "m"
    The joints hierarchy is expectes to be so that the parent's id is always less than the child's
    */
    SMPLWrapper(char, const char*);
    ~SMPLWrapper();

    char getGender() const                    { return gender_; };
    const E::MatrixXi& getFaces() const              { return this->faces_; };
    const E::MatrixXd& getTemplateVertices() const   { return this->verts_template_; };
    const E::VectorXd& getTemplateMeanPoint() const  { return this->template_mean_point_; };

    // Pose/shape parameters can be nullptr: allows to get template/pose without shaping/shaping of the T-pose
    template <typename T>
    MatrixXt<T> calcModelTemplate(const T * const, const T * const) const;
    // non-templated version that can calculate jacobian
    E::MatrixXd calcModel(const double * const, const double * const, E::MatrixXd* pose_jac = nullptr, E::MatrixXd* shape_jac = nullptr) const;

    // Warning! Function doesn't work properly "sometimes". See issue #364 on Quire
    E::MatrixXd calcJointLocations(const double*);

    // Pose/shape parameters can be nullptr: allows to get template/pose without shaping/shaping of the T-pose
    void saveToObj(const double*, const double*, const double*, const std::string) const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    // initial info
    char gender_;
    std::string gender_path_;
    std::string general_path_;

    // model info
    E::MatrixXi faces_;
    E::MatrixXd verts_template_;
    E::VectorXd template_mean_point_;
    E::MatrixXd shape_diffs_[10];  // store only differences between blendshapes and template
    E::MatrixXd jointRegressorMat_;
    int joints_parents_[JOINTS_NUM];
    E::SparseMatrix<double> weights_;

    // private functions
    void readTemplate_();
    void readJointMat_();
    void readShapes_();
    void readWeights_();
    void readHierarchy_();

    // for evaluation uses vertex info from the last parameter and uses last parameter for output
    template <typename T>
    void shapeSMPLTemplate_(const T * const, MatrixXt<T>&) const;
    // non-templated version that can calculate jacobian
    // if not nullptr, shape_jac is expected to be an array of MatrixXd, one matrix for each shape parameter
    void shapeSMPL_(const double * const, E::MatrixXd&, E::MatrixXd* shape_jac = nullptr) const;
    // for evaluation uses vertex info from the last parameter and uses last parameter for output
    template <typename T>
    void poseSMPLTemplate_(const T * const, MatrixXt<T>&) const;
    // non-templated version that can calculate jacobian
    void poseSMPL_(const double * const, E::MatrixXd&, E::MatrixXd* pose_jac = nullptr) const;

    // Assumes that SPACE_DIM == 3
    // Assumes the default joint angles to be all zeros
    // Returns matrix of dimentions (SPACE_DIM + 1) * JOINTS_NUM  x  SPACE_DIM 
    // of stacked transposed global transformation matrices of each joint, with the row of homogenious coordinates removed
    template <typename T>
    MatrixXt<T> getJointsTransposedGlobalTransformationTemplate_(const T * const, MatrixXt<T>&) const;
    // non-templated version that can calculate jacobian
    E::MatrixXd getJointsTransposedGlobalTransformation_(const double * const, E::MatrixXd&, E::MatrixXd* jac = nullptr) const;
    
    // Assumes that SPACE_DIM == 3
    template <typename T, typename Derived2>
    MatrixXt<T> get3DLocalTransformMatTemplate_(const T * const jointAxisAngleRotation, const E::MatrixBase<Derived2>&) const;
    // non-templated version that can calculate jacobian
    // fills the dependence of the Transformation matric on all three coordinates for the input rotation
    E::MatrixXd get3DLocalTransformMat_(const double * const jointAxisAngleRotation, const E::MatrixXd&, E::MatrixXd* jac = nullptr) const;
    
    // Assumes that SPACE_DIM == 3
    template <typename T, typename Derived>
    MatrixXt<T> get3DTranslationMatTemplate_(const E::MatrixBase<Derived>&) const;
    // non-templated version that can calculate jacobian
    E::MatrixXd get3DTranslationMat_(const E::MatrixXd&) const;
    
    // Composes weights (object local) and given vertices in the rest pose into (Sparse) LBSMatrix. 
    // Both are expected to be filled and alive at the moment of invocation.
    // Inspired by igl::lbs_matrix(..)
    template <typename T>
    E::SparseMatrix<T> getLBSMatrixTemplate_(MatrixXt<T>&) const;
    // non-templated version
    E::SparseMatrix<double> getLBSMatrix_(E::MatrixXd&) const;
};


template<typename T>
inline MatrixXt<T> SMPLWrapper::calcModelTemplate(const T * const pose, const T *  const shape) const
{
    // assignment won't work without cast
    MatrixXt<T> verts = this->verts_template_.cast<T>();
#ifdef DEBUG
    std::cout << "Calc model" << std::endl;
#endif // DEBUG

    if (shape != nullptr)
    {
        this->shapeSMPLTemplate_(shape, verts);
    }

    if (pose != nullptr)
    {
        this->poseSMPLTemplate_(pose, verts);
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
inline void SMPLWrapper::shapeSMPLTemplate_(const T * const shape, MatrixXt<T>& verts) const
{
    for (int i = 0; i < this->SHAPE_SIZE; i++)
    {
        verts += shape[i] * this->shape_diffs_[i];
    }
}


template<typename T>
inline void SMPLWrapper::poseSMPLTemplate_(const T *  const pose, MatrixXt<T>& verts) const
{
#ifdef DEBUG
    std::cout << "pose" << std::endl;
#endif // DEBUG
    // doesn't work without cast, regardless of https://groups.google.com/forum/#!topic/ceres-solver/7ZH21XX6HWU
    MatrixXt<T> jointLocations = this->jointRegressorMat_.cast<T>() * verts;
    MatrixXt<T> jointsTransformation = this->getJointsTransposedGlobalTransformationTemplate_(pose, jointLocations);

    // TODO Use sparce matrices for LBS
    E::SparseMatrix<T> LBSMat = this->getLBSMatrixTemplate_<T>(verts);

    verts = LBSMat * jointsTransformation;
}


template<typename T>
inline MatrixXt<T> SMPLWrapper::getJointsTransposedGlobalTransformationTemplate_(const T * const pose, MatrixXt<T>& jointLocations) const
{
#ifdef DEBUG
    std::cout << "global transform" << std::endl;
#endif // DEBUG

    // uses functions that assume input in 3D (see below)
    assert(SMPLWrapper::SPACE_DIM == 3 && "The function can only be used in 3D world");

    const int homo_size = (SMPLWrapper::SPACE_DIM + 1);
    E::Matrix<T, SMPLWrapper::SPACE_DIM + 1, SMPLWrapper::SPACE_DIM + 1> jointGlobalMats[SMPLWrapper::JOINTS_NUM];
    // Stacked (transposed) global transformation matrices for points
    MatrixXt<T> pointTransformTotal(homo_size * SMPLWrapper::JOINTS_NUM, SMPLWrapper::SPACE_DIM);

    jointGlobalMats[0] = this->get3DLocalTransformMatTemplate_<T>(pose, jointLocations.row(0));
    MatrixXt<T> tmpPointGlobalTransform = jointGlobalMats[0] * this->get3DTranslationMatTemplate_<T>(- jointLocations.row(0));
    pointTransformTotal.block(0, 0, homo_size, SMPLWrapper::SPACE_DIM) 
        = tmpPointGlobalTransform.transpose().leftCols(SMPLWrapper::SPACE_DIM);

    for (int i = 1; i < SMPLWrapper::JOINTS_NUM; i++)
    {
        // Forward Kinematics Formula
        jointGlobalMats[i] = jointGlobalMats[this->joints_parents_[i]] 
            * this->get3DLocalTransformMatTemplate_<T>((pose + i*3),
                jointLocations.row(i) - jointLocations.row(this->joints_parents_[i]));
        
        tmpPointGlobalTransform = jointGlobalMats[i] * this->get3DTranslationMatTemplate_<T>(-jointLocations.row(i));

        pointTransformTotal.block(i * homo_size, 0, homo_size, SMPLWrapper::SPACE_DIM) 
            = tmpPointGlobalTransform.transpose().leftCols(SMPLWrapper::SPACE_DIM);
    }

    return pointTransformTotal;
}


template<typename T, typename Derived2>
inline MatrixXt<T> SMPLWrapper::get3DLocalTransformMatTemplate_(const T * const jointAxisAngleRotation,
    const E::MatrixBase<Derived2>& jointLocation) const 
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
inline MatrixXt<T> SMPLWrapper::get3DTranslationMatTemplate_(const E::MatrixBase<Derived>& translationVector) const
{
    MatrixXt<T> translation;
    translation.setIdentity(4, 4);  // in homogenious coordinates
    translation.block(0, 3, 3, 1) = translationVector.transpose();

    return translation;
}


template<typename T>
inline E::SparseMatrix<T> SMPLWrapper::getLBSMatrixTemplate_(MatrixXt<T>& verts) const
{
  
    const int dim = SMPLWrapper::SPACE_DIM;
    const int nVerts = SMPLWrapper::VERTICES_NUM;
    const int nJoints = SMPLWrapper::JOINTS_NUM;  // Number of joints
#ifdef DEBUG
    std::cout << "LBSMat: start" << std::endl;
#endif // DEBUG
    // +1 goes for homogenious coordinates
    E::SparseMatrix<T> LBSMat(nVerts, (dim + 1) * nJoints);
    std::vector<E::Triplet<T>> LBSTripletList;
    LBSTripletList.reserve(nVerts * (dim + 1) * SMPLWrapper::WEIGHTS_BY_VERTEX);     // for faster filling performance

    // go over non-zero weight elements
    for (int k = 0; k < this->weights_.outerSize(); ++k)
    {
        for (E::SparseMatrix<double>::InnerIterator it(this->weights_, k); it; ++it)
        {
            T weight = (T)it.value();
            int idx_vert = it.row();
            int idx_joint = it.col();
            // premultiply weigths by vertex homogenious coordinates
            for (int idx_dim = 0; idx_dim < dim; idx_dim++)
                LBSTripletList.push_back(E::Triplet<T>(idx_vert, idx_joint * (dim + 1) + idx_dim, weight * verts(idx_vert, idx_dim)));
            LBSTripletList.push_back(E::Triplet<T>(idx_vert, idx_joint * (dim + 1) + dim, weight));
        }
    }
    LBSMat.setFromTriplets(LBSTripletList.begin(), LBSTripletList.end());

    return LBSMat;
}
