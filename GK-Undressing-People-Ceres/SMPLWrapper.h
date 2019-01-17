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
    */
    SMPLWrapper(char, const char*);
    ~SMPLWrapper();

    char getGender()                    { return gender_; };
    E::MatrixXi getFaces()             { return this->faces_; };
    E::MatrixXd getTemplateVertices()  { return this->verts_template_; };

    // Pose/shape parameters can be nullptr: allows to get template/pose without shaping/shaping of the T-pose
    template <typename T>
    MatrixXt<T> calcModel(const T * const, const T * const) const;

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
    // The joints hierarchy is expectes to be so that the 
    int joints_parents_[JOINTS_NUM];
    E::MatrixXd weights_;
    E::MatrixXd joints_default_;

    // private functions
    void readTemplate_();
    void readJointMat_();
    void readShapes_();
    void readWeights_();
    void readHierarchy_();

    template <typename T>
    void shapeSMPL_(const T * const, MatrixXt<T>&) const;
    template <typename T>
    void poseSMPL_(const T * const, MatrixXt<T>&) const;

    // Used with SMPL python module as a reference 
    template <typename T>
    MatrixXt<T> getJointsGlobalTransformation_(const T * const, MatrixXt<T>&) const;
    // TODO Try to derive the T type from the Derived1
    // TODO add checks to the functions below
    // Assumes that SPACE_DIM == 3
    template <typename T, typename Derived1, typename Derived2>
    MatrixXt<T> get3DLocalTransformMat_(const E::MatrixBase<Derived1>&, const E::MatrixBase<Derived2>&) const;
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
    std::cout << "Calc model" << std::endl;

    if (shape != nullptr)
    {
        this->shapeSMPL_(shape, verts);
    }

    if (pose != nullptr)
    {
        this->poseSMPL_(pose, verts);
        std::cout << "Fin posing " << verts.rows() << " x " << verts.cols() << std::endl;
    }

    std::cout << "Fin calculating" << std::endl;

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
    std::cout << "pose" << std::endl;

    MatrixXt<T> jointLocations = this->jointRegressorMat_.cast<T>() * verts;
    //std::cout << jointLocations.rows() << ";  " << jointLocations.cols() << std::endl;

    MatrixXt<T> jointsTransformation = this->getJointsGlobalTransformation_(pose, jointLocations); // .transpose();
    // remove extra column that indicates homogenious coordinates
    //MatrixXt<T> nonHomoTransformation = jointsTransformation.leftCols(SMPLWrapper::SPACE_DIM);

    // Get the LBS matrix
    // TODO Use sparce matrices for LBS
    MatrixXt<T> LBSMat = this->getLBSMatrix_<T>(verts);
    
    // Get the final vertices
    //(*verts) = LBSMat * nonHomoTransformation;
    MatrixXt<T> homoVerts = (LBSMat * jointsTransformation);


    //for (int i = 0; i < SMPLWrapper::VERTICES_NUM; i++)
    //{
    //    for (int j = 0; j < SMPLWrapper::SPACE_DIM + 1; j++)
    //    {
    //        std::cout << homoVerts(i, j) << "; ";
    //    }
    //    std::cout << std::endl;
    //}

    std::cout << "Before cutting the column " << homoVerts.rows() << " x " << homoVerts.cols() << std::endl;

    verts = homoVerts.leftCols(SMPLWrapper::SPACE_DIM);
}


template<typename T>
inline MatrixXt<T> SMPLWrapper::getJointsGlobalTransformation_(const T * const pose, MatrixXt<T>& jointLocations) const
{
    // Form the world transformation for joints (exclude thansformation due to the rest pose -- check if that's identical matrix)
    
    // TODO decide if it's a good thing to store duplicates of result matrices in total and Result Mats

    std::cout << "global transform" << std::endl;

    const int homo_size = (SMPLWrapper::SPACE_DIM + 1);
    // Map pose to Eigen vector
    const E::Map<const E::Matrix<T, SMPLWrapper::JOINTS_NUM, SMPLWrapper::SPACE_DIM>, 0, E::Stride<1, SMPLWrapper::SPACE_DIM>> ePose(pose);
    // Create array for joint's transformation matrices 
    E::Matrix<T, SMPLWrapper::SPACE_DIM + 1, SMPLWrapper::SPACE_DIM + 1> resultMats[SMPLWrapper::JOINTS_NUM];
    // Array for stacked transformation matrices 
    MatrixXt<T> total(homo_size * SMPLWrapper::JOINTS_NUM, homo_size);
    
    //std::cout << "Pose \n";
    //for (int i = 0; i < SMPLWrapper::JOINTS_NUM; i++)
    //{
    //    for (int j = 0; j < SMPLWrapper::SPACE_DIM; j++)
    //    {
    //        std::cout << ePose(i, j) << "/" << pose[i*SMPLWrapper::SPACE_DIM + j] << "; ";
    //    }
    //    std::cout << std::endl;
    //}
    

    resultMats[0] = this->get3DLocalTransformMat_<T>(ePose.row(0), jointLocations.row(0)) 
        * this->get3DTranslationMat_<T>(- jointLocations.row(0));

    std::cout << "First mat \n";
    for (int i = 0; i < SMPLWrapper::SPACE_DIM + 1; i++)
    {
        for (int j = 0; j < SMPLWrapper::SPACE_DIM + 1; j++)
        {
            std::cout << resultMats[0](i, j) << "; ";
        }
        std::cout << std::endl;
    }

    total.block(0, 0, homo_size, homo_size) = resultMats[0].transpose();
    for (int i = 1; i < SMPLWrapper::JOINTS_NUM; i++)
    {
        resultMats[i] = this->get3DLocalTransformMat_<T>(ePose.row(i), jointLocations.row(i))
            * resultMats[this->joints_parents_[i]] 
            * this->get3DTranslationMat_<T>(-jointLocations.row(i));
        //std::cout << "after matrix calc" << std::endl;
        // stack the matrices
        total.block(i * homo_size, 0, homo_size, homo_size) = resultMats[i].transpose();
    }

    /*
    std::cout << "Total mat \n";
    for (int i = 0; i < homo_size * SMPLWrapper::JOINTS_NUM; i++)
    {
        for (int j = 0; j < SMPLWrapper::SPACE_DIM + 1; j++)
        {
            std::cout << total(i, j) << "; ";
        }
        std::cout << std::endl;
    }
    */

    return total;
}


template<typename T, typename Derived1, typename Derived2>
inline MatrixXt<T> SMPLWrapper::get3DLocalTransformMat_(const E::MatrixBase<Derived1>& jointAxisAngleRotation,
    const E::MatrixBase<Derived2>& jointLocation) const //E::EigenBase<const T>
{    
    /*
    std::cout << "Joint Rotation" << std::endl;
    for (int i = 0; i < 3; i++)
    {
        std::cout << jointAxisAngleRotation(i) << "; ";
    }

    std::cout << "\nJoint Location" << std::endl;
    for (int i = 0; i < 3; i++)
    {
        std::cout << jointLocation(i) << "; ";
    }
    */
    // apply Rodrigues formula
    E::Matrix<T, 3, 3> exponent;
    exponent.setIdentity();
    T norm = jointAxisAngleRotation.norm();

    if (norm > 0.0001)  // don't waste computations on zero joint movement
    {
        // construct skew-simm matrix
        E::Matrix<T, 3, 3> skew;
        //skew << 0, -jointAxisAngleTransform(2), jointAxisAngleTransform(1),
        //    jointAxisAngleTransform(2), 0, -jointAxisAngleTransform(0),
        //    -jointAxisAngleTransform(1), jointAxisAngleTransform(0), 0;
        // TODO remove Auto to make this piece faster
        auto axis = jointAxisAngleRotation / norm;
        skew.setZero();
        skew(0, 1) = - axis(2);
        skew(0, 2) = axis(1);
        skew(1, 0) = axis(2);
        skew(1, 2) = - axis(0);
        skew(2, 0) = - axis(1);
        skew(2, 1) = axis(0);

        std::cout << "\nSkew" << std::endl;
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                std::cout << skew(i, j) << "; ";
            }
            std::cout << std::endl;
        }

        exponent += skew * sin(norm) + skew * skew * ((T)1. - cos(norm));
    }
    
    /*
    std::cout << "Exponent" << std::endl;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            std::cout << exponent(i, j) << "; ";
        }
        std::cout << std::endl;
    }
    */
    // construct final matrix 
    E::Matrix<T, 4, 4> localTransform;
    localTransform.block(0, 0, 3, 3) = exponent;
    localTransform.block(0, 3, 3, 1) = jointLocation.transpose();
    localTransform.block(3, 0, 1, 3) = E::Matrix<T, 3, 1>::Zero();
    localTransform(3, 3) = (T) 1.;

    /*
    std::cout << "Final" << std::endl;
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            std::cout << localTransform(i, j) << "; ";
        }
        std::cout << std::endl;
    }
    */

    return MatrixXt<T>(localTransform);
}


template<typename T, typename Derived>
inline MatrixXt<T> SMPLWrapper::get3DTranslationMat_(const E::MatrixBase<Derived>& translationVector) const
{
    E::Matrix<T, 4, 4> translation;
    translation.block(0, 0, 3, 3) = E::Matrix<T, 3, 3>::Identity();
    translation.block(0, 3, 3, 1) = translationVector.transpose();
    translation.block(3, 0, 1, 3) = E::Matrix<T, 3, 1>::Zero();
    translation(3, 3) = (T) 1.;

    return MatrixXt<T>(translation);
}


template<typename T>
inline MatrixXt<T> SMPLWrapper::getLBSMatrix_(MatrixXt<T>& verts) const
{
    MatrixXt<T> weights = this->weights_.cast<T>();
    MatrixXt<T> LBSMat;
    const int dim = SMPLWrapper::SPACE_DIM;
    const int nVerts = SMPLWrapper::VERTICES_NUM;
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
