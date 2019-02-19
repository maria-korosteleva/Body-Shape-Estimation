#pragma once
/*
The class is a wrapper around SMPL model. In 
The class is able to calculate the SMPL model output based on pose and shape parameters.

TODO: 
    - Add pose blendshape 
*/

//#define DEBUG

#include <assert.h>
#include <map>

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

using Dictionary = std::map<std::string, int>;
using DictEntry = std::pair<std::string, int>;

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
    SMPLWrapper(char gender, const char* path);
    ~SMPLWrapper();

    char getGender() const                    { return gender_; };
    const E::MatrixXi& getFaces() const              { return this->faces_; };
    const E::MatrixXd& getTemplateVertices() const   { return this->verts_template_; };
    const E::VectorXd& getTemplateMeanPoint() const  { return this->template_mean_point_; };
    const Dictionary& getKeyVertices() const         { return this->key_vertices_; }

    // Pose/shape parameters can be nullptr: allows to get template/pose without shaping/shaping of the T-pose
    // When initialized pose_jac is expected to have space for POSE_SIZE Matrices, 
    // shape_jac is expected to have space for SHAPE_SIZE Matrices
    E::MatrixXd calcModel(const double * const pose, const double * const shape, E::MatrixXd * pose_jac = nullptr, E::MatrixXd * shape_jac = nullptr) const;

    E::MatrixXd calcJointLocations(const double * shape);

    // Pose/shape parameters can be nullptr: allows to get template/pose without shaping/shaping of the T-pose
    void saveToObj(const double * translation, const double * pose, const double* shape, const std::string path) const;

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
    // vertices on the important parts of the body
    Dictionary key_vertices_;

    // private functions
    void readTemplate_();
    void readJointMat_();
    void readShapes_();
    void readWeights_();
    void readHierarchy_();
    void readKeyVertices_();

    // for evaluation uses vertex info from the last parameter and uses last parameter for output
    // if not nullptr, shape_jac is expected to be an array of SHAPE_SIZE of MatrixXd, one matrix for each shape parameter
    void shapeSMPL_(const double * const shape, E::MatrixXd &verts, E::MatrixXd* shape_jac = nullptr) const;
    // for evaluation uses vertex info from the last parameter and uses last parameter for output
    // if not nullptr, pose_jac is expected to be an array of POSE_SIZE of MatrixXd, one matrix for each pose parameter
    void poseSMPL_(const double * const pose, E::MatrixXd & verts, E::MatrixXd * pose_jac = nullptr) const;

    // Assumes that SPACE_DIM == 3
    // Assumes the default joint angles to be all zeros
    // Returns matrix of dimentions (SPACE_DIM + 1) * JOINTS_NUM  x  SPACE_DIM 
    // of stacked transposed global transformation matrices of each joint, with the row of homogenious coordinates removed
    // can calculate analytic jacobian
    E::MatrixXd getJointsTransposedGlobalTransformation_(const double * const pose, E::MatrixXd & jointLocations, E::MatrixXd * jacsTotal = nullptr) const;
    
    // Assumes that SPACE_DIM == 3
    // fills the dependence of the Transformation matric on all three coordinates for the input rotation. 
    // When initialized jac is expected to have space for POSE_SIZE Matrices
    E::MatrixXd get3DLocalTransformMat_(const double * const jointAxisAngleRotation, const E::MatrixXd & jointToParentDist, E::MatrixXd* localTransformJac = nullptr) const;
    
    // Assumes that SPACE_DIM == 3
    E::MatrixXd get3DTranslationMat_(const E::MatrixXd & translationVector) const;
    
    // Composes weights (object local) and given vertices in the rest pose into (Sparse) LBSMatrix. 
    // Both are expected to be filled and alive at the moment of invocation.
    // Inspired by igl::lbs_matrix(..)
    E::SparseMatrix<double> getLBSMatrix_(E::MatrixXd & verts) const;
};

