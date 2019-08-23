#pragma once
/*
The class is a wrapper around SMPL model. It is able to calculate the SMPL model output based on pose and shape parameters.

Limitations:
    - Wrapper does not keep the tranformed vertices and joint locations corresponding to the current
    wrapper state_. Since the state is allowed to be modified from the outside, there is no way to know
    "the freshness" of the atrefacts, if saved.

TODO: - Add pose blendshape 
*/

//#define DEBUG

#include <assert.h>
#include <map>

#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include <igl/readOBJ.h>
#include <igl/writeOBJ.h>
#include <igl/per_vertex_normals.h>


namespace E = Eigen;
using ERMatrixXd = E::Matrix<double, -1, -1, E::RowMajor>;

class SMPLWrapper
{
public:
    struct State {
        double* pose = nullptr;
        double* shape = nullptr;
        double* translation = nullptr;
        ERMatrixXd displacements;

        State();
        ~State();
    };
    static constexpr std::size_t SHAPE_SIZE = 10;
    static constexpr std::size_t SPACE_DIM = 3; // needs to be 3 for most of the routines to work
    static constexpr std::size_t HOMO_SIZE = SMPLWrapper::SPACE_DIM + 1;
    static constexpr std::size_t POSE_SIZE = 72;
    static constexpr std::size_t JOINTS_NUM = POSE_SIZE / SPACE_DIM;
    static constexpr std::size_t VERTICES_NUM = 6890;
    static constexpr std::size_t WEIGHTS_BY_VERTEX = 4;     // number of joints each vertex depend on

    using DictionaryInt = std::map<std::string, int>;
    using DictEntryInt = std::pair<std::string, int>;
    using EHomoCoordMatrix = E::Matrix<double, HOMO_SIZE, HOMO_SIZE>;

    /*
    Class should be initialized with the gender of the model to use and with the path to the model folder,
    that contains the files in a pre-defined structure.
    gender is either "f" or "m"
    The joints hierarchy is expectes to be so that the parent's id is always less than the child's
    */
    SMPLWrapper(char gender, const std::string path);
    ~SMPLWrapper();

    char getGender() const                    { return gender_; };
    const E::MatrixXi& getFaces() const              { return faces_; };
    const E::MatrixXd& getTemplateVertices() const   { return verts_template_normalized_; };
    const E::VectorXd& getTemplateMeanPoint() const  { return E::Vector3d(0, 0, 0); };
    // !! returns pointer to the inner arrays
    State& getStatePointers() { return state_; }

    void rotateLimbToDirection(const std::string joint_name, const E::Vector3d& direction);
    // Matching both directions exaclty is not guaranteed -- hips direction will be matched approximately
    void rotateRoot(const E::Vector3d& body_up, const E::Vector3d& body_right_to_left);
    // shoulder_dir points from right to left 
    void twistBack(const E::Vector3d& shoulder_dir);

    // re-calculates translation to move the posed/shaped mesh center to the specified point
    void translateTo(const E::VectorXd& center_point);

    // *_jacs are expected to have space for POSE_SIZE and SHAPE_SIZE Matrices
    E::MatrixXd calcModel(const double * const translation, 
        const double * const pose, 
        const double * const shape,
        const ERMatrixXd * displacement,
        E::MatrixXd * pose_jac = nullptr, 
        E::MatrixXd * shape_jac = nullptr,
        E::MatrixXd * displacement_jac = nullptr);
    // calculate for the supplied vertices (calcModel output)
    E::MatrixXd calcVertexNormals(const E::MatrixXd* verts);
    // using current SMPLWrapper state
    E::MatrixXd calcModel();
    E::MatrixXd calcJointLocations();

    // using current SMPLWrapper state
    void saveToObj(const std::string path);
    void saveWithDisplacementToObj(const std::string path);
    void savePosedOnlyToObj(const std::string path);
    void saveShapedOnlyToObj(const std::string path);
    void saveShapedWithDisplacementToObj(const std::string path);
    void logParameters(const std::string path);

private:
    void readTemplate_();
    void readJointMat_();
    void readJointNames_();
    void readShapes_();
    void readWeights_();
    void readHierarchy_();

    void saveToObj_(const double * translation, const double * pose, const double* shape, const ERMatrixXd* displacements,
        const std::string path);

    // For individual joint rotation calculation
    static E::Vector3d angle_axis_(const E::Vector3d& from, const E::Vector3d& to);
    static E::Vector3d rotate_by_angle_axis_(const E::Vector3d& vector, const E::Vector3d& angle_axis_rotation);
    static E::Vector3d combine_two_angle_axis_(const E::Vector3d& first, const E::Vector3d& second);
    // pass fk_transforms_ to be explicit of which version of fk_transforms is used for calculations
    void assignJointGlobalRotation_(int joint_id, E::VectorXd rotation, 
        const EHomoCoordMatrix(&fk_transform)[SMPLWrapper::JOINTS_NUM]);
   
    // Model calculation
    // if not nullptr, shape_jac is expected to be an array of SHAPE_SIZE of MatrixXd, one matrix for each shape parameter
    void shapeSMPL_(const double * const shape, E::MatrixXd &verts, E::MatrixXd* shape_jac = nullptr);
    void poseSMPL_(const double * const pose, E::MatrixXd & verts, E::MatrixXd * pose_jac = nullptr);
    // Jaconian is not provided because it's always an identity: dv_i / d_tj == 1 => don't want to waste memory on it
    void translate_(const double * const translation, E::MatrixXd & verts);

    E::MatrixXd calcJointLocations_(const double * translation = nullptr, 
        const double * shape = nullptr, const double * pose = nullptr);

    // pass fk_transforms_ to be explicit of which version of fk_transforms is used for calculations
    static E::MatrixXd extractJointLocationFromFKTransform_(const EHomoCoordMatrix(&fk_transform)[SMPLWrapper::JOINTS_NUM]);

    // result passed in the form of stacked transposed matrices with the homo row clipped: JOINTS_NUM * 4 x 3
    static E::MatrixXd extractLBSJointTransformFromFKTransform_(
        const EHomoCoordMatrix (&fk_transform) [SMPLWrapper::JOINTS_NUM], const E::MatrixXd & t_pose_joints_locations,
        const E::MatrixXd (*FKDerivatives)[SMPLWrapper::JOINTS_NUM][SMPLWrapper::POSE_SIZE] = nullptr, 
        E::MatrixXd * jacsTotal = nullptr);

    // Posing routines: all sssumes that SPACE_DIM == 3
    // Assumes the default joint angles to be all zeros
    // Updates fk_*
    void updateJointsFKTransforms_(const double * const pose, 
        const E::MatrixXd & t_pose_joints_locations, bool calc_derivatives = false);
    
    static E::MatrixXd get3DLocalTransformMat_(
        const double * const jointAxisAngleRotation, 
        const E::MatrixXd & jointToParentDist);

    static void get3DLocalTransformJac_(
        const double * const jointAxisAngleRotation,
        const E::MatrixXd & transform_mat,
        E::MatrixXd* local_transform_jac_out);

    static E::MatrixXd get3DTranslationMat_(const E::MatrixXd & translationVector);
    
    // Composes weights (object local) and given vertices in the rest pose into (Sparse) LBSMatrix. 
    // Inspired by igl::lbs_matrix(..)
    E::SparseMatrix<double> getLBSMatrix_(const E::MatrixXd & verts) const;

    // initial info
    char gender_;
    std::string gender_path_;
    std::string general_path_;

    // constant model info
    E::MatrixXi faces_;
    E::MatrixXd verts_template_;
    E::MatrixXd verts_template_normalized_;
    E::MatrixXd joint_locations_template_;
    E::MatrixXd shape_diffs_[10];  // store only differences between blendshapes and template
    E::MatrixXd jointRegressorMat_;
    static int joints_parents_[JOINTS_NUM];
    DictionaryInt joint_names_;
    E::SparseMatrix<double> weights_;

    // current state
    State state_;

    
    // These vars indicate the last model re-calculation 
    // !! Params are allowed to be changed directly, so make sure it's fresh before using
    EHomoCoordMatrix fk_transforms_[SMPLWrapper::JOINTS_NUM];
    E::MatrixXd fk_derivatives_[SMPLWrapper::JOINTS_NUM][SMPLWrapper::POSE_SIZE];

    //E::MatrixXd joints_global_transform_;
    //E::MatrixXd shaped_joints_locations_;
    //E::MatrixXd joints_locations_;  // all shaped, posed and translated
    //E::MatrixXd verts_;

public:
    // I use fixed-size eigen objects as class members, 
    // https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

