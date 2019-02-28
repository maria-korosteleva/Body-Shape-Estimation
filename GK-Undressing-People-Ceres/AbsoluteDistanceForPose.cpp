#include "pch.h"
#include "AbsoluteDistanceForPose.h"


AbsoluteDistanceForPose::AbsoluteDistanceForPose(SMPLWrapper* smpl, GeneralMesh * toMesh, double * shape)
    : toMesh_(toMesh), smpl_(smpl), shape_(shape)
{
    this->key_verts_num_ = toMesh->getKeyPoints().size();

    //this->set_num_residuals(this->key_verts_num_);
    //this->set_num_residuals(this->key_verts_num_ + SMPLWrapper::VERTICES_NUM);
    this->set_num_residuals(SMPLWrapper::VERTICES_NUM);

    this->mutable_parameter_block_sizes()->push_back(SMPLWrapper::POSE_SIZE);
    this->mutable_parameter_block_sizes()->push_back(SMPLWrapper::SPACE_DIM);
}


AbsoluteDistanceForPose::~AbsoluteDistanceForPose()
{
}

bool AbsoluteDistanceForPose::Evaluate(double const * const * parameters, double * residuals, double ** jacobians) const
{
    assert(SMPLWrapper::SPACE_DIM == 3 && "Distance evaluation is only implemented in 3D");
    assert(this->parameter_block_sizes()[0] == SMPLWrapper::POSE_SIZE && "Pose parameter size is set as expected");
    assert(this->parameter_block_sizes()[1] == SMPLWrapper::SPACE_DIM && "Translation parameter size is set as expected");

    Eigen::MatrixXd pose_jac[SMPLWrapper::POSE_SIZE];
    Eigen::MatrixXd verts;
    if (jacobians != NULL && jacobians[0] != NULL)
    {
        verts = this->smpl_->calcModel(parameters[0], this->shape_, pose_jac, nullptr);
    }
    else
    {
        verts = this->smpl_->calcModel(parameters[0], this->shape_);
    }

    // translate
    for (int i = 0; i < SMPLWrapper::VERTICES_NUM; i++)
    {
        for (int j = 0; j < SMPLWrapper::SPACE_DIM; j++)
        {
            verts(i, j) += parameters[1][j];
        }
    }

    Eigen::VectorXd sqrD;
    Eigen::MatrixXd closest_points;
    Eigen::VectorXi closest_face_ids;

    igl::point_mesh_squared_distance(verts, this->toMesh_->getVertices(), this->toMesh_->getFaces(), sqrD, closest_face_ids, closest_points);

    assert(sqrD.size() == SMPLWrapper::VERTICES_NUM && "Size of the set of distances should equal main parameters");
    assert(closest_points.rows() == SMPLWrapper::VERTICES_NUM && "Size of the set of distances should equal main parameters");

    for (int i = 0; i < SMPLWrapper::VERTICES_NUM; ++i)
    {
        residuals[i] = sqrD(i); 
    }

    // Jacobians
    // w.r.t. pose
    if (jacobians != NULL && jacobians[0] != NULL) 
    {
        for (int v_id = 0; v_id < SMPLWrapper::VERTICES_NUM; ++v_id)
        {
            for (int p_id = 0; p_id < SMPLWrapper::POSE_SIZE; ++p_id)
            {
                jacobians[0][(v_id) * SMPLWrapper::POSE_SIZE + p_id]
                    = 2. * (verts.row(v_id) - closest_points.row(v_id)).dot(pose_jac[p_id].row(v_id)); 
            }
        }
    }

    // wrt translation
    if (jacobians != NULL && jacobians[1] != NULL) 
    {
        for (int v_id = 0; v_id < SMPLWrapper::VERTICES_NUM; ++v_id)
        {
            for (int k = 0; k < SMPLWrapper::SPACE_DIM; ++k)
            {
                jacobians[1][(v_id) * SMPLWrapper::SPACE_DIM + k]
                    = 2 * (verts(v_id, k) - closest_points(v_id, k));  
            }
        }
    }

    return true;
}
