#include "AbsoluteDistanceForShape.h"


AbsoluteDistanceForShape::AbsoluteDistanceForShape(SMPLWrapper* smpl, GeneralMesh * toMesh, const double param = 1.)
    : toMesh_(toMesh), smpl_(smpl), gm_coef_(param)
{
    this->set_num_residuals(SMPLWrapper::VERTICES_NUM);

    this->mutable_parameter_block_sizes()->push_back(SMPLWrapper::SHAPE_SIZE);
    this->mutable_parameter_block_sizes()->push_back(SMPLWrapper::SPACE_DIM);
}


AbsoluteDistanceForShape::~AbsoluteDistanceForShape()
{
}


bool AbsoluteDistanceForShape::Evaluate(double const * const * parameters, double * residuals, double ** jacobians) const
{
    assert(SMPLWrapper::SPACE_DIM == 3 && "Distance evaluation is only implemented in 3D");
    assert(this->parameter_block_sizes()[0] == SMPLWrapper::SHAPE_SIZE && "Shape parameter size is set as expected");
    assert(this->parameter_block_sizes()[1] == SMPLWrapper::SPACE_DIM && "Translation parameter size is set as expected");

    // params inside the smpl_ object ARE NOT the same as the oned passed through parameters argument
    Eigen::MatrixXd shape_jac[SMPLWrapper::SHAPE_SIZE];
    Eigen::MatrixXd verts;
    if (jacobians != NULL && jacobians[0] != NULL)
    {
        verts = smpl_->calcModel(parameters[1], smpl_->getStatePointers().pose, parameters[0], nullptr, shape_jac);
    }
    else
    {
        verts = smpl_->calcModel(parameters[1], smpl_->getStatePointers().pose, parameters[0]);
    }

    //Eigen::VectorXd sqrD;
    Eigen::VectorXd signedDists; Eigen::MatrixXd closest_points;
    Eigen::VectorXi closest_face_ids; Eigen::MatrixXd normals;

    //igl::point_mesh_squared_distance(verts, this->toMesh_->getVertices(), this->toMesh_->getFaces(), sqrD, closest_face_ids, closest_points);
    igl::SignedDistanceType type = igl::SIGNED_DISTANCE_TYPE_PSEUDONORMAL;
    igl::signed_distance(verts, 
        toMesh_->getNormalizedVertices(), 
        toMesh_->getFaces(),
        type, signedDists, closest_face_ids, closest_points, normals);

    assert(signedDists.size() == SMPLWrapper::VERTICES_NUM && "Size of the set of distances should equal main parameters");
    assert(closest_points.rows() == SMPLWrapper::VERTICES_NUM && "Size of the set of distances should equal main parameters");

    for (int i = 0; i < SMPLWrapper::VERTICES_NUM; ++i)
    {
        residuals[i] = residual_elem_(signedDists(i));
    }

    // Jacobians
    // w.r.t. shape
    if (jacobians != NULL && jacobians[0] != NULL) {
        for (int v_id = 0; v_id < SMPLWrapper::VERTICES_NUM; ++v_id)
        {
            for (int sh_id = 0; sh_id < SMPLWrapper::SHAPE_SIZE; ++sh_id)
            {
                jacobians[0][v_id * SMPLWrapper::SHAPE_SIZE + sh_id]
                    = shape_jac_elem_(verts.row(v_id), closest_points.row(v_id), signedDists(v_id),
                        shape_jac[sh_id].row(v_id));
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
                jacobians[1][v_id* SMPLWrapper::SPACE_DIM + k]
                    = translation_jac_elem_(verts(v_id, k), closest_points(v_id, k), signedDists(v_id));
            }
        }
    }
    return true;
}
