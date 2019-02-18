#include "pch.h"
#include "AbsoluteVertsToMeshDistance.h"


AbsoluteVertsToMeshDistance::AbsoluteVertsToMeshDistance(SMPLWrapper* smpl, GeneralMesh * toMesh)
    : toMesh_(toMesh), smpl_(smpl)
{}


AbsoluteVertsToMeshDistance::~AbsoluteVertsToMeshDistance()
{
}

bool AbsoluteVertsToMeshDistance::Evaluate(double const * const * parameters, double * residuals, double ** jacobians) const
{
    assert(SMPLWrapper::SPACE_DIM == 3 && "Distance evaluation is only implemented in 3D");
    assert(this->parameter_block_sizes()[0] == SMPLWrapper::POSE_SIZE && "Pose parameter size is set as expected");
    assert(this->parameter_block_sizes()[1] == SMPLWrapper::SPACE_DIM && "Translation parameter size is set as expected");
    //assert(this->parameter_block_sizes()[2] == SMPLWrapper::SHAPE_SIZE && "Shape parameter size is set as expected");
    
#ifdef DEBUG
    std::cout << "Num of Param blocks " << this->parameter_block_sizes().size() << std::endl;
    std::cout << "Abs dists evaluate" << std::endl;
#endif // DEBUG

    //Eigen::MatrixXd shape_jac[SMPLWrapper::SHAPE_SIZE];
    Eigen::MatrixXd pose_jac[SMPLWrapper::POSE_SIZE];
    //Eigen::MatrixXd verts = this->smpl_->calcModel(parameters[0], parameters[1], pose_jac, shape_jac);
    Eigen::MatrixXd verts = this->smpl_->calcModel(parameters[0], nullptr, pose_jac, nullptr);

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

#ifdef DEBUG
    std::cout << "Abs dists evaluate: start igl calculation" << std::endl;
#endif // DEBUG

    igl::point_mesh_squared_distance(verts, this->toMesh_->getVertices(), this->toMesh_->getFaces(), sqrD, closest_face_ids, closest_points);

#ifdef DEBUG
    std::cout << "Abs dists evaluate: Fin igl calculation" << std::endl;
    std::cout << "Dist Sum " << sqrD.sum() << std::endl;
    std::cout << "Dist Max " << sqrD.maxCoeff() << std::endl;
#endif // DEBUG

    assert(sqrD.size() == SMPLWrapper::VERTICES_NUM && "Size of the set of distances should equal main parameters");
    assert(closest_points.rows() == SMPLWrapper::VERTICES_NUM && "Size of the set of distances should equal main parameters");

    for (int i = 0; i < SMPLWrapper::VERTICES_NUM; ++i)
    {
        residuals[i] = sqrD(i);
    }

    // Jacobians
    // jacobians[i] is a row - major array of size num_residuals x parameter_block_sizes_[i].
        // If jacobians[i] is not NULL, the user is required to compute the Jacobian of the residual vector 
        // with respect to parameters[i] and store it in this array, i.e.
        // jacobians[i][r * parameter_block_sizes_[i] + c] = d residual[r] / d parameters[i][c]
    
    // Main idea for point-to-surface distance jacobian: Gradient for each vertex correspondes to the distance from this vertex to the input mesh.

    // w.r.t. pose
    if (jacobians != NULL && jacobians[0] != NULL) {
#ifdef DEBUG
        std::cout << "Final pose Jacobian evaluation" << std::endl;
#endif // DEBUG

        for (int res_id = 0; res_id < SMPLWrapper::VERTICES_NUM; ++res_id)
        {
            for (int p_id = 0; p_id < SMPLWrapper::POSE_SIZE; ++p_id)
            {
                jacobians[0][res_id * SMPLWrapper::POSE_SIZE + p_id] =
                    2. * (verts.row(res_id) - closest_points.row(res_id)).dot(pose_jac[p_id].row(res_id));
            }
        }
    }

    // wrt translation
    if (jacobians != NULL && jacobians[0] != NULL) {
        
#ifdef DEBUG
        std::cout << "Abs dists evaluate: jacobian evaluation" << std::endl;
#endif // DEBUG
        for (int j = 0; j < SMPLWrapper::VERTICES_NUM; ++j)
        {
            for (int k = 0; k < SMPLWrapper::SPACE_DIM; ++k)
            {
                jacobians[1][j * SMPLWrapper::SPACE_DIM + k]
                    = 2 * (verts(j, k) - closest_points(j, k));
            }
        }
    }

    // w.r.t. shape
    //if (jacobians != NULL && jacobians[1] != NULL) {
    //    for (int res_id = 0; res_id < SMPLWrapper::VERTICES_NUM; ++res_id)
    //    {
    //        for (int sh_id = 0; sh_id < SMPLWrapper::SHAPE_SIZE; ++sh_id)
    //        {
    //            jacobians[2][res_id * SMPLWrapper::SHAPE_SIZE + sh_id] = 
    //                2. * (verts.row(res_id) - closest_points.row(res_id)).dot(shape_jac[sh_id].row(res_id));
    //        }
    //    }
    //}

    return true;
}
