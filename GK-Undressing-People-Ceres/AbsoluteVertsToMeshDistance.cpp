#include "pch.h"
#include "AbsoluteVertsToMeshDistance.h"


AbsoluteVertsToMeshDistance::AbsoluteVertsToMeshDistance(SMPLWrapper* smpl, GeneralMesh * toMesh)
    : toMesh_(toMesh), smpl_(smpl)
{
    this->key_verts_num_ = toMesh->getKeyVertices().size();

    //this->set_num_residuals(this->key_verts_num_);
    this->set_num_residuals(this->key_verts_num_ + SMPLWrapper::VERTICES_NUM);

    this->mutable_parameter_block_sizes()->push_back(SMPLWrapper::POSE_SIZE);
    this->mutable_parameter_block_sizes()->push_back(SMPLWrapper::SPACE_DIM);
}


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
    Eigen::MatrixXd verts;
    if (jacobians != NULL && jacobians[0] != NULL)
    {
        verts = this->smpl_->calcModel(parameters[0], nullptr, pose_jac, nullptr);
    }
    else
    {
        verts = this->smpl_->calcModel(parameters[0], nullptr);
    }

    // translate
    for (int i = 0; i < SMPLWrapper::VERTICES_NUM; i++)
    {
        for (int j = 0; j < SMPLWrapper::SPACE_DIM; j++)
        {
            verts(i, j) += parameters[1][j];
        }
    }

    // TODO divide into sub-functions
    /// key_vertices
    Dictionary inputKeyVerts = this->toMesh_->getKeyVertices();
    Dictionary smplKeyVerts = this->smpl_->getKeyVertices();
    int res_id = 0;
    for (auto const& keyIterator : inputKeyVerts)
    {
        Eigen::VectorXd in_point = this->toMesh_->getVertices().row(keyIterator.second);
        int model_v_id = smplKeyVerts[keyIterator.first];
        Eigen::VectorXd model_point = verts.row(model_v_id);
        Eigen::VectorXd diff = model_point - in_point;
        residuals[res_id] = this->coef_key_vertices_ * diff.norm() * diff.norm();

        // jacobian
        // w.r.t. pose
        if (jacobians != NULL && jacobians[0] != NULL)
        {
            for (int p_id = 0; p_id < SMPLWrapper::POSE_SIZE; ++p_id)
            {
                jacobians[0][res_id * SMPLWrapper::POSE_SIZE + p_id] =
                    this->coef_key_vertices_ * 2. * diff.dot(pose_jac[p_id].row(model_v_id));
            }
        }
        // w.r.t. translation
        if (jacobians != NULL && jacobians[1] != NULL)
        {
            for (int k = 0; k < SMPLWrapper::SPACE_DIM; ++k)
            {
                jacobians[1][res_id * SMPLWrapper::SPACE_DIM + k]
                    = this->coef_key_vertices_ * 2 * diff(k);
            }
        }

        // w.r.t. shape
        //if (jacobians != NULL && jacobians[2] != NULL) {
            //for (int sh_id = 0; sh_id < SMPLWrapper::SHAPE_SIZE; ++sh_id)
            //{
            //    jacobians[2][res_id * SMPLWrapper::SHAPE_SIZE + sh_id] = 
            //        this->coef_key_vertices_ * 2. * diff.dot(shape_jac[sh_id].row(res_id));
            //}
        //}

        res_id++;
    }

    // point-to-surface dist component
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
        residuals[this->key_verts_num_ + i] = sqrD(i);
    }

    // Jacobians
    // w.r.t. pose
    if (jacobians != NULL && jacobians[0] != NULL) 
    {
#ifdef DEBUG
        std::cout << "Final pose Jacobian evaluation" << std::endl;
#endif // DEBUG

        for (int v_id = 0; v_id < SMPLWrapper::VERTICES_NUM; ++v_id)
        {
            for (int p_id = 0; p_id < SMPLWrapper::POSE_SIZE; ++p_id)
            {
                jacobians[0][(this->key_verts_num_ + v_id) * SMPLWrapper::POSE_SIZE + p_id] =
                    2. * (verts.row(v_id) - closest_points.row(v_id)).dot(pose_jac[p_id].row(v_id));
            }
        }
    }

    // wrt translation
    if (jacobians != NULL && jacobians[1] != NULL) 
    {
        
#ifdef DEBUG
        std::cout << "Abs dists evaluate: jacobian evaluation" << std::endl;
#endif // DEBUG
        for (int v_id = 0; v_id < SMPLWrapper::VERTICES_NUM; ++v_id)
        {
            for (int k = 0; k < SMPLWrapper::SPACE_DIM; ++k)
            {
                jacobians[1][(this->key_verts_num_ + v_id) * SMPLWrapper::SPACE_DIM + k]
                    = 2 * (verts(v_id, k) - closest_points(v_id, k));
            }
        }
    }

    // w.r.t. shape
    //if (jacobians != NULL && jacobians[2] != NULL) {
    //    for (int res_id = 0; res_id < SMPLWrapper::VERTICES_NUM; ++res_id)
    //    {
    //        for (int sh_id = 0; sh_id < SMPLWrapper::SHAPE_SIZE; ++sh_id)
    //        {
    //            jacobians[2][(this->key_verts_num_ + v_id) * SMPLWrapper::SHAPE_SIZE + sh_id] = 
    //                2. * (verts.row(res_id) - closest_points.row(res_id)).dot(shape_jac[sh_id].row(res_id));
    //        }
    //    }
    //}

    return true;
}
