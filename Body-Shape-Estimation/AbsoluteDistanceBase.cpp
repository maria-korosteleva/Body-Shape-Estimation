#include "AbsoluteDistanceBase.h"

AbsoluteDistanceBase::DistanceResult AbsoluteDistanceBase::last_result_;

AbsoluteDistanceBase::AbsoluteDistanceBase(SMPLWrapper* smpl, GeneralMesh * toMesh,
    ParameterType parameter, DistanceType dist_type,  double pruning_threshold, std::size_t vertex_id)
    : ceres::EvaluationCallback(),
    toMesh_(toMesh), smpl_(smpl),
    pruning_threshold_(pruning_threshold),
    parameter_type_(parameter), vertex_id_for_displacement_(vertex_id), dist_evaluation_type_(dist_type)
{
    switch (parameter)
    {
        case TRANSLATION:
            this->set_num_residuals(SMPLWrapper::VERTICES_NUM);
            this->mutable_parameter_block_sizes()->push_back(SMPLWrapper::SPACE_DIM);
            break;
        case SHAPE:
            this->set_num_residuals(SMPLWrapper::VERTICES_NUM);
            this->mutable_parameter_block_sizes()->push_back(SMPLWrapper::SHAPE_SIZE);
            break;
        case POSE:
            this->set_num_residuals(SMPLWrapper::VERTICES_NUM);
            this->mutable_parameter_block_sizes()->push_back(SMPLWrapper::POSE_SIZE);
            break;
        case DISPLACEMENT: 
            this->set_num_residuals(1);     // special case -- displacements are optimized per-vertex
            this->mutable_parameter_block_sizes()->push_back(SMPLWrapper::SPACE_DIM);
            break;
        default:
            std::cout << "DistanceBase initialization::WARNING:: no parameter type specified\n";
    }
}

AbsoluteDistanceBase::~AbsoluteDistanceBase()
{
}

void AbsoluteDistanceBase::PrepareForEvaluation(bool evaluate_jacobians, bool new_evaluation_point)
{
    if (evaluate_jacobians || new_evaluation_point)
        updateDistanceCalculations(evaluate_jacobians, last_result_);
}

bool AbsoluteDistanceBase::Evaluate(double const * const * parameters, double * residuals, double ** jacobians) const
{
    assert(SMPLWrapper::SPACE_DIM == 3 && "Distance evaluation is only implemented in 3D");
    
    // TODO add the checks for the expected parameter size and the one used for calculating last_result
    DistanceResult& distance_to_use = last_result_;

    // fill resuduals
    const Eigen::MatrixXd& input_face_normals = toMesh_->getFaceNormals();
    if (parameter_type_ == DISPLACEMENT)
    {
        // special case == only one residual
        residuals[0] = residual_elem_(
            distance_to_use.signedDists(vertex_id_for_displacement_),
            distance_to_use.verts_normals.row(vertex_id_for_displacement_),
            input_face_normals.row(distance_to_use.closest_face_ids(vertex_id_for_displacement_)));
    }
    else
    {
        for (int i = 0; i < SMPLWrapper::VERTICES_NUM; ++i)
        {
            residuals[i] = residual_elem_(distance_to_use.signedDists(i),
                distance_to_use.verts_normals.row(i),
                input_face_normals.row(distance_to_use.closest_face_ids(i)));
        }
    }

    // fill out jacobians
    if (jacobians != NULL && jacobians[0] != NULL)
    {
        switch (parameter_type_)
        {
        case TRANSLATION:
            fillTranslationJac(distance_to_use, residuals, jacobians[0]);
            break;
        case SHAPE:
        case POSE:
            fillJac(distance_to_use, residuals, jacobians[0]);
            break;
        case DISPLACEMENT:
            fillDisplacementJac(distance_to_use, residuals, jacobians[0]);
            break;
        default:
            throw std::invalid_argument("DistanceBase Caclulation::WARNING:: no parameter type specified");
        }
    }

    return true;
}

void AbsoluteDistanceBase::updateDistanceCalculations(bool with_jacobian, DistanceResult& out_distance_result)
{
    bool calc_jac = 
        parameter_type_ == TRANSLATION || parameter_type_ == DISPLACEMENT && displacement_jac_evaluated
        ? false : with_jacobian;

    if (calc_jac)
    {
        out_distance_result.jacobian.resize(parameter_block_sizes()[0]);

        switch (parameter_type_)
        {
        case SHAPE:
            out_distance_result.verts = smpl_->calcModel(
                &smpl_->getStatePointers().translation, 
                &smpl_->getStatePointers().pose, 
                &smpl_->getStatePointers().shape, 
                &smpl_->getStatePointers().displacements,
                nullptr, &out_distance_result.jacobian[0], nullptr);
            break;
        case POSE:
            out_distance_result.verts = smpl_->calcModel(
                &smpl_->getStatePointers().translation, 
                &smpl_->getStatePointers().pose, 
                &smpl_->getStatePointers().shape,
                &smpl_->getStatePointers().displacements,
                &out_distance_result.jacobian[0], nullptr, nullptr);
            break;
        case DISPLACEMENT:
            out_distance_result.verts = smpl_->calcModel(
                &smpl_->getStatePointers().translation,
                &smpl_->getStatePointers().pose,
                &smpl_->getStatePointers().shape,
                &smpl_->getStatePointers().displacements,
                nullptr, nullptr, &out_distance_result.jacobian[0]);
            displacement_jac_evaluated = true;
            break;
        default:
            throw std::invalid_argument("DistanceBase Update::WARNING:: no parameter type for Jac calculation specified");
        }
    }
    else
    {
        out_distance_result.verts = smpl_->calcModel(
            &smpl_->getStatePointers().translation, 
            &smpl_->getStatePointers().pose, 
            &smpl_->getStatePointers().shape, 
            &smpl_->getStatePointers().displacements);
    }
    // get vertex normals
    out_distance_result.verts_normals = smpl_->calcVertexNormals(&out_distance_result.verts);
        
    calcSignedDistByVertecies(out_distance_result);
}

void AbsoluteDistanceBase::calcSignedDistByVertecies(DistanceResult & out_distance_result) const
{
    igl::SignedDistanceType type = igl::SIGNED_DISTANCE_TYPE_PSEUDONORMAL;
    igl::signed_distance(out_distance_result.verts,
        toMesh_->getNormalizedVertices(),
        toMesh_->getFaces(),
        type,
        out_distance_result.signedDists,
        out_distance_result.closest_face_ids,
        out_distance_result.closest_points,
        out_distance_result.normals_for_sign);

    assert(out_distance_result.signedDists.size() == SMPLWrapper::VERTICES_NUM
        && "Size of the set of distances should equal main parameters");
    assert(out_distance_result.closest_points.rows() == SMPLWrapper::VERTICES_NUM
        && "Size of the set of distances should equal main parameters");
}

void AbsoluteDistanceBase::fillJac(const DistanceResult& distance_res, const double* residuals, double * jacobian) const
{
    for (int v_id = 0; v_id < SMPLWrapper::VERTICES_NUM; ++v_id)
    {
        for (int param_id = 0; param_id < parameter_block_sizes()[0]; ++param_id)
        {
            jacobian[v_id * parameter_block_sizes()[0] + param_id]
                = jac_elem_(distance_res.verts.row(v_id), 
                    distance_res.closest_points.row(v_id), 
                    residuals[v_id],
                    distance_res.jacobian[param_id].row(v_id));
        }
    }
}

void AbsoluteDistanceBase::fillDisplacementJac(const DistanceResult & distance_res, const double * residuals, double * jacobian) const
{
    double distance = abs(distance_res.signedDists(vertex_id_for_displacement_));
    for (int axis_id = 0; axis_id < parameter_block_sizes()[0]; ++axis_id)
    {
        jacobian[axis_id]
            = jac_elem_(distance_res.verts.row(vertex_id_for_displacement_),
                distance_res.closest_points.row(vertex_id_for_displacement_),
                residuals[0],
                distance_res.jacobian[axis_id].row(vertex_id_for_displacement_));

    }
}

void AbsoluteDistanceBase::fillTranslationJac(const DistanceResult& distance_res, const double* residuals, double * jacobian) const
{
    for (int v_id = 0; v_id < SMPLWrapper::VERTICES_NUM; ++v_id)
    {
        for (int p_id = 0; p_id < parameter_block_sizes()[0]; ++p_id)
        {
            jacobian[v_id * parameter_block_sizes()[0] + p_id]
                = translation_jac_elem_(distance_res.verts(v_id, p_id),
                    distance_res.closest_points(v_id, p_id),
                    residuals[v_id]);
        }
    }
}
