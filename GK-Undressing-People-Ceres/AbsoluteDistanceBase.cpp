#include "AbsoluteDistanceBase.h"

AbsoluteDistanceBase::DistanceResult AbsoluteDistanceBase::last_result_;

AbsoluteDistanceBase::AbsoluteDistanceBase(SMPLWrapper* smpl, GeneralMesh * toMesh,
    ParameterType parameter, DistanceType dist_type,
    bool use_pre_computation,
    double pruning_threshold)
    : ceres::EvaluationCallback(),
    toMesh_(toMesh), smpl_(smpl),
    pruning_threshold_(pruning_threshold),
    parameter_type_(parameter), dist_evaluation_type_(dist_type),
    use_evaluation_callback_(use_pre_computation)
{
    this->set_num_residuals(SMPLWrapper::VERTICES_NUM);

    switch (parameter)
    {
        case TRANSLATION:
            this->mutable_parameter_block_sizes()->push_back(SMPLWrapper::SPACE_DIM);
            break;
        case SHAPE:
            this->mutable_parameter_block_sizes()->push_back(SMPLWrapper::SHAPE_SIZE);
            break;
        case POSE:
            this->mutable_parameter_block_sizes()->push_back(SMPLWrapper::POSE_SIZE);
            break;
        case DISPLACEMENT: 
            this->mutable_parameter_block_sizes()->push_back(SMPLWrapper::VERTICES_NUM * SMPLWrapper::SPACE_DIM);
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
    
    std::unique_ptr<DistanceResult> immediate_distance_result;
    DistanceResult* distance_to_use;

    if (use_evaluation_callback_)
    {
        distance_to_use = &last_result_;
        // TODO add the checks for the expected paramter size and the one used for calculating last_result
    }
    else  // allow to run the code without EvaluationCallback calculations 
    {
        // TODO check if this mode is needed
        immediate_distance_result = std::move(calcDistance(parameters[0], jacobians != NULL && jacobians[0] != NULL));
        distance_to_use = immediate_distance_result.get();
    }


    // fill resuduals
    const Eigen::MatrixXd& input_face_normals = toMesh_->getFaceNormals();
    for (int i = 0; i < SMPLWrapper::VERTICES_NUM; ++i)
    {
        residuals[i] = residual_elem_(distance_to_use->signedDists(i),
            distance_to_use->verts_normals.row(i),
            input_face_normals.row(distance_to_use->closest_face_ids(i)));
    }

    // fill out jacobians
    if (jacobians != NULL && jacobians[0] != NULL)
    {
        switch (parameter_type_)
        {
        case TRANSLATION:
            fillTranslationJac(*distance_to_use, residuals, jacobians[0]);
            break;
        case SHAPE:
        case POSE:
        case DISPLACEMENT:
            fillJac(*distance_to_use, residuals, jacobians[0]);
            break;
        default:
            throw std::exception("DistanceBase Caclulation::WARNING:: no parameter type specified");
        }
    }

    return true;
}

std::unique_ptr<AbsoluteDistanceBase::DistanceResult> AbsoluteDistanceBase::calcDistance(
    double const * parameter, bool with_jacobian) const
{
    std::unique_ptr<DistanceResult> distance_res = std::unique_ptr<DistanceResult>(new DistanceResult);

    // vertices for passed paramter
    if (with_jacobian)
        distance_res->jacobian.resize(parameter_block_sizes()[0]);

    switch (parameter_type_)
    {
    case TRANSLATION:
        distance_res->verts = smpl_->calcModel(parameter, smpl_->getStatePointers().pose, smpl_->getStatePointers().shape);
        break;

    case SHAPE:
        if (with_jacobian)
            distance_res->verts = smpl_->calcModel(smpl_->getStatePointers().translation,
                smpl_->getStatePointers().pose, parameter, nullptr, &distance_res->jacobian[0]);
        else
            distance_res->verts = smpl_->calcModel(smpl_->getStatePointers().translation,
                smpl_->getStatePointers().pose, parameter);
        break;

    case POSE:
        if (with_jacobian)
            distance_res->verts = smpl_->calcModel(smpl_->getStatePointers().translation,
                parameter, smpl_->getStatePointers().shape, &distance_res->jacobian[0], nullptr);
        else
            distance_res->verts = smpl_->calcModel(smpl_->getStatePointers().translation,
                parameter, smpl_->getStatePointers().shape);
        break;

    case DISPLACEMENT:
        // TODO add distance calculation
        break;

    default:
        throw std::exception("DistanceBase Caclulation::WARNING:: no parameter type specified");
    }

    // get normals
    distance_res->verts_normals = smpl_->calcVertexNormals(&distance_res->verts);

    // distnaces
    calcSignedDistByVertecies(*distance_res);

    return distance_res;
}

void AbsoluteDistanceBase::updateDistanceCalculations(bool with_jacobian, DistanceResult& out_distance_result)
{
    if (with_jacobian)
    {
        out_distance_result.jacobian.resize(parameter_block_sizes()[0]);

        switch (parameter_type_)
        {
        case TRANSLATION:
            out_distance_result.verts = smpl_->calcModel(
                smpl_->getStatePointers().translation, smpl_->getStatePointers().pose, smpl_->getStatePointers().shape);
        case SHAPE:
            out_distance_result.verts = smpl_->calcModel(
                smpl_->getStatePointers().translation, smpl_->getStatePointers().pose, smpl_->getStatePointers().shape, 
                nullptr, &out_distance_result.jacobian[0]);
            break;
        case POSE:
            out_distance_result.verts = smpl_->calcModel(
                smpl_->getStatePointers().translation, smpl_->getStatePointers().pose, smpl_->getStatePointers().shape,
                &out_distance_result.jacobian[0], nullptr);
            break;
        case DISPLACEMENT:
            // TODO add updating distance calculation
            break;
        default:
            throw std::exception("DistanceBase Update::WARNING:: no parameter type specified");
        }
    }
    else
    {
        out_distance_result.verts = smpl_->calcModel(
            smpl_->getStatePointers().translation, smpl_->getStatePointers().pose, smpl_->getStatePointers().shape);
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

    assert(distance_res->signedDists.size() == SMPLWrapper::VERTICES_NUM
        && "Size of the set of distances should equal main parameters");
    assert(distance_res->closest_points.rows() == SMPLWrapper::VERTICES_NUM
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
