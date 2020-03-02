#include "ShapeUnderClothOptimizer.h"


ShapeUnderClothOptimizer::ShapeUnderClothOptimizer(std::shared_ptr<SMPLWrapper> smpl, 
    std::shared_ptr<GeneralMesh> input)
{
    // note: could be nullptr
    smpl_ = std::move(smpl);
    input_ = std::move(input);

    // parameters
    // NOTE: the parameters might be reset from the outside later on
    config_.shape_reg_weight = 0.01;
    config_.pose_reg_weight = 0.001;
    config_.displacement_reg_weight = 0.001;
    config_.displacement_smoothing_weight = 0.1;
    config_.shape_prune_threshold = 0.05;
    config_.gm_saturation_threshold = 0.033;
    config_.in_verts_scaling_weight = 0.1;
}

ShapeUnderClothOptimizer::~ShapeUnderClothOptimizer()
{}

void ShapeUnderClothOptimizer::setNewSMPLModel(std::shared_ptr<SMPLWrapper> smpl)
{
    smpl_ = std::move(smpl);
}

void ShapeUnderClothOptimizer::setNewInput(std::shared_ptr<GeneralMesh> input)
{
    input_ = std::move(input);
}

void ShapeUnderClothOptimizer::findOptimalSMPLParameters(std::vector<Eigen::MatrixXd>* iteration_results)
{
    // Use the current state of smpl_ as init parameter
    // smpl is moved to zero, because we use normalized input - equivalent to translation guess
    smpl_->translateTo(E::Vector3d(0., 0., 0.));
    // Put our trust into the initial pose
    ceres::Matrix initial_pose_as_prior = smpl_->getStatePointers().pose;

    // Setup solver options
    config_.ceres.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY; // analytic jacobian is dense
    config_.ceres.minimizer_progress_to_stdout = true;
    config_.ceres.max_num_iterations = 500;   // usually converges way faster

    SMPLVertsLoggingCallBack* callback = nullptr;
    if (iteration_results != nullptr)
    {
        callback = new SMPLVertsLoggingCallBack(smpl_, iteration_results);
        config_.ceres.callbacks.push_back(callback);
        config_.ceres.update_state_every_iteration = true;
    }

    checkCeresOptions(config_.ceres);

    auto start_time = std::chrono::system_clock::now();
    // just some number of cycles
    for (int i = 0; i < 3; ++i)
    {
        std::cout << "***********************" << std::endl
            << "    Cycle Shape: #" << i << std::endl
            << "***********************" << std::endl;
        
        translationEstimation_(config_);
        shapeEstimation_(config_);
        poseEstimation_(config_, initial_pose_as_prior);
    }

    for (int i = 0; i < 0; ++i)
    {
        std::cout << "***********************" << std::endl
            << "    Cycle Displacement: #" << i << std::endl
            << "***********************" << std::endl;
 
        displacementEstimation_(config_);
        translationEstimation_(config_);
        poseEstimation_(config_, initial_pose_as_prior);
    }

    auto end_time = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end_time - start_time;
    std::time_t end_time_t = std::chrono::system_clock::to_time_t(end_time);

    std::cout << "***********************" << std::endl
        << "Finished at " << std::ctime(&end_time_t) << std::endl
        << "Total time " << elapsed_seconds.count() << "s" << std::endl
        << "***********************" << std::endl;

    // cleanup
    if (callback != nullptr)
    {
        delete callback;
    }
}

void ShapeUnderClothOptimizer::gmLossTest()
{
    // sigma = 10
    GemanMcClareLoss func(0.1);

    std::array<double, 3> outs;

    for (auto x : { 0., 0.05, 0.1, 0.2, 0.4, 0.5 })
    {
        func.Evaluate(x*x, &outs[0]);
        std::cout << "GM(" << x << ") = " << outs[0] << std::endl;
    }
}

void ShapeUnderClothOptimizer::translationEstimation_(OptimizationOptions& config)
{
    std::cout << "-----------------------" << std::endl
              << "      Translation" << std::endl
              << "-----------------------" << std::endl;

    Problem problem;

    // send raw pointers because inner class were not refactored
    AbsoluteDistanceBase* cost_function = new AbsoluteDistanceBase(smpl_.get(), input_.get(),
        AbsoluteDistanceBase::TRANSLATION, AbsoluteDistanceBase::BOTH_DIST);
    // for pre-computation
    config.ceres.evaluation_callback = cost_function;
    
    problem.AddResidualBlock(cost_function, nullptr, smpl_->getStatePointers().translation.data());

    // Run the solver!
    Solver::Summary summary;
    Solve(config.ceres, &problem, &summary);

    // Print summary
    std::cout << "Translation estimation summary:" << std::endl;
    std::cout << summary.FullReport() << std::endl;

    // clear the options from the update for smooth future use
    config.ceres.evaluation_callback = NULL;
}

void ShapeUnderClothOptimizer::poseEstimation_(OptimizationOptions& config, ceres::Matrix & prior_pose)
{
    std::cout << "-----------------------" << std::endl
              << "          Pose" << std::endl
              << "-----------------------" << std::endl;

    Problem problem;

    // Main cost
    if (input_->isClothSegmented())
        poseMainCostClothAware_(problem, config);
    else
        poseMainCostNoSegmetation_(problem, config);

    // Regularizer
    // Note that we exploit the row-major here!
    ceres::Vector prior_pose_as_vector = Eigen::Map<Eigen::VectorXd>(prior_pose.data(), prior_pose.size());
    CostFunction* prior = new NormalPrior(smpl_->getPoseStiffness(), prior_pose_as_vector);
    LossFunction* scale_prior = new ScaledLoss(NULL, config.pose_reg_weight, ceres::TAKE_OWNERSHIP);    // 0.0007
    problem.AddResidualBlock(prior, scale_prior, 
        smpl_->getStatePointers().pose.data());

    // Run the solver!
    Solver::Summary summary;
    Solve(config.ceres, &problem, &summary);

    // Print summary
    std::cout << "Pose estimation summary:" << std::endl;
    std::cout << summary.FullReport() << std::endl;

    // clear the options from the update for smooth future use
    config.ceres.evaluation_callback = NULL;
}

void ShapeUnderClothOptimizer::poseMainCostNoSegmetation_(Problem & problem, OptimizationOptions& config)
{
    // send raw pointers because inner class were not refactored
    AbsoluteDistanceBase* out_cost_function = new AbsoluteDistanceBase(smpl_.get(), input_.get(),
        AbsoluteDistanceBase::POSE, AbsoluteDistanceBase::OUT_DIST);
    AbsoluteDistanceBase* in_cost_function = new AbsoluteDistanceBase(smpl_.get(), input_.get(),
        AbsoluteDistanceBase::POSE, AbsoluteDistanceBase::IN_DIST);

    problem.AddResidualBlock(out_cost_function, nullptr,
        smpl_->getStatePointers().pose.data());

    // in_verts distance needs scaling 
    problem.AddResidualBlock(in_cost_function, innerVerticesLoss_(config),
        smpl_->getStatePointers().pose.data());

    // set any of the defined costs for pre-computation
    config.ceres.evaluation_callback = out_cost_function;
}

void ShapeUnderClothOptimizer::poseMainCostClothAware_(Problem & problem, OptimizationOptions& config)
{
    // send raw pointers because inner class was not refactored
    AbsoluteDistanceBase* cloth_out_cost = new AbsoluteDistanceBase(smpl_.get(), input_.get(),
        AbsoluteDistanceBase::POSE, AbsoluteDistanceBase::CLOTH_OUT);
    AbsoluteDistanceBase* cloth_in_cost = new AbsoluteDistanceBase(smpl_.get(), input_.get(),
        AbsoluteDistanceBase::POSE, AbsoluteDistanceBase::CLOTH_IN);
    AbsoluteDistanceBase* skin_cost = new AbsoluteDistanceBase(smpl_.get(), input_.get(),
        AbsoluteDistanceBase::POSE, AbsoluteDistanceBase::SKIN_BOTH);

    problem.AddResidualBlock(skin_cost, nullptr,
        smpl_->getStatePointers().pose.data());

    problem.AddResidualBlock(cloth_out_cost, nullptr,
        smpl_->getStatePointers().pose.data());

    problem.AddResidualBlock(cloth_in_cost, innerVerticesLoss_(config),
        smpl_->getStatePointers().pose.data());

    // set any of the defined costs for pre-computation
    config.ceres.evaluation_callback = cloth_out_cost;
}

void ShapeUnderClothOptimizer::shapeEstimation_(OptimizationOptions& config)
{
    std::cout << "-----------------------" << std::endl
        << "          Shape" << std::endl
        << "-----------------------" << std::endl;

    Problem problem;

    // Main cost
    if (input_->isClothSegmented())
        shapeMainCostClothAware_(problem, config);
    else
        shapeMainCostNoSegmetation_(problem, config);

    // Regularization
    CostFunction* prior = new NormalPrior(
        Eigen::MatrixXd::Identity(SMPLWrapper::SHAPE_SIZE, SMPLWrapper::SHAPE_SIZE), 
        Eigen::VectorXd::Zero(SMPLWrapper::SHAPE_SIZE));
    LossFunction* scale_prior = new ScaledLoss(NULL, config.shape_reg_weight, ceres::TAKE_OWNERSHIP);
    problem.AddResidualBlock(prior, scale_prior,
        smpl_->getStatePointers().shape.data());

    // Run the solver!
    Solver::Summary summary;
    Solve(config.ceres, &problem, &summary);

    // Print summary
    std::cout << "Shape estimation summary:" << std::endl;
    std::cout << summary.FullReport() << std::endl;

    // clear the options from the update for smooth future use
    config.ceres.evaluation_callback = NULL;
}

void ShapeUnderClothOptimizer::shapeMainCostNoSegmetation_(Problem & problem, OptimizationOptions& config)
{
    AbsoluteDistanceBase* out_cost_function = new AbsoluteDistanceBase(smpl_.get(), input_.get(),
        AbsoluteDistanceBase::SHAPE, AbsoluteDistanceBase::OUT_DIST, config.shape_prune_threshold);
    AbsoluteDistanceBase* in_cost_function = new AbsoluteDistanceBase(smpl_.get(), input_.get(),
        AbsoluteDistanceBase::SHAPE, AbsoluteDistanceBase::IN_DIST);  // no threshold

    // add Residuals 
    problem.AddResidualBlock(out_cost_function, nullptr,
        smpl_->getStatePointers().shape.data());

    problem.AddResidualBlock(in_cost_function, innerVerticesLoss_(config),
        smpl_->getStatePointers().shape.data());

    // set any of the defined costs for pre-computation
    config.ceres.evaluation_callback = out_cost_function;
}

void ShapeUnderClothOptimizer::shapeMainCostClothAware_(Problem & problem, OptimizationOptions& config)
{
    // send raw pointers because inner class was not refactored
    AbsoluteDistanceBase* cloth_out_cost = new AbsoluteDistanceBase(smpl_.get(), input_.get(),
        AbsoluteDistanceBase::SHAPE, AbsoluteDistanceBase::CLOTH_OUT);
    AbsoluteDistanceBase* cloth_in_cost = new AbsoluteDistanceBase(smpl_.get(), input_.get(),
        AbsoluteDistanceBase::SHAPE, AbsoluteDistanceBase::CLOTH_IN);
    AbsoluteDistanceBase* skin_cost = new AbsoluteDistanceBase(smpl_.get(), input_.get(),
        AbsoluteDistanceBase::SHAPE, AbsoluteDistanceBase::SKIN_BOTH);

    problem.AddResidualBlock(skin_cost, nullptr,
        smpl_->getStatePointers().shape.data());

    problem.AddResidualBlock(cloth_out_cost, nullptr,
        smpl_->getStatePointers().shape.data());

    problem.AddResidualBlock(cloth_in_cost, innerVerticesLoss_(config),
        smpl_->getStatePointers().shape.data());

    // set any of the defined costs for pre-computation
    config.ceres.evaluation_callback = cloth_out_cost;
}

void ShapeUnderClothOptimizer::displacementEstimation_(OptimizationOptions& config)
{
    std::cout << "-----------------------" << std::endl
        << "          Displacement" << std::endl
        << "-----------------------" << std::endl;

    Problem problem;

    // prepapre losses for regularization
    CostFunction* prior = new NormalPrior(
        Eigen::MatrixXd::Identity(SMPLWrapper::SPACE_DIM, SMPLWrapper::SPACE_DIM),
        Eigen::VectorXd::Zero(SMPLWrapper::SPACE_DIM));
    LossFunction* L2_scale_loss = new ScaledLoss(NULL, config.displacement_reg_weight, ceres::TAKE_OWNERSHIP);
    LossFunction* smoothing_scale_loss = new ScaledLoss(NULL, config.displacement_smoothing_weight, ceres::TAKE_OWNERSHIP);

    // Main cost -- for each vertex
    bool eval_callback_added = false;
    for (int v_id = 0; v_id < SMPLWrapper::VERTICES_NUM; v_id++) //int v_id = 1084;
    {
        AbsoluteDistanceBase* out_cost_function = new AbsoluteDistanceBase(smpl_.get(), input_.get(),
            AbsoluteDistanceBase::DISPLACEMENT, AbsoluteDistanceBase::OUT_DIST,
            config.shape_prune_threshold, v_id);    // TODO recheck thresholding for displacements shape_prune_threshold_
        AbsoluteDistanceBase* in_cost_function = new AbsoluteDistanceBase(smpl_.get(), input_.get(),
            AbsoluteDistanceBase::DISPLACEMENT, AbsoluteDistanceBase::IN_DIST, 
            100., v_id);  // no threshold
        // nothe that it requres the dispalacements params to be pushed to smpl object 
        // -> evaluation callback of the out/in_cost_function will work
        SmoothDisplacementCost* smoothing_cost_function = new SmoothDisplacementCost(smpl_, v_id);

        // add for performing pre-computation
        if (!eval_callback_added)
        {
            config.ceres.evaluation_callback = out_cost_function;
            eval_callback_added = true;
        }

        // add out Residuals for corresponding vertex
        problem.AddResidualBlock(out_cost_function, nullptr,
            smpl_->getStatePointers().displacements.data() + v_id * SMPLWrapper::SPACE_DIM);

        // add in residuals for corresponding vertex
        problem.AddResidualBlock(in_cost_function, innerVerticesLoss_(config),
            smpl_->getStatePointers().displacements.data() + v_id * SMPLWrapper::SPACE_DIM);

        // add regularization resuiduals
        problem.AddResidualBlock(prior, L2_scale_loss,
            smpl_->getStatePointers().displacements.data() + v_id * SMPLWrapper::SPACE_DIM);
        problem.AddResidualBlock(smoothing_cost_function, smoothing_scale_loss,
            smpl_->getStatePointers().displacements.data() + v_id * SMPLWrapper::SPACE_DIM);
    }

    // Run the solver!
    Solver::Summary summary;
    Solve(config.ceres, &problem, &summary);

    // Print summary
    std::cout << "Displacement estimation summary:" << std::endl;
    std::cout << summary.FullReport() << std::endl;

    // clear the options from the update for smooth future use
    config.ceres.evaluation_callback = NULL;
}

ceres::ComposedLoss* ShapeUnderClothOptimizer::innerVerticesLoss_(const OptimizationOptions& config)
{
    LossFunction* scale_in_cost = new ScaledLoss(NULL, config.in_verts_scaling_weight, ceres::TAKE_OWNERSHIP);
    LossFunction* geman_mcclare_cost = new GemanMcClareLoss(config.gm_saturation_threshold);

    return new ceres::ComposedLoss(
        scale_in_cost, ceres::TAKE_OWNERSHIP,
        geman_mcclare_cost, ceres::TAKE_OWNERSHIP);
}

void ShapeUnderClothOptimizer::checkCeresOptions(const Solver::Options & options)
{
    std::string error_text;
    if (!options.IsValid(&error_text))
        throw std::runtime_error(("Ceres Options Error: " + error_text).c_str());
}

ceres::CallbackReturnType ShapeUnderClothOptimizer::SMPLVertsLoggingCallBack::operator()(const ceres::IterationSummary & summary)
{
    Eigen::MatrixXd verts = smpl_->calcModel();
    smpl_verts_results_->push_back(verts);

    return ceres::SOLVER_CONTINUE;
}
