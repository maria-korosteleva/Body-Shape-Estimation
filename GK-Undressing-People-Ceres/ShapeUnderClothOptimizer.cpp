#include "ShapeUnderClothOptimizer.h"


ShapeUnderClothOptimizer::ShapeUnderClothOptimizer(std::shared_ptr<SMPLWrapper> smpl, 
    std::shared_ptr<GeneralMesh> input, const std::string path_to_prior)
{
    // note: could be nullptr
    smpl_ = std::move(smpl);
    input_ = std::move(input);

    // read prior info
    std::string path(path_to_prior);
    path += '/';
    readAveragePose_deprecated_(path);
    readStiffness_(path);

    // parameters
    // NOTE: the parameters might be reset from the outside
    shape_reg_weight_ = 0.01;
    pose_reg_weight_ = 0.001;
    shape_prune_threshold_ = 0.05;
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

void ShapeUnderClothOptimizer::setNewPriorPath(const char * prior_path)
{
    std::string path(prior_path);
    readAveragePose_deprecated_(path);
    readStiffness_(path);
}

void ShapeUnderClothOptimizer::findOptimalSMPLParameters(std::vector<Eigen::MatrixXd>* iteration_results, const double parameter)
{
    // Use the current state of smpl_ as init parameter
    // smpl is moved to zero, because we use normalized input - equivalent to translation guess
    smpl_->translateTo(E::Vector3d(0., 0., 0.));
    // Put our trust into the initial pose
    ceres::Vector initial_pose_as_prior 
        = copyArray_(smpl_->getStatePointers().pose, SMPLWrapper::POSE_SIZE);

    // Setup solvers options
    Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY; // analytic jacobian is dense
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 500;   // usually converges way faster

    SMPLVertsLoggingCallBack* callback = nullptr;
    if (iteration_results != nullptr)
    {
        callback = new SMPLVertsLoggingCallBack(smpl_, iteration_results);
        options.callbacks.push_back(callback);
        options.update_state_every_iteration = true;
    }

    checkCeresOptions(options);

    auto start_time = std::chrono::system_clock::now();
    // just some number of cycles
    for (int i = 0; i < 3; ++i)
    {
        std::cout << "***********************" << std::endl
            << "    Cycle #" << i << std::endl
            << "***********************" << std::endl;
        
        translationEstimation_(options);
        shapeEstimation_(options, parameter);
        poseEstimation_(options, initial_pose_as_prior);
    }

    // Refine diplacement once
    displacementEstimation_(options);


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

void ShapeUnderClothOptimizer::translationEstimation_(Solver::Options & options)
{
    std::cout << "-----------------------" << std::endl
              << "      Translation" << std::endl
              << "-----------------------" << std::endl;

    Problem problem;

    // send raw pointers because inner class were not refactored
    CostFunction* cost_function = new AbsoluteDistanceBase(smpl_.get(), input_.get(), 
        AbsoluteDistanceBase::TRANSLATION, AbsoluteDistanceBase::BOTH_DIST);
    problem.AddResidualBlock(cost_function, nullptr, smpl_->getStatePointers().translation);

    // Run the solver!
    Solver::Summary summary;
    Solve(options, &problem, &summary);

    // Print summary
    std::cout << "Translation estimation summary:" << std::endl;
    std::cout << summary.FullReport() << std::endl;
}

void ShapeUnderClothOptimizer::poseEstimation_(Solver::Options& options, ceres::Vector& prior_pose, const double parameter)
{
    std::cout << "-----------------------" << std::endl
              << "          Pose" << std::endl
              << "-----------------------" << std::endl;

    Problem problem;

    // Main cost
    // send raw pointers because inner class were not refactored
    AbsoluteDistanceBase* out_cost_function = new AbsoluteDistanceBase(smpl_.get(), input_.get(),
        AbsoluteDistanceBase::POSE, AbsoluteDistanceBase::OUT_DIST, true);
    AbsoluteDistanceBase* in_cost_function = new AbsoluteDistanceBase(smpl_.get(), input_.get(),
        AbsoluteDistanceBase::POSE, AbsoluteDistanceBase::IN_DIST, true);

    // for pre-computation
    options.evaluation_callback = out_cost_function;

    problem.AddResidualBlock(out_cost_function, nullptr,
        smpl_->getStatePointers().pose);

    LossFunction* scale_in_cost = new ScaledLoss(NULL, 0.1, ceres::TAKE_OWNERSHIP);
    LossFunction* geman_mcclare_cost = new GemanMcClareLoss(0.033);
    ceres::ComposedLoss* composed_loss = new ceres::ComposedLoss(
        scale_in_cost, ceres::TAKE_OWNERSHIP,
        geman_mcclare_cost, ceres::TAKE_OWNERSHIP);
    problem.AddResidualBlock(in_cost_function, composed_loss,
        smpl_->getStatePointers().pose);

    // Regularizer
    CostFunction* prior = new NormalPrior(stiffness_, prior_pose);
    LossFunction* scale_prior = new ScaledLoss(NULL, pose_reg_weight_, ceres::TAKE_OWNERSHIP);    // 0.0007
    problem.AddResidualBlock(prior, scale_prior, 
        smpl_->getStatePointers().pose);

    // Run the solver!
    Solver::Summary summary;
    Solve(options, &problem, &summary);

    // Print summary
    std::cout << "Pose estimation summary:" << std::endl;
    std::cout << summary.FullReport() << std::endl;

    // clear the options from the update for smooth future use
    options.evaluation_callback = NULL;
}

void ShapeUnderClothOptimizer::shapeEstimation_(Solver::Options & options, const double gm_parameter)
{
    std::cout << "-----------------------" << std::endl
        << "          Shape" << std::endl
        << "-----------------------" << std::endl;

    Problem problem;

    // Main cost
    AbsoluteDistanceBase* out_cost_function = new AbsoluteDistanceBase(smpl_.get(), input_.get(),
        AbsoluteDistanceBase::SHAPE, AbsoluteDistanceBase::OUT_DIST, true,
        shape_prune_threshold_);
    AbsoluteDistanceBase* in_cost_function = new AbsoluteDistanceBase(smpl_.get(), input_.get(),
        AbsoluteDistanceBase::SHAPE, AbsoluteDistanceBase::IN_DIST, true);  // no threshold

    // add for performing pre-computation
    options.evaluation_callback = out_cost_function;

    // add Residuals 
    problem.AddResidualBlock(out_cost_function, nullptr,
        smpl_->getStatePointers().shape);

    LossFunction* scale_in_cost = new ScaledLoss(NULL, 0.1, ceres::TAKE_OWNERSHIP);
    LossFunction* geman_mcclare_cost = new GemanMcClareLoss(0.033);
    ceres::ComposedLoss* composed_loss = new ceres::ComposedLoss(
        scale_in_cost, ceres::TAKE_OWNERSHIP,
        geman_mcclare_cost, ceres::TAKE_OWNERSHIP);
    problem.AddResidualBlock(in_cost_function, composed_loss,
        smpl_->getStatePointers().shape);

    // Regularization
    CostFunction* prior = new NormalPrior(
        Eigen::MatrixXd::Identity(SMPLWrapper::SHAPE_SIZE, SMPLWrapper::SHAPE_SIZE), 
        Eigen::VectorXd::Zero(SMPLWrapper::SHAPE_SIZE));
    LossFunction* scale_prior = new ScaledLoss(NULL, shape_reg_weight_, ceres::TAKE_OWNERSHIP);
    problem.AddResidualBlock(prior, scale_prior,
        smpl_->getStatePointers().shape);

    // Run the solver!
    Solver::Summary summary;
    Solve(options, &problem, &summary);

    // Print summary
    std::cout << "Shape estimation summary:" << std::endl;
    std::cout << summary.FullReport() << std::endl;

    // clear the options from the update for smooth future use
    options.evaluation_callback = NULL;
}

void ShapeUnderClothOptimizer::displacementEstimation_(Solver::Options& options)
{
    std::cout << "-----------------------" << std::endl
        << "          Displacement" << std::endl
        << "-----------------------" << std::endl;

    Problem problem;

    // prepare losses for inner distances
    LossFunction* scale_in_cost = new ScaledLoss(NULL, 0.1, ceres::TAKE_OWNERSHIP);
    LossFunction* geman_mcclare_cost = new GemanMcClareLoss(0.033);
    ceres::ComposedLoss* composed_loss = new ceres::ComposedLoss(
        scale_in_cost, ceres::TAKE_OWNERSHIP,
        geman_mcclare_cost, ceres::TAKE_OWNERSHIP);

    // prepapre losses for regularization
    CostFunction* prior = new NormalPrior(
        Eigen::MatrixXd::Identity(SMPLWrapper::SPACE_DIM, SMPLWrapper::SPACE_DIM),
        Eigen::VectorXd::Zero(SMPLWrapper::SPACE_DIM));
    LossFunction* scale_prior = new ScaledLoss(NULL, 1., ceres::TAKE_OWNERSHIP);

    // Main cost -- for each vertex
    bool eval_callback_added = false;
    for (int v_id = 0; v_id < SMPLWrapper::VERTICES_NUM; v_id++)
    {
        AbsoluteDistanceBase* out_cost_function = new AbsoluteDistanceBase(smpl_.get(), input_.get(),
            AbsoluteDistanceBase::DISPLACEMENT, AbsoluteDistanceBase::OUT_DIST, true,
            shape_prune_threshold_, v_id);    // TODO recheck thresholding for displacements
        AbsoluteDistanceBase* in_cost_function = new AbsoluteDistanceBase(smpl_.get(), input_.get(),
            AbsoluteDistanceBase::DISPLACEMENT, AbsoluteDistanceBase::IN_DIST, 
            true, 100., v_id);  // no threshold

        // add for performing pre-computation
        if (!eval_callback_added)
        {
            options.evaluation_callback = out_cost_function;
            eval_callback_added = true;
        }

        // add out Residuals for corresponding vertex
        problem.AddResidualBlock(out_cost_function, nullptr,
            smpl_->getStatePointers().displacements.data() + v_id * SMPLWrapper::SPACE_DIM);

        // add in residuals for corresponding vertex
        problem.AddResidualBlock(in_cost_function, composed_loss,
            smpl_->getStatePointers().displacements.data() + v_id * SMPLWrapper::SPACE_DIM);

        // add regularization resuiduals
        problem.AddResidualBlock(prior, scale_prior,
            smpl_->getStatePointers().displacements.data() + v_id * SMPLWrapper::SPACE_DIM);
    }

    // Run the solver!
    Solver::Summary summary;
    Solve(options, &problem, &summary);

    // Print summary
    std::cout << "Displacement estimation summary:" << std::endl;
    std::cout << summary.FullReport() << std::endl;

    // clear the options from the update for smooth future use
    options.evaluation_callback = NULL;
}

void ShapeUnderClothOptimizer::readAveragePose_deprecated_(const std::string path)
{
    //std::string filename = path + "mean_pose.txt";
    std::string filename = path + "A_pose.txt";

    std::fstream inFile;
    inFile.open(filename, std::ios_base::in);
    int size;
    inFile >> size;
    // Sanity check
    if (size != SMPLWrapper::POSE_SIZE - SMPLWrapper::SPACE_DIM)
        throw std::exception("Striffness matrix size doesn't match the number of non-root pose parameters");
    
    average_pose_deprecated_.resize(SMPLWrapper::POSE_SIZE);
    // For convinient use of the mean pose with full pose vectors, root rotation is set to zero
    for (int i = 0; i < SMPLWrapper::SPACE_DIM; i++)
        average_pose_deprecated_(i) = 0.;
    // Now read from file
    for (int i = SMPLWrapper::SPACE_DIM; i < SMPLWrapper::POSE_SIZE; i++)
        inFile >> average_pose_deprecated_(i);

    inFile.close();
}

void ShapeUnderClothOptimizer::readStiffness_(const std::string path)
{
    std::string stiffness_filename = path + "stiffness.txt";

    std::fstream inFile;
    inFile.open(stiffness_filename, std::ios_base::in);
    constexpr int NON_ROOT_POSE_SIZE = SMPLWrapper::POSE_SIZE - SMPLWrapper::SPACE_DIM;
    int rows, cols;
    inFile >> rows;
    inFile >> cols;
    // Sanity check
    if (rows != cols)
        throw std::exception("Striffness matrix is not a square matrix");
    if (rows != SMPLWrapper::POSE_SIZE - SMPLWrapper::SPACE_DIM)
        throw std::exception("Striffness matrix size doesn't match the number of non-root pose parameters");

    // To make matrix applicable to full pose vector
    this->stiffness_.resize(SMPLWrapper::POSE_SIZE, SMPLWrapper::POSE_SIZE);
    for (int i = 0; i < SMPLWrapper::SPACE_DIM; i++)
        for (int j = 0; j < SMPLWrapper::POSE_SIZE; j++)
            this->stiffness_(i, j) = this->stiffness_(j, i) = 0.;

    // Now read from file
    for (int i = SMPLWrapper::SPACE_DIM; i < SMPLWrapper::POSE_SIZE; i++)
        for (int j = SMPLWrapper::SPACE_DIM; j < SMPLWrapper::POSE_SIZE; j++)
            inFile >> this->stiffness_(i, j);

    inFile.close();

#ifdef DEBUG
    std::cout << "Read stiffness matrix" << std::endl;
    //for (int i = 0; i < SMPLWrapper::POSE_SIZE; i++)
    //{
    //    for (int j = 0; j < SMPLWrapper::POSE_SIZE; j++)
    //    {
    //        std::cout << this->stiffness_(i, j) << " ";
    //    }
    //    std::cout << std::endl;
    //}
    
#endif // DEBUG

}

void ShapeUnderClothOptimizer::zeros_(double * arr, std::size_t size)
{
    for (int i = 0; i < size; i++)
    {
        arr[i] = 0.;
    }
}

void ShapeUnderClothOptimizer::printArray_(double * arr, std::size_t size)
{
    for (int i = 0; i < size; i++)
    {
        std::cout << arr[i] << "; ";
    }
    std::cout << std::endl;
}

ceres::Vector ShapeUnderClothOptimizer::copyArray_(double * arr, std::size_t size)
{
    ceres::Vector copy(size);

    for (int i = 0; i < size; ++i)
        copy[i] = arr[i];

    return copy;
}

void ShapeUnderClothOptimizer::checkCeresOptions(const Solver::Options & options)
{
    std::string error_text;
    if (!options.IsValid(&error_text))
        throw std::exception(("Ceres Options Error: " + error_text).c_str());
}

ceres::CallbackReturnType ShapeUnderClothOptimizer::SMPLVertsLoggingCallBack::operator()(const ceres::IterationSummary & summary)
{
    Eigen::MatrixXd verts = smpl_->calcModel();
    smpl_verts_results_->push_back(verts);

    return ceres::SOLVER_CONTINUE;
}
