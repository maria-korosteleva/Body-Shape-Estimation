#include "ShapeUnderClothOptimizer.h"


ShapeUnderClothOptimizer::ShapeUnderClothOptimizer(std::shared_ptr<SMPLWrapper> smpl, 
    std::shared_ptr<GeneralMesh> input, const std::string path_to_prior)
{
    smpl_ = std::move(smpl);
    input_ = std::move(input);

    assert(input->getVertices().cols() == SMPLWrapper::SPACE_DIM && "World dimentions should be equal for SMPL and input mesh");

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
    options.use_nonmonotonic_steps = true;

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

void ShapeUnderClothOptimizer::translationEstimation_(Solver::Options & options)
{
    std::cout << "-----------------------" << std::endl
              << "      Translation" << std::endl
              << "-----------------------" << std::endl;

    Problem problem;

    // send raw pointers because inner class were not refactored
    CostFunction* cost_function = new AbsoluteDistanceForTranslation(smpl_.get(), input_.get());
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
    CostFunction* cost_function = new AbsoluteDistanceForPose(smpl_.get(), input_.get());
    problem.AddResidualBlock(cost_function, nullptr, 
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

#ifdef DEBUG
    // Print last Gradient (?) and Jacobian
    std::vector<double> gradient;
    ceres::CRSMatrix jac;
    problem.Evaluate(Problem::EvaluateOptions(), NULL, NULL, &gradient, &jac);
    std::cout << "Gradient" << std::endl;
    for (auto i = gradient.begin(); i != gradient.end(); ++i)
    {
        std::cout << *i << " ";
    }
    std::cout << std::endl;

    std::cout << "Jacobian (sparse)" << std::endl;

    for (int i = 0; i < jac.num_rows; ++i)
    {
        for (int j = jac.rows[i]; j < jac.rows[i + 1]; ++j)   // only pose jacobian
        {
            std::cout << jac.values[j] << " ";
        }
        std::cout << std::endl;
    }

#endif // DEBUG
}

void ShapeUnderClothOptimizer::shapeEstimation_(Solver::Options & options, const double parameter)
{
    std::cout << "-----------------------" << std::endl
        << "          Shape" << std::endl
        << "-----------------------" << std::endl;

    Problem problem;

    CostFunction* cost_function = new AbsoluteDistanceForShape(smpl_.get(), input_.get(),
        shape_prune_threshold_, parameter);
    problem.AddResidualBlock(cost_function, nullptr,
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

#ifdef DEBUG
    // Print last Gradient (?) and Jacobian
    std::vector<double> gradient;
    ceres::CRSMatrix jac;
    problem.Evaluate(Problem::EvaluateOptions(), NULL, NULL, &gradient, &jac);
    std::cout << "Gradient" << std::endl;
    for (auto i = gradient.begin(); i != gradient.end(); ++i)
    {
        std::cout << *i << " ";
    }
    std::cout << std::endl;

    std::cout << "Jacobian (sparse)" << std::endl;

    for (int i = 0; i < jac.num_rows; ++i)
    {
        for (int j = jac.rows[i]; j < jac.rows[i + 1]; ++j)   // only pose jacobian
        {
            std::cout << jac.values[j] << " ";
        }
        std::cout << std::endl;
    }

#endif // DEBUG
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

#ifdef DEBUG
    std::cout << "Read mean pose" << std::endl;
    //for (int i = 0; i < SMPLWrapper::POSE_SIZE; i++)
    //{
    //    std::cout << this->mean_pose_(i) << " ";
    //}
    //std::cout << std::endl;
#endif // DEBUG
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
