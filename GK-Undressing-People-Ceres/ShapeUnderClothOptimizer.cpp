#include "ShapeUnderClothOptimizer.h"


ShapeUnderClothOptimizer::ShapeUnderClothOptimizer(SMPLWrapper* smpl, GeneralMesh* input, const std::string path_to_prior)
{
    smpl_ = smpl;
    input_ = input;

    assert(input->getVertices().cols() == SMPLWrapper::SPACE_DIM && "World dimentions should be equal for SMPL and input mesh");

    // read prior info
    std::string path(path_to_prior);
    path += '/';
    readAttractivePose_(path);
    readStiffness_(path);
}

ShapeUnderClothOptimizer::~ShapeUnderClothOptimizer()
{}

void ShapeUnderClothOptimizer::setNewSMPLModel(SMPLWrapper* smpl)
{
    smpl_ = smpl;
}

void ShapeUnderClothOptimizer::setNewInput(GeneralMesh * input)
{
    input_ = input;
}

void ShapeUnderClothOptimizer::setNewPriorPath(const char * prior_path)
{
    std::string path(prior_path);
    readAttractivePose_(path);
    readStiffness_(path);
}

void ShapeUnderClothOptimizer::findOptimalSMPLParameters(std::vector<Eigen::MatrixXd>* iteration_results, const double parameter)
{
    // Use the current state of smpl_ as init parameters

    // zero guess, because we assume normalized input and normalized smpl_
    E::VectorXd translation_guess = input_->getMeanPoint() - smpl_->getTemplateMeanPoint();

    // Setup solvers options
    Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;   // analytic jacobian is dense
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 500;   // usually converges faster

    // to debug jacobian
    //options.check_gradients = true;

    SMPLVertsLoggingCallBack* callback = nullptr;
    if (iteration_results != nullptr)
    {
        callback = new SMPLVertsLoggingCallBack(smpl_, iteration_results);
        options.callbacks.push_back(callback);
        options.update_state_every_iteration = true;
    }

    auto start_time = std::chrono::system_clock::now();
    // just some number of cycles
    for (int i = 0; i < 3; ++i)
    {
        std::cout << "***********************" << std::endl
            << "    Cycle #" << i << std::endl
            << "***********************" << std::endl;

        generalPoseEstimation_(options, 1.);

        shapeEstimation_(options, parameter);
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

void ShapeUnderClothOptimizer::generalPoseEstimation_(Solver::Options& options, const double parameter)
{
    std::cout << "-----------------------" << std::endl
              << "          Pose" << std::endl
              << "-----------------------" << std::endl;

    Problem problem;

    // Main cost
    CostFunction* cost_function = new AbsoluteDistanceForPose(smpl_, input_, parameter);
    problem.AddResidualBlock(cost_function, nullptr, 
        smpl_->getStatePointers().pose,
        smpl_->getStatePointers().translation);

    // Regularizer
    CostFunction* prior = new NormalPrior(stiffness_, attractive_pose_);
    LossFunction* scale_prior = new ScaledLoss(NULL, 0.007, ceres::TAKE_OWNERSHIP);    // 0.0007
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

    CostFunction* cost_function = new AbsoluteDistanceForShape(smpl_, input_, parameter);
    problem.AddResidualBlock(cost_function, nullptr, 
        smpl_->getStatePointers().shape, 
        smpl_->getStatePointers().translation);

    // TODO add light regularization

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

void ShapeUnderClothOptimizer::readAttractivePose_(const std::string path)
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
    
    this->attractive_pose_.resize(SMPLWrapper::POSE_SIZE);
    // For convinient use of the mean pose with full pose vectors, root rotation is set to zero
    for (int i = 0; i < SMPLWrapper::SPACE_DIM; i++)
        attractive_pose_(i) = 0.;
    // Now read from file
    for (int i = SMPLWrapper::SPACE_DIM; i < SMPLWrapper::POSE_SIZE; i++)
        inFile >> attractive_pose_(i);

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

ceres::CallbackReturnType ShapeUnderClothOptimizer::SMPLVertsLoggingCallBack::operator()(const ceres::IterationSummary & summary)
{
    // TODO get last calculated vertices
    Eigen::MatrixXd verts = smpl_->calcModel();
    smpl_verts_results_->push_back(verts);

    return ceres::SOLVER_CONTINUE;
}
