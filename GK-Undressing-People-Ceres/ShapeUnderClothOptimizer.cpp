#include "pch.h"
#include "ShapeUnderClothOptimizer.h"


ShapeUnderClothOptimizer::ShapeUnderClothOptimizer(SMPLWrapper* smpl, Eigen::MatrixXd* input_verts)
{
    this->smpl_ = smpl;
    this->input_verts_ = input_verts;
}


ShapeUnderClothOptimizer::~ShapeUnderClothOptimizer()
{
    this->erase_params_();
}


void ShapeUnderClothOptimizer::setSMPLModel(SMPLWrapper* smpl)
{
    this->smpl_ = smpl;
}


void ShapeUnderClothOptimizer::setInput(Eigen::MatrixXd* input_verts)
{
    this->input_verts_ = input_verts;
}


double * ShapeUnderClothOptimizer::getEstimatesPoseParams()
{
    if (this->pose_ != nullptr)
    {
        double * last_pose = new double[SMPLWrapper::POSE_SIZE];
        for (int i = 0; i < SMPLWrapper::POSE_SIZE; i++)
            last_pose[i] = this->pose_[i];
        return last_pose;
    }
    else
        return nullptr;
}


double * ShapeUnderClothOptimizer::getEstimatesShapeParams()
{
    if (this->shape_ != nullptr)
    {
        double * last_shape = new double[SMPLWrapper::SHAPE_SIZE];
        for (int i = 0; i < SMPLWrapper::SHAPE_SIZE; i++)
            last_shape[i] = this->shape_[i];
        return last_shape;
    }
    else 
        return nullptr;
}


void ShapeUnderClothOptimizer::findOptimalParameters()
{
    google::InitGoogleLogging("ShapeUnderClothing");

    this->erase_params_();

    // init parameters
    this->shape_ = new double[SMPLWrapper::SHAPE_SIZE];
    this->zeros_(this->shape_, SMPLWrapper::SHAPE_SIZE);
    this->pose_ = new double[SMPLWrapper::POSE_SIZE];
    this->zeros_(this->pose_, SMPLWrapper::POSE_SIZE);

    // construct a problem
    Problem problem;
    CostFunction* cost_function =
        new AutoDiffCostFunction<DistCost, SMPLWrapper::VERTICES_NUM, SMPLWrapper::POSE_SIZE, SMPLWrapper::SHAPE_SIZE>(new DistCost(this->smpl_, this->input_verts_));
    problem.AddResidualBlock(cost_function, nullptr, this->pose_, this->shape_);

    // Run the solver!
    Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = true;
    //options.max_num_iterations = 150;
    //options.use_nonmonotonic_steps

    // print summary
    Solver::Summary summary;
    Solve(options, &problem, &summary);
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

    //std::cout << "Jacobian (sparse)" << std::endl;
    //std::vector<double> values = jac.values;
    //for (auto i = values.begin(); i != values.end(); ++i)
    //{
    //    std::cout << *i << " ";
    //}
    //std::cout << std::endl;

#endif // DEBUG
}


void ShapeUnderClothOptimizer::erase_params_()
{
    if (this->pose_ != nullptr)
    {
        delete[] this->pose_;
        this->pose_ = nullptr;
    }

    if (this->shape_ != nullptr)
    {
        delete[] this->shape_;
        this->shape_ = nullptr;
    }
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


