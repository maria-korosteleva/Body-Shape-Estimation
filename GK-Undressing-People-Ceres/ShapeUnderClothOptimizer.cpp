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

    // construct a problem
    Problem problem;
    CostFunction* cost_function =
        new AutoDiffCostFunction<DistCost, SMPLWrapper::NUM_VERTICES, SMPLWrapper::SHAPE_SIZE>(new DistCost(this->smpl_, this->input_verts_));
    problem.AddResidualBlock(cost_function, nullptr, this->shape_);

    // Run the solver!
    Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = true;

    // print summary
    Solver::Summary summary;
    Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;
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


