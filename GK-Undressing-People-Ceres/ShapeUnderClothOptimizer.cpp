#include "pch.h"
#include "ShapeUnderClothOptimizer.h"


ShapeUnderClothOptimizer::ShapeUnderClothOptimizer()
{
}


ShapeUnderClothOptimizer::~ShapeUnderClothOptimizer()
{
}

void ShapeUnderClothOptimizer::setSMPLModel(SMPLWrapper* smpl)
{
    this->smpl_ = smpl;
}

void ShapeUnderClothOptimizer::setInput(Eigen::VectorXd* input_verts)
{
    this->input_verts_ = input_verts;
}


class SimpleCostFunctor {
public:
    SimpleCostFunctor(SMPLWrapper* smpl, Eigen::VectorXd* input_verts)
        : smpl_(smpl), input_verts_(input_verts)
    {}

    template <typename T>
    bool operator()(const T* const shape, T* residual) const {
        // shape smpl
        Eigen::VectorXd epose;
        Eigen::VectorXd eshape = Eigen::Map<VectorXd>(shape, this->smpl_->getShapeSize());
        Eigen::MatrixXd* verts = this->smpl_->calcModel(epose, eshape);

        // difference with input
        Eigen::MatrixXd verts_diff = *this->input_verts_ - *verts;

        // final residuals
        for (int i = 0; i < smpl_->getNumberVertices(); i++)
        {
            residual[i] = sqrt(verts_diff[i][0] * verts_diff[i][0] + verts_diff[i][1] * verts_diff[i][1] + verts_diff[i][2] * verts_diff[i][2])
        }
        return true;
    }
private:
    SMPLWrapper* smpl_;
    Eigen::VectorXd* input_verts_;
};

void ShapeUnderClothOptimizer::findOptimalParameters()
{
    // here is all the fun stuff

    google::InitGoogleLogging("ShapeUnderClothing");

    Eigen::VectorXd e_shape = Eigen::VectorXd::Zero(this->smpl_->getShapeSize());

    Problem problem;

    CostFunction* cost_function =
        new AutoDiffCostFunction<SimpleCostFunctor, 6890, 10>(new SimpleCostFunctor(this->smpl_, this->input_verts_));
    problem.AddResidualBlock(cost_function, NULL, e_shape.data());

    // Run the solver!
    Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = true;
    Solver::Summary summary;
    Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";

    std::cout << e_shape;
}
