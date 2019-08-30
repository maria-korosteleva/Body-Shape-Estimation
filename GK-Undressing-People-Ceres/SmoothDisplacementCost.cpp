#include "pch.h"
#include "SmoothDisplacementCost.h"

SmoothDisplacementCost::SmoothDisplacementCost(std::shared_ptr<SMPLWrapper> smpl, int vert_id)
    : vert_id_(vert_id)
{
    smpl_ = std::move(smpl);

    set_num_residuals(SMPLWrapper::SPACE_DIM);
    mutable_parameter_block_sizes()->push_back(SMPLWrapper::SPACE_DIM);
}

SmoothDisplacementCost::~SmoothDisplacementCost()
{
}

bool SmoothDisplacementCost::Evaluate(double const * const * parameters, double * residuals, double ** jacobians) const
{
    Eigen::VectorXd average = vertNeighboursAverageDisplacement_();
    // fill residuals
    for (int axis = 0; axis < parameter_block_sizes()[0]; axis++)
    {
        residuals[axis] = parameters[0][axis] - average[axis];
    }

    // fill jacobian -- always an Identity matrix
    if (jacobians != NULL && jacobians[0] != NULL)
    {
        for (int residual_axis = 0; residual_axis < num_residuals(); residual_axis++)
        {
            for (int param_axis = 0; param_axis < parameter_block_sizes()[0]; param_axis++)
            {
                jacobians[0][residual_axis * parameter_block_sizes()[0] + param_axis] = (residual_axis == param_axis);
            }
        }
    }

    return true;
}

Eigen::VectorXd SmoothDisplacementCost::vertNeighboursAverageDisplacement_() const
{
    const SMPLWrapper::NeighboursList& neighbours = smpl_->getVertNeighbours(vert_id_);
    const SMPLWrapper::ERMatrixXd& displacements = smpl_->getStatePointers().displacements;

    Eigen::VectorXd average = Eigen::VectorXd::Zero(SMPLWrapper::SPACE_DIM);

    for (auto neighbour : neighbours)
    {
        average = average + displacements.row(neighbour);
    }
    average = average / neighbours.size();

    return average;
}
