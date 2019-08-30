#pragma once

#include <ceres/ceres.h>

#include "SMPLWrapper.h"

class SmoothDisplacementCost : public ceres::CostFunction
{
public:
    SmoothDisplacementCost(std::shared_ptr<SMPLWrapper> smpl, int vert_id);
    ~SmoothDisplacementCost();

    // expects that the smpl_->displacements are up to date
    virtual bool Evaluate(double const* const* parameters,
        double* residuals,
        double** jacobians) const;

private:
    Eigen::VectorXd vertNeighboursAverageDisplacement_() const;

    // state
    std::shared_ptr<SMPLWrapper> smpl_;
    int vert_id_;
};

