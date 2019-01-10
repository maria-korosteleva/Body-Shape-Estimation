#pragma once

#include "SMPLWrapper.h"

#include <Eigen/Eigen/Dense>
#include "ceres/ceres.h"
#include "glog/logging.h"

using ceres::AutoDiffCostFunction;
using ceres::NumericDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;


class ShapeUnderClothOptimizer
{
public:
    ShapeUnderClothOptimizer();
    ~ShapeUnderClothOptimizer();
    
    void setSMPLModel(SMPLWrapper*);
    void setInput(Eigen::VectorXd*);

    void findOptimalParameters();

private:
    // fixed
    SMPLWrapper* smpl_;
    Eigen::VectorXd* input_verts_;

    // last params
    Eigen::VectorXd pose_;
    Eigen::VectorXd shape_;

};

