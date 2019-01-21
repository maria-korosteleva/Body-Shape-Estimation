#pragma once
#define GLOG_NO_ABBREVIATED_SEVERITIES
#define DEBUG

#include <Eigen/Eigen/Dense>
#include "ceres/ceres.h"
#include "glog/logging.h"

#include "SMPLWrapper.h"
#include "ModelToInputDistanceCostFunctor.h"
using DistCost = ModelToInputDistanceCostFunctor;

using ceres::AutoDiffCostFunction;
using ceres::NumericDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;


class ShapeUnderClothOptimizer
{
public:
    ShapeUnderClothOptimizer(SMPLWrapper*, Eigen::MatrixXd*);
    ~ShapeUnderClothOptimizer();
    
    void setSMPLModel(SMPLWrapper*);
    void setInput(Eigen::MatrixXd*);

    double* getEstimatesPoseParams();
    double* getEstimatesShapeParams();

    void findOptimalParameters();

private:
    // fixed
    SMPLWrapper* smpl_ = nullptr;
    Eigen::MatrixXd* input_verts_ = nullptr;

    // last params
    double* pose_ = nullptr;
    double* shape_ = nullptr;

    void erase_params_();

    // utils
    static void zeros_(double*, std::size_t);
    static void printArray_(double*, std::size_t);
};

