#pragma once
#define GLOG_NO_ABBREVIATED_SEVERITIES
//#define DEBUG

#include <Eigen/Eigen/Dense>
#include "ceres/ceres.h"
#include "ceres/normal_prior.h"
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
using ceres::NormalPrior;
using ceres::LossFunction;
using ceres::ScaledLoss;


class ShapeUnderClothOptimizer
{
public:
    ShapeUnderClothOptimizer(SMPLWrapper*, Eigen::MatrixXd*, const char*);
    ~ShapeUnderClothOptimizer();
    
    void setNewSMPLModel(SMPLWrapper*);
    void setNewInput(Eigen::MatrixXd*);
    void setNewPriorPath(const char*);

    double* getEstimatesTranslationParams();
    double* getEstimatesPoseParams();
    double* getEstimatesShapeParams();

    void findOptimalParameters();

private:
    // fixed
    SMPLWrapper* smpl_ = nullptr;
    Eigen::MatrixXd* input_verts_ = nullptr;
    ceres::Matrix stiffness_;
    ceres::Vector mean_pose_;

    // last params
    double* translation_ = nullptr;
    double* pose_ = nullptr;
    double* shape_ = nullptr;

    void erase_params_();

    // utils
    void readMeanPose_(const std::string);
    void readStiffness_(const std::string);
    static void zeros_(double*, std::size_t);
    static void printArray_(double*, std::size_t);
};

