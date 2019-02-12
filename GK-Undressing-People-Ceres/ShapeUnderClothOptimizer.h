#pragma once
#define GLOG_NO_ABBREVIATED_SEVERITIES
//#define DEBUG

#include <Eigen/Dense>
#include "ceres/ceres.h"
#include "ceres/normal_prior.h"
#include "glog/logging.h"

#include "GeneralMesh.h"
#include "SMPLWrapper.h"
#include "ModelToInputDistanceCostFunctor.h"
#include "AbsoluteVertsToMeshDistance.h"
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
    ShapeUnderClothOptimizer(SMPLWrapper*, GeneralMesh*, const char*);
    ~ShapeUnderClothOptimizer();
    
    void setNewSMPLModel(SMPLWrapper*);
    void setNewInput(GeneralMesh*);
    void setNewPriorPath(const char*);

    // The getters below return a copy of the corresponding parameters
    // So the caller is responsible for the memory allocated
    double * getEstimatesTranslationParams();
    double * getEstimatesPoseParams();
    double * getEstimatesShapeParams();

    void findOptimalParameters();

private:
    // fixed
    SMPLWrapper * smpl_ = nullptr;
    GeneralMesh * input_ = nullptr;
    ceres::Matrix stiffness_;
    ceres::Vector mean_pose_;

    // last params
    double * translation_ = nullptr;
    double * pose_ = nullptr;
    double * shape_ = nullptr;

    void erase_params_();

    // utils
    void readMeanPose_(const std::string);
    void readStiffness_(const std::string);
    static void zeros_(double *, std::size_t);
    static void printArray_(double*, std::size_t);
};

