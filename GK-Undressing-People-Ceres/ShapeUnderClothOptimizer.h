#pragma once
#define GLOG_NO_ABBREVIATED_SEVERITIES
#define DEBUG

#include <Eigen/Dense>
#include "ceres/ceres.h"
#include "ceres/normal_prior.h"
#include "glog/logging.h"

#include "GeneralMesh.h"
#include "SMPLWrapper.h"
// cost functions
#include "AbsoluteVertsToMeshDistance.h"
#include "DirBasedDistanceForPose.h"

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
    ShapeUnderClothOptimizer(SMPLWrapper* smpl, GeneralMesh* input, const char* path_to_prior);
    ~ShapeUnderClothOptimizer();
    
    void setNewSMPLModel(SMPLWrapper*);
    void setNewInput(GeneralMesh*);
    void setNewPriorPath(const char*);

    // The getters below return a copy of the corresponding parameters
    // So the caller is responsible for the memory allocated
    double * getEstimatesTranslationParams();
    double * getEstimatesPoseParams();
    double * getEstimatesShapeParams();

    void findOptimalParameters(std::vector<Eigen::MatrixXd>* iteration_results = nullptr);

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

    // inidividual optimizers
    // expect the params to be initialized outside
    void direstionalPoseEstimation_(Solver::Options& options);
    void generalPoseEstimation_(Solver::Options& options);

    // utils
    void erase_params_();
    void readMeanPose_(const std::string);
    void readStiffness_(const std::string);
    static void zeros_(double *, std::size_t);
    static void printArray_(double*, std::size_t);

    // inner classes
    class SMPLVertsLoggingCallBack : public ceres::IterationCallback
    {
    public:
        explicit SMPLVertsLoggingCallBack(SMPLWrapper* smpl,
            const double const * pose, const double const * shape, const double const * translation, 
            std::vector<Eigen::MatrixXd>* smpl_verts_iteration_results)
            : smpl_(smpl), 
            pose_(pose), 
            shape_(shape), 
            translation_(translation), 
            smpl_verts_results_(smpl_verts_iteration_results) {}

        ~SMPLVertsLoggingCallBack() {}

        ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary);
    private:
        SMPLWrapper* smpl_;
        std::vector<Eigen::MatrixXd>* smpl_verts_results_;
        // parameter trackers
        const double const * pose_; 
        const double const * shape_;
        const double const * translation_;

    };
};


