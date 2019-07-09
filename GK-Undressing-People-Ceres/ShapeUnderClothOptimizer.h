#pragma once
#define GLOG_NO_ABBREVIATED_SEVERITIES
//#define DEBUG

#include <Eigen/Dense>
#include "ceres/ceres.h"
#include "ceres/normal_prior.h"
#include "glog/logging.h"

#include <GeneralMesh/GeneralMesh.h>
#include "SMPLWrapper.h"
// cost functions
#include "AbsoluteDistanceForPose.h"
#include "AbsoluteDistanceForShape.h"
#include "AbsoluteDistanceForTranslation.h"

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
    ShapeUnderClothOptimizer(std::shared_ptr<SMPLWrapper> smpl, std::shared_ptr<GeneralMesh> input, const std::string path_to_prior);
    ~ShapeUnderClothOptimizer();
    
    void setNewSMPLModel(std::shared_ptr<SMPLWrapper>);
    void setNewInput(std::shared_ptr<GeneralMesh>);
    void setNewPriorPath(const char*);
    void setShapeRegularizationWeight(double weight) { shape_reg_weight_ = weight; };

    std::shared_ptr<SMPLWrapper> getLastSMPL() const { return smpl_; }

    // parameter is some parameter of the underlying procedures; used for experiments; the semantics should be controlled by the programmer
    void findOptimalSMPLParameters(std::vector<Eigen::MatrixXd>* iteration_results = nullptr, const double parameter = 0.);

private:
    // inidividual optimizers
    // expect the params to be initialized outside
    void translationEstimation_(Solver::Options& options);
    void poseEstimation_(Solver::Options& options, ceres::Vector& prior_pose, const double parameter = 1.);
    void shapeEstimation_(Solver::Options& options, const double parameter = 1.);

    // utils
    void readAveragePose_deprecated_(const std::string);
    void readStiffness_(const std::string);
    static void zeros_(double *, std::size_t);
    static void printArray_(double*, std::size_t);
    static ceres::Vector copyArray_(double*, std::size_t);
    void checkCeresOptions(const Solver::Options& options);

    // data
    // use the shared_ptr to make sure objects won't dissapear in-between calls to this class
    std::shared_ptr<SMPLWrapper> smpl_ = nullptr;
    std::shared_ptr<GeneralMesh> input_ = nullptr;
    ceres::Matrix stiffness_;
    ceres::Vector average_pose_deprecated_;

    // parameters
    double shape_reg_weight_;

    // inner classes
    class SMPLVertsLoggingCallBack : public ceres::IterationCallback
    {
    public:
        explicit SMPLVertsLoggingCallBack(std::shared_ptr<SMPLWrapper> smpl,
            std::vector<Eigen::MatrixXd>* smpl_verts_iteration_results)
            : smpl_verts_results_(smpl_verts_iteration_results) {
            smpl_ = std::move(smpl);
        }

        ~SMPLVertsLoggingCallBack() {}

        ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary);
    private:
        std::shared_ptr<SMPLWrapper> smpl_;
        std::vector<Eigen::MatrixXd>* smpl_verts_results_;
    };
};


