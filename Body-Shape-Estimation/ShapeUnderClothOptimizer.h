#pragma once
#define GLOG_NO_ABBREVIATED_SEVERITIES
//#define DEBUG

#include <Eigen/Dense>
#include "ceres/ceres.h"
#include "ceres/normal_prior.h"
#include "glog/logging.h"
#include "igl/signed_distance.h"

#include <GeneralMesh/GeneralMesh.h>
#include "SMPLWrapper.h"
// cost functions
#include "AbsoluteDistanceBase.h"
#include "SmoothDisplacementCost.h"

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
    struct OptimizationOptions
    {
        Solver::Options ceres;

        // hyperparams
        double shape_reg_weight;
        double displacement_reg_weight;
        double displacement_smoothing_weight;
        double pose_reg_weight;
        double shape_prune_threshold;
        double gm_saturation_threshold;
        double in_verts_scaling_weight;

        OptimizationOptions()
        { // defaults
            shape_reg_weight = 0.01;
            pose_reg_weight = 0.001;
            displacement_reg_weight = 0.001;
            displacement_smoothing_weight = 0.1;
            shape_prune_threshold = 0.05;
            gm_saturation_threshold = 2;
            in_verts_scaling_weight = 0.1;
        }
    };

    ShapeUnderClothOptimizer(std::shared_ptr<SMPLWrapper> smpl, std::shared_ptr<GeneralMesh> input);
    ~ShapeUnderClothOptimizer();
    
    void setNewSMPLModel(std::shared_ptr<SMPLWrapper>);
    void setNewInput(std::shared_ptr<GeneralMesh>);
    void setConfig(const OptimizationOptions& config) { config_ = config; };
    void setShapeRegularizationWeight(double weight) { config_.shape_reg_weight = weight; };
    void setDisplacementRegWeight(double weight) { config_.displacement_reg_weight = weight; };
    void setDisplacementSmoothingWeight(double weight) { config_.displacement_smoothing_weight = weight; };
    void setShapePruningThreshold(double value) { config_.shape_prune_threshold = value; };
    void setPoseRegularizationWeight(double weight) { config_.pose_reg_weight = weight; };
    void setGMSaturationThreshold(double threshold) { config_.gm_saturation_threshold = threshold;  }
    void setInScailingWeight(double weight) { config_.in_verts_scaling_weight = weight; }

    std::shared_ptr<SMPLWrapper> getLastSMPL() const { return smpl_; }

    // parameter is some parameter of the underlying procedures; used for experiments; the semantics should be controlled by the programmer
    void findOptimalSMPLParameters(std::vector<Eigen::MatrixXd>* iteration_results = nullptr);

    // tmp
    void gmLossTest();

private:

    // inidividual optimizers
    // expect the params to be initialized outside
    void translationEstimation_(OptimizationOptions& config);

    void poseEstimation_(OptimizationOptions& config, ceres::Matrix & prior_pose);
    void poseMainCostNoSegmetation_(Problem& problem, OptimizationOptions& config);
    void poseMainCostClothAware_(Problem& problem, OptimizationOptions& config);

    void shapeEstimation_(OptimizationOptions& config);
    void shapeMainCostNoSegmetation_(Problem& problem, OptimizationOptions& config);
    void shapeMainCostClothAware_(Problem& problem, OptimizationOptions& config);

    void displacementEstimation_(OptimizationOptions& config);

    // utils
    ceres::ComposedLoss* innerVerticesLoss_(const OptimizationOptions& config);
    void checkCeresOptions(const Solver::Options& config);

    // data
    // use the shared_ptr to make sure objects won't dissapear in-between calls to this class
    std::shared_ptr<SMPLWrapper> smpl_ = nullptr;
    std::shared_ptr<GeneralMesh> input_ = nullptr;
    OptimizationOptions config_;

    // inner classes

    class GemanMcClareLoss : public ceres::LossFunction
    {
    public:
        // GM function is x^2 / (1 + sigma^2 * x^2) with x as independent variable, sigma as paramter to control saturation
        // in our case x is the distance of the model vertex to the input surface
        // => GM loss is s / (1 + sigma^2 * s)
        //
        // ~(1/sigma) = saturation_threshold : about halfway through the fucntion values (1/(2 * sigma^2))
        // the value of the sqrt(s)=x where the loss stops being ~linear and start saturation
        // ~(saturation_threshold^2) = the upper limit GM value approaches when saturated - 
        // larger when the saturation_threshold is larger
        GemanMcClareLoss(double saturation_threshold) : sigma(1 / saturation_threshold) {};

        virtual void Evaluate(double s, double out[3]) const
        {
            double sigma_sqr = sigma * sigma;
            double denominator = 1 + s * sigma_sqr;

            out[0] = s / denominator;
            out[1] = 1 / (denominator * denominator);
            out[2] = -2 * sigma_sqr * out[1] / denominator;
        }
    private:
        double sigma = 0.;
    };


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


