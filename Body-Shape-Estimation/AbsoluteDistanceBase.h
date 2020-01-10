#pragma once
#include <ceres/ceres.h>
#include <igl/point_mesh_squared_distance.h>
#include <igl/signed_distance.h>

#include <GeneralMesh/GeneralMesh.h>
#include "SMPLWrapper.h"

class AbsoluteDistanceBase : public ceres::CostFunction, public ceres::EvaluationCallback
{
public:
    enum ParameterType{
        BASE, 
        TRANSLATION, 
        SHAPE, 
        POSE,
        DISPLACEMENT
    };
    enum DistanceType {
        IN_DIST, 
        OUT_DIST, 
        BOTH_DIST, 
        CLOTH_IN, 
        CLOTH_OUT, 
        SKIN_BOTH
    };

    AbsoluteDistanceBase(SMPLWrapper*, GeneralMesh *,
        ParameterType parameter = BASE, DistanceType dist_type = BOTH_DIST,
        double pruning_threshold = 100.,
        std::size_t vertex_id = 0);
    ~AbsoluteDistanceBase();

    // Callback to be called before the evaluation of the optimization step
    // the new optimization parameter values are pushed to the smpl_ parameters 
    virtual void PrepareForEvaluation(bool evaluate_jacobians, bool new_evaluation_point);

    // parameters[0] <-> this->parameter_type_
    // Main idea for point-to-surface distance jacobian: 
    // Gradient for each vertex correspondes to the distance from this vertex to the input mesh.
    virtual bool Evaluate(double const* const* parameters,
        double* residuals,
        double** jacobians) const;

protected:
    struct DistanceResult {
        Eigen::MatrixXd verts;
        Eigen::MatrixXd verts_normals;
        std::vector<Eigen::MatrixXd> jacobian;
        // libigl output
        Eigen::VectorXd signedDists; 
        Eigen::VectorXi closest_face_ids; 
        Eigen::MatrixXd closest_points;
        Eigen::MatrixXd normals_for_sign;
    };

    void updateDistanceCalculations(bool with_jacobian, DistanceResult& out_distance_result);
    void calcSignedDistByVertecies(DistanceResult& out_distance_result) const;

    void fillJac(const DistanceResult& distance_res, const double* residuals, double * jacobian) const;
    void fillDisplacementJac(const DistanceResult& distance_res, const double* residuals, double * jacobian) const;
    void fillTranslationJac(const DistanceResult& distance_res, const double* residuals, double * jacobian) const;

    // 
    template<typename Row1, typename Row2>
    inline double residual_elem_(const double signed_dist, 
        const Row1 vertex_normal, const Row2 input_normal, const double cloth_prob = 1.) const
    {
        if ((dist_evaluation_type_ == IN_DIST || dist_evaluation_type_ == CLOTH_IN) && signed_dist > 0       // is outside, want inside
            || (dist_evaluation_type_ == OUT_DIST || dist_evaluation_type_ == CLOTH_OUT) && signed_dist < 0 // is inside, want outside
            || abs(signed_dist) > pruning_threshold_            // too far
            || vertex_normal.dot(input_normal) <= 0)            // looks the wrong way. Check last, as it's most expensive
        {
            return 0;
        }

        switch (dist_evaluation_type_)
        {
        case CLOTH_IN:
        case CLOTH_OUT:
            return sqrt(cloth_prob) * abs(signed_dist);
        case SKIN_BOTH:
            return sqrt(1. - cloth_prob) * abs(signed_dist);
        default:
            return abs(signed_dist);
        }
    }

    // Jac values are set to zero whenever the residual is zero/close to zero
    template<typename Row1, typename Row2, typename Row3>
    inline double jac_elem_(const Row1&& vertex,
        const Row2&& closest_input_point,
        double abs_dist,
        const Row3&& grad, 
        const double cloth_prob = 1.) const
    {
        double jac_entry = abs_dist < 1e-5
            ? 0
            : (vertex - closest_input_point).dot(grad) / abs_dist;
        
        switch (dist_evaluation_type_)
        {
        case CLOTH_IN:
        case CLOTH_OUT:
            return sqrt(cloth_prob) * jac_entry;
        case SKIN_BOTH:
            return sqrt(1. - cloth_prob) * jac_entry;
        default:
            return jac_entry;
        }
    }

    inline double translation_jac_elem_(const double vert_coord,
        const double input_coord, double abs_dist) const
    {
        double jac_entry = abs_dist < 1e-5
            ? 0
            : (vert_coord - input_coord) / abs_dist;
        return jac_entry;
    }

    GeneralMesh * toMesh_;
    SMPLWrapper * smpl_;
    double pruning_threshold_;

    // instance type
    ParameterType parameter_type_;
    std::size_t vertex_id_for_displacement_ = 0;  // for the DISPLACEMENT only 
    bool displacement_jac_evaluated = false;      // for the DISPLACEMENT only
    DistanceType dist_evaluation_type_;

    // last evaluated result
    static DistanceResult last_result_;
};

