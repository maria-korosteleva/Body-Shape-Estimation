#pragma once
#include "ceres/ceres.h"
#include "igl/point_mesh_squared_distance.h"
#include "igl/signed_distance.h"

#include <GeneralMesh/GeneralMesh.h>
#include "SMPLWrapper.h"

//#define DEBUG

class AbsoluteDistanceForPose : public ceres::CostFunction
{
public:
    AbsoluteDistanceForPose(SMPLWrapper*, GeneralMesh *, const double inside_coef);
    ~AbsoluteDistanceForPose();

    // parameters[0] <-> pose, parameters[1] <-> translation
    // Main idea for point-to-surface distance jacobian: 
    // Gradient for each vertex correspondes to the distance from this vertex to the input mesh.
    virtual bool Evaluate(double const* const* parameters,
        double* residuals,
        double** jacobians) const;
private:
    template<typename Row1, typename Row2, typename Row3>
    inline double pose_jac_elem_(const Row1&& vertex,
        const Row2&& closest_input_point,
        double signed_dist,
        const Row3&& grad) const
    {
        double jac_entry = signed_dist > 0
            ? 2. * (vertex - closest_input_point).dot(grad)
            : inside_coef_ * 2. * (vertex - closest_input_point).dot(grad);

        return jac_entry;
    }

    inline double translation_jac_elem_(const double vert_coord,
        const double input_coord, double signed_dist) const
    {
        double jac_entry = signed_dist > 0
            ? 2. * (vert_coord - input_coord)
            : inside_coef_ * 2. * (vert_coord - input_coord);
        return jac_entry;
    }

    GeneralMesh * toMesh_;
    SMPLWrapper * smpl_;

    // optional 
    double inside_coef_ = 1.;
};

