#pragma once

#include "ceres/ceres.h"
#include "igl/point_mesh_squared_distance.h"
#include "igl/signed_distance.h"

#include <GeneralMesh/GeneralMesh.h>
#include "SMPLWrapper.h"

#define SQR(x) ((x)*(x)) 

class AbsoluteDistanceForShape : public ceres::CostFunction
{
public:
    AbsoluteDistanceForShape(SMPLWrapper*, GeneralMesh *, const double param);
    ~AbsoluteDistanceForShape();

    // parameters[0] <-> shape, parameters[1] <-> translation
    // Main idea for point-to-surface distance jacobian: 
    // Gradient for each vertex correspondes to the distance from this vertex to the input mesh.
    virtual bool Evaluate(double const* const* parameters,
        double* residuals,
        double** jacobians) const;

private:
    // 
    inline double residual_elem_(const double signed_dist) const
    {
        //residuals[i] = sqrD(i); 
        return signed_dist > 0 ? 
            SQR(signed_dist)
            : SQR(signed_dist) / (1 + gm_coef_ * SQR(signed_dist));     // inner distance regularized
    }

    //
    template<typename Row1, typename Row2, typename Row3>
    inline double shape_jac_elem_(const Row1&& vertex,
        const Row2&& closest_input_point,
        double signed_dist,
        const Row3&& grad) const
    {
        double jac_entry = signed_dist >= 0.
            ? 2. * (vertex - closest_input_point).dot(grad)
            : 2. * (vertex - closest_input_point).dot(grad) / SQR(1 - gm_coef_ * signed_dist);

        return jac_entry;
    }

    //
    inline double translation_jac_elem_(const double vert_coord,
        const double input_coord, double signed_dist) const
    {
        double jac_entry = signed_dist >= 0.
            ? 2. * (vert_coord - input_coord)
            : 2. * (vert_coord - input_coord) / SQR(1 - gm_coef_ * signed_dist);

        return jac_entry;
    }


    GeneralMesh * toMesh_;
    SMPLWrapper * smpl_;

    // optional
    double gm_coef_ = 0.;
};

#undef SQR(x)

