#pragma once
#include "ceres/ceres.h"
#include "igl/point_mesh_squared_distance.h"
#include "igl/signed_distance.h"

#include <GeneralMesh/GeneralMesh.h>
#include "SMPLWrapper.h"

#define SQR(x) ((x)*(x)) 

class AbsoluteDistanceForPose : public ceres::CostFunction
{
public:
    AbsoluteDistanceForPose(SMPLWrapper*, GeneralMesh *);
    ~AbsoluteDistanceForPose();

    // parameters[0] <-> pose, parameters[1] <-> translation
    // Main idea for point-to-surface distance jacobian: 
    // Gradient for each vertex correspondes to the distance from this vertex to the input mesh.
    virtual bool Evaluate(double const* const* parameters,
        double* residuals,
        double** jacobians) const;
private:
    // 
    inline double residual_elem_(const double signed_dist) const
    {
        return abs(signed_dist);
    }

    //
    template<typename Row1, typename Row2, typename Row3>
    inline double pose_jac_elem_(const Row1&& vertex,
        const Row2&& closest_input_point,
        double signed_dist,
        const Row3&& grad) const
    {
        double jac_entry = abs(signed_dist) < 1e-5
            ? 0
            : (vertex - closest_input_point).dot(grad) / abs(signed_dist);

        return jac_entry;
    }

    //
    inline double translation_jac_elem_(const double vert_coord,
        const double input_coord, double signed_dist) const
    {
        double jac_entry = abs(signed_dist) < 1e-5
            ? 0
            : (vert_coord - input_coord) / abs(signed_dist);
        return jac_entry;
    }

    GeneralMesh * toMesh_;
    SMPLWrapper * smpl_;
};

#undef SQR(x)