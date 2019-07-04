#pragma once
#include "ceres/ceres.h"
#include "igl/point_mesh_squared_distance.h"
#include "igl/signed_distance.h"

#include <GeneralMesh/GeneralMesh.h>
#include "SMPLWrapper.h"

class AbsoluteDistanceBase : public ceres::CostFunction
{
public:
    AbsoluteDistanceBase(SMPLWrapper*, GeneralMesh *);
    ~AbsoluteDistanceBase();

protected:

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
    template<typename Row1, typename Row2, typename Row3>
    inline double shape_jac_elem_(const Row1&& vertex,
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

