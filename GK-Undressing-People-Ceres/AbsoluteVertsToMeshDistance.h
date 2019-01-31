#pragma once

#include "ceres/ceres.h"
#include "GeneralMesh.h"
#include "SMPLWrapper.h"
#include "igl/point_mesh_squared_distance.h"

class AbsoluteVertsToMeshDistance : public ceres::SizedCostFunction<SMPLWrapper::VERTICES_NUM, SMPLWrapper::VERTICES_NUM * SMPLWrapper::SPACE_DIM>
{
public:
    AbsoluteVertsToMeshDistance(GeneralMesh *);
    ~AbsoluteVertsToMeshDistance();

    virtual bool Evaluate(double const* const* parameters,
        double* residuals,
        double** jacobians) const;
private:
    GeneralMesh * toMesh_;
};

