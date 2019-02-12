#pragma once

#include "ceres/ceres.h"
#include "GeneralMesh.h"
#include "SMPLWrapper.h"
#include "igl/point_mesh_squared_distance.h"

//#define DEBUG

class AbsoluteVertsToMeshDistance : 
    public ceres::SizedCostFunction<SMPLWrapper::VERTICES_NUM, SMPLWrapper::SPACE_DIM, SMPLWrapper::SHAPE_SIZE, SMPLWrapper::POSE_SIZE>  // SMPLWrapper::VERTICES_NUM *
{
public:
    AbsoluteVertsToMeshDistance(SMPLWrapper*, GeneralMesh *);
    ~AbsoluteVertsToMeshDistance();

    virtual bool Evaluate(double const* const* parameters,
        double* residuals,
        double** jacobians) const;
private:
    GeneralMesh * toMesh_;
    SMPLWrapper * smpl_;
};

