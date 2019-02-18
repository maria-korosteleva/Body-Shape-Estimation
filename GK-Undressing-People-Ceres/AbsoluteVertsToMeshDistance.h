#pragma once

#include "ceres/ceres.h"
#include "GeneralMesh.h"
#include "SMPLWrapper.h"
#include "igl/point_mesh_squared_distance.h"

//#define DEBUG

class AbsoluteVertsToMeshDistance : 
    public ceres::SizedCostFunction<SMPLWrapper::VERTICES_NUM, SMPLWrapper::POSE_SIZE, SMPLWrapper::SPACE_DIM>  // SMPLWrapper::SPACE_DIM,  SMPLWrapper::VERTICES_NUM * SMPLWrapper::SHAPE_SIZE, 
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

