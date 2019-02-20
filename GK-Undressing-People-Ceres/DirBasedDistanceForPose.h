#pragma once

#include "ceres/ceres.h"
#include "GeneralMesh.h"
#include "SMPLWrapper.h"

class DirBasedDistanceForPose :
    public ceres::CostFunction
{
public:
    DirBasedDistanceForPose(SMPLWrapper *, GeneralMesh *);
    ~DirBasedDistanceForPose();

    // parameters[0] <-> pose
    virtual bool Evaluate(double const* const* parameters,
        double* residuals,
        double** jacobians) const;

private:
    GeneralMesh * toMesh_;
    int key_dirs_num_;
    SMPLWrapper * smpl_;
};

