#pragma once
#include "AbsoluteDistanceBase.h"

class AbsoluteDistanceForPose : public AbsoluteDistanceBase
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
};