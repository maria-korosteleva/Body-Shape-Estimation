#pragma once

#include "AbsoluteDistanceBase.h"

#define SQR(x) ((x)*(x)) 

class AbsoluteDistanceForTranslation : public AbsoluteDistanceBase
{
public:
    AbsoluteDistanceForTranslation(SMPLWrapper*, GeneralMesh *);
    ~AbsoluteDistanceForTranslation();

    // parameters[0] <-> translation
    // Main idea for point-to-surface distance jacobian: 
    // Gradient for each vertex correspondes to the distance from this vertex to the input mesh.
    virtual bool Evaluate(double const* const* parameters,
        double* residuals,
        double** jacobians) const;
};

#undef SQR(x)