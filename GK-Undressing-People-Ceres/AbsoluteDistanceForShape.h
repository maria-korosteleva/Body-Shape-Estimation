#pragma once

#include "AbsoluteDistanceBase.h"

class AbsoluteDistanceForShape : public AbsoluteDistanceBase
{
public:
    AbsoluteDistanceForShape(SMPLWrapper*, GeneralMesh *, double pruning_threshold = 100., const double param = 0.);
    ~AbsoluteDistanceForShape();

    // parameters[0] <-> shape, parameters[1] <-> translation
    // Main idea for point-to-surface distance jacobian: 
    // Gradient for each vertex correspondes to the distance from this vertex to the input mesh.
    virtual bool Evaluate(double const* const* parameters,
        double* residuals,
        double** jacobians) const;

private:

    // optional
    double gm_coef_ = 0.;
};

#undef SQR(x)

