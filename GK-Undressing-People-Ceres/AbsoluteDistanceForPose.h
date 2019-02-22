#pragma once

#include "ceres/ceres.h"
#include "GeneralMesh.h"
#include "SMPLWrapper.h"
#include "igl/point_mesh_squared_distance.h"

//#define DEBUG

class AbsoluteDistanceForPose : 
    public ceres::CostFunction
{
public:
    static constexpr double coef_key_vertices_ = 10;

    AbsoluteDistanceForPose(SMPLWrapper*, GeneralMesh *, double * shape = nullptr);
    ~AbsoluteDistanceForPose();

    // parameters[0] <-> pose, parameters[1] <-> translation
    // Main idea for point-to-surface distance jacobian: 
    // Gradient for each vertex correspondes to the distance from this vertex to the input mesh.
    virtual bool Evaluate(double const* const* parameters,
        double* residuals,
        double** jacobians) const;
private:
    GeneralMesh * toMesh_;
    int key_verts_num_;
    SMPLWrapper * smpl_;
    double * shape_;

    // uses NULL for compatibility with ceres
    //bool EvaluateKeyVerticesDiff(E::MatrixXd& verts, E::MatrixXd * pose_jac, double * wrt_pose, double * wrt_transl);
};

