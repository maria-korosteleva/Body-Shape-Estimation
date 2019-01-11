#pragma once

#include <Eigen/Eigen/Dense>
#include "SMPLWrapper.h"

class ModelToInputDistanceCostFunctor
{
public:
    ModelToInputDistanceCostFunctor(SMPLWrapper*, Eigen::MatrixXd*);
    ~ModelToInputDistanceCostFunctor();

    template <typename T>
    bool operator()(const T* const, T*) const;

private:
    SMPLWrapper* smpl_;
    Eigen::MatrixXd* input_verts_;
};


template<typename T>
inline bool ModelToInputDistanceCostFunctor::operator()(const T * const shape, T * residual) const
{
    // shape smpl
    T* pose = nullptr;
    E::Matrix<T, E::Dynamic, E::Dynamic> verts = this->smpl_->calcModel<T>(pose, shape);

    // difference with input
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> verts_diff = (*this->input_verts_).cast<T>() - verts;

    // final residuals
    for (int i = 0; i < SMPLWrapper::NUM_VERTICES; i++)
    {
        residual[i] = sqrt(verts_diff(i, 0) * verts_diff(i, 0) + verts_diff(i, 0) * verts_diff(i, 0) + verts_diff(i, 0) * verts_diff(i, 0));
    }
    return true;
}