#pragma once

#include <Eigen/Eigen/Dense>
#include "SMPLWrapper.h"

class ModelToInputDistanceCostFunctor
{
public:
    ModelToInputDistanceCostFunctor(SMPLWrapper*, Eigen::MatrixXd*);
    ~ModelToInputDistanceCostFunctor();

    template <typename T>
    bool operator()(const T * const, const T * const, const T * const, T *) const;

private:
    SMPLWrapper* smpl_;
    Eigen::MatrixXd* input_verts_;
};


template<typename T>
inline bool ModelToInputDistanceCostFunctor::operator()(const T * const translation, const T * const pose, const T * const shape, T * residual) const
{
    // evaluate smpl
    E::Matrix<T, E::Dynamic, E::Dynamic> verts = this->smpl_->calcModel<T>(pose, shape);

    // translate
    for (int i = 0; i < SMPLWrapper::VERTICES_NUM; i++)
    {
        for (int j = 0; j < SMPLWrapper::SPACE_DIM; ++j)
        {
            verts(i, j) += translation[j];
        }
    }

    // difference with input
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> verts_diff = (*this->input_verts_).cast<T>() - verts;

    // final residuals
    for (int i = 0; i < SMPLWrapper::VERTICES_NUM; i++)
    {
        residual[i] = sqrt(verts_diff(i, 0) * verts_diff(i, 0) + verts_diff(i, 1) * verts_diff(i, 1) + verts_diff(i, 2) * verts_diff(i, 2));
    }

    return true;
}