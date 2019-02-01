#pragma once

#include <Eigen/Dense>
#include "ceres/ceres.h"

#include "SMPLWrapper.h"
#include "GeneralMesh.h"
#include "AbsoluteVertsToMeshDistance.h"
#include "igl/point_mesh_squared_distance.h"

//#define DEBUG

class AbsoluteVertsToMeshDistanceFunctor {
public:
    AbsoluteVertsToMeshDistanceFunctor(GeneralMesh * input)
        : toMesh_(input) {};
    ~AbsoluteVertsToMeshDistanceFunctor() {};
    bool operator()(double const * parameters, double * residuals) const;
private:
    GeneralMesh * toMesh_;
};


class ModelToInputDistanceCostFunctor
{
public:
    ModelToInputDistanceCostFunctor(SMPLWrapper*, GeneralMesh*);

    ~ModelToInputDistanceCostFunctor();

    template <typename T>
    bool operator()(const T * const, T *) const;

private:
    SMPLWrapper * smpl_;
    GeneralMesh * input_;

    ceres::CostFunctionToFunctor<SMPLWrapper::VERTICES_NUM, SMPLWrapper::VERTICES_NUM * SMPLWrapper::SPACE_DIM> absDist_;
};


template<typename T>
inline bool ModelToInputDistanceCostFunctor::operator()(const T * const translation, T * residual) const     // const T * const pose, const T * const shape, 
{
    // evaluate smpl
    E::Matrix<T, E::Dynamic, E::Dynamic> verts = this->smpl_->calcModel<T>(nullptr, nullptr);

    // translate
    for (int i = 0; i < SMPLWrapper::VERTICES_NUM; i++)
    {
        for (int j = 0; j < SMPLWrapper::SPACE_DIM; j++)
        {
            verts(i, j) += translation[j];
        }
    }

    // difference with input
    // Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> verts_diff = (this->input_vertices_).cast<T>() - verts;
    // distance to input
    //T * dists = new T[SMPLWrapper::VERTICES_NUM];

    this->absDist_(verts.data(), residual);

    // final residuals
    //for (int i = 0; i < SMPLWrapper::VERTICES_NUM; i++)
    //{
    //    //residual[i] = sqrt(verts_diff(i, 0) * verts_diff(i, 0) + verts_diff(i, 1) * verts_diff(i, 1) + verts_diff(i, 2) * verts_diff(i, 2));
    //    residual[i] = sqrt(dists[i]);
    //}

    //delete[] dists;

    return true;
}