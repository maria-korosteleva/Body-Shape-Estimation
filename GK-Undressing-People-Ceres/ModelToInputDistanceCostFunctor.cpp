#include "pch.h"
#include "ModelToInputDistanceCostFunctor.h"


ModelToInputDistanceCostFunctor::ModelToInputDistanceCostFunctor(SMPLWrapper * smpl, Eigen::MatrixXd * input_verts)
    : smpl_(smpl), input_verts_(input_verts)
{}


ModelToInputDistanceCostFunctor::~ModelToInputDistanceCostFunctor()
{
}
