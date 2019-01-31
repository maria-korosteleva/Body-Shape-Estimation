#include "pch.h"
#include "ModelToInputDistanceCostFunctor.h"


ModelToInputDistanceCostFunctor::ModelToInputDistanceCostFunctor(SMPLWrapper * smpl, GeneralMesh * input)
    : smpl_(smpl), input_(input), absDist_(new AbsoluteVertsToMeshDistance(input))
{
    this->input_vertices_ = this->input_->getVertices();
}


ModelToInputDistanceCostFunctor::~ModelToInputDistanceCostFunctor()
{
}
