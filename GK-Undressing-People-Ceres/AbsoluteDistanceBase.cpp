#include "AbsoluteDistanceBase.h"



AbsoluteDistanceBase::AbsoluteDistanceBase(SMPLWrapper* smpl, GeneralMesh * toMesh, double pruning_threshold)
    :toMesh_(toMesh), smpl_(smpl), pruning_threshold_(pruning_threshold)
{
    this->set_num_residuals(SMPLWrapper::VERTICES_NUM);
}


AbsoluteDistanceBase::~AbsoluteDistanceBase()
{
}
