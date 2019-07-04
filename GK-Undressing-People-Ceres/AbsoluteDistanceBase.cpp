#include "AbsoluteDistanceBase.h"



AbsoluteDistanceBase::AbsoluteDistanceBase(SMPLWrapper* smpl, GeneralMesh * toMesh)
    :toMesh_(toMesh), smpl_(smpl)
{
    this->set_num_residuals(SMPLWrapper::VERTICES_NUM);
}


AbsoluteDistanceBase::~AbsoluteDistanceBase()
{
}
