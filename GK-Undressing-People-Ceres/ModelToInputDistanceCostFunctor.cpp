#include "pch.h"
#include "ModelToInputDistanceCostFunctor.h"


ModelToInputDistanceCostFunctor::ModelToInputDistanceCostFunctor(SMPLWrapper * smpl, GeneralMesh * input)
    : smpl_(smpl), input_(input), 
    absDist_(new AbsoluteVertsToMeshDistance(input))
        //new ceres::NumericDiffCostFunction<AbsoluteVertsToMeshDistanceFunctor, ceres::CENTRAL, SMPLWrapper::VERTICES_NUM, SMPLWrapper::VERTICES_NUM * SMPLWrapper::SPACE_DIM>(
        //    new AbsoluteVertsToMeshDistanceFunctor(input)))
{
#ifdef DEBUG
    std::cout << "ModelToInputDistanceCost" << std::endl;
#endif // DEBUG
}


ModelToInputDistanceCostFunctor::~ModelToInputDistanceCostFunctor()
{
}

bool AbsoluteVertsToMeshDistanceFunctor::operator()(double const * parameters, double * residuals) const
{
#ifdef DEBUG
 //   std::cout << this->parameter_block_sizes().size() << std::endl;
    
#endif // DEBUG
    std::cout << "Abs dists evaluate" << std::endl;
    const Eigen::MatrixXd vertices = Eigen::Map<const Eigen::MatrixXd>(parameters, SMPLWrapper::VERTICES_NUM, SMPLWrapper::SPACE_DIM);

    Eigen::VectorXd sqrD;
    Eigen::MatrixXd closest_points;
    Eigen::VectorXi closest_face_ids;

#ifdef DEBUG
    std::cout << "Abs dists evaluate: start igl calculation" << std::endl;
#endif // DEBUG

    igl::point_mesh_squared_distance(vertices, this->toMesh_->getVertices(), this->toMesh_->getFaces(), sqrD, closest_face_ids, closest_points);

#ifdef DEBUG
    std::cout << "Abs dists evaluate: Fin igl calculation" << std::endl;
#endif // DEBUG

    assert(sqrD.size() == SMPLWrapper::VERTICES_NUM && "Size of the set of distances should equal main parameters");
    assert(closest_points.size() == SMPLWrapper::VERTICES_NUM && "Size of the set of distances should equal main parameters");

    for (int i = 0; i < SMPLWrapper::VERTICES_NUM; ++i)
        residuals[i] = sqrD(i);

    return true;
}
