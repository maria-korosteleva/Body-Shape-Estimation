#include "pch.h"
#include "AbsoluteVertsToMeshDistance.h"


AbsoluteVertsToMeshDistance::AbsoluteVertsToMeshDistance(GeneralMesh * toMesh)
    : toMesh_(toMesh)
{}


AbsoluteVertsToMeshDistance::~AbsoluteVertsToMeshDistance()
{
}

bool AbsoluteVertsToMeshDistance::Evaluate(double const * const * parameters, double * residuals, double ** jacobians) const
{
    assert(SMPLWrapper::SPACE_DIM == 3 && "Distance evaluation is only implemented in 3D");
    //assert(this->parameter_block_sizes()[0] == SMPLWrapper::VERTICES_NUM * SMPLWrapper::SPACE_DIM && "Input parameter size is set as expected");
    
#ifdef DEBUG
    std::cout << this->parameter_block_sizes().size() << std::endl;
    std::cout << "Abs dists evaluate" << std::endl;
#endif // DEBUG

    //Eigen::Map<const Eigen::Matrix<const double, Eigen::Dynamic, Eigen::Dynamic>> vertices(parameters[0], SMPLWrapper::VERTICES_NUM, SMPLWrapper::SPACE_DIM);
    //Eigen::Map<const Eigen::MatrixXd> vertices(parameters[0], SMPLWrapper::VERTICES_NUM, SMPLWrapper::SPACE_DIM);
    const Eigen::MatrixXd vertices = Eigen::Map<const Eigen::MatrixXd>(parameters[0], SMPLWrapper::VERTICES_NUM, SMPLWrapper::SPACE_DIM);

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

    if (jacobians != NULL && jacobians[0] != NULL) {
        //jacobians[i] is a row - major array of size num_residuals x parameter_block_sizes_[i].
        // If jacobians[i] is not NULL, the user is required to compute the Jacobian of the residual vector 
        // with respect to parameters[i] and store it in this array, i.e.
        // jacobians[i][r * parameter_block_sizes_[i] + c] = d residual[r] / d parameters[i][c]
        // jacobians[0][0] = -1;
        // non-zero elements
        // Gradient for each vertex correspondes to the distance from this vertex to the input mesh. (Heuritics). 
#ifdef DEBUG
        std::cout << "Abs dists evaluate: jacobian evaluation" << std::endl;
#endif // DEBUG
        for (int i = 0; i < SMPLWrapper::VERTICES_NUM; ++i)
        {
            for (int j = 0; i < SMPLWrapper::VERTICES_NUM; ++i)
            {
                if (i == j)
                    for (int k = 0; k < SMPLWrapper::SPACE_DIM; ++k)
                    {
                        jacobians[0][i * SMPLWrapper::VERTICES_NUM * SMPLWrapper::SPACE_DIM + j * SMPLWrapper::SPACE_DIM + k]
                            = 2 * (vertices(j, k) - closest_points(j, k));
                    }
                else
                    for (int k = 0; k < SMPLWrapper::SPACE_DIM; ++k)
                    {
                        jacobians[0][i * SMPLWrapper::VERTICES_NUM * SMPLWrapper::SPACE_DIM + j * SMPLWrapper::SPACE_DIM + k] = 0.;
                    }
                /*std::cout << jacobians[0][i * SMPLWrapper::VERTICES_NUM * SMPLWrapper::SPACE_DIM + j * SMPLWrapper::SPACE_DIM + 0]
                    << " "
                    << jacobians[0][i * SMPLWrapper::VERTICES_NUM * SMPLWrapper::SPACE_DIM + j * SMPLWrapper::SPACE_DIM + 1]
                    << " "
                    << jacobians[0][i * SMPLWrapper::VERTICES_NUM * SMPLWrapper::SPACE_DIM + j * SMPLWrapper::SPACE_DIM + 2]
                    << std::endl;*/
            }
        }
    }

    return true;
}
