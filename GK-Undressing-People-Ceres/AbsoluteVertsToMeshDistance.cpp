#include "pch.h"
#include "AbsoluteVertsToMeshDistance.h"


AbsoluteVertsToMeshDistance::AbsoluteVertsToMeshDistance(SMPLWrapper* smpl, GeneralMesh * toMesh)
    : toMesh_(toMesh), smpl_(smpl)
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
    //const Eigen::MatrixXd vertices = Eigen::Map<const Eigen::MatrixXd>(parameters[0], SMPLWrapper::VERTICES_NUM, SMPLWrapper::SPACE_DIM);

    // shape [1] and pose [2]
    Eigen::MatrixXd shape_jac[SMPLWrapper::SHAPE_SIZE];
    Eigen::MatrixXd verts = this->smpl_->calcModel(nullptr, parameters[1], shape_jac);

    // translate
    for (int i = 0; i < SMPLWrapper::VERTICES_NUM; i++)
    {
        for (int j = 0; j < SMPLWrapper::SPACE_DIM; j++)
        {
            verts(i, j) += parameters[0][j];
        }
    }

    Eigen::VectorXd sqrD;
    Eigen::MatrixXd closest_points;
    Eigen::VectorXi closest_face_ids;

#ifdef DEBUG
    std::cout << "Abs dists evaluate: start igl calculation" << std::endl;
#endif // DEBUG

    igl::point_mesh_squared_distance(verts, this->toMesh_->getVertices(), this->toMesh_->getFaces(), sqrD, closest_face_ids, closest_points);

#ifdef DEBUG
    std::cout << "Abs dists evaluate: Fin igl calculation" << std::endl;
    std::cout << "Dist Sum " << sqrD.sum() << std::endl;
#endif // DEBUG

    assert(sqrD.size() == SMPLWrapper::VERTICES_NUM && "Size of the set of distances should equal main parameters");
    assert(closest_points.size() == SMPLWrapper::VERTICES_NUM && "Size of the set of distances should equal main parameters");

    for (int i = 0; i < SMPLWrapper::VERTICES_NUM; ++i)
    {
        residuals[i] = sqrt(sqrD(i));
    }

    // Jacobians
    // jacobians[i] is a row - major array of size num_residuals x parameter_block_sizes_[i].
        // If jacobians[i] is not NULL, the user is required to compute the Jacobian of the residual vector 
        // with respect to parameters[i] and store it in this array, i.e.
        // jacobians[i][r * parameter_block_sizes_[i] + c] = d residual[r] / d parameters[i][c]
    // Main idea: Gradient for each vertex correspondes to the distance from this vertex to the input mesh.

    // wrt translation
    if (jacobians != NULL && jacobians[0] != NULL) {
        
#ifdef DEBUG
        std::cout << "Abs dists evaluate: jacobian evaluation" << std::endl;
#endif // DEBUG
        //for (int i = 0; i < SMPLWrapper::VERTICES_NUM * SMPLWrapper::VERTICES_NUM * SMPLWrapper::SPACE_DIM; ++i)
        //{
        //    jacobians[0][i] = 0;
        //}
        for (int j = 0; j < SMPLWrapper::VERTICES_NUM; ++j)
        {
            //std::cout << j << std::endl;
            for (int k = 0; k < SMPLWrapper::SPACE_DIM; ++k)
            {
                jacobians[0][j * SMPLWrapper::SPACE_DIM + k]
                    = (verts(j, k) - closest_points(j, k)) / residuals[j];
                //std::cout << jacobians[0][j * SMPLWrapper::SPACE_DIM + k]
                ///   << " ";
            }
            /*double length = (verts(j, 0) - closest_points(j, 0)) * (vertices(j, 0) - closest_points(j, 0))
                + (vertices(j, 1) - closest_points(j, 1)) * (vertices(j, 1) - closest_points(j, 1))
                + (vertices(j, 2) - closest_points(j, 2)) * (vertices(j, 2) - closest_points(j, 2));*/
            //std::cout << length << " ? " << sqrD(j) << std::endl;
            //std::cout << std::endl;
        }

        //for (int i = 0; i < SMPLWrapper::VERTICES_NUM; ++i)
        //{
        //    for (int j = 0; j < SMPLWrapper::VERTICES_NUM * SMPLWrapper::SPACE_DIM; ++j)
        //    {
        //        std::cout << jacobians[0][i * SMPLWrapper::VERTICES_NUM * SMPLWrapper::SPACE_DIM + j]
        //            << " ";
        //    }
        //    std::cout << std::endl;
        //}
    }

    // w.r.t. shape
    if (jacobians != NULL && jacobians[1] != NULL) {
        for (int res_id = 0; res_id < SMPLWrapper::VERTICES_NUM; ++res_id)
        {
            //std::cout << j << std::endl;
            for (int sh_id = 0; sh_id < SMPLWrapper::SHAPE_SIZE; ++sh_id)
            {
                jacobians[1][res_id * SMPLWrapper::SHAPE_SIZE + sh_id] = 0;
                for (int d = 0; d < SMPLWrapper::SPACE_DIM; ++d)
                {
                    // sum derivatives for each coordinate w.r.t. current shape parameter
                    jacobians[1][res_id * SMPLWrapper::SHAPE_SIZE + sh_id]
                        += (verts(res_id, d) - closest_points(res_id, d)) * shape_jac[sh_id](res_id, d);
                }
                jacobians[1][res_id * SMPLWrapper::SHAPE_SIZE + sh_id] /= residuals[res_id];
                //std::cout << jacobians[0][j * SMPLWrapper::SPACE_DIM + k]
                ///   << " ";
            }
            /*double length = (verts(j, 0) - closest_points(j, 0)) * (vertices(j, 0) - closest_points(j, 0))
                + (vertices(j, 1) - closest_points(j, 1)) * (vertices(j, 1) - closest_points(j, 1))
                + (vertices(j, 2) - closest_points(j, 2)) * (vertices(j, 2) - closest_points(j, 2));*/
            //std::cout << length << " ? " << sqrD(j) << std::endl;
            //std::cout << std::endl;
        }
    }

    return true;
}
