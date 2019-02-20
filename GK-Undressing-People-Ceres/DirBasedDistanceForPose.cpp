#include "pch.h"
#include "DirBasedDistanceForPose.h"


DirBasedDistanceForPose::DirBasedDistanceForPose(SMPLWrapper * smpl, GeneralMesh * toMesh)
    : toMesh_(toMesh), smpl_(smpl)
{
    this->key_dirs_num_ = smpl->getKeyDirections().size();

    this->set_num_residuals(this->key_dirs_num_);

    this->mutable_parameter_block_sizes()->push_back(SMPLWrapper::POSE_SIZE);
}


DirBasedDistanceForPose::~DirBasedDistanceForPose()
{
}


bool DirBasedDistanceForPose::Evaluate(double const * const * parameters, double * residuals, double ** jacobians) const
{
    assert(SMPLWrapper::SPACE_DIM == 3 && "Distance evaluation is only implemented in 3D");
    assert(this->parameter_block_sizes()[0] == SMPLWrapper::POSE_SIZE && "Pose parameter size is set as expected");

    // pose the model
    Eigen::MatrixXd pose_jac[SMPLWrapper::POSE_SIZE];
    Eigen::MatrixXd verts;
    //if (jacobians != NULL && jacobians[0] != NULL)
    //{
    //    
    //}
    //else
    //{
    //    verts = this->smpl_->calcModel(parameters[0], nullptr);
    //}

    verts = this->smpl_->calcModel(parameters[0], nullptr, pose_jac, nullptr);

    //std::cout << "Setted num of residuals " << this->num_residuals() << std::endl;

    // calc the cost and jacobians
    CoordsDictionary inputKeyPoints = this->toMesh_->getKeyPoints();
    Dictionary smplKeyVerts = this->smpl_->getKeyVertices();
    int res_id = 0;
    for (auto const& keyDirsIterator : this->smpl_->getKeyDirections())
    {
        //std::cout << "Dir: " << keyDirsIterator.first << " to " << keyDirsIterator.second;

        if (inputKeyPoints.count(keyDirsIterator.first) < 1
            || inputKeyPoints.count(keyDirsIterator.second) < 1
            || smplKeyVerts.count(keyDirsIterator.first) < 1
            || smplKeyVerts.count(keyDirsIterator.second) < 1)
        {
            throw std::exception("Key used in key directions does not exist among key vertices of the model or input");
        }

        Eigen::VectorXd in_dir = 
            inputKeyPoints[keyDirsIterator.first] - inputKeyPoints[keyDirsIterator.second];

        int model_v_1 = smplKeyVerts[keyDirsIterator.first];
        int model_v_2 = smplKeyVerts[keyDirsIterator.second];
        Eigen::VectorXd model_dir = verts.row(model_v_1) - verts.row(model_v_2);

        //std::cout << std::endl << model_dir << std::endl;

        Eigen::VectorXd diff = model_dir / model_dir.norm() - in_dir / in_dir.norm();
        residuals[res_id] = diff.dot(diff);

        //std::cout << "Residual " << residuals[res_id] << std::endl;

        // jacobian w.r.t. pose
        if (jacobians != NULL && jacobians[0] != NULL)
        {
            for (int p_id = 0; p_id < SMPLWrapper::POSE_SIZE; ++p_id)
            {
                jacobians[0][res_id * SMPLWrapper::POSE_SIZE + p_id] =
                    2. * diff.dot((pose_jac[p_id].row(model_v_1) - pose_jac[p_id].row(model_v_2)) / model_dir.norm());
                //std::cout << jacobians[0][res_id * SMPLWrapper::POSE_SIZE + p_id] << ", ";
            }
            //std::cout << std::endl;
        }

        res_id++;
    }
    //std::cout << res_id << std::endl;

    return true;
}
