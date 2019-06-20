#include "SMPLWrapper.h"


SMPLWrapper::SMPLWrapper(char gender, const std::string path)
{
    // set the info
    if (gender != 'f' && gender != 'm') 
    {
        std::string message("Wrong gender supplied: ");
        message += gender;
        throw std::exception(message.c_str());
    }
    this->gender_ = gender;

    // !!!! expects a pre-defined file structure
    general_path_ = path + "/";
    gender_path_ = general_path_ + gender + "_smpl/";

    readTemplate_();
    readJointMat_();
    readJointNames_();
    readShapes_();
    readWeights_();
    readHierarchy_();
    readKeyVertices_();
    readKeyDirections_();

    mean_point_template_ = verts_template_.colwise().mean();
    joint_locations_template_ = calcJointLocations(nullptr, nullptr);
}

SMPLWrapper::~SMPLWrapper()
{
}

void SMPLWrapper::rotateJointToDirection(const std::string joint_name, E::Vector3d direction)
{
    assert(SMPLWrapper::SPACE_DIM == 3 && "rotateJointToDirection() can only be used in 3D world");

    std::cout << "Setting Bone Direction for joint " << joint_name << std::endl
        << "To direction \n" << direction << std::endl;
    
    // find id
    int joint_id;
    try { joint_id = joint_names_.at(joint_name); }
    catch (std::out_of_range& e)
    {
        std::cout << "SMPLWRapper::setBoneConnection was supplied with unknown joint" << std::endl;
        throw e;
    }

    // find child
    int child_id;
    for (int i = 0; i < JOINTS_NUM; i++)
    {
        if (joints_parents_[i] == joint_id)     
            child_id = i;
    }

    // get default bone direction
    E::Vector3d default_dir =
        (joint_locations_template_.row(child_id) - joint_locations_template_.row(joint_id)).transpose();

    std::cout << "Default direction \n" << default_dir << std::endl;

    // control for zero vectors
    if (direction.norm() * default_dir.norm() > 0.0)
    {
        E::Vector3d axis = default_dir.cross(direction);
        double angle = asin(axis.norm() / (direction.norm() * default_dir.norm()));
        axis.normalize();
        axis = angle * axis;

        std::cout << "Angle-axis rotation \n" << axis << std::endl;

        for (int i = 0; i < SPACE_DIM; i++)
        {
            state_.pose[joint_id * SPACE_DIM + i] = axis(i);
        }
    }
}

E::MatrixXd SMPLWrapper::calcModel(const double * const pose, const double * const shape, E::MatrixXd * pose_jac, E::MatrixXd * shape_jac) const
{
    // assignment won't work without cast
    E::MatrixXd verts = this->verts_template_;
#ifdef DEBUG
    std::cout << "Calc model" << std::endl;
#endif // DEBUG

    if (shape != nullptr)
    {
        this->shapeSMPL_(shape, verts, shape_jac);
    }

    if (pose != nullptr)
    {
        this->poseSMPL_(pose, verts, pose_jac);

        if (shape_jac != nullptr)
        {
            // WARNING! This will significantly increse iteration time
            for (int i = 0; i < SMPLWrapper::SHAPE_SIZE; ++i)
            {
                // TODO: add the use of pre-computed LBS Matrices 
                // TODO: test
                this->poseSMPL_(pose, shape_jac[i]);
            }
        }
#ifdef DEBUG
        std::cout << "Fin posing " << verts.rows() << " x " << verts.cols() << std::endl;
#endif // DEBUG
    }

#ifdef DEBUG
    std::cout << "Fin calculating" << std::endl;
    //if (std::is_same_v<T, double>)
    //{
    //    for (int i = 0; i < SMPLWrapper::VERTICES_NUM; i++)
    //    {
    //        for (int j = 0; j < SMPLWrapper::SPACE_DIM; j++)
    //        {
    //            std::cout << verts(i, j) << " ";
    //        }
    //        std::cout << std::endl;
    //    }
    //}
#endif // DEBUG

    return verts;
}

E::MatrixXd SMPLWrapper::calcModel(E::MatrixXd * pose_jac, E::MatrixXd * shape_jac) const
{
    return calcModel(state_.pose, state_.shape, pose_jac, shape_jac);
}

E::MatrixXd SMPLWrapper::calcJointLocations(const double * shape, const double * pose = nullptr)  const
{
    E::MatrixXd verts = this->calcModel(nullptr, shape);

    E::MatrixXd baseJointLocations = this->jointRegressorMat_ * verts;

    if (pose == nullptr)
    {
        return baseJointLocations;
    }

    E::MatrixXd posedJointLocations(SMPLWrapper::JOINTS_NUM, SMPLWrapper::SPACE_DIM);
    this->getJointsTransposedGlobalTransformation_(pose, baseJointLocations, nullptr, &posedJointLocations);

    return posedJointLocations;
}

E::MatrixXd SMPLWrapper::calcJointLocations() const
{
    return calcJointLocations(state_.shape, state_.pose);
}

void SMPLWrapper::saveToObj(const double* translation, const double* pose, const double* shape, const std::string path) const
{
    E::MatrixXd verts = this->calcModel(pose, shape);
    
    if (translation != nullptr)
    {
        for (int i = 0; i < SMPLWrapper::VERTICES_NUM; i++)
        {
            for (int j = 0; j < SMPLWrapper::SPACE_DIM; ++j)
            {
                verts(i, j) += translation[j];
            }
        }
    }

    igl::writeOBJ(path, verts, this->faces_);
}

void SMPLWrapper::saveToObj(const std::string path) const
{
    saveToObj(state_.translation, state_.pose, state_.shape, path);
}

void SMPLWrapper::savePosedOnlyToObj(const std::string path) const
{
    saveToObj(state_.translation, state_.pose, nullptr, path);
}

void SMPLWrapper::saveShapedOnlyToObj(const std::string path) const
{
    saveToObj(state_.translation, nullptr, state_.shape, path);
}

void SMPLWrapper::logParameters(const std::string path) const
{
    std::ofstream out(path);

    out << "Translation \n[ ";
    for (int i = 0; i < SMPLWrapper::SPACE_DIM; i++)
        out << state_.translation[i] << " , ";
    out << "]" << std::endl;

    out << std::endl << "Pose params [ \n";
    for (int i = 0; i < SMPLWrapper::JOINTS_NUM; i++)
    {
        for (int j = 0; j < SMPLWrapper::SPACE_DIM; j++)
        {
            out << state_.pose[i * SMPLWrapper::SPACE_DIM + j] << " , ";
        }
        out << std::endl;
    }
    out << "]" << std::endl;

    out << std::endl << "Shape (betas) params [ \n";
    for (int i = 0; i < SMPLWrapper::SHAPE_SIZE; i++)
        out << state_.shape[i] << " , ";
    out << std::endl << "]" << std::endl;

    out << std::endl << "Joints locations for posed and shaped model [\n";
    Eigen::MatrixXd translatedJointLoc(calcJointLocations());
    // translate
    for (int i = 0; i < translatedJointLoc.rows(); ++i)
    {
        for (int j = 0; j < SMPLWrapper::SPACE_DIM; ++j)
        {
            translatedJointLoc(i, j) += state_.translation[j];
        }
    }
    out << translatedJointLoc << std::endl;
    out << "]" << std::endl;

    out.close();
}

void SMPLWrapper::readTemplate_()
{
    std::string file_name(this->gender_path_);
    file_name += this->gender_;
    file_name += "_shapeAv.obj";

    bool success = igl::readOBJ(file_name, this->verts_template_, this->faces_);
    if (!success)
    {
        std::string message("Abort: Could not read SMPL template at ");
        message += file_name;
        throw std::exception(message.c_str());
    }
}

void SMPLWrapper::readJointMat_()
{
    std::string file_name(this->gender_path_);
    file_name += this->gender_;
    file_name += "_joints_mat.txt";

    // copy from Meekyong code example
    std::fstream inFile;
    inFile.open(file_name, std::ios_base::in);
    int joints_n, verts_n;
    inFile >> joints_n;
    inFile >> verts_n;
    // Sanity check
    if (joints_n != SMPLWrapper::JOINTS_NUM || verts_n != SMPLWrapper::VERTICES_NUM)
        throw std::exception("Joint matrix info (number of joints and vertices) is incompatible with the model");

    this->jointRegressorMat_.resize(joints_n, verts_n);
    for (int i = 0; i < joints_n; i++)
        for (int j = 0; j < verts_n; j++)
            inFile >> this->jointRegressorMat_(i, j);

    inFile.close();
}

void SMPLWrapper::readJointNames_()
{
    std::string file_name(this->general_path_);
    file_name += "joint_names.txt";

    std::fstream inFile;
    inFile.open(file_name, std::ios_base::in);
    int joints_n;
    inFile >> joints_n;
    // Sanity check
    if (joints_n != JOINTS_NUM)
    {
        throw std::exception("Number of joint names specified doesn't match current SMPLWrapper settings");
    }

    std::string joint_name;
    int jointId;
    for (int i = 0; i < joints_n; i++)
    {
        inFile >> joint_name;
        inFile >> jointId;
        joint_names_.insert(DictEntryInt(joint_name, jointId));
    }

    inFile.close();
}

void SMPLWrapper::readShapes_()
{
    std::string file_path(this->gender_path_);
    file_path += this->gender_;
    file_path += "_blendshape/shape";

    Eigen::MatrixXi fakeFaces(SMPLWrapper::VERTICES_NUM, SMPLWrapper::SPACE_DIM);

    for (int i = 0; i < SMPLWrapper::SHAPE_SIZE; i++)
    {
        std::string file_name(file_path);
        file_name += std::to_string(i);
        file_name += ".obj";

        igl::readOBJ(file_name, this->shape_diffs_[i], fakeFaces);

        this->shape_diffs_[i] -= this->verts_template_;
    }
}

void SMPLWrapper::readWeights_()
{
    std::string file_name(this->gender_path_);
    file_name += this->gender_;
    file_name += "_weight.txt";

    std::fstream inFile;
    inFile.open(file_name, std::ios_base::in);
    int joints_n, verts_n;
    inFile >> joints_n;
    inFile >> verts_n;
    // Sanity check
    if (joints_n != SMPLWrapper::JOINTS_NUM || verts_n != SMPLWrapper::VERTICES_NUM)
        throw std::exception("Weights info (number of joints and vertices) is incompatible with the model");

    std::vector<E::Triplet<double>> tripletList;
    tripletList.reserve(verts_n * SMPLWrapper::WEIGHTS_BY_VERTEX);     // for faster filling performance
    double tmp;
    for (int i = 0; i < verts_n; i++)
    {
        for (int j = 0; j < joints_n; j++)
        {
            inFile >> tmp;
            if (tmp > 0.00001)  // non-zero weight
                tripletList.push_back(E::Triplet<double>(i, j, tmp));
        }
    }
    this->weights_.resize(verts_n, joints_n);
    this->weights_.setFromTriplets(tripletList.begin(), tripletList.end());

#ifdef DEBUG
    std::cout << "Weight sizes " << this->weights_.outerSize() << " " << this->weights_.innerSize() << std::endl;
#endif // DEBUG

    inFile.close();
}

void SMPLWrapper::readHierarchy_()
{
    std::string file_name(this->general_path_);
    file_name += "jointsHierarchy.txt";

    std::fstream inFile;
    inFile.open(file_name, std::ios_base::in);
    int joints_n;
    inFile >> joints_n;
    // Sanity check
    if (joints_n != SMPLWrapper::JOINTS_NUM)
    {
        throw std::exception("Number of joints in joints hierarchy info is incompatible with the model");
    }
    
    int tmpId;
    for (int j = 0; j < joints_n; j++)
    {
        inFile >> tmpId;
        inFile >> joints_parents_[tmpId];
    }

    inFile.close();
}

void SMPLWrapper::readKeyVertices_()
{
    std::string file_name(this->general_path_);
    file_name += "key_vertices.txt";

    std::fstream inFile;
    inFile.open(file_name, std::ios_base::in);
    int keys_n;
    inFile >> keys_n;
    // Sanity check
    if (keys_n <= 0)
    {
        throw std::exception("Number of key vertices should be a positive number!");
    }

    std::string key_name;
    int vertexId;
    for (int i = 0; i < keys_n; i++)
    {
        inFile >> key_name;
        inFile >> vertexId;
        this->key_vertices_.insert(DictEntryInt(key_name, vertexId));
    }

    inFile.close();
}

void SMPLWrapper::readKeyDirections_()
{
    std::string file_name(this->general_path_);
    file_name += "key_directions.txt";

    std::fstream inFile;
    inFile.open(file_name, std::ios_base::in);
    int dirs_n;
    inFile >> dirs_n;
    // Sanity check
    if (dirs_n <= 0)
    {
        throw std::exception("Number of key directions should be a positive number!");
    }

    std::string key_1;
    std::string key_2;
    for (int i = 0; i < dirs_n; i++)
    {
        inFile >> key_1;
        inFile >> key_2;
        this->key_directions_.push_back(DirPair(key_1, key_2));
    }

    inFile.close();
}

void SMPLWrapper::shapeSMPL_(const double * const shape, E::MatrixXd &verts, E::MatrixXd* shape_jac) const
{
#ifdef DEBUG
    std::cout << "shape (analytic)" << std::endl;
#endif // DEBUG
    for (int i = 0; i < this->SHAPE_SIZE; i++)
    {
        verts += shape[i] * this->shape_diffs_[i];
    }

    if (shape_jac != nullptr)
    {
        for (int i = 0; i < this->SHAPE_SIZE; i++)
        {
            shape_jac[i] = this->shape_diffs_[i];
        }
    }
}

void SMPLWrapper::poseSMPL_(const double * const pose, E::MatrixXd & verts, E::MatrixXd * pose_jac) const
{
#ifdef DEBUG
    std::cout << "pose (analytic)" << std::endl;
#endif // DEBUG

    E::MatrixXd jointLocations = this->jointRegressorMat_ * verts;
    E::MatrixXd jointsTransformation;

    jointsTransformation = this->getJointsTransposedGlobalTransformation_(pose, jointLocations, pose_jac);

    E::SparseMatrix<double> LBSMat = this->getLBSMatrix_(verts);

    verts = LBSMat * jointsTransformation;

    if (pose_jac != nullptr)
    {
        // pose_jac[i] at this point has the same structure as jointsTransformation
        for (int i = 0; i < SMPLWrapper::POSE_SIZE; ++i)
        {
            pose_jac[i].applyOnTheLeft(LBSMat);
        }
    }
}

E::MatrixXd SMPLWrapper::getJointsTransposedGlobalTransformation_(
    const double * const pose,
    E::MatrixXd & jointLocations,
    E::MatrixXd * jacsTotal, 
    E::MatrixXd * finJointLocations
) const
{
#ifdef DEBUG
    std::cout << "global transform (analytic)" << std::endl;
#endif // DEBUG

    // uses functions that assume input in 3D (see below)
    assert(SMPLWrapper::SPACE_DIM == 3 && "The function can only be used in 3D world");

    static constexpr int HOMO_SIZE = SMPLWrapper::SPACE_DIM + 1;
    E::Matrix<double, HOMO_SIZE, HOMO_SIZE> forwardKinematicsTransforms[SMPLWrapper::JOINTS_NUM];
    // for jacobian calculations, each entry is 4x4 or empty. FK=Forward Kinematics
    E::MatrixXd FKDerivatives[SMPLWrapper::JOINTS_NUM][SMPLWrapper::POSE_SIZE];
    // Stacked (transposed) global transformation matrices for points
    E::MatrixXd pointTransformTotal(HOMO_SIZE * SMPLWrapper::JOINTS_NUM, SMPLWrapper::SPACE_DIM);

    // root 
    if (jacsTotal != nullptr)
    {
        for (int i = 0; i < SMPLWrapper::POSE_SIZE; ++i)
        {
            jacsTotal[i].setZero(HOMO_SIZE * SMPLWrapper::JOINTS_NUM, SMPLWrapper::SPACE_DIM);
        }

        forwardKinematicsTransforms[0] = this->get3DLocalTransformMat_(pose, jointLocations.row(0), FKDerivatives[0]);

        // fill derivatives w.r.t. coordinates of rotation of the first joint  in the exponential coordinates
        E::MatrixXd tmpPointGlobalJac;
        for (int i = 0; i < SMPLWrapper::SPACE_DIM; ++i)
        {
            tmpPointGlobalJac = FKDerivatives[0][i] * this->get3DTranslationMat_(-jointLocations.row(0));
            jacsTotal[i].block(0, 0, HOMO_SIZE, SMPLWrapper::SPACE_DIM) = tmpPointGlobalJac.transpose().leftCols(SMPLWrapper::SPACE_DIM);
        }
    }
    else
    {
        forwardKinematicsTransforms[0] = this->get3DLocalTransformMat_(pose, jointLocations.row(0));
    }

    if (finJointLocations != nullptr)
    {
        for (int i = 0; i < SMPLWrapper::SPACE_DIM; ++i)
        {
            (*finJointLocations)(0, i) = forwardKinematicsTransforms[0](i, SMPLWrapper::SPACE_DIM);
        }
    }

    E::MatrixXd tmpPointGlobalTransform = forwardKinematicsTransforms[0] * this->get3DTranslationMat_(-jointLocations.row(0));
    pointTransformTotal.block(0, 0, HOMO_SIZE, SMPLWrapper::SPACE_DIM)
        = tmpPointGlobalTransform.transpose().leftCols(SMPLWrapper::SPACE_DIM);

    // the rest of the joints
    for (int joint_id = 1; joint_id < SMPLWrapper::JOINTS_NUM; joint_id++)
    {
        if (jacsTotal == nullptr)
        {
            // Forward Kinematics Formula
            forwardKinematicsTransforms[joint_id] = forwardKinematicsTransforms[this->joints_parents_[joint_id]]
                * this->get3DLocalTransformMat_((pose + joint_id * 3),
                    jointLocations.row(joint_id) - jointLocations.row(this->joints_parents_[joint_id]));

            // collect transform for final matrix
            tmpPointGlobalTransform = forwardKinematicsTransforms[joint_id] * this->get3DTranslationMat_(-jointLocations.row(joint_id));
            pointTransformTotal.block(joint_id * HOMO_SIZE, 0, HOMO_SIZE, SMPLWrapper::SPACE_DIM)
                = tmpPointGlobalTransform.transpose().leftCols(SMPLWrapper::SPACE_DIM);
        }
        else // calc jacobian
        {
            E::MatrixXd localTransform;
            E::MatrixXd localTransformJac[SMPLWrapper::SPACE_DIM];

            localTransform = this->get3DLocalTransformMat_((pose + joint_id * SMPLWrapper::SPACE_DIM),
                jointLocations.row(joint_id) - jointLocations.row(this->joints_parents_[joint_id]),
                localTransformJac); // ask to calc jacobian

            // Forward Kinematics Formula
            forwardKinematicsTransforms[joint_id] = forwardKinematicsTransforms[this->joints_parents_[joint_id]] * localTransform;
            // collect transform for final matrix
            tmpPointGlobalTransform = forwardKinematicsTransforms[joint_id] * this->get3DTranslationMat_(-jointLocations.row(joint_id));
            pointTransformTotal.block(joint_id * HOMO_SIZE, 0, HOMO_SIZE, SMPLWrapper::SPACE_DIM)
                = tmpPointGlobalTransform.transpose().leftCols(SMPLWrapper::SPACE_DIM);

            E::MatrixXd tmpPointGlobalJac;
            // jac w.r.t current joint rot coordinates
            for (int j = 0; j < SMPLWrapper::SPACE_DIM; ++j)
            {
                FKDerivatives[joint_id][joint_id * SMPLWrapper::SPACE_DIM + j] = forwardKinematicsTransforms[this->joints_parents_[joint_id]] * localTransformJac[j];

                tmpPointGlobalJac
                    = FKDerivatives[joint_id][joint_id * SMPLWrapper::SPACE_DIM + j] * this->get3DTranslationMat_(-jointLocations.row(joint_id));

                jacsTotal[joint_id * SMPLWrapper::SPACE_DIM + j].block(joint_id * HOMO_SIZE, 0, HOMO_SIZE, SMPLWrapper::SPACE_DIM)
                    = tmpPointGlobalJac.transpose().leftCols(SMPLWrapper::SPACE_DIM);
            }

            // jac w.r.t. ancessors rotation coordinates         
            for (int j = 0; j < (this->joints_parents_[joint_id] + 1) * SMPLWrapper::SPACE_DIM; ++j)
            {
                if (FKDerivatives[this->joints_parents_[joint_id]][j].size() > 0)
                {
                    FKDerivatives[joint_id][j] = FKDerivatives[this->joints_parents_[joint_id]][j] * localTransform;

                    tmpPointGlobalJac = FKDerivatives[joint_id][j] * this->get3DTranslationMat_(-jointLocations.row(joint_id));
                    jacsTotal[j].block(joint_id * HOMO_SIZE, 0, HOMO_SIZE, SMPLWrapper::SPACE_DIM)
                        = tmpPointGlobalJac.transpose().leftCols(SMPLWrapper::SPACE_DIM);
                }
            }
        }

        if (finJointLocations != nullptr)
        {
            for (int i = 0; i < SMPLWrapper::SPACE_DIM; ++i)
            {
                (*finJointLocations)(joint_id, i) = forwardKinematicsTransforms[joint_id](i, SMPLWrapper::SPACE_DIM);
            }
        }

    }

    return pointTransformTotal;
}

E::MatrixXd SMPLWrapper::get3DLocalTransformMat_(const double * const jointAxisAngleRotation, const E::MatrixXd & jointToParentDist, E::MatrixXd* localTransformJac) const
{
    // init
    E::MatrixXd localTransform;
    localTransform.setIdentity(4, 4);   // in homogenious coordinates
    localTransform.block(0, 3, 3, 1) = jointToParentDist.transpose(); // g(0)

    if (localTransformJac != nullptr)
    {
        for (int i = 0; i < 3; ++i)
        {
            localTransformJac[i].setZero(4, 4);
            localTransformJac[i].block(0, 3, 3, 1) = jointToParentDist.transpose();   // For the default pose transformation
        }
    }

    // prepare the info
    E::Vector3d w = E::Map<const E::Vector3d>(jointAxisAngleRotation);
    double norm = w.norm();
    E::Matrix3d w_skew;
    w_skew <<
        0, -w[2], w[1],
        w[2], 0, -w[0],
        -w[1], w[0], 0;
    w_skew /= norm;
    E::Matrix3d exponent = E::Matrix3d::Identity();

    // calculate
    if (norm > 0.0001)  // don't waste computations on zero joint movement
    {
        // apply Rodrigues formula
        exponent += w_skew * sin(norm) + w_skew * w_skew * (1. - cos(norm));
        localTransform.block(0, 0, 3, 3) = exponent;
    }
    
    // jacobian
    if (localTransformJac != nullptr)
    {
        if (norm > 0.0001)
        {
            for (int i = 0; i < 3; ++i)
            {
                // compact formula from https://arxiv.org/pdf/1312.0788.pdf
                E::Vector3d cross = w.cross((E::Matrix3d::Identity() - exponent).col(i)) / (norm * norm);
                E::Matrix3d cross_skew;
                cross_skew <<
                    0, -cross[2], cross[1],
                    cross[2], 0, -cross[0],
                    -cross[1], cross[0], 0;

                localTransformJac[i].block(0, 0, 3, 3) = 
                    (w_skew * w[i] / norm + cross_skew) * exponent;
            }
        }
        else // zero case 
        {
            localTransformJac[0].block(0, 0, 3, 3) <<
                0, 0, 0,
                0, 0, -1,
                0, 1, 0;
            localTransformJac[1].block(0, 0, 3, 3) <<
                0, 0, 1,
                0, 0, 0,
                -1, 0, 0;
            localTransformJac[2].block(0, 0, 3, 3) <<
                0, -1, 0,
                1, 0, 0,
                0, 0, 0;
        }
    }

    return localTransform;
}

E::MatrixXd SMPLWrapper::get3DTranslationMat_(const E::MatrixXd & translationVector) const
{
    E::MatrixXd translation;
    translation.setIdentity(4, 4);  // in homogenious coordinates
    translation.block(0, 3, 3, 1) = translationVector.transpose();

    return translation;
}

E::SparseMatrix<double> SMPLWrapper::getLBSMatrix_(E::MatrixXd & verts) const
{
    const int dim = SMPLWrapper::SPACE_DIM;
    const int nVerts = SMPLWrapper::VERTICES_NUM;
    const int nJoints = SMPLWrapper::JOINTS_NUM;  // Number of joints
#ifdef DEBUG
    std::cout << "LBSMat: start (analytic)" << std::endl;
#endif // DEBUG
    // +1 goes for homogenious coordinates
    E::SparseMatrix<double> LBSMat(nVerts, (dim + 1) * nJoints);
    std::vector<E::Triplet<double>> LBSTripletList;
    LBSTripletList.reserve(nVerts * (dim + 1) * SMPLWrapper::WEIGHTS_BY_VERTEX);     // for faster filling performance

    // go over non-zero weight elements
    for (int k = 0; k < this->weights_.outerSize(); ++k)
    {
        for (E::SparseMatrix<double>::InnerIterator it(this->weights_, k); it; ++it)
        {
            double weight = it.value();
            int idx_vert = it.row();
            int idx_joint = it.col();
            // premultiply weigths by vertex homogenious coordinates
            for (int idx_dim = 0; idx_dim < dim; idx_dim++)
                LBSTripletList.push_back(E::Triplet<double>(idx_vert, idx_joint * (dim + 1) + idx_dim, weight * verts(idx_vert, idx_dim)));
            LBSTripletList.push_back(E::Triplet<double>(idx_vert, idx_joint * (dim + 1) + dim, weight));
        }
    }
    LBSMat.setFromTriplets(LBSTripletList.begin(), LBSTripletList.end());

    return LBSMat;
}

SMPLWrapper::State::State()
{
    pose = new double[POSE_SIZE];
    for (int i = 0; i < POSE_SIZE; i++)
        pose[i] = 0.;

    shape = new double[SHAPE_SIZE];
    for (int i = 0; i < SHAPE_SIZE; i++)
        shape[i] = 0.;

    translation = new double[SPACE_DIM];
    for (int i = 0; i < SPACE_DIM; i++)
        translation[i] = 0.;
}

SMPLWrapper::State::~State()
{
    delete[] pose;
    delete[] shape;
    delete[] translation;
}
