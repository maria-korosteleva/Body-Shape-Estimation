#include "SMPLWrapper.h"

int SMPLWrapper::joints_parents_[JOINTS_NUM];

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

    joint_locations_template_ = calcJointLocations();
}

SMPLWrapper::~SMPLWrapper()
{
}

void SMPLWrapper::rotateLimbToDirection(const std::string joint_name, const E::Vector3d& direction)
{
    assert(SMPLWrapper::SPACE_DIM == 3 && "rotateLimbToDirection() can only be used in 3D world");

    std::cout << "----- Setting Bone Direction for joint " << joint_name << " -----" << std::endl
        << "To direction \n" << direction << std::endl;
    
    int joint_id;
    try { 
        joint_id = joint_names_.at(joint_name); 
        if (joint_id == 0) // root
            throw "SMPLWrapper::ERROR::use specialized method to set up root";
        if (joint_name == "LowBack" || joint_name == "MiddleBack" || joint_name == "TopBack")
            throw "SMPLWrapper::ERROR::use specialized method to set up back twist";
    }
    catch (std::out_of_range& e)
    {
        std::cout << "SMPLWRapper::rotateLimbToDirection was supplied with incorrect joint" << std::endl;
        throw e;
    }

    int child_id;
    for (int i = 0; i < JOINTS_NUM; i++)
    {
        if (joints_parents_[i] == joint_id)
        {
            child_id = i;
            break;
        }
    }
    
    std::cout << "SMPL Joint pair " << joint_id << " -> " << child_id << std::endl;

    // get default bone direction; it also updates joint_global_transform_
    E::MatrixXd joint_locations = calcJointLocations_(nullptr, nullptr, state_.pose);
    E::Vector3d default_dir =
        (joint_locations.row(child_id) - joint_locations.row(joint_id)).transpose();

    std::cout << "Default direction \n" << default_dir << std::endl;

    // skip calculations for zero vectors
    if (direction.norm() * default_dir.norm() > 0.0)
    {
        E::Vector3d axis = angle_axis_(default_dir, direction);
        assignJointGlobalRotation_(joint_id, axis, fk_transforms_);
    }

    // sanity check
    // TODO add efficiency flag
    E::MatrixXd new_joint_locations = calcJointLocations_(nullptr, nullptr, state_.pose);
    E::Vector3d new_dir = (new_joint_locations.row(child_id) - new_joint_locations.row(joint_id)).transpose();
    std::cout << "Difference with the target " << std::endl
        << new_dir.normalized() - direction.normalized() << std::endl;

    E::Vector3d fact_axis = angle_axis_(default_dir, new_dir);
    std::cout << "Fact Turned Angle: " << fact_axis.norm() * 180 / 3.1415 << std::endl;
}

void SMPLWrapper::rotateRoot(const E::Vector3d& body_up, const E::Vector3d& body_right_to_left)
{
    assert(SMPLWrapper::SPACE_DIM == 3 && "rotateRoot() can only be used in 3D world");

    std::cout << "----- Setting Root Rotation -----" << std::endl
        << "With UP (Y) to \n" << body_up << std::endl
        << "With Right (X) to \n" << body_right_to_left << std::endl;

    E::Vector3d default_Y(0., 1., 0.);
    E::Vector3d rotation_match_body_up = angle_axis_(default_Y, body_up);

    std::cout << "Vertical Angle-axis rotation with angle_for_up " << rotation_match_body_up.norm() * 180 / 3.1415
        << "\n" << rotation_match_body_up << std::endl;
    
    E::Vector3d X_updated = rotate_by_angle_axis_(E::Vector3d(1., 0., 0), rotation_match_body_up);
    E::Vector3d Y_matched = rotate_by_angle_axis_(default_Y, rotation_match_body_up);
    
    // use rotation around Y + projection instead of the full vector
    // to gurantee the Y stays where it is and hips are matched as close as possible
    E::Vector3d right_to_left_projected = body_right_to_left - body_right_to_left.dot(Y_matched) * Y_matched;
    double angle = atan2(
        X_updated.cross(right_to_left_projected).norm(), 
        X_updated.dot(right_to_left_projected));

    E::Vector3d rotation_match_hips = angle * Y_matched;

    E::Vector3d combined_rotation = combine_two_angle_axis_(rotation_match_body_up, rotation_match_hips);

    std::cout << "Combined rotation with angle " << combined_rotation.norm() * 180 / 3.1415
        << "\n" << combined_rotation << std::endl;

    assignJointGlobalRotation_(0, combined_rotation, fk_transforms_);
}

void SMPLWrapper::twistBack(const E::Vector3d& shoulder_dir)
{
    assert(SMPLWrapper::SPACE_DIM == 3 && "rotateRoot() can only be used in 3D world");

    std::cout << "----- Setting shoulders -----" << std::endl
        << "To direction \n" << shoulder_dir << std::endl;

    // get default bone direction; it also updates joint_global_transform_
    E::MatrixXd joint_locations = calcJointLocations_(nullptr, nullptr, state_.pose);

    int Rshoulder_id = joint_names_.at("RShoulder");
    int Lshoulder_id = joint_names_.at("LShoulder");

    E::Vector3d default_dir =
        (joint_locations.row(Lshoulder_id) - joint_locations.row(Rshoulder_id)).transpose();

    std::cout << "Default direction \n" << default_dir << std::endl;

    E::Vector3d rotation = angle_axis_(default_dir, shoulder_dir);
    double angle = rotation.norm();
    E::Vector3d axis = rotation.normalized();

    std::cout << "Angle-axis rotation with angle " << angle * 180 / 3.1415
        << "\n" << rotation << std::endl;

    // divide between the back joints
    assignJointGlobalRotation_(joint_names_.at("LowBack"), axis * angle / 3, fk_transforms_);

    updateJointsFKTransforms_(state_.pose, joint_locations_template_);
    assignJointGlobalRotation_(joint_names_.at("MiddleBack"), axis * angle / 3, fk_transforms_);

    updateJointsFKTransforms_(state_.pose, joint_locations_template_);
    assignJointGlobalRotation_(joint_names_.at("TopBack"), axis * angle / 3, fk_transforms_);
}

void SMPLWrapper::translateTo(const E::VectorXd & center_point)
{
    E::MatrixXd verts = calcModel();
    E::VectorXd mean_point = verts.colwise().mean();

    for (int i = 0; i < SPACE_DIM; i++)
    {
        state_.translation[i] = center_point(i) - mean_point(i);
    }
}

E::MatrixXd SMPLWrapper::calcModel(const double * const translation, const double * const pose, const double * const shape,
    const ERMatrixXd * displacement,
    E::MatrixXd * pose_jac, E::MatrixXd * shape_jac, E::MatrixXd * displacement_jac)
{
    // assignment won't work without cast
    E::MatrixXd verts = verts_template_normalized_;

    if (displacement != nullptr)
    {
        verts = verts + *displacement;  // should be able to combine row-major and col-major automatically
        if (displacement_jac != nullptr)
        {
            for (int axis = 0; axis < SPACE_DIM; axis++)
            {
                displacement_jac[axis] = E::MatrixXd::Zero(VERTICES_NUM, SPACE_DIM);
                displacement_jac[axis].col(axis).setOnes();
            }
        }
    }

    if (shape != nullptr)
        shapeSMPL_(shape, verts, shape_jac);

    if (pose != nullptr)
    {
        poseSMPL_(pose, verts, pose_jac);

        // should be updated for the given pose
        // TODO: use pre-calculated pose matrix?
        if (shape_jac != nullptr)
            for (int i = 0; i < SMPLWrapper::SHAPE_SIZE; ++i)
                this->poseSMPL_(pose, shape_jac[i]); // TODO: add the use of pre-computed LBS Matrices 
        if (displacement_jac != nullptr)
            for (int axis = 0; axis < SPACE_DIM; axis++)
            {
                this->poseSMPL_(pose, displacement_jac[axis], nullptr, true, false);
            }
    }

    if (translation != nullptr)
        translate_(translation, verts);

    return verts;
}

E::MatrixXd SMPLWrapper::calcVertexNormals(const E::MatrixXd * verts)
{
    E::MatrixXd normals;
    igl::per_vertex_normals(*verts, faces_, normals);

    return normals;
}

E::MatrixXd SMPLWrapper::calcModel()
{
    return calcModel(state_.translation, state_.pose, state_.shape, &state_.displacements);
}

E::MatrixXd SMPLWrapper::calcJointLocations()
{
    return calcJointLocations_(state_.translation, state_.shape, state_.pose);
}

void SMPLWrapper::saveToObj_(const double* translation, const double* pose, const double* shape, 
    const ERMatrixXd* displacements, const std::string path)
{
    E::MatrixXd verts = calcModel(translation, pose, shape, displacements);

    igl::writeOBJ(path, verts, faces_);
}

void SMPLWrapper::saveToObj(const std::string path) 
{
    saveToObj_(state_.translation, state_.pose, state_.shape, nullptr, path);
}

void SMPLWrapper::saveWithDisplacementToObj(const std::string path)
{
    saveToObj_(state_.translation, state_.pose, state_.shape, &state_.displacements, path);
}

void SMPLWrapper::savePosedOnlyToObj(const std::string path) 
{
    saveToObj_(state_.translation, state_.pose, nullptr, nullptr, path);
}

void SMPLWrapper::saveShapedOnlyToObj(const std::string path) 
{
    saveToObj_(state_.translation, nullptr, state_.shape, nullptr, path);
}

void SMPLWrapper::saveShapedWithDisplacementToObj(const std::string path)
{
    saveToObj_(state_.translation, nullptr, state_.shape, &state_.displacements, path);
}

void SMPLWrapper::logParameters(const std::string path)
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

    out << calcJointLocations() << std::endl;

    out << "]" << std::endl;

    out.close();
}

void SMPLWrapper::readTemplate_()
{
    std::string file_name = gender_path_ + gender_ + "_shapeAv.obj";

    bool success = igl::readOBJ(file_name, verts_template_, faces_);
    if (!success)
    {
        std::string message("Abort: Could not read SMPL template at ");
        message += file_name;
        throw std::exception(message.c_str());
    }

    E::VectorXd mean_point = verts_template_.colwise().mean();
    verts_template_normalized_ = verts_template_.rowwise() - mean_point.transpose();
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
    std::string file_path = gender_path_ + gender_ + "_blendshape/shape";

    Eigen::MatrixXi fakeFaces(SMPLWrapper::VERTICES_NUM, SMPLWrapper::SPACE_DIM);

    for (int i = 0; i < SMPLWrapper::SHAPE_SIZE; i++)
    {
        std::string file_name(file_path);
        file_name += std::to_string(i);
        file_name += ".obj";

        igl::readOBJ(file_name, shape_diffs_[i], fakeFaces);

        shape_diffs_[i] -= verts_template_;
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

E::Vector3d SMPLWrapper::angle_axis_(const E::Vector3d& from, const E::Vector3d& to)
{
    assert(SMPLWrapper::SPACE_DIM == 3 && "angle_axis_() can only be used in 3D world");

    if (from.norm() * to.norm() > 0.0)
    {
        E::Vector3d axis = from.cross(to);
        double sin_a = axis.norm() / (from.norm() * to.norm());
        double cos_a = from.dot(to) / (from.norm() * to.norm());
        double angle = atan2(sin_a, cos_a);

        axis.normalize();
        axis = angle * axis;
        return axis;
    }
    return E::Vector3d(0, 0, 0);
}

E::Vector3d SMPLWrapper::rotate_by_angle_axis_(const E::Vector3d& vector, const E::Vector3d& angle_axis_rotation)
{
    double angle = angle_axis_rotation.norm();
    E::Vector3d axis = angle_axis_rotation.normalized();

    // Rodrigues' rotation formula
    // https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
    E::Vector3d rotated_vec =
        cos(angle) * vector
        + sin(angle) * axis.cross(vector)
        + (1 - cos(angle)) * axis.dot(vector) * axis;

    return rotated_vec;
}

E::Vector3d SMPLWrapper::combine_two_angle_axis_(const E::Vector3d& first, const E::Vector3d& second)
{
    double angle_first = first.norm();
    E::Vector3d axis_first = first.normalized();
    double angle_second = second.norm();
    E::Vector3d axis_second = second.normalized();

    // Rodrigues' Formula 
    // https://math.stackexchange.com/questions/382760/composition-of-two-axis-angle-rotations
    // get axis
    E::Vector3d axis_sin_scaled =
        cos(angle_first / 2) * sin(angle_second / 2) * axis_second
        + sin(angle_first / 2) * cos(angle_second / 2) * axis_first
        + sin(angle_first / 2) * sin(angle_second / 2) * second.cross(first);
    E::Vector3d axis = axis_sin_scaled.normalized();

    // get angle
    double angle_half_sin = axis_sin_scaled.norm();
    double angle_half_cos = cos(angle_first / 2) * cos(angle_second / 2)
        - axis_first.dot(axis_second) * sin(angle_first / 2) * sin(angle_second / 2);

    double angle = 2 * atan2(angle_half_sin, angle_half_cos);

    return angle * axis;
}

void SMPLWrapper::assignJointGlobalRotation_(int joint_id, E::VectorXd rotation, 
    const EHomoCoordMatrix(&fk_transform)[SMPLWrapper::JOINTS_NUM])
{
    Eigen::Vector3d rotation_local;
    if (joint_id > 0)
    {
        Eigen::MatrixXd joint_inverse_rotation =
            fk_transform[joint_id].block(0, 0, SPACE_DIM, SPACE_DIM).transpose();

        rotation_local = joint_inverse_rotation * rotation;
    }
    else
    {
        rotation_local = rotation;
    }
    

    for (int i = 0; i < SPACE_DIM; ++i)
    {
        state_.pose[joint_id * SPACE_DIM + i] = rotation_local(i);
    }
}

void SMPLWrapper::shapeSMPL_(const double * const shape, E::MatrixXd &verts, E::MatrixXd* shape_jac)
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

void SMPLWrapper::poseSMPL_(const double * const pose, E::MatrixXd & verts, E::MatrixXd * pose_jac, 
    bool use_previous_pose_matrix, bool ignore_translation)
{
    E::SparseMatrix<double> LBSMat = this->getLBSMatrix_(verts);

    if (!use_previous_pose_matrix)
    {
        joint_locations_ = jointRegressorMat_ * verts;
        updateJointsFKTransforms_(pose, joint_locations_, pose_jac != nullptr);
    }

    E::MatrixXd joints_global_transform = extractLBSJointTransformFromFKTransform_(
        fk_transforms_, joint_locations_,
        &fk_derivatives_, pose_jac, ignore_translation);

    verts = LBSMat * joints_global_transform;

    if (pose_jac != nullptr)
    {
        // pose_jac[i] at this point has the same structure as jointsTransformation
        for (int i = 0; i < SMPLWrapper::POSE_SIZE; ++i)
        {
            pose_jac[i].applyOnTheLeft(LBSMat);
        }
    }
}

void SMPLWrapper::translate_(const double * const translation, E::MatrixXd & verts)
{
    for (int i = 0; i < verts.rows(); ++i)
        for (int j = 0; j < SMPLWrapper::SPACE_DIM; ++j)
            verts(i, j) += translation[j];

    // Jac w.r.t. translation is identity: dv_i / d_tj == 1 
}

E::MatrixXd SMPLWrapper::calcJointLocations_(const double * translation, 
    const double * shape, const double * pose)
{
    E::MatrixXd joint_locations;

    if (joint_locations_template_.size() > 0)
    {
        joint_locations = joint_locations_template_;
    }
    else
    {
        joint_locations = jointRegressorMat_ * verts_template_normalized_;
    }

    if (shape != nullptr)
    {
        E::MatrixXd verts = verts_template_normalized_;
        shapeSMPL_(shape, verts);
        joint_locations = jointRegressorMat_ * verts;
    }

    if (pose != nullptr)
    {
        updateJointsFKTransforms_(pose, joint_locations);
        joint_locations = extractJointLocationFromFKTransform_(fk_transforms_);
    }

    if (translation != nullptr)
    {
        translate_(translation, joint_locations);
    }

    return joint_locations;
}

E::MatrixXd SMPLWrapper::extractJointLocationFromFKTransform_(
    const EHomoCoordMatrix(&fk_transform)[SMPLWrapper::JOINTS_NUM])
{
    // Go over the fk_transform and gather joint locations
    E::MatrixXd joints_locations(JOINTS_NUM, SPACE_DIM);
    for (int j = 0; j < JOINTS_NUM; j++)
    {
        // translation info is in the last column
        joints_locations.row(j) = fk_transform[j].block(0, SPACE_DIM, SPACE_DIM, 1).transpose();
    }

    return joints_locations;
}

E::MatrixXd SMPLWrapper::extractLBSJointTransformFromFKTransform_(
    const EHomoCoordMatrix(&fk_transform)[SMPLWrapper::JOINTS_NUM], 
    const E::MatrixXd & t_pose_joints_locations,
    const E::MatrixXd(*FKDerivatives)[SMPLWrapper::JOINTS_NUM][SMPLWrapper::POSE_SIZE],
    E::MatrixXd * jacsTotal, 
    bool zero_out_translation)
{
    E::MatrixXd joints_transform(HOMO_SIZE * SMPLWrapper::JOINTS_NUM, SMPLWrapper::SPACE_DIM);

    // utils vars: not to recreate them on each loop iteration
    E::MatrixXd inverse_t_pose_translate;
    E::MatrixXd tmpPointGlobalTransform;
    E::MatrixXd tmpPointGlobalJac;

    if (jacsTotal != nullptr)
        for (int i = 0; i < SMPLWrapper::POSE_SIZE; ++i)
            jacsTotal[i].setZero(HOMO_SIZE * SMPLWrapper::JOINTS_NUM, SMPLWrapper::SPACE_DIM);

    // Go over the fk_transform_ matrix and create LBS-compatible matrix
    for (int j = 0; j < JOINTS_NUM; j++)
    {
        tmpPointGlobalTransform = fk_transform[j];
        if (zero_out_translation)
        {
            // ignore translation part
            tmpPointGlobalTransform.col(SPACE_DIM).setZero();
        }
        else
        {
            // inverse is needed to transform verts coordinates to local coordinate system
            inverse_t_pose_translate = get3DTranslationMat_(-t_pose_joints_locations.row(j));
            tmpPointGlobalTransform = fk_transform[j] * inverse_t_pose_translate;
        }

        joints_transform.block(HOMO_SIZE * j, 0, HOMO_SIZE, SMPLWrapper::SPACE_DIM)
            = tmpPointGlobalTransform.transpose().leftCols(SMPLWrapper::SPACE_DIM);

        // and fill corresponding jac format
        if (jacsTotal != nullptr && FKDerivatives != nullptr)
        {
            // jac w.r.t current joint rotation coordinates
            for (int dim = 0; dim < SMPLWrapper::SPACE_DIM; ++dim)
            {
                tmpPointGlobalJac = 
                    (*FKDerivatives)[j][j * SMPLWrapper::SPACE_DIM + dim] * inverse_t_pose_translate;
                jacsTotal[j * SMPLWrapper::SPACE_DIM + dim].block(j * HOMO_SIZE, 0, HOMO_SIZE, SMPLWrapper::SPACE_DIM)
                    = tmpPointGlobalJac.transpose().leftCols(SMPLWrapper::SPACE_DIM);
            }

            // jac w.r.t. ancessors rotation coordinates         
            for (int parent_dim = 0; parent_dim < (joints_parents_[j] + 1) * SMPLWrapper::SPACE_DIM; ++parent_dim)
            {
                if ((*FKDerivatives)[joints_parents_[j]][parent_dim].size() > 0)
                {
                    tmpPointGlobalJac = (*FKDerivatives)[j][parent_dim] * inverse_t_pose_translate;
                    jacsTotal[parent_dim].block(j * HOMO_SIZE, 0, HOMO_SIZE, SMPLWrapper::SPACE_DIM)
                        = tmpPointGlobalJac.transpose().leftCols(SMPLWrapper::SPACE_DIM);
                }
            }
        }
    }

    return joints_transform;
}

void SMPLWrapper::updateJointsFKTransforms_(
    const double * const pose, const E::MatrixXd & t_pose_joints_locations, bool calc_derivatives) 
{
#ifdef DEBUG
    std::cout << "global transform (analytic)" << std::endl;
#endif // DEBUG

    // uses functions that assume input in 3D (see below)
    assert(SMPLWrapper::SPACE_DIM == 3 && "The function can only be used in 3D world");

    // root as special case
    fk_transforms_[0] = get3DLocalTransformMat_(pose, t_pose_joints_locations.row(0));
    if (calc_derivatives)
    {
        get3DLocalTransformJac_(pose, fk_transforms_[0], fk_derivatives_[0]);
    }

    E::MatrixXd localTransform, localTransformJac[SMPLWrapper::SPACE_DIM];
    for (int joint_id = 1; joint_id < SMPLWrapper::JOINTS_NUM; joint_id++)
    {
        localTransform = get3DLocalTransformMat_((pose + joint_id * 3),
            t_pose_joints_locations.row(joint_id) - t_pose_joints_locations.row(joints_parents_[joint_id]));

        // Forward Kinematics Formula
        fk_transforms_[joint_id] = fk_transforms_[joints_parents_[joint_id]] * localTransform;

        if (calc_derivatives)
        {
            get3DLocalTransformJac_((pose + joint_id * 3), localTransform, localTransformJac);

            // jac w.r.t current joint rot coordinates
            for (int j = 0; j < SMPLWrapper::SPACE_DIM; ++j)
            {
                fk_derivatives_[joint_id][joint_id * SMPLWrapper::SPACE_DIM + j] = 
                    fk_transforms_[joints_parents_[joint_id]] * localTransformJac[j];
            }

            // jac w.r.t. ancessors rotation coordinates         
            for (int j = 0; j < (joints_parents_[joint_id] + 1) * SMPLWrapper::SPACE_DIM; ++j)
            {
                if (fk_derivatives_[joints_parents_[joint_id]][j].size() > 0)
                {
                    fk_derivatives_[joint_id][j] = 
                        fk_derivatives_[joints_parents_[joint_id]][j] * localTransform;
                }
            }
        }

    }

    // now the fk_* are updated
}

E::MatrixXd SMPLWrapper::get3DLocalTransformMat_(const double * const jointAxisAngleRotation, 
    const E::MatrixXd & jointToParentDist)
{
    // init
    E::MatrixXd localTransform;
    localTransform.setIdentity(4, 4);   // in homogenious coordinates
    localTransform.block(0, 3, 3, 1) = jointToParentDist.transpose(); // g(0)

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

    if (norm > 0.0001)  // don't waste computations on zero joint movement
    {
        // apply Rodrigues formula
        exponent += w_skew * sin(norm) + w_skew * w_skew * (1. - cos(norm));
        localTransform.block(0, 0, 3, 3) = exponent;
    }

    return localTransform;
}

void SMPLWrapper::get3DLocalTransformJac_(const double * const jointAxisAngleRotation, 
    const E::MatrixXd & transform_mat, E::MatrixXd* local_transform_jac_out)
{
    for (int i = 0; i < SPACE_DIM; ++i)
    {
        local_transform_jac_out[i].setZero(4, 4);
        // For the default pose transformation g(0)
        local_transform_jac_out[i].col(SPACE_DIM) = transform_mat.col(SPACE_DIM);   
        // local_transform_jac_out[i](3, 3) = 0;   
        // (3, 3) should be 1 to account for the positions of the previous and subsequent joints in jac calculation
    }

    E::Vector3d w = E::Map<const E::Vector3d>(jointAxisAngleRotation);
    double norm = w.norm();

    if (norm > 0.0001)
    {
        E::Matrix3d w_skew;
        w_skew <<
            0, -w[2], w[1],
            w[2], 0, -w[0],
            -w[1], w[0], 0;
        w_skew /= norm;

        E::MatrixXd rot_mat = transform_mat.block(0, 0, SPACE_DIM, SPACE_DIM);

        for (int i = 0; i < SPACE_DIM; ++i)
        {
            // compact formula from https://arxiv.org/pdf/1312.0788.pdf
            E::Vector3d cross = 
                w.cross((E::Matrix3d::Identity() - rot_mat).col(i))
                / (norm * norm);
            E::Matrix3d cross_skew;
            cross_skew <<
                0, -cross[2], cross[1],
                cross[2], 0, -cross[0],
                -cross[1], cross[0], 0;

            local_transform_jac_out[i].block(0, 0, 3, 3) =
                (w_skew * w[i] / norm + cross_skew) * rot_mat;
        }
    }
    else // zero case 
    {
        local_transform_jac_out[0].block(0, 0, 3, 3) <<
            0, 0, 0,
            0, 0, -1,
            0, 1, 0;
        local_transform_jac_out[1].block(0, 0, 3, 3) <<
            0, 0, 1,
            0, 0, 0,
            -1, 0, 0;
        local_transform_jac_out[2].block(0, 0, 3, 3) <<
            0, -1, 0,
            1, 0, 0,
            0, 0, 0;
    }

}

E::MatrixXd SMPLWrapper::get3DTranslationMat_(const E::MatrixXd & translationVector)
{
    E::MatrixXd translation;
    translation.setIdentity(4, 4);  // in homogenious coordinates
    translation.block(0, 3, 3, 1) = translationVector.transpose();

    return translation;
}

E::SparseMatrix<double> SMPLWrapper::getLBSMatrix_(const E::MatrixXd & verts) const
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

    displacements.setZero(VERTICES_NUM, SPACE_DIM);
}

SMPLWrapper::State::~State()
{
    delete[] pose;
    delete[] shape;
    delete[] translation;
}
