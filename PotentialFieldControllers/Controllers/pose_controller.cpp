/**
 * Pose Controller
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#include <Controllers/pose_controller.h>

using controllers::PoseController;

// CONSTRUCTORS/DESTRUCTORS
PoseController::PoseController(std::shared_ptr<RobotSystem> robot_model_in,
                               int num_virtual_joints,
                               std::vector<int> virtual_rotation_joints,
                               std::string robot_name,
                               std::string ref_frame)
    : PotentialFieldController(robot_model_in, num_virtual_joints, virtual_rotation_joints, robot_name, ref_frame) {
    kp_ = 0.835; // default gain

    // set topic names
    ref_topic_ = std::string("nstgro20/reference_pose");
    cmd_topic_ = std::string("nstgro20/joint_commands");

    // setup IK module
    ik_.setDebug(false);
}

PoseController::~PoseController() {
}

// CONTROLLER FUNCTIONS
void PoseController::init(ros::NodeHandle& nh,
                          std::string group_name,
                          std::vector<std::string> joint_names,
                          std::vector<int> joint_indices,
                          std::string frame_name, int frame_idx) {
    // initialize controller
    PotentialFieldController::init(nh, group_name, joint_names, joint_indices, frame_name, frame_idx);

    // set task
    pose_task_ = std::make_shared<Task6DPose>(Task6DPose(robot_model_, frame_idx));
    ik_.addTaskToList(pose_task_);
    ik_.setDefaultTaskGains();
    // TODO HELPER FUNCTION FOR SETTING APPROPRIATE IK TASKS
    return;
}

void PoseController::start() {
    // start controller
    PotentialFieldController::start();
    return;
}

void PoseController::stop() {
    // stop controller
    PotentialFieldController::stop();
    return;
}

void PoseController::reset() {
    // reset controller
    PotentialFieldController::reset();
    return;
}

void PoseController::update() {
    // update controller
    PotentialFieldController::update();
    return;
}

// CONNECTIONS
void PoseController::initializeConnections() {
    // reference subscriber
    ref_sub_ = nh_.subscribe(ref_topic_, 1, &PoseController::refCallback, this);
    // command publisher
    cmd_pub_ = nh_.advertise<sensor_msgs::JointState>(cmd_topic_, 1);

    return;
}

// GET CONTROLLER INFO
std::string PoseController::getReferenceType() {
    return std::string("geometry_msgs::PoseStamped");
}

std::string PoseController::getCommandType() {
    return std::string("sensor_msgs::JointState");
}

std::string PoseController::getFullName() {
    return std::string("controllers/PoseController");
}

std::string PoseController::getName() {
    return std::string("PoseController");
}

// CALLBACK
void PoseController::refCallback(const geometry_msgs::PoseStamped& msg) {
    // check if initialized
    if( !initialized_ ) {
        ROS_ERROR("%s::refCallback() -- cannot accept reference pose, controller is not initialized", getName().c_str());
        return;
    }

    // set reference pose message
    ref_pose_msg_ = msg;

    return;
}

// HELPER FUNCTIONS
void PoseController::updateReferencePose() {
    // get message frame name
    std::string msg_frame_name = ref_pose_msg_.header.frame_id;

    // intialize reference message
    geometry_msgs::Pose ref_pose;

    // check if message frame is empty
    if( msg_frame_name.empty() ) {
        ROS_ERROR("%s::updateReferencePose() -- empty frame id for reference message", getName().c_str());
    }

    // check that that message frame matches reference frame
    if( msg_frame_name != ref_frame_name_ ) {
        // frames are not the same, transformation needed
        std::string err_msg;
        geometry_msgs::PoseStamped transformed_ref_pose_msg;
        try {
            // check if transform exists
            if( !tf_.waitForTransform(msg_frame_name, ref_frame_name_, ros::Time(0), ros::Duration(1.0), // TODO these frames might need to be switched? (target, source)
                                      ros::Duration(0.01), &err_msg)) { // default polling sleep duration, pointer to error message
                ROS_ERROR("%s::updateReferencePose() -- no transform from %s to %s; error: %s",
                          getName().c_str(), ref_frame_name_.c_str(), msg_frame_name.c_str(), err_msg.c_str());
            }
            else {
                // transform reference message into reference frame
                tf_.transformPose(ref_frame_name_, ref_pose_msg_, transformed_ref_pose_msg);
                // update reference message
                ref_pose = transformed_ref_pose_msg.pose;
            }
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s::updateReferencePose() -- trouble getting transform from %s to %s; TransformException: %s",
                      getName().c_str(), ref_frame_name_.c_str(), msg_frame_name.c_str(), ex.what());
        }
    }
    else {
        // no transformation needed, update reference message
        ref_pose = ref_pose_msg_.pose;
    }

    // set reference pose
    ref_pos_ << ref_pose.position.x, ref_pose.position.y, ref_pose.position.z;
    ref_quat_.x() = ref_pose.orientation.x;
    ref_quat_.y() = ref_pose.orientation.y;
    ref_quat_.z() = ref_pose.orientation.z;
    ref_quat_.w() = ref_pose.orientation.w;

    // update reference pose in attractive potential field
    att_potential_.setGoal(ref_pos_, ref_quat_);
    // update reference pose in 6dpose task
    pose_task_->setTarget(ref_pos_, ref_quat_);

    // set reference flag
    reference_set_ = true;

    return;
}

// CONTROL LAW FUNCTIONS
double PoseController::potential() {
    // compute potential using attractive potential field
    double pot = att_potential_.potential(curr_pos_, curr_quat_);

    return pot;
}

double PoseController::potential(dynacore::Vect3 _curr_pos, dynacore::Quaternion _curr_quat, dynacore::Vector _curr_q) {
    // compute potential using attractive potential field
    double pot = att_potential_.potential(curr_pos_, curr_quat_);

    return pot;
}

void PoseController::gradient(dynacore::Vector& _grad) {
    // compute gradient using attractive potential field
    att_potential_.gradient(curr_pos_, curr_quat_, _grad);

    return;
}

void PoseController::getDx(Eigen::Vector3d& _dx_p, Eigen::Vector3d& _dx_r) {
    // compute change in configuration using attractive potential field
    att_potential_.getDx(curr_pos_, curr_quat_, _dx_p, _dx_r);

    // resize error vector
    err_.resize(6);
    
    // set error vector
    err_.head(3) = _dx_p;
    err_.tail(3) = _dx_r;
    
    return;
}

void PoseController::objectiveJacobian(dynacore::Matrix& _J, dynacore::Matrix& _Jinv) {
    // compute Jacobian using pose task
    pose_task_->computeTaskJacobian(_J);

    // compute pseudoinverse of Jacobian
    dynacore::pInv(_J, _Jinv);
    
    return;
}

void PoseController::getDq(dynacore::Vector& _dq) {
    // update configuration
    updateConfiguration();

    // update current and reference poses
    updateCurrentPose();
    updateReferencePose();

    // compute pose error
    dynacore::Vect3 dx_p;
    dynacore::Vect3 dx_r;
    getDx(dx_p, dx_r);

    // solve IK problem
    dynacore::Vector q_solution;
    ik_.solve(q_solution);

    // compute change in configuration
    _dq = kp_ * (q_solution - q_);

    return;
}
