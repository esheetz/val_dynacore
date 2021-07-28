/**
 * Orientation Task Class
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#include <Tasks/task_orientation.h>

// CONSTRUCTORS/DESTRUCTORS
TaskOrientation::TaskOrientation() {
    // initialize parameters for task
    initializeTaskParameters();
}

TaskOrientation::TaskOrientation(std::shared_ptr<RobotSystem> robot_model_in, int frame_idx_in) {
    // set frame and robot
    task_frame_ = frame_idx_in;
    robot_model_ = robot_model_in;

    // initialize parameters for task
    initializeTaskParameters();
}

TaskOrientation::~TaskOrientation() {
}

// GETTERS/SETTERS
void TaskOrientation::getTarget(dynacore::Quaternion& ref_quat_out) {
    ref_quat_out = ref_quat_;
    return;
}

void TaskOrientation::setTarget(dynacore::Quaternion ref_quat_in) {
    ref_quat_ = ref_quat_in;
    return;
}

void TaskOrientation::initializeTaskParameters() {
    // set task name
    task_name_ = "TaskOrientation";

    // set task dimension
    task_dim_ = 3;

    // initialize current orientation
    curr_quat_.setIdentity();

    // intialize reference orientation
    ref_quat_.setIdentity();

    return;
}

// TASK RELATED COMPUTATIONS
void TaskOrientation::computeTaskResidual(dynacore::Vector& r_task) {
    // update current orientation of frame
    robot_model_->getOri(task_frame_, curr_quat_);

    if( debug_ ) {
        dynacore::pretty_print(curr_quat_, std::cout, "Current quaternion:");
        dynacore::pretty_print(ref_quat_, std::cout, "Target quaternion:");
    }

    // compute rotation difference as axis angle
    Eigen::AngleAxisd axis_angle;
    axis_angle = ref_quat_ * curr_quat_.inverse();
    dynacore::Vect3 error_rot = axis_angle.angle() * axis_angle.axis(); // representation of rotation matrix (rotation vector) in exponential coordinates

    // set 3x1 residual
    residual_.resize(task_dim_);
    residual_ = error_rot;

    // multiply by task gain
    residual_ *= kp_task_gain_;

    if( debug_ ) {
        dynacore::pretty_print(residual_, std::cout, "Task residual:");
    }

    // set input variable
    r_task = residual_;

    return;
}

void TaskOrientation::computeTaskJacobian(dynacore::Matrix& J_task) {
    // create temporary linear Jacobian, linear Jacobian pseudoinverse, and angular Jacobian pseudoinverse matrices
    dynacore::Matrix Jlin_tmp, Jlininv_tmp, Janginv_tmp;

    // get Jacobian from robot model and invert RBDL Jacobian to be [linear; rotational]
    RobotUtils::getRobotModelJacobians(robot_model_, task_frame_,
                                       Jlin_tmp, J_task,
                                       Jlininv_tmp, Janginv_tmp);

    if( debug_ ) {
        dynacore::pretty_print(J_task, std::cout, "Task Jacobian:");
    }

    return;
}
