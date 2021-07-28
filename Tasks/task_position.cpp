/**
 * Position Task Class
 * Emily Sheetz, NSTGRO VTE 2021 
 **/

#include <Tasks/task_position.h>

// CONSTRUCTORS/DESTRUCTORS
TaskPosition::TaskPosition() {
    // initialize parameters for task
    initializeTaskParameters();
}

TaskPosition::TaskPosition(std::shared_ptr<RobotSystem> robot_model_in, int frame_idx_in) {
    // set frame and robot
    task_frame_ = frame_idx_in;
    robot_model_ = robot_model_in;

    // initialize parameters for task
    initializeTaskParameters();
}

TaskPosition::~TaskPosition() {
}

// GETTERS/SETTERS
void TaskPosition::getTarget(dynacore::Vect3& ref_pos_out) {
    ref_pos_out = ref_pos_;
    return;
}

void TaskPosition::setTarget(dynacore::Vect3 ref_pos_in) {
    ref_pos_ = ref_pos_in;
    return;
}

void TaskPosition::initializeTaskParameters() {
    // set task name
    task_name_ = "TaskPosition";

    // set task dimension
    task_dim_ = 3;

    // initialize current position
    curr_pos_.setZero();

    // initialize reference position
    ref_pos_.setZero();

    return;
}

// TASK RELATED COMPUTATIONS
void TaskPosition::computeTaskResidual(dynacore::Vector& r_task) {
    // update current position of frame
    robot_model_->getPos(task_frame_, curr_pos_);

    if( debug_ ) {
        dynacore::pretty_print(curr_pos_, std::cout, "Current position:");
        dynacore::pretty_print(ref_pos_, std::cout, "Target position:");
    }

    // compute position difference
    dynacore::Vect3 error_pos = ref_pos_ - curr_pos_;

    // set 3x1 residual
    residual_.resize(task_dim_);
    residual_ = error_pos;

    // multiply by task gain
    residual_ *= kp_task_gain_;

    if( debug_ ) {
        dynacore::pretty_print(residual_, std::cout, "Task residual:");
    }

    // set input variable
    r_task = residual_;

    return;
}

void TaskPosition::computeTaskJacobian(dynacore::Matrix& J_task) {
    // create temporary angular Jacobian, angular Jacobian pseudoinverse, and linear Jacobian pseudoinverse matrices
    dynacore::Matrix Jang_tmp, Janginv_tmp, Jlininv_tmp;

    // get Jacobian from robot model and invert RBDL Jacobian to be [linear; rotational]
    RobotUtils::getRobotModelJacobians(robot_model_, task_frame_,
                                       J_task, Jang_tmp,
                                       Jlininv_tmp, Janginv_tmp);

    if( debug_ ) {
        dynacore::pretty_print(J_task, std::cout, "Task Jacobian:");
    }

    return;
}
