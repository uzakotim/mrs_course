#include <task_01_controller/utils.h>

#include <student_headers/controller.h>

#include <iostream>

namespace task_01_controller
{

using namespace Eigen;

/**
 * @brief The controller initialization method. It is called ONLY ONCE in the lifetime of the controller.
 * Use this method to do any heavy pre-computations.
 *
 * @param mass UAV mass [kg]
 * @param user_params user-controllable parameters
 * @param g gravitational acceleration [m/s^2]
 * @param action_handlers methods for the user
 */
void Controller::init(const double mass, const UserParams_t user_params, const double g, ActionHandlers_t &action_handlers) {

  // copy the mass and the gravity acceleration
  this->_mass_ = mass;
  this->_g_    = g;

  // the action handlers will allow you to plot data
  this->action_handlers_ = action_handlers;

  // INITIALIZE YOUR CONTROLLER HERE

  // INITIALIZE YOUR KALMAN FILTER HERE
  // SET THE STATE AND THE COVARIANCE MATRICES AS GLOBAL VARIABLES
}

/**
 * @brief This method is called to reset the internal state of the controller, e.g., just before
 * the controller's activation. Use it to, e.g., reset the controllers integrators and estimators.
 */
void Controller::reset() {

  // IT WOULD BE GOOD TO RESET THE PID'S INTEGRALS

  // IT WOULD BE NICE TO RESET THE KALMAN'S STATE AND COVARIANCE

  // ALSO, THE NEXT iteration calculateControlSignal() IS GOING TO BE "THE 1ST ITERATION"
}

/**
 * @brief the main routine, is called to obtain the control signals
 *
 * @param uav_state the measured UAV state, contains position and acceleration
 * @param user_params user-controllable parameters
 * @param control_reference the desired state of the UAV, position, velocity, acceleration, heading
 * @param dt the time difference in seconds between now and the last time calculateControlSignal() got called
 *
 * @return the desired control signal: the total thrust force and the desired orientation
 */
std::pair<double, Matrix3d> Controller::calculateControlSignal(const UAVState_t uav_state, const UserParams_t user_params,
                                                               const ControlReference_t control_reference, const double dt) {

  // Publish the following values as "ROS topics" such that they can be plotted
  // * plotting can be achived using, e.g., the tool called PlotJuggler
  // * try the "plot.sh" script, which will run PlotJuggler
  //
  // action_handlers_.plotValue("pos_x", uav_state.position[0]);
  // action_handlers_.plotValue("pos_y", uav_state.position[1]);
  // action_handlers_.plotValue("pos_z", uav_state.position[2]);

  // publish the following pose as "ROS topic", such that it can be plotted by Rviz
  //
  // action_handlers_.visualizePose("uav_pose_offset", uav_state.position[0], uav_state.position[1], uav_state.position[2] + 1.0, uav_state.heading);

  // | ---------- calculate the output control signals ---------- |

  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // THIS IS THE PLACE FOR YOUR CODE
  std::cout<<"Hello, world!"<<'\n';

  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

  // LATER, CALL THE lkfPredict() AND lkfCorrect() FUNCTIONS HERE TO OBTAIN THE FILTERED POSITION STATE
  // DON'T FORGET TO INITIALZE THE STATE DURING THE FIRST ITERATION

  double des_tilt_x  = 0;  // [rad]
  double des_tilt_y  = 0;  // [rad]
  double des_accel_z = 0;  // [m/s^2]

  // | ---------------- add gravity compensation ---------------- |

  des_accel_z += _g_;

  // | --------------- return the control signals --------------- |

  double   body_thrust;
  Matrix3d desired_orientation;

  std::tie(body_thrust, desired_orientation) = augmentInputs(des_tilt_x, des_tilt_y, des_accel_z * _mass_, control_reference.heading);

  return {body_thrust, desired_orientation};
};

}  // namespace task_01_controller
