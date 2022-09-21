#define VERSION "1.0.0"

/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/attitude_converter.h>

#include <student_headers/controller.h>

#include <random>

#include <stdio.h>

//}

namespace task_01_evaluation
{

/* class Kalmanrmse_velocity //{ */

class KalmanTest : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  bool            is_initialized_ = false;
  std::string     _version_;

  // | ----------------------- parameters ----------------------- |

  std::string _file_path_;
  bool        _output_to_file_ = false;
  FILE*       _output_file_;

  double _requirement_rmse_pos_;
  double _requirement_rmse_vel_;

  task_01_controller::UserParams_t user_params_;

  // | ------------------ student's controller ------------------ |

  std::unique_ptr<task_01_controller::Controller> controller_;

  // | --------------------- action handlers -------------------- |

  task_01_controller::ActionHandlers_t action_handlers_;

  void visualizePose(const std::string name, const double x, const double y, const double z, const double heading);
  void plotValue(const std::string name, const double value);

  // | ----------------- random number generator ---------------- |

  std::default_random_engine random_engine_;

  // | ---------------------- random engine --------------------- |

  double randd(const double from, const double to);
  int    randi(const int from, const int to);

  bool isVectorFinite(const Eigen::VectorXd& vector, const std::string name);
  bool isMatrixFinite(const Eigen::MatrixXd& matrix, const std::string name);

  void runtimeError();
};

//}

/* onInit() //{ */

void KalmanTest::onInit() {

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  srand(static_cast<unsigned int>(ros::Time::now().nsec));

  mrs_lib::ParamLoader param_loader(nh_, "KalmanTest");

  param_loader.loadParam("version", _version_);

  if (_version_ != VERSION) {

    ROS_ERROR("[KalmanTest]: the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
    ros::shutdown();
  }

  param_loader.loadParam("file", _file_path_);

  if (_file_path_ != "") {
    _output_to_file_ = true;
  }

  param_loader.loadParam("param1", user_params_.param1);
  param_loader.loadParam("param2", user_params_.param2);
  param_loader.loadParam("param3", user_params_.param3);
  param_loader.loadParam("param4", user_params_.param4);
  param_loader.loadParam("param5", user_params_.param5);
  param_loader.loadParam("param6", user_params_.param6);
  param_loader.loadParam("param7", user_params_.param7);
  param_loader.loadParam("param8", user_params_.param8);
  param_loader.loadParam("param9", user_params_.param9);
  param_loader.loadParam("param10", user_params_.param10);
  param_loader.loadParam("param11", user_params_.param11);
  param_loader.loadParam("param12", user_params_.param12);

  param_loader.loadParam("requirements/rmse_position", _requirement_rmse_pos_);
  param_loader.loadParam("requirements/rmse_velocity", _requirement_rmse_vel_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[ParamServer]: Could not load all parameters!");
    ros::shutdown();
  }

  action_handlers_.plotValue     = std::bind(&KalmanTest::plotValue, this, std::placeholders::_1, std::placeholders::_2);
  action_handlers_.visualizePose = std::bind(&KalmanTest::visualizePose, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
                                             std::placeholders::_4, std::placeholders::_5);

  double dt = 0.01;

  task_01_controller::Matrix9x9d A = task_01_controller::Matrix9x9d::Identity();
  A.diagonal().tail<3>()           = task_01_controller::Vector3d(0.95, 0.95, 0.00);
  A.diagonal(3)                    = dt * task_01_controller::Vector6d::Ones();
  A.diagonal(6)                    = 0.5 * dt * dt * task_01_controller::Vector3d::Ones();

  task_01_controller::Matrix9x3d B = task_01_controller::Matrix9x3d::Zero();
  B.block<3, 3>(6, 0).diagonal()   = task_01_controller::Vector3d(0.05, 0.05, 0.01);

  task_01_controller::Vector9d x = task_01_controller::Vector9d::Zero();

  // | ------------ initialize the student's library ------------ |

  controller_ = std::make_unique<task_01_controller::Controller>();
  controller_->init(2.0, user_params_, 9.8, action_handlers_);

  // | ---------------- prepare the input vector ---------------- |

  int input_n_steps = randi(5, 15);

  std::vector<double> input_step_lens;

  for (int i = 0; i < input_n_steps; i++) {
    input_step_lens.push_back(randd(1, 3));
  }

  std::vector<double> input_step_signal_x;
  std::vector<double> input_step_signal_y;
  std::vector<double> input_step_signal_z;

  for (int i = 0; i < input_n_steps; i++) {
    input_step_signal_x.push_back(randd(-2, 2));
    input_step_signal_y.push_back(randd(-2, 2));
    input_step_signal_z.push_back(randd(-2, 2));
  }

  // | ------------------------ simulate ------------------------ |

  task_01_controller::Vector9d   x_student  = task_01_controller::Vector9d::Zero();
  task_01_controller::Matrix9x9d covariance = task_01_controller::Matrix9x9d::Identity();

  ROS_INFO("[KalmanTest]: simulating");

  std::normal_distribution<double> norm_robot_xyz_pos(0, 0.1 / 1.73);
  std::normal_distribution<double> norm_robot_xyz_acc(0, 0.1 / 1.73);

  double error_velocity = 0;
  double error_position = 0;

  int n_steps = 0;

  // for each input step
  for (int i = 0; i < input_n_steps; i++) {

    // how manu simulation samplea should be made
    int n_samples = int(input_step_lens[i] / dt);

    // for each simulation sample, apply the input and create artificial measurements
    for (int j = 0; j < n_samples; j++) {

      n_steps++;

      Eigen::Vector3d input = Eigen::Vector3d(input_step_signal_x[i], input_step_signal_y[i], input_step_signal_z[i]);

      std::tie(x_student, covariance) = controller_->lkfPredict(x_student, covariance, input, dt);

      if (x_student.size() != 9) {
        ROS_ERROR("[KalmanTest]: lkfPredict() return 'x' with a wrong size");
        runtimeError();
      }

      if (covariance.cols() != 9 || covariance.rows() != 9) {
        ROS_ERROR("[KalmanTest]: lkfPredict() return 'covariance' with wrong dimensions");
        runtimeError();
      }

      if (!isVectorFinite(x_student, "x_student")) {
        ROS_ERROR("[KalmanTest]: NaN/Inf found in the student's state vector, coming out of lkfPredict()");
        runtimeError();
      }

      if (!isMatrixFinite(covariance, "covariance")) {
        ROS_ERROR("[KalmanTest]: NaN/Inf found in the student's covariance matrix, coming out of lkfPredict()");
        runtimeError();
      }

      x = A * x + B * input;

      // make the measurements noisy

      task_01_controller::Vector6d measurement;
      measurement.head<3>() = x.head<3>();
      measurement.tail<3>() = x.tail<3>();
      for (int it = 0; it < 3; it++)
        measurement(it) += norm_robot_xyz_pos(random_engine_);
      for (int it = 3; it < 6; it++)
        measurement(it) += norm_robot_xyz_acc(random_engine_);

      std::tie(x_student, covariance) = controller_->lkfCorrect(x_student, covariance, measurement, dt);

      if (x_student.size() != 9) {
        ROS_ERROR("[KalmanTest]: lkfPredict() return 'x' with a wrong size");
        runtimeError();
      }

      if (covariance.cols() != 9 || covariance.rows() != 9) {
        ROS_ERROR("[KalmanTest]: lkfPredict() return 'covariance' with wrong dimensions");
        runtimeError();
      }

      if (!isVectorFinite(x_student, "x_student")) {
        ROS_ERROR("[KalmanTest]: NaN/Inf found in the student's state vector, coming out of lkfCorrect()");
        runtimeError();
      }

      if (!isMatrixFinite(covariance, "covariance")) {
        ROS_ERROR("[KalmanTest]: NaN/Inf found in the student's covariance matrix, coming out of lkfCorrect()");
        runtimeError();
      }

      error_position += (x.head<3>() - x_student.head<3>()).squaredNorm();
      error_velocity += (x.segment<3>(3) - x_student.segment<3>(3)).squaredNorm();
    }
  }

  double rmse_position = sqrt(error_position / n_steps);
  double rmse_velocity = sqrt(error_velocity / n_steps);

  if (_output_to_file_) {
    _output_file_ = fopen(_file_path_.c_str(), "w+");
  }

  if (rmse_position < _requirement_rmse_pos_ && rmse_velocity < _requirement_rmse_vel_) {

    ROS_INFO("[KalmanTest]: simulation finished, PASS, RMSE in pos = %.2f (limit %.2f), RMSE in vel = %.2f (limit %.2f)", rmse_position, _requirement_rmse_pos_,
             rmse_velocity, _requirement_rmse_vel_);

    if (_output_to_file_) {
      fprintf(_output_file_, "1 %.2f %.2f %.2f %.2f", rmse_position, _requirement_rmse_pos_, rmse_velocity, _requirement_rmse_vel_);
    }

  } else {

    ROS_ERROR("[KalmanTest]: simulation finished, FAIL, RMSE in pos = %.2f (limit %.2f), RMSE in vel = %.2f (limit %.2f)", rmse_position,
              _requirement_rmse_pos_, rmse_velocity, _requirement_rmse_vel_);

    if (_output_to_file_) {
      fprintf(_output_file_, "0 %.2f %.2f %.2f %.2f", rmse_position, _requirement_rmse_pos_, rmse_velocity, _requirement_rmse_vel_);
    }
  }

  if (_output_to_file_) {
    fclose(_output_file_);
  }

  ros::Duration(1.0).sleep();

  ros::shutdown();
}

//}

// | ---------------- action handlers callbacks --------------- |

/* plotValue() //{ */

void KalmanTest::plotValue(const std::string name, const double value) {
}

//}

/* visualizePose() //{ */

void KalmanTest::visualizePose(const std::string name, const double x, const double y, const double z, const double heading) {
}

//}

// | ------------------------ routines ------------------------ |

/* randd() //{ */

double KalmanTest::randd(const double from, const double to) {

  double zero_to_one = double((float)rand()) / double(RAND_MAX);

  return (to - from) * zero_to_one + from;
}

//}

/* randi() //{ */

int KalmanTest::randi(const int from, const int to) {

  double zero_to_one = double((float)rand()) / double(RAND_MAX);

  return int(double(to - from) * zero_to_one + from);
}

//}

/* isVectorFinite() //{ */

bool KalmanTest::isVectorFinite(const Eigen::VectorXd& vector, const std::string name) {

  for (int i = 0; i < vector.size(); i++) {
    if (!std::isfinite(vector[i])) {
      ROS_ERROR("[KalmanTest]: NaN detected in \"%s[%d]\"!!!", name.c_str(), i);
      return false;
    }
  }

  return true;
}

//}

/* isMatrixFinite() //{ */

bool KalmanTest::isMatrixFinite(const Eigen::MatrixXd& matrix, const std::string name) {

  for (int i = 0; i < matrix.cols(); i++) {
    for (int j = 0; j < matrix.rows(); j++) {
      if (!std::isfinite(matrix(i, j))) {
        ROS_ERROR("[KalmanTest]: NaN detected in \"%s[%d, %d]\"!!!", name.c_str(), i, j);
        return false;
      }
    }
  }

  return true;
}

//}

/* runtimeError() //{ */

void KalmanTest::runtimeError() {

  if (_output_to_file_) {

    _output_file_ = fopen(_file_path_.c_str(), "w+");

    fprintf(_output_file_, "0 %s %.2f %s %.2f", "Inf", _requirement_rmse_pos_, "Inf", _requirement_rmse_vel_);
    fclose(_output_file_);
  }

  ros::Duration(1.0).sleep();

  ros::shutdown();
}

//}

}  // namespace task_01_evaluation

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(task_01_evaluation::KalmanTest, nodelet::Nodelet)
