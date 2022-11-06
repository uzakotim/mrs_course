#ifndef FORMATION_H
#define FORMATION_H
#include <task_02_formation/task_02_formation.h>
#include <student_headers/astar.h>

namespace task_02_formation
{
typedef Eigen::Matrix<double, 6, 6> Matrix6x6d;
typedef Eigen::Matrix<double, 6, 3> Matrix6x3d;
typedef Eigen::Matrix<double, 3, 6> Matrix3x6d;
typedef Eigen::Matrix<double, 3, 3> Matrix3x3d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 3, 1> Vector3d;

class Formation : public Task02Formation {

public:

  /**
   * @brief The formation controller initialization method. This method will be called ONLY ONCE in the lifetime of the controller.
   * Use this method do do any heavy pre-computations.
   */
  void init();
  std::tuple<Vector6d, Matrix6x6d> lkfPredict(const Vector6d &x, const Matrix6x6d &x_cov,const double &dt);
  std::tuple<Vector6d, Matrix6x6d> lkfCorrect(const Vector6d &x, const Matrix6x6d &x_cov, const Vector3d &measurement);
  void resetKF();
  /**
   * @brief Function to create points around the main point
   * 
   * @param input 
   * @param resolution 
   * @return std::vector<Eigen::Vector3d> 
   */
  std::vector<Eigen::Vector3d> createMinkowskyPoints(astar::Position input,double resolution);
  std::vector<Eigen::Vector3d> createMinkowskyPointsHard(astar::Position input,double resolution);
  /**
   * @brief This method calculates paths for each UAV from an initial state towards a desired final state.
   * This method is supposed to be filled in by the student.
   *
   * @param initial_states A vector of 3D initial positions for each UAV.
   * @param final_states A vector of 3D final positions of each UAV.
   *
   * @return A vector of paths, each path being a vector of 3D positions for each UAV. The initial and final states are supposed
   * to be the part of the path for each UAV. The expected result for point I, as the starting point for a UAV and point F as the final
   * point for a UAV, can be, e.g.:
   *   I -> F
   *   I -> B -> F
   *   I -> B -> C -> F
   * The following paths are considered invalid:
   *   I
   *   F
   *   D -> D
   *   I -> D
   *   F -> I
   */
  std::vector<std::vector<Eigen::Vector3d>> getPathsReshapeFormationHard(const std::vector<Eigen::Vector3d> &initial_states,
                                                                     const std::vector<Eigen::Vector3d> &final_states);
  std::vector<std::vector<Eigen::Vector3d>> getPathsReshapeFormation(const std::vector<Eigen::Vector3d> &initial_states,
                                                                     const std::vector<Eigen::Vector3d> &final_states);
  std::vector<std::vector<Eigen::Vector3d>> assignPaths(const FormationState_t &formation_state,const Eigen::Vector3d &one, const Eigen::Vector3d &two, const Eigen::Vector3d &three);

  /**
   * @brief The method for calculating a 3D position of source of signal based on the positions of UAVs and the measured distances to the source.
   *
   * @param uav_states Vector of 3D positions of each UAV.
   * @param distances Vector of the measured distances from each UAV to the source of signal.
   *
   * @return the estimated 3D position of the source of radiation.
   */
  Eigen::Vector3d multilateration(const std::vector<Eigen::Vector3d> &uav_states, const Eigen::VectorXd &distances);

  /**
   * @brief The main routine for controlling the experiment. The method is called regularly at 10 Hz.
   *
   * @param formation_state The current state of the formation. The state contains:
   * - absolute position of the virtual leader UAV
   * - positions of the follower UAVs relative the virtual leader
   * - flag stating whether the formation is moving or whether it is stationary
   * @param ranging A structure containing the measured distances form each UAV to the source of radio signal.
   * @param time_stamp Current time in seconds.
   * @param action_handlers This structure provides users with functions to control the formation:
   *   reshapeFormation() will reshape the formation relative the the virtual leader's position.
   *   moveFormation() will move the formation by moving the virtual leader. The followers will follow.
   * Moreover, the action_handlers structure provides additional methods for data visualization.
   */
  void update(const FormationState_t &formation_state, const Ranging_t &ranging, const double &time_stamp, ActionHandlers_t &action_handlers);

private:
  // | -------- Put any custom variables and methods here ------- |

  int current_stage_ = 0;
  int count_data_ = 0;
  std::deque<Eigen::Vector3d> measurements;
  Eigen::Vector3d avg_target;
  Eigen::Vector3d estimated_state;
  bool xTrueYfalse = true;
  bool success = false;
  double time_last_call_;
  Vector3d init_input;
  Matrix6x6d Q;
  Matrix3x3d R;

  Vector6d new_x;
  Matrix6x6d new_cov;
};

}  // namespace task_02_formation

#endif  // FORMATION_H
