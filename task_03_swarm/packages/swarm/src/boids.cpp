#include <task_03_boids/boids.h>

namespace task_03_boids
{

typedef Eigen::Matrix<double, 8,8> Matrix8x8d;
typedef Eigen::Matrix<double, 4,8> Matrix4x8d;
typedef Eigen::Matrix<double, 8,4> Matrix8x4d;
typedef Eigen::Matrix<double, 4,4> Matrix4x4d;
typedef Eigen::Matrix<double, 8, 1> Vector8d;
typedef Eigen::Matrix<double, 4, 1> Vector4d;
// | ---------- HERE YOU MAY WRITE YOUR OWN FUNCTIONS --------- |

// Example function, can be deleted
double multiply(const double a, const double b) {
  return a * b;
}

std::tuple<Vector4d, Matrix4x4d> lkfPredict(const Vector4d &x, const Matrix4x4d &x_cov,const double &q){

  // x[k+1] = A*x[k] + B*u[k]
  Matrix4x4d A; 
  A <<1,0,0,0,
      0,1,0,0,
      0,0,1,0,
      0,0,0,1;
  Vector4d   new_x;      // the updated state vector, x[k+1]
  Matrix4x4d new_x_cov;  // the updated covariance matrix
  
  Matrix4x4d Q; 
  Q <<q,0,0,0,
      0,q,0,0,
      0,0,q,0,
      0,0,0,q;
  new_x     = A*x;
  new_x_cov = A*x_cov*A.transpose()+Q;
  
  return {new_x, new_x_cov};
}

/**
 * @brief LKF filter correction step
 *
 * @param x current state vector: x = [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, acc_x, acc_y, acc_z]^T
 * @param x_cov current state covariance: x_cov in R^{9x9}
 * @param measurement measurement vector: measurement = [pos_x, pos_y, pos_z, acc_x, acc_y, acc_z]^T
 * @param dt the time difference in seconds between now and the last iteration
 *
 * @return <new_state, new_covariance>
 */
std::tuple<Vector4d, Matrix4x4d> lkfCorrect(const Vector4d &x, const Matrix4x4d &x_cov, const Vector4d &measurement,const double &r) {

  Vector4d   new_x;      // the updated state vector, x[k+1]
  Matrix4x4d new_x_cov;  // the updated covariance matrix

  Matrix4x4d H;
  H << 1,0,0,0,
       0,1,0,0,
       0,0,1,0,
       0,0,0,1;

  Matrix4x4d R; 
  R <<r,0,0,0,
      0,r,0,0,
      0,0,r,0,
      0,0,0,r;
  // Kalman Gain
  Matrix4x4d K = x_cov*H.transpose()*((H*x_cov*H.transpose()+R).inverse()); 
  // update
  new_x = x+K*(measurement-H*x);

  Matrix4x4d Id4x4;
  Id4x4.setIdentity();

  new_x_cov = (Id4x4 - K*H)*x_cov;
  return {new_x, new_x_cov};
}
// | ------------- FILL COMPULSORY FUNCTIONS BELOW ------------ |

/* updateAgentState() //{ */

/**
 * @brief Calculate a next-iteration action of one agent given relative information of its neighbors and the direction towards a target.
 *        This method is supposed to be filled in by the student.
 *
 * @param AgentState_t Current state of the agent as defined in agent_state.h.
 * @param user_params user-controllable parameters
 * @param action_handlers functions for visualization
 *  - visualizeArrow() will publish the given arrow in the agent frame within the visualization
 *
 * @return
 *    1) XYZ vector in frame of the agent to be set as velocity command. Beware that i) the vector z-axis component will be set to 0, ii) the vector magnitude
 * will be clamped into <v_min, v_max> limits and iii) azimuth of the vector's XY-projection will be saturated such that the azimuth between the agent's current
 * velocity and the vector does not exceed a maximal change.
 *
 *       Example 1: Maximal change is d=45deg, v_min=0.1, v_max=0.2, and current velocity is (0.2, 0, 0) -> vector (0, 1, 1) will be clamped and saturated to
 * 0.2*(cos(d), sin(d), 0).
 *
 *       Example 2: Maximal change is d=45deg, v_min=0.1, v_max=0.2, and current velocity is (0.2, 0, 0) -> vector (0, -0.05, 1) will
 * be clamped and saturated to 0.1*(cos(-d), sin(-d), 0).
 *
 *    2) Probability distribution of colors to be set to the agent for next iteration. Beware that output distribution.dim() has to equal input
 * state.distribution.dim().
 */
std::tuple<Eigen::Vector3d, Distribution> Boids::updateAgentState(const AgentState_t &state, const UserParams_t &user_params,
                                                                  const ActionHandlers_t &action_handlers) {

  // TODO STUDENTS: Finish this method. The example code below can be removed, it's there just for an inspiration.

  // | --------------- MY CODE START ------------------- |
  // 1) Setup the output action
  Eigen::Vector3d action = Eigen::Vector3d::Zero();
  Eigen::Vector3d target = state.target;

  Eigen::Vector3d alignment  = state.velocity;
  Eigen::Vector3d cohesion   = Eigen::Vector3d::Zero();
  Eigen::Vector3d separation = Eigen::Vector3d::Zero();

  Eigen::Vector4d beacon_color      = Eigen::Vector4d::Zero();  
  Eigen::Vector4d neighbours_color  = Eigen::Vector4d::Zero();  
  Eigen::Vector4d own_color         = Eigen::Vector4d::Zero();  
  Eigen::Vector4d result_color         = Eigen::Vector4d::Zero();  
  
  // 2) Access my own prob. distribution of colors
  Distribution my_distribution = state.distribution;
  int          dim             = my_distribution.dim();
  
  own_color << my_distribution.get(0),my_distribution.get(1),my_distribution.get(2),my_distribution.get(3);
  neighbours_color = own_color;
  if (state.neighbors_states.size()>0)
  {
      // 3) Iterate over the states of the visible neighbors
      for (const auto &n_state : state.neighbors_states) {

        auto &[n_pos_rel, n_vel_global, n_distribution] = n_state;
        // distance to the neighbour
        [[maybe_unused]] double n_dist = n_pos_rel.norm();
        // check if the size of my prob. distribution matches the size of the neighbour's distribution
        if (dim != n_distribution.dim()) {
          std::cout << "This should never happen. If it did, you set the previous distribution wrongly." << std::endl;
        }
        alignment += n_vel_global;
        cohesion  += n_pos_rel;
        separation-= n_pos_rel;

        neighbours_color(0)+=n_distribution.get(0);
        neighbours_color(1)+=n_distribution.get(1);
        neighbours_color(2)+=n_distribution.get(2);
        neighbours_color(3)+=n_distribution.get(3);
      }

      // 4) Scale the action by user parameter
      alignment  /=(state.neighbors_states.size()+1);
      // printVector3d(alignment, "Alignment: ");
      cohesion   /=state.neighbors_states.size();
      // printVector3d(cohesion, "Cohesion: ");
      separation /=state.neighbors_states.size();
      // printVector3d(separation, "Separation: ");
      neighbours_color(0)/=(state.neighbors_states.size()+1.0);
      neighbours_color(1)/=(state.neighbors_states.size()+1.0);
      neighbours_color(2)/=(state.neighbors_states.size()+1.0);
      neighbours_color(3)/=(state.neighbors_states.size()+1.0);
  }

  action = user_params.param1 * alignment + user_params.param2*cohesion + user_params.param3*separation + user_params.param4* alignment.norm()*target;
  
  // double q= 1.0;
  // Matrix4x4d x_cov;  
  // x_cov << 1,0,0,0,
  //          0,1,0,0,
  //          0,0,1,0,
  //          0,0,0,1;
  
  // Am I nearby a beacon?
  Distribution beacon_distribution;  

  if (state.nearby_beacon) {
    beacon_distribution = state.beacon_distribution;
    beacon_color << beacon_distribution.get(0),beacon_distribution.get(1),beacon_distribution.get(2),beacon_distribution.get(3);
    result_color = 0.6*beacon_color + 0.4*neighbours_color;
  }
  else 
  {
    result_color = 0.4*beacon_color + 0.6*neighbours_color;
  }
  // std::tie(beacon_color,x_cov) = lkfPredict(neighbours_color,x_cov,user_params.param8);
  // std::tie(result_color,x_cov) = lkfCorrect(neighbours_color,x_cov,beacon_color,user_params.param9);
  
  // result_color = user_params.param5*beacon_color + user_params.param6*neighbours_color;
  // double total = result_color.sum();
  // result_color(0)/=total;
  // result_color(1)/=total;
  // result_color(2)/=total;
  // result_color(3)/=total;
  
  my_distribution.set(0,result_color(0));
  my_distribution.set(1,result_color(1));
  my_distribution.set(2,result_color(2));
  my_distribution.set(3,result_color(3));
  
  // 5) Print the output action
  // printVector3d(action, "Action: ");

  // 6) Visualize the arrow in RViz
  action_handlers.visualizeArrow("action", action, Color_t{0.0, 0.0, 0.0, 1.0});
  // | ------------------- EXAMPLE CODE START ------------------- |

  // Eigen::Vector3d action = Eigen::Vector3d::Zero();
  // Eigen::Vector3d target = state.target;


  // // Call custom functions, e.g., useful for dynamic weighting
  // [[maybe_unused]] double x = multiply(5.0, 10.0);

  // // Access my own prob. distribution of colors
  // Distribution my_distribution = state.distribution;
  // int          dim             = my_distribution.dim();

  // // Am I nearby a beacon?
  // Distribution beacon_distribution;
  // if (state.nearby_beacon) {
  //   beacon_distribution = state.beacon_distribution;
  // }

  // // Iterate over the states of the visible neighbors
  // for (const auto &n_state : state.neighbors_states) {

  //   auto &[n_pos_rel, n_vel_global, n_distribution] = n_state;

  //   // distance to the neighbour
  //   [[maybe_unused]] double n_dist = n_pos_rel.norm();

  //   // check if the size of my prob. distribution matches the size of the neighbour's distribution
  //   if (dim != n_distribution.dim()) {
  //     std::cout << "This should never happen. If it did, you set the previous distribution wrongly." << std::endl;
  //   }
  // }

  // // Example: scale the action by user parameter
  // action = user_params.param1 * target;

  // // Print the output action
  // printVector3d(action, "Action: ");

  // // Visualize the arrow in RViz
  // action_handlers.visualizeArrow("action", action, Color_t{0.0, 0.0, 0.0, 1.0});

  // | -------------------- EXAMPLE CODE END -------------------- |

  return {action, my_distribution};
}

//}

}  // namespace task_03_boids
