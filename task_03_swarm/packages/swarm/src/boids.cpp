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
  Eigen::Vector3d action = Eigen::Vector3d::Zero();
  Eigen::Vector3d target = state.target;

  Eigen::Vector3d alignment  = Eigen::Vector3d::Zero();
  Eigen::Vector3d separation = Eigen::Vector3d::Zero();
  Eigen::Vector3d cohesion   = Eigen::Vector3d::Zero();
  Eigen::Vector4d neighbours_color = Eigen::Vector4d::Zero();
  Distribution my_distribution = state.distribution;
  
  neighbours_color << my_distribution.get(0),my_distribution.get(1),my_distribution.get(2),my_distribution.get(3);


  int          dim             = my_distribution.dim();
  const int n_neighbours = state.neighbors_states.size();

  for (const auto &n_state : state.neighbors_states) {

    
    auto &[n_pos_rel, n_vel_global, n_distribution] = n_state;
    double coef = 0;
    [[maybe_unused]] double n_dist = n_pos_rel.norm();

    if(n_dist<=1.0)
    {
      coef = 10;
    }
    else
    {
      if (n_dist>10)
      {
        coef = 0.0;
      }
      else
      {
        coef = 0.5/(n_dist-1.0);
      }
    }
    
    // check if the size of my prob. distribution matches the size of the neighbour's distribution
    if (dim != n_distribution.dim()) {
      std::cout << "This should never happen. If it did, you set the previous distribution wrongly." << std::endl;
    }
    alignment  += 0.5*n_vel_global;
    alignment  += 0.5*coef*n_vel_global;
    separation += coef*n_pos_rel;
    cohesion   += n_pos_rel;

    for (int i =0;i<4;i++)
    {
      neighbours_color(i)+=n_distribution.get(i);
    }

  }
  if (n_neighbours>0)
  {
      alignment  *= (1.0/n_neighbours);
      separation *= (-1.0/n_neighbours); // use weighting function for separation
      cohesion   *= (1.0/n_neighbours);
      
      for (int i =0;i<4;i++)
      {
        neighbours_color(i)*=(1.0/(n_neighbours+1));
      }
  }  
  Distribution beacon_distribution;  
  if (state.nearby_beacon) {
    my_distribution = state.beacon_distribution;
    for (int i =0;i<4;i++)
    {
        my_distribution.set(i,0.5*state.beacon_distribution.get(i)+0.5*neighbours_color(i));
    }
  }
  else
  {
    for(int i=0;i<4;i++)
    {
      my_distribution.set(i,neighbours_color(i));
    }
  }

  action = user_params.param1*alignment + user_params.param2*target + user_params.param4*separation + user_params.param3*cohesion;  
  return {action, my_distribution};
}

//}

}  // namespace task_03_boids
