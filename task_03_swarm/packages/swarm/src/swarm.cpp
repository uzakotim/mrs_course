#include <student_headers/swarm.h>

namespace task_03_swarm
{

// --------------------------------------------------------------
// |                    the library interface                   |
// --------------------------------------------------------------

/* init() //{ */

/**
 * @brief The swarm controller (single unit) initialization method. This method will be called ONLY ONCE in the lifetime of the controller.
 * Use this method to do any heavy pre-computations.
 *
 * @param visibility radius of the agent
 */
void Swarm::init(const double visibility_radius) {

  _visibility_radius_ = visibility_radius;
  no_decision = true;
  major_idx = 0;
  prev_major_idx = 0;
  result = Eigen::Vector3d(0,0,0);
  prev_result = Eigen::Vector3d(0,0,0);
}

//}

// | ------ Compulsory functions to be filled by students ----- |

/* updateAction() //{ */

/**
 * @brief This method calculates a next-iteration action of one UAV given relative information of its neighbors and obstacles; and the direction towards the
 *        moving target. This method is supposed to be filled in by the student.
 *
 * @param perception Current perceptual information of this UAV. Defined in perception.h. It contains:
 *  - current time
 *  - target vector: 3D vector towards the moving robot in the UAV body frame
 *  - neighbors defined by:
 *    - their position in the UAV body frame
 *    - the variables shared through the communication network
 *  - obstacles consisting of:
 *    - 3D vector from the body frame to the closest obstacle in the environment
 *    - 4 gates (pairs of 2 gate edges) in the UAV body frame
 * @param user_params user-controllable parameters
 * @param action_handlers functions for visualization and data sharing among the UAVs:
 *  - shareVariables(int, int, double) will share the three basic-type variables among the UAVs
 *  - visualizeArrow() will publish the given arrow in the UAV body frame within the visualization
 *  - visualizeArrowFrom() will publish the given arrow at position given in the UAV body frame within the visualization
 *  - visualizeCube() will publish a cube in the UAV body frame within the visualization
 *
 * @return Next-iteration action for this UAV given as a 3D vector. Zero vector is expected if no action should be performed. Beware that i) the vector z-axis
 * component will be set to 0, ii) the vector vector magnitude will be clamped into <0, v_max> limits, and iii) the vector will be passed to a velocity
 * controller of the UAV.
 *
 *       Example 1: v_max=0.75 -> vector (1, 0, 1) will be saturated to 0.75*(1, 0, 0).
 */
Eigen::Vector3d Swarm::calculateCohesion(const Perception_t &perception,const UserParams_t &user_params)
{

  // int idx = selectNeighClosest(perception);
  Eigen::Vector3d result (0,0,0);
  for (auto& uav : perception.neighbors)
  {
      result += uav.position;
  }
  result *= (1.0/perception.neighbors.size());
  return result;
}
Eigen::Vector3d Swarm::calculateCohesionNeigh(const Perception_t &perception,const UserParams_t &user_params)
{

  int idx = selectNeighClosest(perception);
  Eigen::Vector3d result (0,0,0);
  result = perception.neighbors[idx].position;
  return result;
}
Eigen::Vector3d Swarm::calculateAttraction(const Eigen::Vector3d &result,const Perception_t &perception,const UserParams_t &user_params){ 
  return result;
}
Eigen::Vector3d Swarm::calculateSeparation(const Perception_t &perception,const UserParams_t &user_params){

  Eigen::Vector3d result (0,0,0);
  double final_weight;
  for (auto& uav : perception.neighbors)
  {
    auto [defined, weight] = weightingFunction(uav.position.norm(),_visibility_radius_,user_params.param8,0.0);
    if (defined)
    {
      final_weight = weight;
    }
    else
    {
      final_weight = 1e3;
    }
    result += final_weight*uav.position;
  } 
  for (auto& gate: perception.obstacles.gates)
  {
    auto [defined1, weight1] = weightingFunction(gate.first.norm(),_visibility_radius_,user_params.param9,0.0);
    if (defined1)
    {
      final_weight = weight1;
    }
    else
    {
      final_weight = 1e3;
    }

    result += final_weight*gate.first;
    
    auto [defined2, weight2] = weightingFunction(gate.second.norm(),_visibility_radius_,user_params.param9,0.0);
    if (defined2)
    {
      final_weight = weight2;
    }
    else
    {
      final_weight = 1e3;
    }

    result += final_weight*gate.second;
  }
  result *= (-1.0/(perception.neighbors.size()+2.0*perception.obstacles.gates.size()));
  return result;
}
Eigen::Vector3d Swarm::calculateAvoidance(const Perception_t &perception,const UserParams_t &user_params){

  Eigen::Vector3d result (0,0,0);

  auto [defined3, weight3] = weightingFunction(perception.obstacles.closest.norm(),_visibility_radius_,user_params.param9,0.0);
  double final_weight;
  if (defined3)
  {
    final_weight = weight3;
  }
  else
  {
    final_weight = 1e3;
  }
  result -= final_weight*perception.obstacles.closest;
  return result;
}

Eigen::Vector3d Swarm::updateAction(const Perception_t &perception, const UserParams_t &user_params, const ActionHandlers_t &action_handlers) {

  // return desired velocity
  double neigh_0_angle = perception.neighbors[0].shared_variables.dbl;
  double neigh_1_angle = perception.neighbors[1].shared_variables.dbl;

  int niegh_0_priority = perception.neighbors[0].shared_variables.int2;
  int niegh_1_priority = perception.neighbors[1].shared_variables.int2;
  unsigned int idx_closest = Swarm::selectNeighClosest(perception);

  if ((perception.neighbors[idx_closest].shared_variables.int2==int_var_2)&&(perception.neighbors[idx_closest].position.norm()<3.0))
  {
    int_var_2++;
  }
  if (perception.neighbors[idx_closest].position.norm()>3.0)
  {
    int_var_2 = 0;
  }

  unsigned int i = Swarm::selectGateClosest(perception.obstacles);
  double cur_distance_to_gate = (perception.obstacles.gates[i].first.norm()+perception.obstacles.gates[i].second.norm())/2.0;
  double threshold = user_params.param5;
  double cohesion_reduction = 1.0; 
  double avoidance_reduction = 1.0;
  double turn_on_attraction  = 1.0;
  int neigh_flag_1,neigh_flag_2;

  neigh_flag_1 = perception.neighbors[0].shared_variables.int1;
  neigh_flag_2 = perception.neighbors[1].shared_variables.int1;


  double double_var_3 = std::atan2(perception.target_vector[1],perception.target_vector[0]);
  if ((std::abs(double_var_3 - neigh_0_angle)>threshold) && (std::abs(double_var_3 - neigh_1_angle)>threshold))
  {
    result = Eigen::Vector3d (perception.target_vector.norm()*cos((neigh_0_angle+neigh_1_angle)/2.0),perception.target_vector.norm()*sin((neigh_0_angle+neigh_1_angle)/2.0),0);
  }
  else
  {
    result = perception.target_vector;
  }
  
  unsigned int idx_farthest = Swarm::selectNeighFarthest(perception);
  double cur_x = std::abs(result(0));
  double cur_y = std::abs(result(1));
  double gate_threshold = 1.5;
  
  direction = Swarm::calculateDirection(perception, result);
  directions.push_back(targetToDirection(direction));
  if (counter >user_params.param6)
  {
      std::map<int,int> direction_map = countIntegers(directions);
      if ((cur_y>user_params.param7) && ((prev_major_idx == 1)||(prev_major_idx==3))) 
      {   
        if (cur_x>user_params.param7)
        {
          major_idx = prev_major_idx;
        }
        else
        {
          if (cur_distance_to_gate>gate_threshold)
            major_idx = getMajority(direction_map);
        }
      }
      if ((cur_y<=user_params.param7) && ((prev_major_idx == 1)||(prev_major_idx==3))) 
      {   
        if (cur_x>user_params.param7)
        {
            //problem x
            if (cur_distance_to_gate>gate_threshold)
            {
              if (result(0)>0)
              {
                major_idx = 1;
              }
              else 
              {
                major_idx = 3;
              }
            }
        }
        else
        {
          filtered_direction = Eigen::Vector3d(0,0,0);
          major_idx = 0;
          std::cout<<"Found robot!!"<<'\n';
        }
      }

      if ((cur_y>user_params.param7) && ((prev_major_idx == 2)||(prev_major_idx==4))) 
      {   
        if (cur_x>user_params.param7)
        {
          if (cur_distance_to_gate>gate_threshold)
            major_idx = getMajority(direction_map);
        }
        else
        {
          //problem y
          if (cur_distance_to_gate>gate_threshold)
          {
            if (result(1)>0)
            {
              major_idx = 2;
            }
            else 
            {
              major_idx = 4;
            }
          }
        }
      }

      if ((cur_y<=user_params.param7) && ((prev_major_idx == 2)||(prev_major_idx==4))) 
      {      
        if (cur_x>user_params.param7)
        {
          if (cur_distance_to_gate>gate_threshold)
            major_idx = getMajority(direction_map);
        }
        else
        {
          filtered_direction = Eigen::Vector3d(0,0,0);
          major_idx = 0;
          std::cout<<"Found robot!!"<<'\n';
        }
      }
      if (prev_major_idx==0)
      {
        major_idx = getMajority(direction_map);
      }

      std::cout<<"current direction: "<<major_idx<<'\n';
      int_var_1 = major_idx;
      std::deque<int> agreement = {major_idx,perception.neighbors[0].shared_variables.int1,perception.neighbors[1].shared_variables.int1};
      std::map<int,int> agreement_map = countIntegers(agreement);
      if (cur_distance_to_gate>1.5)
      {
        major_idx = getMajority(agreement_map);
        std::cout<<"agreed direction:" <<major_idx<<'\n';
      }
      if (major_idx != 0)
      { 
          filtered_direction = (perception.obstacles.gates[major_idx-1].first+perception.obstacles.gates[major_idx-1].second)/2.0;
          filtered_direction /= filtered_direction.norm();
      }
      // resetting counter and directions
      counter-=user_params.param6/2;
      for (size_t k = 0;k<user_params.param6/2;k++)
      {
        directions.pop_front();
      }
  }
  Eigen::Vector3d cohesion = cohesion_reduction*user_params.param1*calculateCohesion(perception,user_params);
  if (int_var_2>(perception.neighbors[idx_closest].shared_variables.int2))
  {
    // if my priority is greater than priority of closest neighbour
    //      I do not fly to target
    turn_on_attraction = 0.0;
    if (perception.neighbors[idx_closest].position.norm()>3.0)
      cohesion        = cohesion_reduction*user_params.param1*calculateCohesionNeigh(perception,user_params);
  }

  if (perception.neighbors[idx_farthest].position.norm()>6.0)
  {
    turn_on_attraction = 0.0;
  }
  if (((perception.obstacles.gates[i].first+perception.obstacles.gates[i].second)/2.0).norm()<=1.5)
  {
    filtered_direction = prev_direction;
    turn_on_attraction = 0.5;
  }
  Eigen::Vector3d separation      = user_params.param2*calculateSeparation(perception,user_params);
  Eigen::Vector3d avoidance       = avoidance_reduction*user_params.param3*calculateAvoidance(perception,user_params); 

  Eigen::Vector3d attraction      = turn_on_attraction*user_params.param4*calculateAttraction(filtered_direction,perception,user_params);

  action_handlers.visualizeArrow("cohesion", cohesion, Color_t{0.0, 1.0, 0.0, 1.0});
  action_handlers.visualizeArrow("separation", separation, Color_t{1.0, 1.0, 1.0, 0.5});
  action_handlers.visualizeArrow("avoidance", avoidance, Color_t{1.0, 0.0, 0.0, 1.0});
  action_handlers.visualizeArrow("direction", filtered_direction, Color_t{0.0, 0.0, 1.0, 1.0});

  counter++;
  prev_major_idx = major_idx;
  prev_direction = filtered_direction;
  prev_distance_to_gate = cur_distance_to_gate;
  prev_result = result;
  action_handlers.shareVariables(int_var_1,int_var_2,double_var_3);
  return cohesion + attraction + separation + avoidance;
}

//}

/* weightingFunction() //{ */

/**
 * @brief Non-linear weighting of forces.
 *
 * The function is to be non-increasing, non-negative, and grows to infinity as the distance is approaching the lower bound (the safety distance). Below the
 * lower bound (including), the function is to be undefined. Over the visibility range, the function shall return 0.
 *
 * @param distance to an agent/obstacle
 * @param visibility visibility range of the UAVs
 * @param safety distance: min distance to other UAVs or obstacles
 * @param desired distance: desired distance to other UAVs or obstacles (does not need to be used)
 *
 * @return
 *   bool:   True if function is defined for the given distance, False otherwise
 *   double: Weight for an agent/obstacle at given distance, if function is defined for the given distance.
 */
std::tuple<bool, double> Swarm::weightingFunction(const double distance, const double visibility, const double safety_distance,
                                                  [[maybe_unused]] const double desired_distance) {

  // TODO: Filling this function is compulsory!
  double coef = 0.0;
  if (distance<=0)
  {
    return {false, 0.0};
  }
  if(distance<=safety_distance)
  {
    return {false,0.0};
  }
  else 
  {
    if (distance>visibility)
    {
      coef = 0.0;
    }
    else
    {
      coef = 0.5/(distance-safety_distance);
    }

  }
  return {true, coef};
}

//}

// | -- Helper methods to be filled in by students if needed -- |

/* targetToDirection() //{ */

int Swarm::targetToDirection(const Eigen::Vector3d &target_vector) {

  double x = target_vector.x();
  double y = target_vector.y();
  
  double result_angle = std::atan2(y,x);
    
  if (result_angle>=2*M_PI)
  {
    result_angle -= 2*M_PI;
  }
  if (result_angle<=0.0)
  {
    result_angle += 2*M_PI;
  }
  
  if (((result_angle>11*M_PI/6.0) && (result_angle<=2*M_PI))||((result_angle>=0.0) && (result_angle<M_PI/6.0)))
  {
    // std::cout<<"RIGHT"<<'\n';
    return 1;
  } else if ((result_angle>=M_PI/6.0) && (result_angle<5*M_PI/6.0))
  {
    // std::cout<<"UP"<<'\n';
    return 2;
  }else if ((result_angle>=5*M_PI/6.0) && (result_angle<7*M_PI/6.0))
  {
    // std::cout<<"LEFT"<<'\n';
    return 3;
  }
  else if ((result_angle>=7*M_PI/6.0) && (result_angle<11*M_PI/6.0))
  {
    // std::cout<<"DOWN"<<'\n';
    return 4;
  }
  else
  {
    return 0;
  }
}

//}

/* robotsInIdenticalStates() //{ */

bool Swarm::robotsInIdenticalStates(const Perception_t &perception) {

  // TODO: fill if want to use
  std::cout << "[ERROR] robotsInIdenticalStates() not implemented. Returning false if there are any neighbors, true otherwise." << std::endl;

  for (unsigned int i = 0; i < perception.neighbors.size(); i++) {
    return false;
  }

  return true;
}

//}

/* anyRobotInState() //{ */

bool Swarm::anyRobotInState(const Perception_t &perception, const State_t &state) {

  // TODO: fill if want to use
  std::cout << "[ERROR] robotsInIdenticalStates() not implemented. Returning true if there are any neighbors, false otherwise." << std::endl;

  for (unsigned int i = 0; i < perception.neighbors.size(); i++) {
    return true;
  }

  return false;
}

//}

// | ------------ Helper methods for data handling ------------ |

/* selectGateInDirection() //{ */

/**
 * @brief Finds the index of the gate in the given direction.
 *
 * @return index of the gate in the obstacles struct (access by: obstacles.gates[dir_idx])
 */
unsigned int Swarm::selectGateInDirection(const Direction_t &direction, const Obstacles_t &obstacles) {

  switch (direction) {

    case UP: {
      return 1;

      break;
    }

    case DOWN: {
      return 3;

      break;
    }

    case LEFT: {
      return 2;

      break;
    }

    case RIGHT: {
      return 0;

      break;
    }

    case NONE: {
      std::cout << "[ERROR] selectGateInDirection() given direction=NONE. Can't determine the gate, returning G1." << std::endl;

      break;
    }
  }

  return 0;
}

//}

/* selectClosestNeighbour() //{ */

/**
 * @brief Finds the index of the gate closest to the agent.
 *
 * @return index of the gate in the obstacles struct (access by: obstacles.gates[min_idx])
 */
unsigned int Swarm::selectNeighClosest(const Perception_t &perception) {

  unsigned int min_idx  = 0;
  double       min_dist = perception.neighbors[0].position.norm();

  for (unsigned int i = 0; i < perception.neighbors.size(); i++) {

    const double   G_dist      = perception.neighbors[i].position.norm();
    if (G_dist < min_dist) {
      min_idx  = i;
      min_dist = G_dist;
    }
  }

  return min_idx;
}
unsigned int Swarm::selectNeighFarthest(const Perception_t &perception) {

  unsigned int max_idx  = 0;
  double       max_dist = perception.neighbors[0].position.norm();

  for (unsigned int i = 0; i < perception.neighbors.size(); i++) {

    const double   G_dist      = perception.neighbors[i].position.norm();
    if (G_dist > max_dist) {
      max_idx  = i;
      max_dist = G_dist;
    }
  }

  return max_idx;
}

unsigned int Swarm::selectGateClosest(const Obstacles_t &obstacles) {

  unsigned int min_idx  = 0;
  double       min_dist = obstacles.gates[0].first.norm();

  for (unsigned int i = 0; i < obstacles.gates.size(); i++) {

    const auto   G      = obstacles.gates[i];
    const double G_dist = (G.first.norm() < G.second.norm()) ? G.first.norm() : G.second.norm();

    if (G_dist < min_dist) {
      min_idx  = i;
      min_dist = G_dist;
    }
  }

  return min_idx;
}
//}

/* computeMutualDistances() //{ */

/**
 * @brief Computes the vector of mutual distances between agents
 *
 * @return vector of all mutual distances (unordered) between all agents (incl. me)
 */
std::vector<double> Swarm::computeMutualDistances(const std::vector<Neighbor_t> &neighbors) {

  // All known positions (incl. mine)
  std::vector<Eigen::Vector3d> positions = {Eigen::Vector3d::Zero()};
  for (const auto &n : neighbors) {
    positions.push_back(n.position);
  }

  // Compute all mutual distances
  std::vector<double> distances;
  for (unsigned int i = 0; i < positions.size(); i++) {
    for (unsigned int j = i + 1; j < positions.size(); j++) {
      distances.push_back((positions[j] - positions[i]).norm());
    }
  }

  return distances;
}

//}

/* integersAreUnique() //{ */

/**
 * @brief Check if integers in a vector are unique
 *
 * @return true if all the integers are unique
 */
bool Swarm::integersAreUnique(const std::deque<int> &integers) {

  const auto count_map = countIntegers(integers);

  return count_map.size() == integers.size();
}

//}

/* countIntegers() //{ */

/* Computes frequency of integers in the given array
 *
 * @return map of counts for each key in the initial list
 * */
std::map<int, int> Swarm::countIntegers(const std::deque<int> &integers) {

  std::map<int, int> count_map;

  for (const int i : integers) {
    if (count_map.find(i) == count_map.end()) {
      count_map[i] = 0;
    }
    count_map[i]++;
  }

  return count_map;
}

//}

/* getMajority() //{ */

/* Return the key and value of the first maximal element in the given idx->count map.
 *
 * @return key, count
 * */
int Swarm::getMajority(const std::map<int, int> &integer_counts) {

  if (integer_counts.empty()) {
    // return {-1, -1};
    return -1;

  }

  int max_idx = 0;
  int max_val = 0;

  for (auto it = integer_counts.begin(); it != integer_counts.end(); ++it) {
    if (it->second > max_val) {
      max_idx = it->first;
      max_val = it->second;
    }
  }

  return max_idx;
}

// std::tuple<int, int> Swarm::getMajority(const std::deque<int> &integers) {
  // return getMajority(countIntegers(integers));
// }

Eigen::Vector3d Swarm::calculateDirection(const Perception_t &perception, const Eigen::Vector3d & result )
{
    Eigen::Vector3d direction(0,0,0);
    double result_angle = std::atan2(result(1),result(0));
    
    if (result_angle>=2*M_PI)
    {
      result_angle -= 2*M_PI;
    }
    if (result_angle<=0.0)
    {
      result_angle += 2*M_PI;
    }
    
    if (((result_angle>11*M_PI/6.0) && (result_angle<=2*M_PI))||((result_angle>=0.0) && (result_angle<M_PI/6.0)))
    {
      // std::cout<<"RIGHT"<<'\n';
      direction = (perception.obstacles.gates[0].first+perception.obstacles.gates[0].second)/2.0;
      direction /= direction.norm();
    }
    if ((result_angle>M_PI/6.0) && (result_angle<=5*M_PI/6.0))
    {
      // std::cout<<"UP"<<'\n';
      direction = (perception.obstacles.gates[1].first+perception.obstacles.gates[1].second)/2.0;
      direction /= direction.norm();
    }
    if ((result_angle>5*M_PI/6.0) && (result_angle<=7*M_PI/6.0))
    {
      // std::cout<<"LEFT"<<'\n';
      direction = (perception.obstacles.gates[2].first+perception.obstacles.gates[2].second)/2.0;
      direction /= direction.norm();
    }
    if ((result_angle>7*M_PI/6.0) && (result_angle<=11*M_PI/6.0))
    {
      // std::cout<<"DOWN"<<'\n';
      direction = (perception.obstacles.gates[3].first+perception.obstacles.gates[3].second)/2.0;
      direction /= direction.norm();
    }
    return direction;
}

//}

// | --------- Helper methods for data-type conversion -------- |

/* stateToInt() //{ */

// Same method exists for Direction_t in task_03_common/direction.h.
int Swarm::stateToInt(const State_t &state) {
  return static_cast<int>(state);
}

//}

/* intToState() //{ */

// Same method exists for Direction_t in task_03_common/direction.h.
State_t Swarm::intToState(const int value) {
  return static_cast<State_t>(value);
}

//}

// | --------------- Helper methods for printing -------------- |

/* stateToString() //{ */

// Same method exists for Direction_t in task_03_common/direction.h.
std::string Swarm::stateToString(const State_t &state) {

  // TODO: fill with your states if you want to use this method
  switch (state) {

    case INIT_STATE: {
      return "INIT_STATE";

      break;
    }

    case AGREEING_ON_DIRECTION: {
      return "AGREEING_ON_DIRECTION";

      break;
    }

    default: {
      break;
    }
  }

  return "UNKNOWN";
}

//}

}  // namespace task_03_swarm
