#include <student_headers/formation.h>
#define RESHAPING_SIZE 10
namespace task_02_formation
{

// --------------------------------------------------------------
// |                    the library interface                   |
// --------------------------------------------------------------

/* init() //{ */

/**
 * @brief The formation controller initialization method. This method will be called ONLY ONCE in the lifetime of the controller.
 * Use this method to do any heavy pre-computations.
 */
void Formation::init() {
}

std::vector<Eigen::Vector3d> Formation::createMinkowskyPoints(Eigen::Vector3d input,double resolution)
{   
    std::vector< std::vector<int> > EXPANSION_DIRECTIONS = {{-1, -1, -1}, {-1, -1, 0}, {-1, -1, 1}, {-1, 0, -1}, {-1, 0, 0}, {-1, 0, 1}, {-1, 1, -1},
                                                            {-1, 1, 0},   {-1, 1, 1},  {0, -1, -1}, {0, -1, 0},  {0, -1, 1}, {0, 0, -1}, {0, 0, 1},
                                                            {0, 1, -1},   {0, 1, 0},   {0, 1, 1},   {1, -1, -1}, {1, -1, 0}, {1, -1, 1}, {1, 0, -1},
                                                            {1, 0, 0},    {1, 0, 1},   {1, 1, -1},  {1, 1, 0},   {1, 1, 1}
                                                            };
    std::vector< std::vector<int> > EXPANSION_DIRECTIONS_HARD = {{ -2 , -2 , -2 },{ -2 , -2 , -1 },{ -2 , -2 , 0 },{ -2 , -2 , 1 },{ -2 , -2 , 2 },{ -2 , -1 , -2 },{ -2 , -1 , -1 },{ -2 , -1 , 0 },{ -2 , -1 , 1 },{ -2 , -1 , 2 },{ -2 , 0 , -2 },{ -2 , 0 , -1 },{ -2 , 0 , 0 },{ -2 , 0 , 1 },{ -2 , 0 , 2 },{ -2 , 1 , -2 },{ -2 , 1 , -1 },{ -2 , 1 , 0 },{ -2 , 1 , 1 },{ -2 , 1 , 2 },{ -2 , 2 , -2 },{ -2 , 2 , -1 },{ -2 , 2 , 0 },{ -2 , 2 , 1 },{ -2 , 2 , 2 },{ -1 , -2 , -2 },{ -1 , -2 , -1 },{ -1 , -2 , 0 },{ -1 , -2 , 1 },{ -1 , -2 , 2 },{ -1 , -1 , -2 },{ -1 , -1 , -1 },{ -1 , -1 , 0 },{ -1 , -1 , 1 },{ -1 , -1 , 2 },{ -1 , 0 , -2 },{ -1 , 0 , -1 },{ -1 , 0 , 0 },{ -1 , 0 , 1 },{ -1 , 0 , 2 },{ -1 , 1 , -2 },{ -1 , 1 , -1 },{ -1 , 1 , 0 },{ -1 , 1 , 1 },{ -1 , 1 , 2 },{ -1 , 2 , -2 },{ -1 , 2 , -1 },{ -1 , 2 , 0 },{ -1 , 2 , 1 },{ -1 , 2 , 2 },{ 0 , -2 , -2 },{ 0 , -2 , -1 },{ 0 , -2 , 0 },{ 0 , -2 , 1 },{ 0 , -2 , 2 },{ 0 , -1 , -2 },{ 0 , -1 , -1 },{ 0 , -1 , 0 },{ 0 , -1 , 1 },{ 0 , -1 , 2 },{ 0 , 0 , -2 },{ 0 , 0 , -1 },{ 0 , 0 , 0 },{ 0 , 0 , 1 },{ 0 , 0 , 2 },{ 0 , 1 , -2 },{ 0 , 1 , -1 },{ 0 , 1 , 0 },{ 0 , 1 , 1 },{ 0 , 1 , 2 },{ 0 , 2 , -2 },{ 0 , 2 , -1 },{ 0 , 2 , 0 },{ 0 , 2 , 1 },{ 0 , 2 , 2 },{ 1 , -2 , -2 },{ 1 , -2 , -1 },{ 1 , -2 , 0 },{ 1 , -2 , 1 },{ 1 , -2 , 2 },{ 1 , -1 , -2 },{ 1 , -1 , -1 },{ 1 , -1 , 0 },{ 1 , -1 , 1 },{ 1 , -1 , 2 },{ 1 , 0 , -2 },{ 1 , 0 , -1 },{ 1 , 0 , 0 },{ 1 , 0 , 1 },{ 1 , 0 , 2 },{ 1 , 1 , -2 },{ 1 , 1 , -1 },{ 1 , 1 , 0 },{ 1 , 1 , 1 },{ 1 , 1 , 2 },{ 1 , 2 , -2 },{ 1 , 2 , -1 },{ 1 , 2 , 0 },{ 1 , 2 , 1 },{ 1 , 2 , 2 },{ 2 , -2 , -2 },{ 2 , -2 , -1 },{ 2 , -2 , 0 },{ 2 , -2 , 1 },{ 2 , -2 , 2 },{ 2 , -1 , -2 },{ 2 , -1 , -1 },{ 2 , -1 , 0 },{ 2 , -1 , 1 },{ 2 , -1 , 2 },{ 2 , 0 , -2 },{ 2 , 0 , -1 },{ 2 , 0 , 0 },{ 2 , 0 , 1 },{ 2 , 0 , 2 },{ 2 , 1 , -2 },{ 2 , 1 , -1 },{ 2 , 1 , 0 },{ 2 , 1 , 1 },{ 2 , 1 , 2 },{ 2 , 2 , -2 },{ 2 , 2 , -1 },{ 2 , 2 , 0 },{ 2 , 2 , 1 },{ 2 , 2 , 2 }
                                                            };
    std::vector< std::vector<int> > EXPANSION_DIRECTIONS_EASY = {{-1, -1, -1}, {-1, -1, 1},
                                                                {-1, 1, -1},{-1, 1, 1},   
                                                                {1, -1, -1},{1, -1, 1}, 
                                                                {1, 1, -1},{1, 1, 1}};
    std::vector<Eigen::Vector3d> points;
    points.push_back(input);
    for (auto vertex: EXPANSION_DIRECTIONS_HARD)
    {
        Eigen::Vector3d point;
        point << input(0)+resolution*vertex[0],input(1)+resolution*vertex[1], input(2)+resolution*vertex[2];
        points.push_back(point);
    }
    return points;
}
//}

/* getPathsReshapeFormation() //{ */

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
std::vector<std::vector<Eigen::Vector3d>> Formation::getPathsReshapeFormation(const std::vector<Eigen::Vector3d> &initial_states,
                                                                              const std::vector<Eigen::Vector3d> &final_states) {

  // how many UAVs do we have
  int n_uavs = initial_states.size();

  // initialize the vector of paths
  std::vector<std::vector<Eigen::Vector3d>> paths;
  // set resolution
  const double resolution = 0.7;
  astar::Astar astar(resolution);
   // initialize obstacles
  std::set<astar::Cell> obstacles_fixed;
  std::set<astar::Cell> obstacles_total;

  Eigen::Vector3d point_temp;

  std::vector<Eigen::Vector3d> inflated_obstacles; 
  std::vector<Eigen::Vector3d> inflated_starts; 
  std::vector<Eigen::Vector3d> inflated_goals; 
  // for each UAV
  for (int i = 0; i < n_uavs; i++) {

    // prepare the path
    std::vector<Eigen::Vector3d> path;
    // path made of two waypoints: I -> F
    std::set<astar::Cell> obstacles_temp = {};

    std::vector<Eigen::Vector3d> inflated_obstacles = {}; 
    std::vector<Eigen::Vector3d> inflated_starts = {}; 
    std::vector<Eigen::Vector3d> inflated_goals = {}; 
    int j = i;
    while(j <n_uavs-1)
    {
      point_temp << initial_states[j+1](0), initial_states[j+1](1), initial_states[j+1](2);
      inflated_starts = Formation::createMinkowskyPoints(point_temp,resolution);
      for (Eigen::Vector3d obst: inflated_starts)
      {
        obstacles_temp.insert(astar.toGrid(obst(0), obst(1), obst(2)));
      }
      point_temp << final_states[j+1](0), final_states[j+1](1), final_states[j+1](2);
      inflated_goals = Formation::createMinkowskyPoints(point_temp,resolution);
      for (Eigen::Vector3d obst: inflated_goals)
      {
          obstacles_temp.insert(astar.toGrid(obst(0), obst(1), obst(2)));
      }
      j++;
    }
    Eigen::Vector3d start,goal;
    start << initial_states[i](0), initial_states[i](1), initial_states[i](2);
    goal  << final_states[i](0),final_states[i](1),final_states[i](2);

    std::set<astar::Cell> obstacles_total{};
    std::merge(obstacles_fixed.begin(), obstacles_fixed.end(),
                obstacles_temp.begin(), obstacles_temp.end(),
                std::inserter(obstacles_total, obstacles_total.begin()));
    path = astar.plan(start, goal, obstacles_total);


    if (path.size()>0) {
      // printf("path found\n");
      paths.push_back(path);
    } else {
      printf("path not found\n");
      return paths;
    }

    for (Eigen::Vector3d point : path)
    {
        inflated_obstacles = Formation::createMinkowskyPoints(point,resolution);
        for (Eigen::Vector3d obst: inflated_obstacles)
        {
          obstacles_fixed.insert(astar.toGrid(obst(0), obst(1), obst(2)));
        }
    }
     
  }

  return paths;
}

//}

/* multilateration() //{ */

/**
 * @brief The method for calculating a 3D position of source of signal based on the positions of UAVs and the measured distances to the source.
 *
 * @param uav_states Vector of 3D positions of each UAV.
 * @param distances Vector of the measured distances from each UAV to the source of signal.
 *
 * @return the estimated 3D position of the source of radiation.
 */
Eigen::Vector3d Formation::multilateration(const std::vector<Eigen::Vector3d> &positions, const Eigen::VectorXd &distances) {

  // THIS IS THE MOST BASIC OPTIMIZATION FOR THE POSITION OF THE ROBOT
  // The method can be improved significantly by:
  // * increasing the number of iterations
  // * trying multiple different initial conditions (xk)
  // * not optimizing for the full 3D position of the robot, we know that the robot rides on the ground, z = 0
  // * using better optimization method (LM)

  const int N = int(positions.size());

  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(N, 3);
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3, 3);
  Eigen::MatrixXd g = Eigen::VectorXd::Zero(N);

  // the solution... initialized as (0, 0, 0)^T, is it a good initialization?
  Eigen::Vector3d s = Eigen::Vector3d(0, 0, 0);

  const int max_iterations = 30;

  for (int n_iterations = 0; n_iterations < max_iterations; n_iterations++) {

    for (int j = 0; j < N; j++) {

      J.row(j) = (s - positions[j]) / (s - positions[j]).norm();
    }

    // distance from xk to the sphere with radius distances[i] and center positions[i]
    for (int i = 0; i < N; i++) {
      g(i) = (s - positions[i]).norm() - distances[i];
    }

    // do the Gauss-Newton iteration
    s = s - (J.transpose() * J + 0.5*I).inverse() * J.transpose() * g;
  }

  return s;
}

//}

/* update() //{ */

/**
 * @brief The main routine for controlling the experiment. The method is called regularly at 10 Hz, and,
 * therefore, it should not be blocking.
 *
 * @param formation_state The current state of the formation. The state contains:
 * - absolute position of the virtual leader
 * - positions of the follower UAVs relative the virtual leader
 * - flag stating whether the formation is moving or whether it is stationary
 * @param ranging A structure containing the measured distances form each UAV to the source of radio signal.
 * @param time_stamp Current time in seconds.
 * @param action_handlers This structure provides users with functions to control the formation:
 *   reshapeFormation() will reshape the formation relative the the virtual leader's position.
 *   moveFormation() will move the formation by moving the virtual leader. The followers will follow.
 * Moreover, the action_handlers structure provides additional methods for data visualization.
 */
void Formation::update(const FormationState_t &formation_state, const Ranging_t &ranging, [[maybe_unused]] const double &time_stamp,
                       ActionHandlers_t &action_handlers) {

  // how many UAVs are there in the formation?
  const int n_uavs = int(formation_state.followers.size());

  // | ------------- calculate the target's position ------------ |

  // calculate the abolsute positions of the formation members
  std::vector<Eigen::Vector3d> abs_positions;

  for (int i = 0; i < n_uavs; i++) {
    abs_positions.push_back(formation_state.followers[i] + formation_state.virtual_leader);
  }

  Eigen::Vector3d target_position = multilateration(abs_positions, ranging.distances);
  // | --------------- Publishing CUBE Rviz marker -------------- |
  // * you can use this function repeatedly with different names to visualize other stuff
  // * the args are:
  //    Position (x, y, z)
  //    Color (r, g, b, alpha), alpha = 1.0 is fully visible
  //    Size (meters)
  // action_handlers.visualizeCube(Position_t{target_position[0], target_position[1], target_position[2]}, Color_t{0.0, 0.0, 1.0, 1.0}, 1.0);

  // | ------------------- Put your code here ------------------- |
  
  // do nothing while the formation is in motion
  if (!formation_state.is_static) {
    return;
  }
        
  std::vector<std::vector<Eigen::Vector3d>> paths = {};

  // this is an example of a "state machine"
  switch (current_stage_) {

    // state machine:
    // 0 - forming triangle
    // 1 - captuing data
    // 2 - deciding which direction to go
    // 3 - reshaping
    // 4 - flying to destination

    // in the fist state, reorganize the formation into a column
    case 0: {
      printf("0: forming triangle\n");

      std::vector<Eigen::Vector3d> formation_triangle;
      formation_triangle.push_back(Eigen::Vector3d(-2.5, 0.0, 3.0));
      formation_triangle.push_back(Eigen::Vector3d(0.0, 2.5, 3.0));
      formation_triangle.push_back(Eigen::Vector3d(2.5, 0.0, 3.0));

      // plan paths to reshape the formation
      std::vector<std::vector<Eigen::Vector3d>> paths = Formation::getPathsReshapeFormation(formation_state.followers, formation_triangle);

      // tell the formation to reshape the formation
      // this will make the UAVs move, the flag "formation_state.is_static" will become false
      bool success = action_handlers.reshapeFormation(paths);

      if (!success) {
        printf("something went wrong while reshaping the formation\n");
        return;
      }

      current_stage_++;
      count_data_ = 0;
      measurements = {};
      avg_target = {};
      break;
    }

    case 1: {

      printf("1: capturing data\n");
      measurements.push_back(target_position);
      count_data_ ++;
      if (count_data_>10)
      {
        double total_x{0.0};
        double total_y{0.0};
        double total_z{0.0};
        for (Eigen::Vector3d target : measurements){
          total_x+=target(0);
          total_y+=target(1);
          total_z+=target(2);
        }
        total_x /= measurements.size();
        total_y /= measurements.size();
        total_z /= measurements.size();
        avg_target << total_x,total_y,total_z;
        std::cout<<"avg x : "<<total_x<<" y: "<<total_y<<" z: "<<total_z<<std::endl;
        current_stage_++;
      }
      else 
      {
        current_stage_=1;
      }
      break;
//----------------------------------------------------------------
    }
    

    case 2: {
     

      printf("2: deciding which way to go\n");

      if (avg_target(0)>avg_target(1))
      {
        xTrueYfalse = true;
      }
      else 
      {
        xTrueYfalse = false;
      }

      current_stage_++;

      break;
      
    }
    case 3: {
      printf("3: reshaping\n");

      std::vector<Eigen::Vector3d> formation_line_x = {};
      formation_line_x.push_back(Eigen::Vector3d(-2.4, 0.0, 3.0));
      formation_line_x.push_back(Eigen::Vector3d(0.0, 0.0, 3.0));
      formation_line_x.push_back(Eigen::Vector3d(2.4, 0.0, 3.0));

      std::vector<Eigen::Vector3d> formation_line_y= {};
      formation_line_y.push_back(Eigen::Vector3d(0.0, -2.4, 3.0));
      formation_line_y.push_back(Eigen::Vector3d(0.0, 0.0, 3.0));
      formation_line_y.push_back(Eigen::Vector3d(0.0, 2.4, 3.0));
      
      std::vector<std::vector<Eigen::Vector3d>> paths = Formation::getPathsReshapeFormation(formation_state.followers, formation_line_y);
      if (xTrueYfalse)
      {
        std::vector<std::vector<Eigen::Vector3d>> paths = Formation::getPathsReshapeFormation(formation_state.followers, formation_line_x);
      } 
      // tell the formation to reshape the formation
      // this will make the UAVs move, the flag "formation_state.is_static" will become false
      bool success = action_handlers.reshapeFormation(paths);

      if (!success) {
        printf("something went wrong while reshaping the formation\n");
        return;
      }

      current_stage_++;

      break;
    }
    case 4: {
      printf("4: flying to destination\n");
    // tell the virtual leader to move to the center of the arena

      // i need difference between current goal and current leader pose
      double x_dif {0.0};
      double y_dif {0.0};

      double leader_pose_x = formation_state.virtual_leader(0);
      double leader_pose_y = formation_state.virtual_leader(1);

      double goal_x = avg_target(0);
      double goal_y = avg_target(1);

      x_dif = goal_x - leader_pose_x;
      y_dif = goal_y - leader_pose_y;
      int x_step = x_dif/10;
      int y_step = y_dif/10;
      bool success = false;
      
      if (xTrueYfalse)
      {
        success = action_handlers.setLeaderPosition(Eigen::Vector3d(10.0*x_step, 0, 3)); // relative to current position vector
      }
      else 
      {
        success = action_handlers.setLeaderPosition(Eigen::Vector3d(0, 10.0*y_step, 3)); // relative to current position vector
      }
      if (!success) {
        printf("something went wrong moving the leader\n");
        return;
      }

      current_stage_=0;

      break;
    }
    default:
    {
      break;
    }
  }
}

//}

}  // namespace task_02_formation
