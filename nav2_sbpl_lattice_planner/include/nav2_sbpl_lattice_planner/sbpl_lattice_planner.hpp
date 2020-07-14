#ifndef NAV2_SBPLLATTICE_PLANNER__SBPLLATTICE_HPP_
#define NAV2_SBPLLATTICE_PLANNER__SBPLLATTICE_HPP_

#include <iostream>
#include <vector>

using namespace std;

// ROS
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

// Costmap used for the map representation
#include <nav2_costmap_2d/costmap_2d_ros.hpp>

// sbpl headers
#include <sbpl/headers.h>

// global representation
#include <nav2_core/global_planner.hpp>
#include <nav2_util/lifecycle_node.hpp>

#include <tf2_ros/buffer.h>

#include <nav2_sbpl_lattice_planner_msgs/msg/sbpl_lattice_planner_stats.hpp>

namespace nav2_sbpl_lattice_planner{

class SBPLLatticePlanner : public nav2_core::GlobalPlanner{
public:
  
  /**
   * @brief  Default constructor for the NavFnROS object
   */
  SBPLLatticePlanner();
  ~SBPLLatticePlanner() = default;

  
  // plugin configure
  void configure(
    rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  // plugin cleanup
  void cleanup() override;

  // plugin activate
  void activate() override;

  // plugin deactivate
  void deactivate() override;

  // This method creates path for given start and goal pose.
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:

  void initialize();

  unsigned char costMapCostToSBPLCost(unsigned char newcost);
  void publishStats(int solution_cost, int solution_size, 
                    const geometry_msgs::msg::PoseStamped& start, 
                    const geometry_msgs::msg::PoseStamped& goal);

  unsigned char computeCircumscribedCost();

  bool initialized_;

  SBPLPlanner* planner_;
  EnvironmentNAVXYTHETALAT* env_;
  
  std::string planner_type_; /**< sbpl method to use for planning.  choices are ARAPlanner and ADPlanner */

  double allocated_time_; /**< amount of time allowed for search */
  double initial_epsilon_; /**< initial epsilon for beginning the anytime search */

  std::string environment_type_; /** what type of environment in which to plan.  choices are 2D and XYThetaLattice. */ 
  std::string cost_map_topic_; /** what topic is being used for the costmap topic */

  bool forward_search_; /** whether to use forward or backward search */
  std::string primitive_filename_; /** where to find the motion primitives for the current robot */
  int force_scratch_limit_; /** the number of cells that have to be changed in the costmap to force the planner to plan from scratch even if its an incremental planner */

  unsigned char lethal_obstacle_;
  unsigned char inscribed_inflated_obstacle_;
  unsigned char circumscribed_cost_;
  unsigned char sbpl_cost_multiplier_;

  std::string name_;
  std::string global_frame_;

  // TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // node ptr
  nav2_util::LifecycleNode::SharedPtr node_;

  // Global Costmap
  nav2_costmap_2d::Costmap2D * costmap_;

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_; /**< manages the cost map for us */
  std::vector<geometry_msgs::msg::Point> footprint_;
  unsigned int current_env_width_;
  unsigned int current_env_height_;

//   rclcpp::Publisher<nav2_sbpl_lattice_planner_msgs::msg::SbplLatticePlannerStats>::SharedPtr stats_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<nav2_sbpl_lattice_planner_msgs::msg::SbplLatticePlannerStats>::SharedPtr stats_publisher_;
};
};

#endif

