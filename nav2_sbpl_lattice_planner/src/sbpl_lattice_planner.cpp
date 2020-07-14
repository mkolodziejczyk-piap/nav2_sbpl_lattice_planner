/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Mike Phillips
*********************************************************************/

#include <nav2_sbpl_lattice_planner/sbpl_lattice_planner.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <nav_msgs/msg/path.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <nav2_costmap_2d/inflation_layer.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_util/node_utils.hpp>

using namespace std;

PLUGINLIB_EXPORT_CLASS(nav2_sbpl_lattice_planner::SBPLLatticePlanner, nav2_core::GlobalPlanner)

// namespace geometry_msgs {
//   bool operator== (const Point &p1, const Point &p2)
//   {
//     return p1.x == p2.x && p1.y == p2.y && p1.z == p2.z;
//   }
// }

namespace nav2_sbpl_lattice_planner{

class LatticeSCQ : public StateChangeQuery{
  public:
    LatticeSCQ(EnvironmentNAVXYTHETALAT* env, std::vector<nav2dcell_t> const & changedcellsV)
      : env_(env), changedcellsV_(changedcellsV) {
    }

    // lazy init, because we do not always end up calling this method
    virtual std::vector<int> const * getPredecessors() const{
      if(predsOfChangedCells_.empty() && !changedcellsV_.empty())
        env_->GetPredsofChangedEdges(&changedcellsV_, &predsOfChangedCells_);
      return &predsOfChangedCells_;
    }

    // lazy init, because we do not always end up calling this method
    virtual std::vector<int> const * getSuccessors() const{
      if(succsOfChangedCells_.empty() && !changedcellsV_.empty())
        env_->GetSuccsofChangedEdges(&changedcellsV_, &succsOfChangedCells_);
      return &succsOfChangedCells_;
    }

    EnvironmentNAVXYTHETALAT * env_;
    std::vector<nav2dcell_t> const & changedcellsV_;
    mutable std::vector<int> predsOfChangedCells_;
    mutable std::vector<int> succsOfChangedCells_;
};

SBPLLatticePlanner::SBPLLatticePlanner()
  : tf_(nullptr), node_(nullptr), costmap_(nullptr) {}

void SBPLLatticePlanner::configure(
  rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
//   costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  initialized_ = false;
  initialize();

}

void SBPLLatticePlanner::initialize()
{
  if(!initialized_){

    RCLCPP_INFO(
      node_->get_logger(), "Name is %s", name_.c_str());

    nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".planner_type", rclcpp::ParameterValue("ARAPlanner"));
    node_->get_parameter(name_ + ".planner_type", planner_type_);

    nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".allocated_time", rclcpp::ParameterValue(10.0));
    node_->get_parameter(name_ + ".allocated_time", allocated_time_);

    nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".initial_epsilon", rclcpp::ParameterValue(3.0));
    node_->get_parameter(name_ + ".initial_epsilon", initial_epsilon_);

    nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".environment_type", rclcpp::ParameterValue("XYThetaLattice"));
    node_->get_parameter(name_ + ".environment_type", environment_type_);

    nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".forward_search", rclcpp::ParameterValue(false));
    node_->get_parameter(name_ + ".forward_search", forward_search_);

    nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".primitive_filename", rclcpp::ParameterValue(""));
    node_->get_parameter(name_ + ".primitive_filename", primitive_filename_);

    nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".force_scratch_limit", rclcpp::ParameterValue(500));
    node_->get_parameter(name_ + ".force_scratch_limit", force_scratch_limit_);

    double nominalvel_mpersecs, timetoturn45degsinplace_secs;

    nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".nominalvel_mpersecs", rclcpp::ParameterValue(0.4));
    node_->get_parameter(name_ + ".nominalvel_mpersecs", nominalvel_mpersecs);

    nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".timetoturn45degsinplace_secs", rclcpp::ParameterValue(0.6));
    node_->get_parameter(name_ + ".timetoturn45degsinplace_secs", timetoturn45degsinplace_secs);

    int lethal_obstacle;
    nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".lethal_obstacle", rclcpp::ParameterValue(20));
    node_->get_parameter(name_ + ".lethal_obstacle", lethal_obstacle);

    lethal_obstacle_ = (unsigned char) lethal_obstacle;
    inscribed_inflated_obstacle_ = lethal_obstacle_-1;
    sbpl_cost_multiplier_ = (unsigned char) (nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE/inscribed_inflated_obstacle_ + 1);
    RCLCPP_DEBUG(
        node_->get_logger(), "SBPL: lethal: %uz, inscribed inflated: %uz, multiplier: %uz",lethal_obstacle,inscribed_inflated_obstacle_,sbpl_cost_multiplier_);

    // name_ = name;
    // costmap_ros_ = costmap_ros;

    footprint_ = costmap_ros_->getRobotFootprint();

    if ("XYThetaLattice" == environment_type_){
      RCLCPP_DEBUG(
        node_->get_logger(), "Using a 3D costmap for theta lattice\n");
      env_ = new EnvironmentNAVXYTHETALAT();
    }
    else{
      RCLCPP_ERROR(
        node_->get_logger(), "XYThetaLattice is currently the only supported environment!\n");
      exit(1);
    }

    circumscribed_cost_ = computeCircumscribedCost();

    if (circumscribed_cost_ == 0) {
      // Unfortunately, the inflation_radius is not taken into account by
      // inflation_layer->computeCost(). If inflation_radius is smaller than
      // the circumscribed radius, SBPL will ignore some obstacles, but we
      // cannot detect this problem. If the cost_scaling_factor is too large,
      // SBPL won't run into obstacles, but will always perform an expensive
      // footprint check, no matter how far the nearest obstacle is.
      RCLCPP_WARN(
        node_->get_logger(),
        "The costmap value at the robot's circumscribed radius (%f m) is 0.", costmap_ros_->getLayeredCostmap()->getCircumscribedRadius());
      RCLCPP_WARN(
        node_->get_logger(), "SBPL performance will suffer.");
      RCLCPP_WARN(
        node_->get_logger(), "Please decrease the costmap's cost_scaling_factor.");
    }
    if(!env_->SetEnvParameter("cost_inscribed_thresh",costMapCostToSBPLCost(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE))){
      RCLCPP_ERROR(
        node_->get_logger(), "Failed to set cost_inscribed_thresh parameter");
      exit(1);
    }
    if(!env_->SetEnvParameter("cost_possibly_circumscribed_thresh", circumscribed_cost_)){
      RCLCPP_ERROR(
        node_->get_logger(), "Failed to set cost_possibly_circumscribed_thresh parameter");
      exit(1);
    }
    int obst_cost_thresh = costMapCostToSBPLCost(nav2_costmap_2d::LETHAL_OBSTACLE);
    vector<sbpl_2Dpt_t> perimeterptsV;
    perimeterptsV.reserve(footprint_.size());
    for (size_t ii(0); ii < footprint_.size(); ++ii) {
      sbpl_2Dpt_t pt;
      pt.x = footprint_[ii].x;
      pt.y = footprint_[ii].y;
      perimeterptsV.push_back(pt);
    }

    bool ret;
    try{
      ret = env_->InitializeEnv(costmap_ros_->getCostmap()->getSizeInCellsX(), // width
                                costmap_ros_->getCostmap()->getSizeInCellsY(), // height
                                0, // mapdata
                                0, 0, 0, // start (x, y, theta, t)
                                0, 0, 0, // goal (x, y, theta)
                                0, 0, 0, //goal tolerance
                                perimeterptsV, costmap_ros_->getCostmap()->getResolution(), nominalvel_mpersecs,
                                timetoturn45degsinplace_secs, obst_cost_thresh,
                                primitive_filename_.c_str());
      current_env_width_ = costmap_ros_->getCostmap()->getSizeInCellsX();
      current_env_height_ = costmap_ros_->getCostmap()->getSizeInCellsY();
    }
    catch(SBPL_Exception *e){
      RCLCPP_ERROR(
        node_->get_logger(), "SBPL encountered a fatal exception: %s", e->what());
      ret = false;
    }
    if(!ret){
      RCLCPP_ERROR(
        node_->get_logger(), "SBPL initialization failed!");
      exit(1);
    }
    for (ssize_t ix(0); ix < costmap_ros_->getCostmap()->getSizeInCellsX(); ++ix)
      for (ssize_t iy(0); iy < costmap_ros_->getCostmap()->getSizeInCellsY(); ++iy)
        env_->UpdateCost(ix, iy, costMapCostToSBPLCost(costmap_ros_->getCostmap()->getCost(ix,iy)));

    if ("ARAPlanner" == planner_type_){
      RCLCPP_INFO(
        node_->get_logger(), "Planning with ARA*");
      planner_ = new ARAPlanner(env_, forward_search_);
    }
    else if ("ADPlanner" == planner_type_){
      RCLCPP_INFO(
        node_->get_logger(), "Planning with AD*");
      planner_ = new ADPlanner(env_, forward_search_);
    }
    else{
      RCLCPP_ERROR(
        node_->get_logger(), "ARAPlanner and ADPlanner are currently the only supported planners!\n");
      exit(1);
    }

    RCLCPP_INFO(
        node_->get_logger(), "[sbpl_lattice_planner] Initialized successfully");
    // plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
    stats_publisher_ = node_->create_publisher<nav2_sbpl_lattice_planner_msgs::msg::SbplLatticePlannerStats>("sbpl_lattice_planner_stats", 1);
    
    initialized_ = true;
  }
}

void SBPLLatticePlanner::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
    name_.c_str());
    stats_publisher_.reset();
}

void SBPLLatticePlanner::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
    name_.c_str());
    stats_publisher_->on_activate();
}

void SBPLLatticePlanner::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
    name_.c_str());
    stats_publisher_->on_deactivate();
}
  
//Taken from Sachin's sbpl_cart_planner
//This rescales the costmap according to a rosparam which sets the obstacle cost

// https://github.com/ros-planning/navigation2/blob/eloquent-devel/nav2_costmap_2d/include/nav2_costmap_2d/cost_values.hpp

unsigned char SBPLLatticePlanner::costMapCostToSBPLCost(unsigned char newcost){
  if(newcost == nav2_costmap_2d::LETHAL_OBSTACLE)
    return lethal_obstacle_;
  else if(newcost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    return inscribed_inflated_obstacle_;
  else if(newcost == 0 || newcost == nav2_costmap_2d::NO_INFORMATION)
    return 0;
  else {
    unsigned char sbpl_cost = newcost / sbpl_cost_multiplier_;
    if (sbpl_cost == 0)
      sbpl_cost = 1;
    return sbpl_cost;
  }
}

void SBPLLatticePlanner::publishStats(int solution_cost, int solution_size, 
                                      const geometry_msgs::msg::PoseStamped& start, 
                                      const geometry_msgs::msg::PoseStamped& goal){
  // Fill up statistics and publish
  nav2_sbpl_lattice_planner_msgs::msg::SbplLatticePlannerStats stats;
  stats.initial_epsilon = initial_epsilon_;
  stats.plan_to_first_solution = false;
  stats.final_number_of_expands = planner_->get_n_expands();
  stats.allocated_time = allocated_time_;

  stats.time_to_first_solution = planner_->get_initial_eps_planning_time();
  stats.actual_time = planner_->get_final_eps_planning_time();
  stats.number_of_expands_initial_solution = planner_->get_n_expands_init_solution();
  stats.final_epsilon = planner_->get_final_epsilon();

  stats.solution_cost = solution_cost;
  stats.path_size = solution_size;
  stats.start = start;
  stats.goal = goal;
  stats_publisher_->publish(stats);
}

unsigned char SBPLLatticePlanner::computeCircumscribedCost() {
  unsigned char result = 0;

  if (!costmap_ros_) {
    RCLCPP_ERROR(
        node_->get_logger(), "Costmap is not initialized");
    return 0;
  }

  // check if the costmap has an inflation layer
  for(std::vector<std::shared_ptr<nav2_costmap_2d::Layer> >::const_iterator layer = costmap_ros_->getLayeredCostmap()->getPlugins()->begin();
      layer != costmap_ros_->getLayeredCostmap()->getPlugins()->end();
      ++layer) {
    std::shared_ptr<nav2_costmap_2d::InflationLayer> inflation_layer = std::dynamic_pointer_cast<nav2_costmap_2d::InflationLayer>(*layer);
    if (!inflation_layer) continue;

    result = costMapCostToSBPLCost(inflation_layer->computeCost(costmap_ros_->getLayeredCostmap()->getCircumscribedRadius() / costmap_ros_->getCostmap()->getResolution()));
  }
  return result;
}

nav_msgs::msg::Path SBPLLatticePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{

  nav_msgs::msg::Path path;

  RCLCPP_INFO(node_->get_logger(), "Create plan.."); 

  if(!initialized_){
    RCLCPP_ERROR(
        node_->get_logger(), "Global planner is not initialized");
    return path;
  }

  bool do_init = false;
  if (current_env_width_ != costmap_ros_->getCostmap()->getSizeInCellsX() ||
      current_env_height_ != costmap_ros_->getCostmap()->getSizeInCellsY()) {
    RCLCPP_INFO(
      node_->get_logger(), "Costmap dimensions have changed from (%d x %d) to (%d x %d), reinitializing sbpl_lattice_planner.",
             current_env_width_, current_env_height_,
             costmap_ros_->getCostmap()->getSizeInCellsX(), costmap_ros_->getCostmap()->getSizeInCellsY());
    do_init = true;
  }
  else if (footprint_ != costmap_ros_->getRobotFootprint()) {
    RCLCPP_INFO(
      node_->get_logger(), "Robot footprint has changed, reinitializing sbpl_lattice_planner.");
    do_init = true;
  }
  else if (circumscribed_cost_ != computeCircumscribedCost()) {
    RCLCPP_INFO(
      node_->get_logger(), "Cost at circumscribed radius has changed, reinitializing sbpl_lattice_planner.");
    do_init = true;
  }

  if (do_init) {
    initialized_ = false;
    delete planner_;
    planner_ = NULL;
    delete env_;
    env_ = NULL;
    initialize();
  }

  RCLCPP_INFO(
    node_->get_logger(), "[sbpl_lattice_planner] getting start point (%g,%g) goal point (%g,%g)",
           start.pose.position.x, start.pose.position.y,goal.pose.position.x, goal.pose.position.y);
  double theta_start = 2 * atan2(start.pose.orientation.z, start.pose.orientation.w);
  double theta_goal = 2 * atan2(goal.pose.orientation.z, goal.pose.orientation.w);

  try{
    int ret = env_->SetStart(start.pose.position.x - costmap_ros_->getCostmap()->getOriginX(), start.pose.position.y - costmap_ros_->getCostmap()->getOriginY(), theta_start);
    if(ret < 0 || planner_->set_start(ret) == 0){
      RCLCPP_ERROR(
        node_->get_logger(), "ERROR: failed to set start state\n");
      return path;
    }
  }
  catch(SBPL_Exception *e){
    RCLCPP_ERROR(
        node_->get_logger(), "SBPL encountered a fatal exception while setting the start state");
    return path;
  }

  try{
    int ret = env_->SetGoal(goal.pose.position.x - costmap_ros_->getCostmap()->getOriginX(), goal.pose.position.y - costmap_ros_->getCostmap()->getOriginY(), theta_goal);
    if(ret < 0 || planner_->set_goal(ret) == 0){
      RCLCPP_ERROR(
        node_->get_logger(), "ERROR: failed to set goal state\n");
      return path;
    }
  }
  catch(SBPL_Exception *e){
    RCLCPP_ERROR(
        node_->get_logger(), "SBPL encountered a fatal exception while setting the goal state");
    return path;
  }
  
  int offOnCount = 0;
  int onOffCount = 0;
  int allCount = 0;
  vector<nav2dcell_t> changedcellsV;

  for(unsigned int ix = 0; ix < costmap_ros_->getCostmap()->getSizeInCellsX(); ix++) {
    for(unsigned int iy = 0; iy < costmap_ros_->getCostmap()->getSizeInCellsY(); iy++) {

      unsigned char oldCost = env_->GetMapCost(ix,iy);
      unsigned char newCost = costMapCostToSBPLCost(costmap_ros_->getCostmap()->getCost(ix,iy));

      if(oldCost == newCost) continue;

      allCount++;

      //first case - off cell goes on

      if((oldCost != costMapCostToSBPLCost(nav2_costmap_2d::LETHAL_OBSTACLE) && oldCost != costMapCostToSBPLCost(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)) &&
          (newCost == costMapCostToSBPLCost(nav2_costmap_2d::LETHAL_OBSTACLE) || newCost == costMapCostToSBPLCost(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE))) {
        offOnCount++;
      }

      if((oldCost == costMapCostToSBPLCost(nav2_costmap_2d::LETHAL_OBSTACLE) || oldCost == costMapCostToSBPLCost(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)) &&
          (newCost != costMapCostToSBPLCost(nav2_costmap_2d::LETHAL_OBSTACLE) && newCost != costMapCostToSBPLCost(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE))) {
        onOffCount++;
      }
      env_->UpdateCost(ix, iy, costMapCostToSBPLCost(costmap_ros_->getCostmap()->getCost(ix,iy)));

      nav2dcell_t nav2dcell;
      nav2dcell.x = ix;
      nav2dcell.y = iy;
      changedcellsV.push_back(nav2dcell);
    }
  }

  try{
    if(!changedcellsV.empty()){
      StateChangeQuery* scq = new LatticeSCQ(env_, changedcellsV);
      planner_->costs_changed(*scq);
      delete scq;
    }

    if(allCount > force_scratch_limit_)
      planner_->force_planning_from_scratch();
  }
  catch(SBPL_Exception *e){
    RCLCPP_ERROR(
        node_->get_logger(), "SBPL failed to update the costmap");
    return path;
  }

  //setting planner parameters
  RCLCPP_DEBUG(
        node_->get_logger(), "allocated:%f, init eps:%f\n",allocated_time_,initial_epsilon_);
  planner_->set_initialsolution_eps(initial_epsilon_);
  planner_->set_search_mode(false);

  RCLCPP_DEBUG(
        node_->get_logger(), "[sbpl_lattice_planner] run planner");
  vector<int> solution_stateIDs;
  int solution_cost;
  try{
    int ret = planner_->replan(allocated_time_, &solution_stateIDs, &solution_cost);
    if(ret)
      RCLCPP_DEBUG(
        node_->get_logger(), "Solution is found\n");
    else{
      RCLCPP_INFO(
        node_->get_logger(), "Solution not found\n");
      publishStats(solution_cost, 0, start, goal);
      return path;
    }
  }
  catch(SBPL_Exception *e){
    RCLCPP_ERROR(
        node_->get_logger(), "SBPL encountered a fatal exception while planning");
    return path;
  }

  RCLCPP_DEBUG(
    node_->get_logger(), "size of solution=%d", (int)solution_stateIDs.size());

  // vector<int> -> vector<EnvNAVXYTHETALAT3Dpt_t>
  
  vector<EnvNAVXYTHETALAT3Dpt_t> sbpl_path;
  try{
    env_->ConvertStateIDPathintoXYThetaPath(&solution_stateIDs, &sbpl_path);
  }
  catch(SBPL_Exception *e){
    RCLCPP_ERROR(
        node_->get_logger(), "SBPL encountered a fatal exception while reconstructing the path");
    return path;
  }
  
  // TODO is it necessary in navigation2?
  // if the plan has zero points, add a single point to make move_base happy
  if( sbpl_path.size() == 0 ) {
    EnvNAVXYTHETALAT3Dpt_t s(
        start.pose.position.x - costmap_ros_->getCostmap()->getOriginX(),
        start.pose.position.y - costmap_ros_->getCostmap()->getOriginY(),
        theta_start);
    sbpl_path.push_back(s);
  }

  RCLCPP_DEBUG(
    node_->get_logger(), "Plan has %d points.\n", (int)sbpl_path.size());
  rclcpp::Time plan_time = node_->now();

  // vector<EnvNAVXYTHETALAT3Dpt_t> -> nav_msgs::msg::Path

  path.poses.reserve(sbpl_path.size());
  path.header.frame_id = global_frame_;
  path.header.stamp = plan_time;
  
  for(unsigned int i=0; i<sbpl_path.size(); i++){
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = plan_time;
    pose.header.frame_id = global_frame_;

    pose.pose.position.x = sbpl_path[i].x + costmap_ros_->getCostmap()->getOriginX();
    pose.pose.position.y = sbpl_path[i].y + costmap_ros_->getCostmap()->getOriginY();
    pose.pose.position.z = start.pose.position.z;

    tf2::Quaternion temp;
    temp.setRPY(0,0,sbpl_path[i].theta);
    pose.pose.orientation.x = temp.getX();
    pose.pose.orientation.y = temp.getY();
    pose.pose.orientation.z = temp.getZ();
    pose.pose.orientation.w = temp.getW();

    // RCLCPP_INFO(node_->get_logger(), "i: %d", i);
    // RCLCPP_INFO(node_->get_logger(), "x: %f", pose.pose.position.x);
    // RCLCPP_INFO(node_->get_logger(), "y: %f", pose.pose.position.y);
    // RCLCPP_INFO(node_->get_logger(), "theta: %f", sbpl_path[i].theta);

    path.poses.push_back(pose);
  }

  RCLCPP_INFO(node_->get_logger(), "Valid plan created");

  publishStats(solution_cost, sbpl_path.size(), start, goal);

  return path;
}
}
