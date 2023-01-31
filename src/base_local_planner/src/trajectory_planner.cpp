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
* Author: Eitan Marder-Eppstein
*********************************************************************/

#include <base_local_planner/trajectory_planner.h>

#include <base_local_planner/global_variable.h>

#include <costmap_2d/footprint.h>
#include <string>
#include <sstream>
#include <math.h>
#include <angles/angles.h>




#include <boost/algorithm/string.hpp>

#include <ros/console.h>

//for computing path distance
#include <queue>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>



using namespace std;
using namespace costmap_2d;

namespace base_local_planner{

  void TrajectoryPlanner::reconfigure(BaseLocalPlannerConfig &cfg)
  {
      BaseLocalPlannerConfig config(cfg);

      boost::mutex::scoped_lock l(configuration_mutex_);

      acc_lim_x_ = config.acc_lim_x;
      acc_lim_y_ = config.acc_lim_y;
      acc_lim_theta_ = config.acc_lim_theta;

      max_vel_x_ = config.max_vel_x;
      min_vel_x_ = config.min_vel_x;
      
      max_vel_th_ = config.max_vel_theta;
      min_vel_th_ = config.min_vel_theta;
      min_in_place_vel_th_ = config.min_in_place_vel_theta;

      sim_time_ = config.sim_time;
      sim_granularity_ = config.sim_granularity;
      angular_sim_granularity_ = config.angular_sim_granularity;

      path_distance_bias_ = config.path_distance_bias;
      goal_distance_bias_ = config.goal_distance_bias;
      occdist_scale_ = config.occdist_scale;

      if (meter_scoring_) {
        //if we use meter scoring, then we want to multiply the biases by the resolution of the costmap
        double resolution = costmap_.getResolution();
        goal_distance_bias_ *= resolution;
        path_distance_bias_ *= resolution;
      }

      oscillation_reset_dist_ = config.oscillation_reset_dist;
      escape_reset_dist_ = config.escape_reset_dist;
      escape_reset_theta_ = config.escape_reset_theta;

      vx_samples_ = config.vx_samples;
      vtheta_samples_ = config.vtheta_samples;

      if (vx_samples_ <= 0) {
          config.vx_samples = 1;
          vx_samples_ = config.vx_samples;
          ROS_WARN("You've specified that you don't want any samples in the x dimension. We'll at least assume that you want to sample one value... so we're going to set vx_samples to 1 instead");
      }
      if(vtheta_samples_ <= 0) {
          config.vtheta_samples = 1;
          vtheta_samples_ = config.vtheta_samples;
          ROS_WARN("You've specified that you don't want any samples in the theta dimension. We'll at least assume that you want to sample one value... so we're going to set vtheta_samples to 1 instead");
      }

      heading_lookahead_ = config.heading_lookahead;

      holonomic_robot_ = config.holonomic_robot;
      
      backup_vel_ = config.escape_vel;

      dwa_ = config.dwa;

      heading_scoring_ = config.heading_scoring;
      heading_scoring_timestep_ = config.heading_scoring_timestep;

      simple_attractor_ = config.simple_attractor;

      //y-vels
      string y_string = config.y_vels;
      vector<string> y_strs;
      boost::split(y_strs, y_string, boost::is_any_of(", "), boost::token_compress_on);

      vector<double>y_vels;
      for(vector<string>::iterator it=y_strs.begin(); it != y_strs.end(); ++it) {
          istringstream iss(*it);
          double temp;
          iss >> temp;
          y_vels.push_back(temp);
          //ROS_INFO("Adding y_vel: %e", temp);
      }

      y_vels_ = y_vels;
      
  }

  TrajectoryPlanner::TrajectoryPlanner(WorldModel& world_model,
      /*const*/ Costmap2D& costmap,
      std::vector<geometry_msgs::Point> footprint_spec,
      double acc_lim_x, double acc_lim_y, double acc_lim_theta,
      double sim_time, double sim_granularity,
      int vx_samples, int vtheta_samples,
      double path_distance_bias, double goal_distance_bias, double occdist_scale,
      double heading_lookahead, double oscillation_reset_dist,
      double escape_reset_dist, double escape_reset_theta,
      bool holonomic_robot,
      double max_vel_x, double min_vel_x,
      double max_vel_th, double min_vel_th, double min_in_place_vel_th,
      double backup_vel,
      bool dwa, bool heading_scoring, double heading_scoring_timestep, bool meter_scoring, bool simple_attractor,
      vector<double> y_vels, double stop_time_buffer, double sim_period, double angular_sim_granularity)
    : path_map_(costmap.getSizeInCellsX(), costmap.getSizeInCellsY()),
      goal_map_(costmap.getSizeInCellsX(), costmap.getSizeInCellsY()),
      costmap_(costmap),
    world_model_(world_model), world_model_legfree(NULL),world_model_legobs(NULL),footprint_spec_(footprint_spec),
    sim_time_(sim_time), sim_granularity_(sim_granularity), angular_sim_granularity_(angular_sim_granularity),
    vx_samples_(vx_samples), vtheta_samples_(vtheta_samples),
    path_distance_bias_(path_distance_bias), goal_distance_bias_(goal_distance_bias), occdist_scale_(occdist_scale),
    acc_lim_x_(acc_lim_x), acc_lim_y_(acc_lim_y), acc_lim_theta_(acc_lim_theta),
    prev_x_(0), prev_y_(0), escape_x_(0), escape_y_(0), escape_theta_(0), heading_lookahead_(heading_lookahead),
    oscillation_reset_dist_(oscillation_reset_dist), escape_reset_dist_(escape_reset_dist),
    escape_reset_theta_(escape_reset_theta), holonomic_robot_(holonomic_robot),
    max_vel_x_(max_vel_x), min_vel_x_(min_vel_x),
    max_vel_th_(max_vel_th), min_vel_th_(min_vel_th), min_in_place_vel_th_(min_in_place_vel_th),
    backup_vel_(backup_vel),
    dwa_(dwa), heading_scoring_(heading_scoring), heading_scoring_timestep_(heading_scoring_timestep),
    simple_attractor_(simple_attractor), y_vels_(y_vels), stop_time_buffer_(stop_time_buffer), sim_period_(sim_period)
  {
    //the robot is not stuck to begin with
    stuck_left = false;
    stuck_right = false;
    stuck_left_strafe = false;
    stuck_right_strafe = false;
    rotating_left = false;
    rotating_right = false;
    strafe_left = false;
    strafe_right = false;

    escaping_ = false;
    final_goal_position_valid_ = false;
	
	costmap_2d::calculateMinAndMaxDistances(footprint_spec_, inscribed_radius_, circumscribed_radius_);

	
	whlLegMode_ = 2;   // 0 : wheel mode , 1: leg mode, 2: leg and wheel hybrid 3: transition (wheel to leg) 4: transition (leg to wheel); 5: closing mode
	ModeTranStart_theta_ = -1024.0; /*-1024 means that no vaild data is available */
	CloseWheel_Duration_ = ros::Duration(1.0);
	
	MarkObsDistFlag_ = false;
	VelSignByWhlLegMode_ = 1;
        WheelLeg_InstallDir_ = 1;     //0: front is wheeled mode   // 1: front is legged mode
	
	if(whlLegMode_ == 1)
	{
		ModeTranStart_x_ = 0;
		ModeTranStart_y_ = 0;
	}
  
    world_model_legfree = new CostmapModel(costmap_); 
	world_model_legobs = new CostmapModel(costmap_);
	
	DebugTraj_TextWrite_ = false; //false;
	writing_count_ = 0;
	
	if(DebugTraj_TextWrite_ == true)
	{
	   
	   std::string dir = "/home/kangneoung/sw_repo/alpha_waltr/src";
       	   
	   LegTrajectoryText_x_.open(dir + "/" + "LegTrajectoryText_x_.txt");
	   LegTrajectoryText_y_.open(dir + "/" + "LegTrajectoryText_y_.txt");
	   LegTrajectoryText_cost_.open(dir + "/" + "LegTrajectoryText_cost_.txt");
	   WhlTrajectoryText_x_.open(dir + "/" + "WhlTrajectoryText_x_.txt");
	   WhlTrajectoryText_y_.open(dir + "/" + "WhlTrajectoryText_y_.txt");
	   WhlTrajectoryText_cost_.open(dir + "/" + "WhlTrajectoryText_cost_.txt");
		
	}
	
	
  }

  TrajectoryPlanner::~TrajectoryPlanner(){
	  
	  if(world_model_legfree != NULL)
      delete world_model_legfree;
  
  	  if(world_model_legobs != NULL)
      delete world_model_legobs;
	  
	  if(DebugTraj_TextWrite_ == true)
	  {
	     LegTrajectoryText_x_.close();
	     LegTrajectoryText_y_.close();
		 LegTrajectoryText_cost_.close();
	     WhlTrajectoryText_x_.close();
	     WhlTrajectoryText_y_.close();
		 WhlTrajectoryText_cost_.close();
	  }
	  //delete costmap_legfree;
	  //delete costmap_legobs;
  }

  bool TrajectoryPlanner::getCellCosts(int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost) {
    MapCell cell = path_map_(cx, cy);
    MapCell goal_cell = goal_map_(cx, cy);
    if (cell.within_robot) {
        return false;
    }
    occ_cost = costmap_.getCost(cx, cy);
    if (cell.target_dist == path_map_.obstacleCosts() ||
        cell.target_dist == path_map_.unreachableCellCosts() ||
        occ_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        return false;
    }
    path_cost = cell.target_dist;
    goal_cost = goal_cell.target_dist;
    total_cost = path_distance_bias_ * path_cost + goal_distance_bias_ * goal_cost + occdist_scale_ * occ_cost;
    return true;
  }

  /**
   * create and score a trajectory given the current pose of the robot and selected velocities
   */
  void TrajectoryPlanner::generateTrajectory(
      double x, double y, double theta,
      double vx, double vy, double vtheta,
      double vx_samp, double vy_samp, double vtheta_samp,
      double acc_x, double acc_y, double acc_theta,
      double impossible_cost,
      Trajectory& traj) {

    // make sure the configuration doesn't change mid run
    boost::mutex::scoped_lock l(configuration_mutex_);

    double x_i = x;
    double y_i = y;
    double theta_i = theta;

    double vx_i, vy_i, vtheta_i;

    vx_i = vx;
    vy_i = vy;
    vtheta_i = vtheta;

    //compute the magnitude of the velocities
    double vmag = hypot(vx_samp, vy_samp);

    //compute the number of steps we must take along this trajectory to be "safe"
    int num_steps;
	
	num_steps = int(sim_time_ / sim_granularity_ + 0.5);
    /*if(!heading_scoring_) {
      num_steps = int(max((vmag * sim_time_) / sim_granularity_, fabs(vtheta_samp) / angular_sim_granularity_) + 0.5);
    } else {
      num_steps = int(sim_time_ / sim_granularity_ + 0.5);
    }*/

    //we at least want to take one step... even if we won't move, we want to score our current position
    if(num_steps == 0) {
      num_steps = 1;
    }

    double dt = sim_time_ / num_steps;
    double time = 0.0;

    //create a potential trajectory
    traj.resetPoints();
    traj.xv_ = vx_samp;
    traj.yv_ = vy_samp;
    traj.thetav_ = vtheta_samp;
    traj.cost_ = -1.0;

    //initialize the costs for the trajectory
    double path_dist = 0.0;
    double goal_dist = 0.0;
    double occ_cost = 0.0;
    double heading_diff = 0.0;

    for(int i = 0; i < num_steps; ++i){
      //get map coordinates of a point
      unsigned int cell_x, cell_y;

      //we don't want a path that goes off the know map
      if(!costmap_.worldToMap(x_i, y_i, cell_x, cell_y)){
        traj.cost_ = -1.0;
        return;
      }

      //check the point on the trajectory for legality
      double footprint_cost = footprintCost(x_i, y_i, theta_i);

      //if the footprint hits an obstacle this trajectory is invalid
      if(footprint_cost < 0){
        traj.cost_ = -1.0;
        return;
        //TODO: Really look at getMaxSpeedToStopInTime... dues to discretization errors and high acceleration limits,
        //it can actually cause the robot to hit obstacles. There may be something to be done to fix, but I'll have to
        //come back to it when I have time. Right now, pulling it out as it'll just make the robot a bit more conservative,
        //but safe.
        /*
        double max_vel_x, max_vel_y, max_vel_th;
        //we want to compute the max allowable speeds to be able to stop
        //to be safe... we'll make sure we can stop some time before we actually hit
        getMaxSpeedToStopInTime(time - stop_time_buffer_ - dt, max_vel_x, max_vel_y, max_vel_th);

        //check if we can stop in time
        if(fabs(vx_samp) < max_vel_x && fabs(vy_samp) < max_vel_y && fabs(vtheta_samp) < max_vel_th){
          ROS_ERROR("v: (%.2f, %.2f, %.2f), m: (%.2f, %.2f, %.2f) t:%.2f, st: %.2f, dt: %.2f", vx_samp, vy_samp, vtheta_samp, max_vel_x, max_vel_y, max_vel_th, time, stop_time_buffer_, dt);
          //if we can stop... we'll just break out of the loop here.. no point in checking future points
          break;
        }
        else{
          traj.cost_ = -1.0;
          return;
        }
        */
      }

      occ_cost = std::max(std::max(occ_cost, footprint_cost), double(costmap_.getCost(cell_x, cell_y)));

      //do we want to follow blindly
      if (simple_attractor_) {
        goal_dist = (x_i - global_plan_[global_plan_.size() -1].pose.position.x) *
          (x_i - global_plan_[global_plan_.size() -1].pose.position.x) +
          (y_i - global_plan_[global_plan_.size() -1].pose.position.y) *
          (y_i - global_plan_[global_plan_.size() -1].pose.position.y);
      } else {

        bool update_path_and_goal_distances = true;

        // with heading scoring, we take into account heading diff, and also only score
        // path and goal distance for one point of the trajectory
        if (heading_scoring_) {
          if (time >= heading_scoring_timestep_ && time < heading_scoring_timestep_ + dt) {
            heading_diff = headingDiff(cell_x, cell_y, x_i, y_i, theta_i);
          } else {
            update_path_and_goal_distances = false;
          }
        }

        if (update_path_and_goal_distances) {
          //update path and goal distances
          path_dist = path_map_(cell_x, cell_y).target_dist;
          goal_dist = goal_map_(cell_x, cell_y).target_dist;

          //if a point on this trajectory has no clear path to goal it is invalid
          if(impossible_cost <= goal_dist || impossible_cost <= path_dist){
//            ROS_DEBUG("No path to goal with goal distance = %f, path_distance = %f and max cost = %f",
//                goal_dist, path_dist, impossible_cost);
            traj.cost_ = -2.0;
            return;
          }
        }
      }


      //the point is legal... add it to the trajectory
      traj.addPoint(x_i, y_i, theta_i);

      //calculate velocities
      vx_i = computeNewVelocity(vx_samp, vx_i, acc_x, dt);
      vy_i = computeNewVelocity(vy_samp, vy_i, acc_y, dt);
      vtheta_i = computeNewVelocity(vtheta_samp, vtheta_i, acc_theta, dt);

      //calculate positions
      x_i = computeNewXPosition(x_i, vx_i, vy_i, theta_i, dt);
      y_i = computeNewYPosition(y_i, vx_i, vy_i, theta_i, dt);
      theta_i = computeNewThetaPosition(theta_i, vtheta_i, dt);

      //increment time
      time += dt;
    } // end for i < numsteps

    //ROS_INFO("OccCost: %f, vx: %.2f, vy: %.2f, vtheta: %.2f", occ_cost, vx_samp, vy_samp, vtheta_samp);
    double cost = -1.0;
    if (!heading_scoring_) {
      cost = path_distance_bias_ * path_dist + goal_dist * goal_distance_bias_ + occdist_scale_ * occ_cost;
    } else {
      cost = occdist_scale_ * occ_cost + path_distance_bias_ * path_dist + 0.3 * heading_diff + goal_dist * goal_distance_bias_;
    }
    traj.cost_ = cost;
  }
  
  
  
    /**
   * create and score a trajectory for Alpha_WaLTR given the current pose of the robot and selected velocities
   */
  void TrajectoryPlanner::generateTrajectory_a_waltr(
      double x, double y, double theta,
      double vx, double vy, double vtheta,
      double vx_samp, double vy_samp, double vtheta_samp,
      double acc_x, double acc_y, double acc_theta,
      double impossible_cost, bool legcell_free_true,
      Trajectory& traj) {
    
    // make sure the configuration doesn't change mid run
    boost::mutex::scoped_lock l(configuration_mutex_);
	
	traj.goal_dist_final_ = 1024; /*uit : m , initialize the goal distance*/

    // x_i and y_i and theta_i are for the sample positions for path extention
    double x_i = x;
    double y_i = y;
	
    double theta_i = theta;
	
	//x_origin, y_origin are for calculating the distance between current robot position and closest leg available obstacle 
	double x_origin = x;
	double y_origin = y;

    double legobs_dist = 1024;  /*unit : m */
	
	 //x_prev, y_prev are for calculating the gradient cost
	double x_prev = x;
	double y_prev = y;
	double theta_prev = theta;

    double vx_i, vy_i, vtheta_i;

    vx_i = vx;
    vy_i = vy;
    vtheta_i = vtheta;

    //compute the magnitude of the velocities
    double vmag = hypot(vx_samp, vy_samp);

    //compute the number of steps we must take along this trajectory to be "safe"
    int num_steps;
	
    num_steps = int(sim_time_ / sim_granularity_ + 0.5);
    /*if(!heading_scoring_) {
      num_steps = int(max((vmag * sim_time_) / sim_granularity_, fabs(vtheta_samp) / angular_sim_granularity_) + 0.5);
    } else {
      num_steps = int(sim_time_ / sim_granularity_ + 0.5);
    }*/

    //we at least want to take one step... even if we won't move, we want to score our current position
    if(num_steps == 0) {
      num_steps = 1;
    }

    double dt = sim_time_ / num_steps;
    double time = 0.0;

    //create a potential trajectory
    traj.resetPoints();
    traj.xv_ = vx_samp;
    traj.yv_ = vy_samp;
    traj.thetav_ = vtheta_samp;
    traj.cost_ = -1.0;

    //initialize the costs for the trajectory
    double path_dist = 0.0;
    double goal_dist = 0.0;
    double occ_cost = 0.0;
	double seg_cost_temp = 0.0;
    double seg_cost = 0.0;
	double cost_final =0.0;
    double heading_diff = 0.0;
	
	double height = 0.0;
    double height_prev = 0.0;	
	double height_cost_temp = 0.0;
	double grad_cost_temp = 0.0;
	double height_cost = 0.0;
	double grad_cost = 0.0;
   
    double ground_cost_dist_sq;
   
    double threshold_obs_h;
	if(legcell_free_true == true) threshold_obs_h = costmap_.leg_available_max_obstacle_height_;
	else threshold_obs_h = costmap_.wheel_available_max_obstacle_height_;
	
     double grad_scale = 0.5;
     double height_scale = 0.5;

   /****************** path extention start ********************/
    for(int i = 0; i < num_steps; ++i){
      //get map coordinates of a point
      unsigned int cell_x, cell_y;
	  //cell_x_prev, cell_y_prev are for calculating the gradient cost
      unsigned int cell_x_prev, cell_y_prev;

      //we don't want a path that goes off the know map
      if(!costmap_.worldToMap(x_i, y_i, cell_x, cell_y)){
        traj.cost_ = -1.0;
        return;
      }

      //check the point on the trajectory for legality
      double footprint_cost = footprintCost_waltr(x_i, y_i, theta_i,legcell_free_true);

      //if the footprint hits an obstacle this trajectory is invalid
      if(footprint_cost < 0){
        traj.cost_ = -1.0;
        return;
        //TODO: Really look at getMaxSpeedToStopInTime... dues to discretization errors and high acceleration limits,
        //it can actually cause the robot to hit obstacles. There may be something to be done to fix, but I'll have to
        //come back to it when I have time. Right now, pulling it out as it'll just make the robot a bit more conservative,
        //but safe.
        /*
        double max_vel_x, max_vel_y, max_vel_th;
        //we want to compute the max allowable speeds to be able to stop
        //to be safe... we'll make sure we can stop some time before we actually hit
        getMaxSpeedToStopInTime(time - stop_time_buffer_ - dt, max_vel_x, max_vel_y, max_vel_th);

        //check if we can stop in time
        if(fabs(vx_samp) < max_vel_x && fabs(vy_samp) < max_vel_y && fabs(vtheta_samp) < max_vel_th){
          ROS_ERROR("v: (%.2f, %.2f, %.2f), m: (%.2f, %.2f, %.2f) t:%.2f, st: %.2f, dt: %.2f", vx_samp, vy_samp, vtheta_samp, max_vel_x, max_vel_y, max_vel_th, time, stop_time_buffer_, dt);
          //if we can stop... we'll just break out of the loop here.. no point in checking future points
          break;
        }
        else{
          traj.cost_ = -1.0;
          return;
        }
        */
      }
	  
	  float temp_cell_cost =  float(costmap_.getCost(cell_x, cell_y));
	  float cost_of_traversable_with_leg = costmap_.leg_available_map_cost_;

      if(legcell_free_true == false)   /*test*/  /*legcell_free_true == false means the traversable cells with leg mode should be obstacle*/
	  {
		  if((temp_cell_cost >= cost_of_traversable_with_leg)&&(temp_cell_cost<LETHAL_OBSTACLE))
		  {
		     temp_cell_cost = LETHAL_OBSTACLE;
			  legobs_dist = 1024;  // unit : m
		  }
		    
	  }
	  else   /*test*/
	  {
          if((temp_cell_cost >= cost_of_traversable_with_leg)&&(temp_cell_cost<(LETHAL_OBSTACLE-50)))
		  {
		     temp_cell_cost = 0;
			 
			  double legobs_dist_temp = hypot(x_i - x_origin, y_i - y_origin);
			  if(legobs_dist_temp < legobs_dist) legobs_dist = legobs_dist_temp;
			  
			  //ROS_INFO("x_i : %lf, x_origin : %lf, y_i : %lf, y_origin : %lf, legobs_dist_temp : %lf, legobs_dist : %lf ", x_i, x_origin, y_i, y_origin, legobs_dist_temp, legobs_dist);
			  
		  }
          else if(temp_cell_cost >(LETHAL_OBSTACLE-50))	
          {
              temp_cell_cost = LETHAL_OBSTACLE;
          }	  
	  }
	  
	  occ_cost = std::max(std::max(occ_cost, footprint_cost), double(temp_cell_cost));

      if (occ_cost == LETHAL_OBSTACLE)
      {
          traj.cost_ = -1.0;
          return;
      }

	  

	  
	  if(costmap_.worldToMap(x_prev, y_prev, cell_x_prev, cell_y_prev))
	  {
	     height_prev = double(costmap_.getHeight(cell_x_prev, cell_y_prev));    // 500 is the resoluon factor of height map,   heigt_prev unit : m
	  }
	  else
	  {
		 height_prev = 0.0; 
	  }
   
     ground_cost_dist_sq = (x_i - x_origin)*(x_i - x_origin) + (y_i - y_origin)*(y_i - y_origin);
     
     //if(ground_cost_dist_sq < (inscribed_radius_*inscribed_radius_))  seg_cost_temp = double(costmap_.getGroundCost(cell_x, cell_y));
	 //else seg_cost_temp = footprintGroundCost_waltr(x_i, y_i, theta_i);
	 seg_cost_temp = double(costmap_.getGroundCost(cell_x, cell_y));
	 
	 double saffety_seg_cost_temp = 0;
	 if(ground_cost_dist_sq > min_vel_x_*sim_time_) saffety_seg_cost_temp = footprintGroundCost_waltr(x_i, y_i, theta_i);
	 
	  if((seg_cost_temp >= LETHAL_OBSTACLE)||(saffety_seg_cost_temp > LETHAL_OBSTACLE))
	  {
		 traj.cost_ = -5.0;
         return;
	  }
      else if((cell_x == cell_x_prev)&&(cell_y == cell_y_prev))
      {
           //if previous cell and current cell is same, do not accumulate the segmentation (terrain) cost
      }
      else
      {
         seg_cost = seg_cost + seg_cost_temp/255;    //divide by 255 means the change the range  (0 ~ 255 to 0 ~ 1)
      }
	  
	  x_prev = x_i;
	  y_prev = y_i;
	  theta_prev = theta_i;
	  
	  height = double(costmap_.getHeight(cell_x, cell_y));   //max value of raw digit in heigtht map = 255 --> 0.51 meter    height unit : m

      double footprint_height =  footprintHeight_waltr(x_i, y_i, theta_i);
	  
	  if((footprint_height*height_scale) > threshold_obs_h)
	  {
		  traj.cost_ = -1.0;
          return;
	  }
	  
	  height = std::max(height,footprint_height);
	  
      height_cost_temp = height * height_scale;
         
	  grad_cost_temp = (height - height_prev)*grad_scale;
	  
	  if(grad_cost_temp<0.0) grad_cost_temp = 0;
	  
	  if((height_cost_temp + grad_cost_temp)>(grad_cost + height_cost))
	  {
		  height_cost = height_cost_temp;
		  grad_cost = grad_cost_temp;
	  }


      if((height_cost + grad_cost) > threshold_obs_h)
      {
		  	traj.height_cost_ = height_cost;
				traj.grad_cost_ = grad_cost;
          traj.cost_ = -4.0;
          return;
       }
	  
	  cost_final = height_cost + seg_cost + grad_cost;
	  
      //do we want to follow blindly
      if (simple_attractor_) {
        goal_dist = std::abs(x_i - global_plan_[global_plan_.size() -1].pose.position.x) + std::abs(y_i - global_plan_[global_plan_.size() -1].pose.position.y);
      } else {

        bool update_path_and_goal_distances = true;

        // with heading scoring, we take into account heading diff, and also only score
        // path and goal distance for one point of the trajectory
        if (heading_scoring_) {
          if (time >= heading_scoring_timestep_ && time < heading_scoring_timestep_ + dt) {
            heading_diff = headingDiff(cell_x, cell_y, x_i, y_i, theta_i);
          } else {
            update_path_and_goal_distances = false;
          }
        }

        if (update_path_and_goal_distances) {
          //update path and goal distances
          path_dist = path_map_(cell_x, cell_y).target_dist;
          goal_dist = goal_map_(cell_x, cell_y).target_dist;

          //if a point on this trajectory has no clear path to goal it is invalid
          if(impossible_cost <= goal_dist || impossible_cost <= path_dist){
//            ROS_DEBUG("No path to goal with goal distance = %f, path_distance = %f and max cost = %f",
//                goal_dist, path_dist, impossible_cost);
            traj.cost_ = -2.0;
            return;
          }
        }
      }
	  
      //the point is legal... add it to the trajectory
      traj.addPoint(x_i, y_i, theta_i);

      //calculate velocities
      vx_i = vx_samp;//computeNewVelocity(vx_samp, vx_i, acc_x, dt);
      vy_i = vy_samp;//computeNewVelocity(vy_samp, vy_i, acc_y, dt);
      vtheta_i = vtheta_samp;// computeNewVelocity(vtheta_samp, vtheta_i, acc_theta, dt);

      //calculate positions
      x_i = computeNewXPosition(x_i, vx_i, vy_i, theta_i, dt);
      y_i = computeNewYPosition(y_i, vx_i, vy_i, theta_i, dt);
      theta_i = computeNewThetaPosition(theta_i, vtheta_i, dt);

      //increment time
      time += dt;
    } // end for i < numsteps

    //ROS_INFO("OccCost: %f, vx: %.2f, vy: %.2f, vtheta: %.2f", occ_cost, vx_samp, vy_samp, vtheta_samp);
    double cost = -1.0;
    if (!heading_scoring_) {
      cost =  goal_dist * goal_distance_bias_ + cost_final;
    } else {
      cost = cost_final  + 0.3 * heading_diff + goal_dist * goal_distance_bias_;
    }
    traj.cost_ = cost;
	
	/*070521 Kangneoung Lee*/
	traj.goal_dist_final_ = goal_dist;
	/*041322 Kangneoung Lee*/
	traj.legobs_dist_ = legobs_dist;
	
	 /*100122 Kangneoung Lee for debugging*/
	traj.goal_bias_cost_ = goal_dist * goal_distance_bias_;
	traj.height_cost_ = height_cost;
	traj.grad_cost_ = grad_cost;
	traj.terrain_t_cost_ = seg_cost;
  }
  
/*
*end of  generateTrajectory_a_waltr
*/
  
  
  

double TrajectoryPlanner::headingDiff(int cell_x, int cell_y, double x, double y, double heading){
    unsigned int goal_cell_x, goal_cell_y;

    // find a clear line of sight from the robot's cell to a farthest point on the path
    for (int i = global_plan_.size() - 1; i >=0; --i) {
      if (costmap_.worldToMap(global_plan_[i].pose.position.x, global_plan_[i].pose.position.y, goal_cell_x, goal_cell_y)) {
        if (lineCost(cell_x, goal_cell_x, cell_y, goal_cell_y) >= 0) {
          double gx, gy;
          costmap_.mapToWorld(goal_cell_x, goal_cell_y, gx, gy);
          return fabs(angles::shortest_angular_distance(heading, atan2(gy - y, gx - x)));
        }
      }
    }
    return DBL_MAX;
  }

  //calculate the cost of a ray-traced line
  double TrajectoryPlanner::lineCost(int x0, int x1,
      int y0, int y1){
    //Bresenham Ray-Tracing
    int deltax = abs(x1 - x0);        // The difference between the x's
    int deltay = abs(y1 - y0);        // The difference between the y's
    int x = x0;                       // Start x off at the first pixel
    int y = y0;                       // Start y off at the first pixel

    int xinc1, xinc2, yinc1, yinc2;
    int den, num, numadd, numpixels;

    double line_cost = 0.0;
    double point_cost = -1.0;

    if (x1 >= x0)                 // The x-values are increasing
    {
      xinc1 = 1;
      xinc2 = 1;
    }
    else                          // The x-values are decreasing
    {
      xinc1 = -1;
      xinc2 = -1;
    }

    if (y1 >= y0)                 // The y-values are increasing
    {
      yinc1 = 1;
      yinc2 = 1;
    }
    else                          // The y-values are decreasing
    {
      yinc1 = -1;
      yinc2 = -1;
    }

    if (deltax >= deltay)         // There is at least one x-value for every y-value
    {
      xinc1 = 0;                  // Don't change the x when numerator >= denominator
      yinc2 = 0;                  // Don't change the y for every iteration
      den = deltax;
      num = deltax / 2;
      numadd = deltay;
      numpixels = deltax;         // There are more x-values than y-values
    } else {                      // There is at least one y-value for every x-value
      xinc2 = 0;                  // Don't change the x for every iteration
      yinc1 = 0;                  // Don't change the y when numerator >= denominator
      den = deltay;
      num = deltay / 2;
      numadd = deltax;
      numpixels = deltay;         // There are more y-values than x-values
    }

    for (int curpixel = 0; curpixel <= numpixels; curpixel++) {
      point_cost = pointCost(x, y); //Score the current point

      if (point_cost < 0) {
        return -1;
      }

      if (line_cost < point_cost) {
        line_cost = point_cost;
      }

      num += numadd;              // Increase the numerator by the top of the fraction
      if (num >= den) {           // Check if numerator >= denominator
        num -= den;               // Calculate the new numerator value
        x += xinc1;               // Change the x as appropriate
        y += yinc1;               // Change the y as appropriate
      }
      x += xinc2;                 // Change the x as appropriate
      y += yinc2;                 // Change the y as appropriate
    }

    return line_cost;
  }

  double TrajectoryPlanner::pointCost(int x, int y){
    unsigned char cost = costmap_.getCost(x, y);
    //if the cell is in an obstacle the path is invalid
    if(cost == LETHAL_OBSTACLE || cost == INSCRIBED_INFLATED_OBSTACLE || cost == NO_INFORMATION){
      return -1;
    }

    return cost;
  }

  void TrajectoryPlanner::updatePlan(const vector<geometry_msgs::PoseStamped>& new_plan, bool compute_dists){
    global_plan_.resize(new_plan.size());
    for(unsigned int i = 0; i < new_plan.size(); ++i){
      global_plan_[i] = new_plan[i];
    }

    if( global_plan_.size() > 0 ){
      geometry_msgs::PoseStamped& final_goal_pose = global_plan_[ global_plan_.size() - 1 ];
      final_goal_x_ = final_goal_pose.pose.position.x;
      final_goal_y_ = final_goal_pose.pose.position.y;
      final_goal_position_valid_ = true;
    } else {
      final_goal_position_valid_ = false;
    }

    if (compute_dists) {
      //reset the map for new operations
      path_map_.resetPathDist();
      goal_map_.resetPathDist();

      //make sure that we update our path based on the global plan and compute costs
      path_map_.setTargetCells(costmap_, global_plan_);
      goal_map_.setLocalGoal(costmap_, global_plan_);
      ROS_DEBUG("Path/Goal distance computed");
    }
  }

  bool TrajectoryPlanner::checkTrajectory(double x, double y, double theta, double vx, double vy,
      double vtheta, double vx_samp, double vy_samp, double vtheta_samp){
    Trajectory t;

    double cost = scoreTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp);

    //if the trajectory is a legal one... the check passes
    if(cost >= 0) {
      return true;
    }
    ROS_WARN("Invalid Trajectory %f, %f, %f, cost: %f", vx_samp, vy_samp, vtheta_samp, cost);

    //otherwise the check fails
    return false;
  }

  double TrajectoryPlanner::scoreTrajectory(double x, double y, double theta, double vx, double vy,
      double vtheta, double vx_samp, double vy_samp, double vtheta_samp) {
    Trajectory t;
    
    bool legcell_free_true;

    if (this->whlLegMode_ == 0) legcell_free_true = false;
    else legcell_free_true = true;
    

    double impossible_cost = path_map_.obstacleCosts();
    generateTrajectory_a_waltr(x, y, theta,
                       vx, vy, vtheta,
                       vx_samp, vy_samp, vtheta_samp,
                       acc_lim_x_, acc_lim_y_, acc_lim_theta_,
                       impossible_cost, legcell_free_true, t);

    // return the cost.
    return double( t.cost_ );
  }

  /*
   * create the trajectories we wish to score
   */
  Trajectory TrajectoryPlanner::createTrajectories(double x, double y, double theta,
      double vx, double vy, double vtheta,
      double acc_x, double acc_y, double acc_theta, std::vector<geometry_msgs::PoseStamped>& local_plan_test_out1,std::vector<geometry_msgs::PoseStamped>& local_plan_test_out2, std::string global_frame_ ) {
    //compute feasible velocity limits in robot space
	
	//ROS_INFO("createTrajectories is entered, current mode is %d", whlLegMode_);
	
	

    //keep track of the best trajectory seen so far
    Trajectory* best_traj = &traj_one;
    best_traj->cost_ = -1.0;

    Trajectory* comp_traj = &traj_two;
    comp_traj->cost_ = -1.0;
	
	
	Trajectory* best_traj_leg = &traj_one_leg;
    best_traj_leg->cost_ = -1.0;

    Trajectory* comp_traj_leg = &traj_two_leg;
    comp_traj_leg->cost_ = -1.0;
	

    Trajectory* swap = NULL;

	/*070721 Kangneoung Lee*/
	Trajectory* temp1 = NULL; /*test*/
	Trajectory* temp2 = NULL; /*test*/

    //any cell with a cost greater than the size of the map is impossible
    double impossible_cost = path_map_.obstacleCosts();
	
	double ParamModeChangeTurningSpeed = 0.5;

    if((whlLegMode_ == 3/*(wheel to leg)*/)||(whlLegMode_ == 4/*(leg to wheel)*/))
	{
		ModeTranStart_x_ = x;
		ModeTranStart_y_ = y;
		
		//if(ModeTranStart_theta_ == -1024.0)   
		//{
		//	ROS_INFO("reset ModeTranStart_theta_ ****");
		//	ModeTranStart_theta_ = theta;     // theta range   -180 degree ~ 180 degree or  -pi (rad) ~ pi (rad)
		//}
		//double ThetaSignConv = 0; 
		
		//if(ModeTranStart_theta_*theta >= 0)
		//{
		//	ThetaSignConv = 1;
		//}
		//else
		//{
		//	ThetaSignConv = -1;
			
		//}
				
		//double ThetaDiff = abs(ModeTranStart_theta_ - ThetaSignConv*theta);
		
		//ROS_INFO("ModeTranStart_theta_ : %lf, theta : %lf, ThetaSignConv : %lf, ThetaDiff : %lf", ModeTranStart_theta_, theta, ThetaSignConv, ThetaDiff);
		
		ModeTran_theta_ = theta;
		
		ModeTran_thetaDiff_1cycle_ = ModeTran_theta_ - ModeTran_thetaPrev_;
		
		
		if(ModeTran_thetaDiff_1cycle_ >= 3.14)  //prevent phase change or theta data stuck 
		{
				ModeTran_thetaDiff_1cycle_ = ModeTran_theta_ - ModeTran_thetaPrev_ - 2*3.14;
		}
		else if(ModeTran_thetaDiff_1cycle_ <= -3.14)
		{
				ModeTran_thetaDiff_1cycle_ = ModeTran_theta_ - ModeTran_thetaPrev_ + 2*3.14;
		}
		
		ModeTran_thetaSum_ = ModeTran_thetaSum_ + ModeTran_thetaDiff_1cycle_;
		
		ROS_INFO("_theta_ : %lf, theta_prev : %lf, _thetaDIff_ : %lf, _thetaDIff_prev : %lf, ModeTran_thetaSum_ : %lf", ModeTran_theta_, ModeTran_thetaPrev_, ModeTran_thetaDiff_1cycle_, ModeTran_thetaDiff_1cyclePrev_,ModeTran_thetaSum_);
		
		ModeTran_thetaPrev_ = ModeTran_theta_;
		ModeTran_thetaDiff_1cyclePrev_ = ModeTran_thetaDiff_1cycle_;
		
		//if(abs(ModeTranStart_theta_ - ThetaSignConv*theta) > 2.9 /*unit : rad*/)
		if(abs(ModeTran_thetaSum_)>3.0)
		{
			if (whlLegMode_ == 3/*(wheel to leg)*/) whlLegMode_ = 1; /*leg mode*/
			else if (whlLegMode_ == 4/*(leg to wheel)*/)  whlLegMode_ = 0; /*wheel mode*/
				
			//ModeTranStart_theta_ == -1024.0;  /*reset the ModeTranStart_theta_*/
			
			/*x y position store for calculating the distance with leg mode later */
			ModeTranStart_x_ = x;
			ModeTranStart_y_ = y;
		}
		else
		{
			best_traj->xv_ = 0;
		    best_traj->yv_ = 0;
		    best_traj->thetav_ = ParamModeChangeTurningSpeed;
		    best_traj->cost_ = 1;
		
		    return *best_traj;
			
		}

	}
	else
	{
		ModeTran_theta_ = theta;
		ModeTran_thetaPrev_ = theta;
		ModeTran_thetaDiff_1cycle_ = 0;
		ModeTran_thetaDiff_1cyclePrev_ = 0;
		ModeTran_thetaSum_ = 0;
	}
	
	if(whlLegMode_==5)
	{
		if((ros::Time::now() - CloseWheel_InitTime_)>CloseWheel_Duration_)
		{
			whlLegMode_ = 2;
		}
		else
		{
                    float close_vel;
                    if(WheelLeg_InstallDir_ == 1) close_vel = -max_vel_x_;
                    else close_vel = max_vel_x_;

		    best_traj->xv_ = close_vel/2;
		    best_traj->yv_ = 0;
		    best_traj->thetav_ = 0;
		    best_traj->cost_ = 1;
		
		    return *best_traj;
			
		}
	}
	else
	{
		CloseWheel_InitTime_ = ros::Time::now();
	}
	
        if (WheelLeg_InstallDir_ == 0)
        {
	   // if the mode is leg mode (whlLegMode_ = 1 or 2), then the negative vx sample should be inputted to the trajectory generation.
	   // otherwise, positive vx sample (original code) should be inputted.
	   if((whlLegMode_== 1)||(whlLegMode_ == 2)) VelSignByWhlLegMode_ = -1;		
	   else  VelSignByWhlLegMode_ = 1;
        }
        else if(WheelLeg_InstallDir_ == 1)
        {
	   if((whlLegMode_== 0)||(whlLegMode_ == 3)||(whlLegMode_ == 4)) VelSignByWhlLegMode_ = -1;		
	   else  VelSignByWhlLegMode_ = 1;     
        }
		
    double max_vel_x = max_vel_x_, max_vel_theta;
    double min_vel_x, min_vel_theta;
	
	/*070121 Kangneoung Lee*/  /*check the cost map*/
	
	//std::string  dir = "/home/kangneoung/move_base_log/costmap.txt";
	//bool pass = costmap_.saveMap(dir);
	
    if( final_goal_position_valid_ ){
      double final_goal_dist = hypot( final_goal_x_ - x, final_goal_y_ - y );
      max_vel_x = min( max_vel_x, final_goal_dist / sim_time_ );
    }

    //should we use the dynamic window approach?
    if (dwa_) {
      max_vel_x = max(min(max_vel_x, vx + acc_x * sim_period_), min_vel_x_);
      min_vel_x = max(min_vel_x_, vx - acc_x * sim_period_);

      max_vel_theta = min(max_vel_th_, vtheta + acc_theta * sim_period_);
      min_vel_theta = max(min_vel_th_, vtheta - acc_theta * sim_period_);
    } else {
      max_vel_x = max(min(max_vel_x, vx + acc_x * sim_time_), min_vel_x_);
      min_vel_x = max(min_vel_x_, vx - acc_x * sim_time_);

      max_vel_theta = min(max_vel_th_, vtheta + acc_theta * sim_time_);
      min_vel_theta = max(min_vel_th_, vtheta - acc_theta * sim_time_);
    }


    //we want to sample the velocity space regularly
    double dvx = VelSignByWhlLegMode_*(max_vel_x - min_vel_x) / (vx_samples_ - 1);
    double dvtheta = (max_vel_theta - min_vel_theta) / (vtheta_samples_ - 1);

    double vx_samp = VelSignByWhlLegMode_*min_vel_x;
    double vtheta_samp = min_vel_theta;
    double vy_samp = 0.0;
	
    
    //if we're performing an escape we won't allow moving forward
    if (!escaping_) {
		
	  bool legmode_flag = false; /* find the path for wheel mode*/
      //loop through all x velocities
      for(int i = 0; i < vx_samples_; ++i) {
        vtheta_samp = 0;
        //first sample the straight trajectory
        generateTrajectory_a_waltr(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp,
            acc_x, acc_y, acc_theta, impossible_cost, legmode_flag, *comp_traj);

		if((DebugTraj_TextWrite_ == true)&&(writing_count_ < 1))
		{
			for (unsigned int j = 0; j < comp_traj->getPointsSize(); ++j)
			{
				double p_x, p_y, p_th;
                comp_traj->getPoint(j, p_x, p_y, p_th);
				
				WhlTrajectoryText_x_ << p_x <<" ";
                WhlTrajectoryText_y_ << p_y <<" ";				
		    }
			WhlTrajectoryText_x_<<std::endl;
			WhlTrajectoryText_y_<<std::endl;
			
			WhlTrajectoryText_cost_ << " linear vel (xv) : " <<  comp_traj->xv_  << "angluar vel (theta v) : " << comp_traj->thetav_ << " Total cost : " << comp_traj->cost_ << " goal_bias_cost_ : " <<  comp_traj->goal_bias_cost_ <<  " height_cost_ : " << comp_traj->height_cost_ <<  " grad_cost_ : " << comp_traj->grad_cost_ << " terrain_t_cost_ : " << comp_traj->terrain_t_cost_ << std::endl;
		}

        //if the new trajectory is better... let's take it
        if(comp_traj->cost_ >= 0 && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)){
          swap = best_traj;
          best_traj = comp_traj;
          comp_traj = swap;
        }
		
        vtheta_samp = min_vel_theta;
        //next sample all theta trajectories
        for(int j = 0; j < vtheta_samples_ - 1; ++j){
          generateTrajectory_a_waltr(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp,
              acc_x, acc_y, acc_theta, impossible_cost, legmode_flag, *comp_traj);
			   
		   if((DebugTraj_TextWrite_ == true)&&(writing_count_ < 1))
		   {
			   for (unsigned int j = 0; j < comp_traj->getPointsSize(); ++j)
			   {
				   double p_x, p_y, p_th;
                   comp_traj->getPoint(j, p_x, p_y, p_th);
				
				   WhlTrajectoryText_x_ << p_x <<" ";
                   WhlTrajectoryText_y_ << p_y <<" ";				
		       }
			   
			   WhlTrajectoryText_x_<<std::endl;
			   WhlTrajectoryText_y_<<std::endl;

			   WhlTrajectoryText_cost_ << " linear vel (xv) : " <<  comp_traj->xv_  << "angluar vel (theta v) : " << comp_traj->thetav_ << " Total cost : " << comp_traj->cost_ << " goal_bias_cost_ : " <<  comp_traj->goal_bias_cost_ <<  " height_cost_ : " << comp_traj->height_cost_ <<  " grad_cost_ : " << comp_traj->grad_cost_ << " terrain_t_cost_ : " << comp_traj->terrain_t_cost_ << std::endl;			   
		   }	  
			  

          //if the new trajectory is better... let's take it
          if(comp_traj->cost_ >= 0 && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)){
            swap = best_traj;
            best_traj = comp_traj;
            comp_traj = swap;
			
          }
          vtheta_samp += dvtheta;
        }
        vx_samp += dvx;
      }
	  
	  
	  if((DebugTraj_TextWrite_ == true)&&(writing_count_ < 1))
	  {
		WhlTrajectoryText_x_ << "best trajectory : "; 
		WhlTrajectoryText_y_ << "best trajectory : ";
		WhlTrajectoryText_cost_ << "best trajectory :";
		 
		for (unsigned int j = 0; j < best_traj->getPointsSize(); ++j)
		{
		   double p_x, p_y, p_th;
           best_traj->getPoint(j, p_x, p_y, p_th);
				
		   WhlTrajectoryText_x_ <<p_x <<" ";
           WhlTrajectoryText_y_ <<p_y <<" ";				
		}
			   
		   WhlTrajectoryText_x_<<std::endl;
		   WhlTrajectoryText_y_<<std::endl;
		   WhlTrajectoryText_cost_ << " linear vel (xv) : " <<  best_traj->xv_  << "angluar vel (theta v) : " << best_traj->thetav_ << " Total cost : " << best_traj->cost_ << " goal_bias_cost_ : " <<  best_traj->goal_bias_cost_ <<  " height_cost_ : " << best_traj->height_cost_ <<  " grad_cost_ : " << best_traj->grad_cost_ << " terrain_t_cost_ : " << best_traj->terrain_t_cost_ << std::endl;
	  }
	  
	  

      legmode_flag = true; /* find the path for leg mode*/
	  /* leg mode */
	  vx_samp = VelSignByWhlLegMode_*min_vel_x; /*reset the vx_samp*/
      //loop through all x velocities
      for(int i = 0; i < vx_samples_; ++i) {
        vtheta_samp = 0;
        //first sample the straight trajectory
        generateTrajectory_a_waltr(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp,
            acc_x, acc_y, acc_theta, impossible_cost, legmode_flag, *comp_traj_leg);
	  
	  if((DebugTraj_TextWrite_ == true)&&(writing_count_ < 1))
		{
			for (unsigned int j = 0; j < comp_traj_leg->getPointsSize(); ++j)
			{
				double p_x, p_y, p_th;
                comp_traj_leg->getPoint(j, p_x, p_y, p_th);
				
				LegTrajectoryText_x_ << p_x <<" ";
                LegTrajectoryText_y_ << p_y <<" ";				
		    }
			LegTrajectoryText_x_<<std::endl;
			LegTrajectoryText_y_<<std::endl;
			
			LegTrajectoryText_cost_ << " linear vel (xv) : " <<  comp_traj_leg->xv_  << " angluar vel (theta v) : " << comp_traj_leg->thetav_ << " Total cost : " << comp_traj_leg->cost_ << " goal_bias_cost_ : " <<  comp_traj_leg->goal_bias_cost_ <<  " height_cost_ : " << comp_traj_leg->height_cost_ <<  " grad_cost_ : " << comp_traj_leg->grad_cost_ << " terrain_t_cost_ : " << comp_traj_leg->terrain_t_cost_ << std::endl;			
		}

        //if the new trajectory is better... let's take it
        if(comp_traj_leg->cost_ >= 0 && (comp_traj_leg->cost_ < best_traj_leg->cost_ || best_traj_leg->cost_ < 0)){
          swap = best_traj_leg;
          best_traj_leg = comp_traj_leg;
          comp_traj_leg = swap;
        }

        vtheta_samp = min_vel_theta;
        //next sample all theta trajectories
        for(int j = 0; j < (vtheta_samples_ - 1); ++j){
          generateTrajectory_a_waltr(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp,
              acc_x, acc_y, acc_theta, impossible_cost, legmode_flag, *comp_traj_leg);
	
	      if((DebugTraj_TextWrite_ == true)&&(writing_count_ < 1))
		  {
			  for (unsigned int j = 0; j < comp_traj_leg->getPointsSize(); ++j)
			  {
				  double p_x, p_y, p_th;
                  comp_traj_leg->getPoint(j, p_x, p_y, p_th);
				
				  LegTrajectoryText_x_ << p_x <<" ";
                  LegTrajectoryText_y_ << p_y <<" ";				
		      }
			  
			  LegTrajectoryText_x_<<std::endl;
			  LegTrajectoryText_y_<<std::endl;
			  LegTrajectoryText_cost_ << " linear vel (xv) : " <<  comp_traj_leg->xv_  << " angluar vel (theta v) : " << comp_traj_leg->thetav_ << " Total cost : " << comp_traj_leg->cost_ << " goal_bias_cost_ : " <<  comp_traj_leg->goal_bias_cost_ <<  " height_cost_ : " << comp_traj_leg->height_cost_ <<  " grad_cost_ : " << comp_traj_leg->grad_cost_ << " terrain_t_cost_ : " << comp_traj_leg->terrain_t_cost_ << std::endl;			
		  
		  }

          //if the new trajectory is better... let's take it
          if(comp_traj_leg->cost_ >= 0 && (comp_traj_leg->cost_ < best_traj_leg->cost_ || best_traj_leg->cost_ < 0)){
            swap = best_traj_leg;
            best_traj_leg = comp_traj_leg;
            comp_traj_leg = swap;
          }
          vtheta_samp += dvtheta;
        }
        vx_samp += dvx;
      }
	  
	  if((DebugTraj_TextWrite_ == true)&&(writing_count_ < 1))
	  {
		   LegTrajectoryText_x_ << "best trajectory : ";
		   LegTrajectoryText_y_ << "best trajectory : ";
		   LegTrajectoryText_cost_ << "best trajectory : ";
		for (unsigned int j = 0; j < best_traj_leg->getPointsSize(); ++j)
		{
		   double p_x, p_y, p_th;
           best_traj_leg->getPoint(j, p_x, p_y, p_th);
				
		   LegTrajectoryText_x_ <<p_x <<" ";
           LegTrajectoryText_y_ <<p_y <<" ";				
		}
			   
		   LegTrajectoryText_x_<<std::endl;
		   LegTrajectoryText_y_<<std::endl;
		   LegTrajectoryText_cost_ << " linear vel (xv) : " <<  best_traj_leg->xv_  << "angluar vel (theta v) : " << best_traj_leg->thetav_ << " Total cost : " << best_traj_leg->cost_ << " goal_bias_cost_ : " <<  best_traj_leg->goal_bias_cost_ <<  " height_cost_ : " << best_traj_leg->height_cost_ <<  " grad_cost_ : " << best_traj_leg->grad_cost_ << " terrain_t_cost_ : " << best_traj_leg->terrain_t_cost_ << std::endl;			
		   
	  }
	  
	 
	  /* leg mode end */
	  writing_count_++;
	  

      //only explore y velocities with holonomic robots
      if (holonomic_robot_) {
        //explore trajectories that move forward but also strafe slightly
        vx_samp = 0.1;
        vy_samp = 0.1;
        vtheta_samp = 0.0;
        generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp,
            acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);

        //if the new trajectory is better... let's take it
        if(comp_traj->cost_ >= 0 && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)){
          swap = best_traj;
          best_traj = comp_traj;
          comp_traj = swap;
        }

        vx_samp = 0.1;
        vy_samp = -0.1;
        vtheta_samp = 0.0;
        generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp,
            acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);

        //if the new trajectory is better... let's take it
        if(comp_traj->cost_ >= 0 && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)){
          swap = best_traj;
          best_traj = comp_traj;
          comp_traj = swap;
        }
      }
    } // end if not escaping
	
	/*070521 Kangneoung Lee*/
	if(best_traj->cost_ >=0) temp1 = best_traj; /*test*/
	if(best_traj_leg->cost_ >= 0) temp2 = best_traj_leg; /*test*/
	
	
	float TravelDistLeg = 0; /*unit : m */
	float ParamTravelDistLegTh = 1; /*unit : m*/
	
	
	float GoalDistDiff;
    if(simple_attractor_)
    {
        GoalDistDiff = (float)(best_traj->goal_dist_final_ - best_traj_leg->goal_dist_final_  );
	}
    else
    {
         GoalDistDiff =(float)(best_traj->goal_dist_final_ - best_traj_leg->goal_dist_final_  )*costmap_.getResolution();   // cell size of goal difference unit conversion  cell -> meter
	}	
	
	//https://www.servocity.com/5203-series-yellow-jacket-planetary-gear-motor-139-1-ratio-24mm-length-8mm-rex-shaft-43-rpm-3-3-5v-encoder/
	//Nominal Voltage : 12V, Stall current : 9.2A, Stall torque : 2570 oz-in, 18.148 Nm, Gear ratio : 139 : 1
	// wheel radius of leg mode : 0.2m
	float t_mc = 2.0; // ClosingTime or mode change time
	float t_delay = 1.0; //climbingup_delay
	float Weight_C = 1;   // callibration out = 1000, nominal value : 1
	float r_w_leg = 0.2;
	float P_o = 12*9.2; // Required power = 12V * 9.2A
	float T_o = 18.148; //Stall torque
	//float ParamGoalDiffWhlToLegTh = (max_vel_x_*t_mc+P_o/T_o*r_w_leg)*Weight_C;
	float ParamGoalDiffWhlToLegTh = 1;//10000; /*10000 is callibration out, resonable value is  0.5m ~ 3m default value : 1 */
	float ParamGoalDiffLegToWhlTh = -1000;/*cal out*/ //0.5; /*minus value is callibration out, resonable value is 0.5m*/
	float ParamLegObsDistTh = 1000; //1.5; /*unit : m*/   callibration out = 1000
	
	float ObsFreeDistTh = (min_vel_x+max_vel_x)/2*sim_time_;  /*unit : m */
	//ROS_INFO("wheel mode goal distance: %lf, leg mode goal distance: %lf, goal_difference: %f, best_traj->cost_ : %lf, best_traj_leg->cost_ : %lf", best_traj->goal_dist_final_,  best_traj_leg->goal_dist_final_, GoalDistDiff, best_traj->cost_, best_traj_leg->cost_); /*test*/
	
		
    if(whlLegMode_ == 0/* wheel mode*/)
	{ 
	
		if((GoalDistDiff>ParamGoalDiffWhlToLegTh)) //||(best_traj->cost_< -0)
	    {
			
			ROS_INFO("Trajectory swaped, GoalDistDiff : %f, ParamGoalDiffWhlToLegTh : %f, whlLegMode_ : %d, best_traj->legobs_dist_ : %f, ParamLegObsDistTh : %f", GoalDistDiff, ParamGoalDiffWhlToLegTh, whlLegMode_, best_traj_leg->legobs_dist_, ParamLegObsDistTh);
		
			if(best_traj_leg->legobs_dist_ < ParamLegObsDistTh /*unit :m*/)
			{
				whlLegMode_ = 3; /*wheel to leg*/
				ROS_INFO("Transformation wheel to leg,  best_traj_leg->legobs_dist_ : %f, ParamLegObsDistTh : %f, whlLegMode_ : %d", best_traj_leg->legobs_dist_, ParamLegObsDistTh, whlLegMode_);
                return *best_traj_leg;				
			}
            else if(best_traj->cost_ >= 0)  return *best_traj;  /*wheel mode trajectory is feasible*/
    				
		}
        else if(best_traj->cost_ >= 0)  return *best_traj;  /*wheel mode trajectory is feasible*/
        else if(best_traj_leg->cost_>=0) return *best_traj_leg; /*wheel mode trajectory is not fesible and leg mode trajectoryis feasble*/ 
		
	}
	else if(whlLegMode_ == 1/* Leg mode*/)
	{

		TravelDistLeg = hypot(ModeTranStart_x_ - x, ModeTranStart_y_ - y);
		
		if((GoalDistDiff<ParamGoalDiffLegToWhlTh))
	    {			 
			if(TravelDistLeg > 2*ParamLegObsDistTh /*unit :m*/)
			{
				whlLegMode_ = 4; /*leg to wheel */
				ROS_INFO("Transformation wheel to leg,  TravelDistLeg : %f, ParamTravelDistLegTh : %f, whlLegMode_ : %d", TravelDistLeg, ParamTravelDistLegTh, whlLegMode_);	
			}			
		}
        else if((TravelDistLeg > 2*ParamLegObsDistTh)&&(best_traj_leg->legobs_dist_ > ObsFreeDistTh))
		{
			whlLegMode_ = 5; //transition (wheel open to closed);
		} 
		if(best_traj_leg->cost_>=0) return *best_traj_leg;
		
	}
	else if(whlLegMode_ == 2 /*leg mode with closed wheel*/)
	{
		if((GoalDistDiff>ParamGoalDiffWhlToLegTh)) //||(best_traj->cost_< -0)
	    {
			ROS_INFO("Follow the leg mode trajectory,  TravelDistLeg : %f, 2*ParamLegObsDistTh : %f, best_traj_leg->legobs_dist_  : %f MarkObsDistFlag_ : %d", TravelDistLeg, 2*ParamLegObsDistTh, best_traj_leg->legobs_dist_,MarkObsDistFlag_ );

		    if((best_traj_leg->legobs_dist_ < ParamLegObsDistTh /*unit :m*/)&&(MarkObsDistFlag_ == false))
		    {
			    MarkObsDistFlag_ = true;
			    ModeTranStart_x_ = x;
			    ModeTranStart_y_ = y;
		    	
		    }
		    else if(MarkObsDistFlag_ == true)
		    {
			    TravelDistLeg = hypot(ModeTranStart_x_ - x, ModeTranStart_y_ - y);
			    ROS_INFO("Precondition leg to wheel mode,  TravelDistLeg : %f, 2*ParamLegObsDistTh : %f, best_traj_leg->legobs_dist_  : %f MarkObsDistFlag_ : %d", TravelDistLeg, 2*ParamLegObsDistTh, best_traj_leg->legobs_dist_,MarkObsDistFlag_ );
			
			    if((TravelDistLeg > 2*ParamLegObsDistTh)&&(best_traj_leg->legobs_dist_ > ObsFreeDistTh))
			    {
                    whlLegMode_ = 5; //transition (wheel open to closed);
			        MarkObsDistFlag_ = false;
			
			    }
			
		    }
			
		    if(best_traj_leg->cost_>=0) return *best_traj_leg;
        }
		else if(best_traj->cost_>=0) return *best_traj;
		else if(best_traj_leg->cost_>=0) return *best_traj_leg;
	}
		

    //next we want to generate trajectories for rotating in place
    vtheta_samp = min_vel_theta;
    vx_samp = 0.0;
    vy_samp = 0.0;

    //let's try to rotate toward open space
    double heading_dist = DBL_MAX;

    for(int i = 0; i < vtheta_samples_; ++i) {
      //enforce a minimum rotational velocity because the base can't handle small in-place rotations
      double vtheta_samp_limited = vtheta_samp > 0 ? max(vtheta_samp, min_in_place_vel_th_)
        : min(vtheta_samp, -1.0 * min_in_place_vel_th_);

      generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp_limited,
          acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);

      //if the new trajectory is better... let's take it...
      //note if we can legally rotate in place we prefer to do that rather than move with y velocity
      if(comp_traj->cost_ >= 0
          && (comp_traj->cost_ <= best_traj->cost_ || best_traj->cost_ < 0 || best_traj->yv_ != 0.0)
          && (vtheta_samp > dvtheta || vtheta_samp < -1 * dvtheta)){
        double x_r, y_r, th_r;
        comp_traj->getEndpoint(x_r, y_r, th_r);
        x_r += heading_lookahead_ * cos(th_r);
        y_r += heading_lookahead_ * sin(th_r);
        unsigned int cell_x, cell_y;

        //make sure that we'll be looking at a legal cell
        if (costmap_.worldToMap(x_r, y_r, cell_x, cell_y)) {
          double ahead_gdist = goal_map_(cell_x, cell_y).target_dist;
          if (ahead_gdist < heading_dist) {
            //if we haven't already tried rotating left since we've moved forward
            if (vtheta_samp < 0 && !stuck_left) {
              swap = best_traj;
              best_traj = comp_traj;
              comp_traj = swap;
              heading_dist = ahead_gdist;
            }
            //if we haven't already tried rotating right since we've moved forward
            else if(vtheta_samp > 0 && !stuck_right) {
              swap = best_traj;
              best_traj = comp_traj;
              comp_traj = swap;
              heading_dist = ahead_gdist;
            }
          }
        }
      }

      vtheta_samp += dvtheta;
    }

    //do we have a legal trajectory
    if (best_traj->cost_ >= 0) {
      if (best_traj->xv_ == 0) ROS_INFO("Can not find vaild path, rotation motion start");
      
      // avoid oscillations of in place rotation and in place strafing
      if ( ! (best_traj->xv_ *VelSignByWhlLegMode_> 0)) {
        if (best_traj->thetav_ < 0) {
          if (rotating_right) {
            stuck_right = true;
          }
          rotating_right = true;
        } else if (best_traj->thetav_ > 0) {
          if (rotating_left){
            stuck_left = true;
          }
          rotating_left = true;
        } else if(best_traj->yv_ > 0) {
          if (strafe_right) {
            stuck_right_strafe = true;
          }
          strafe_right = true;
        } else if(best_traj->yv_ < 0){
          if (strafe_left) {
            stuck_left_strafe = true;
          }
          strafe_left = true;
        }

        //set the position we must move a certain distance away from
        prev_x_ = x;
        prev_y_ = y;
      }

      double dist = hypot(x - prev_x_, y - prev_y_);
      if (dist > oscillation_reset_dist_) {
        rotating_left = false;
        rotating_right = false;
        strafe_left = false;
        strafe_right = false;
        stuck_left = false;
        stuck_right = false;
        stuck_left_strafe = false;
        stuck_right_strafe = false;
      }

      dist = hypot(x - escape_x_, y - escape_y_);
      if(dist > escape_reset_dist_ ||
          fabs(angles::shortest_angular_distance(escape_theta_, theta)) > escape_reset_theta_){
        escaping_ = false;
      }
	  
	  
	  
	 if(temp1!=NULL)
	 {
	    for (unsigned int i = 0; i < temp1->getPointsSize(); ++i) {
         double p_x, p_y, p_th;
         temp1->getPoint(i, p_x, p_y, p_th);
         geometry_msgs::PoseStamped pose;
         pose.header.frame_id = global_frame_;
         pose.header.stamp = ros::Time::now();
         pose.pose.position.x = p_x;
         pose.pose.position.y = p_y;
         pose.pose.position.z = 0.0;
         tf2::Quaternion q;
         q.setRPY(0, 0, p_th);
         tf2::convert(q, pose.pose.orientation);
         local_plan_test_out1.push_back(pose);
        }
	 }
	
	//ROS_INFO("Kangneoung Debug 11");
	 
	 if(temp2!=NULL)
	 {
	    for (unsigned int i = 0; i < temp2->getPointsSize(); ++i) {
         double p_x, p_y, p_th;
         temp2->getPoint(i, p_x, p_y, p_th);
         geometry_msgs::PoseStamped pose;
         pose.header.frame_id = global_frame_;
         pose.header.stamp = ros::Time::now();
         pose.pose.position.x = p_x;
         pose.pose.position.y = p_y;
         pose.pose.position.z = 0.0;
         tf2::Quaternion q;
         q.setRPY(0, 0, p_th);
         tf2::convert(q, pose.pose.orientation);
         local_plan_test_out2.push_back(pose);
	     }
      }

      return *best_traj;
    }

    //only explore y velocities with holonomic robots
    if (holonomic_robot_) {
      //if we can't rotate in place or move forward... maybe we can move sideways and rotate
      vtheta_samp = min_vel_theta;
      vx_samp = 0.0;

      //loop through all y velocities
      for(unsigned int i = 0; i < y_vels_.size(); ++i){
        vtheta_samp = 0;
        vy_samp = y_vels_[i];
        //sample completely horizontal trajectories
        generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp,
            acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);

        //if the new trajectory is better... let's take it
        if(comp_traj->cost_ >= 0 && (comp_traj->cost_ <= best_traj->cost_ || best_traj->cost_ < 0)){
          double x_r, y_r, th_r;
          comp_traj->getEndpoint(x_r, y_r, th_r);
          x_r += heading_lookahead_ * cos(th_r);
          y_r += heading_lookahead_ * sin(th_r);
          unsigned int cell_x, cell_y;

          //make sure that we'll be looking at a legal cell
          if(costmap_.worldToMap(x_r, y_r, cell_x, cell_y)) {
            double ahead_gdist = goal_map_(cell_x, cell_y).target_dist;
            if (ahead_gdist < heading_dist) {
              //if we haven't already tried strafing left since we've moved forward
              if (vy_samp > 0 && !stuck_left_strafe) {
                swap = best_traj;
                best_traj = comp_traj;
                comp_traj = swap;
                heading_dist = ahead_gdist;
              }
              //if we haven't already tried rotating right since we've moved forward
              else if(vy_samp < 0 && !stuck_right_strafe) {
                swap = best_traj;
                best_traj = comp_traj;
                comp_traj = swap;
                heading_dist = ahead_gdist;
              }
            }
          }
        }
      }
    }

    //do we have a legal trajectory
    if (best_traj->cost_ >= 0) {
      if (!(best_traj->xv_ *VelSignByWhlLegMode_> 0)) {
        if (best_traj->thetav_ < 0) {
          if (rotating_right){
            stuck_right = true;
          }
          rotating_left = true;
        } else if(best_traj->thetav_ > 0) {
          if(rotating_left){
            stuck_left = true;
          }
          rotating_right = true;
        } else if(best_traj->yv_ > 0) {
          if(strafe_right){
            stuck_right_strafe = true;
          }
          strafe_left = true;
        } else if(best_traj->yv_ < 0) {
          if(strafe_left){
            stuck_left_strafe = true;
          }
          strafe_right = true;
        }

        //set the position we must move a certain distance away from
        prev_x_ = x;
        prev_y_ = y;

      }

      double dist = hypot(x - prev_x_, y - prev_y_);
      if(dist > oscillation_reset_dist_) {
        rotating_left = false;
        rotating_right = false;
        strafe_left = false;
        strafe_right = false;
        stuck_left = false;
        stuck_right = false;
        stuck_left_strafe = false;
        stuck_right_strafe = false;
      }

      dist = hypot(x - escape_x_, y - escape_y_);
      if(dist > escape_reset_dist_ || fabs(angles::shortest_angular_distance(escape_theta_, theta)) > escape_reset_theta_) {
        escaping_ = false;
      }

      return *best_traj;
    }

    //and finally, if we can't do anything else, we want to generate trajectories that move backwards slowly
    vtheta_samp = 0.0;
    vx_samp = VelSignByWhlLegMode_*backup_vel_;
    vy_samp = 0.0;
    generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp,
        acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);

    ROS_INFO("Can not find vaild path, escape mode enter");
    //if the new trajectory is better... let's take it
    /*
       if(comp_traj->cost_ >= 0 && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)){
       swap = best_traj;
       best_traj = comp_traj;
       comp_traj = swap;
       }
       */

    //we'll allow moving backwards slowly even when the static map shows it as blocked
    swap = best_traj;
    best_traj = comp_traj;
    comp_traj = swap;

    double dist = hypot(x - prev_x_, y - prev_y_);
    if (dist > oscillation_reset_dist_) {
      rotating_left = false;
      rotating_right = false;
      strafe_left = false;
      strafe_right = false;
      stuck_left = false;
      stuck_right = false;
      stuck_left_strafe = false;
      stuck_right_strafe = false;
    }

    //only enter escape mode when the planner has given a valid goal point
    if (!escaping_ && best_traj->cost_ > -2.0) {
      escape_x_ = x;
      escape_y_ = y;
      escape_theta_ = theta;
      escaping_ = true;
    }

    dist = hypot(x - escape_x_, y - escape_y_);

    if (dist > escape_reset_dist_ ||
        fabs(angles::shortest_angular_distance(escape_theta_, theta)) > escape_reset_theta_) {
      escaping_ = false;
    }


    //if the trajectory failed because the footprint hits something, we're still going to back up
    if(best_traj->cost_ == -1.0)
      best_traj->cost_ = 1.0;
  
  
  	 //ROS_INFO("Kangneoung Debug 10");
	 
	 if(temp1!=NULL)
	 {
	    for (unsigned int i = 0; i < temp1->getPointsSize(); ++i) {
         double p_x, p_y, p_th;
         temp1->getPoint(i, p_x, p_y, p_th);
         geometry_msgs::PoseStamped pose;
         pose.header.frame_id = global_frame_;
         pose.header.stamp = ros::Time::now();
         pose.pose.position.x = p_x;
         pose.pose.position.y = p_y;
         pose.pose.position.z = 0.0;
         tf2::Quaternion q;
         q.setRPY(0, 0, p_th);
         tf2::convert(q, pose.pose.orientation);
         local_plan_test_out1.push_back(pose);
        }
	}
	
	//ROS_INFO("Kangneoung Debug 11");
	 
	 if(temp2!=NULL)
	 {
	    for (unsigned int i = 0; i < temp2->getPointsSize(); ++i) {
         double p_x, p_y, p_th;
         temp2->getPoint(i, p_x, p_y, p_th);
         geometry_msgs::PoseStamped pose;
         pose.header.frame_id = global_frame_;
         pose.header.stamp = ros::Time::now();
         pose.pose.position.x = p_x;
         pose.pose.position.y = p_y;
         pose.pose.position.z = 0.0;
         tf2::Quaternion q;
         q.setRPY(0, 0, p_th);
         tf2::convert(q, pose.pose.orientation);
         local_plan_test_out2.push_back(pose);
	     }
    }
  
    return *best_traj;

  }
  
  //given the current state of the robot, find a good trajectory
  Trajectory TrajectoryPlanner::findBestPath(const geometry_msgs::PoseStamped& global_pose,
      geometry_msgs::PoseStamped& global_vel, geometry_msgs::PoseStamped& drive_velocities, std::vector<geometry_msgs::PoseStamped>& local_plan_test_out1,std::vector<geometry_msgs::PoseStamped>& local_plan_test_out2, std::string global_frame_) {

    Eigen::Vector3f pos(global_pose.pose.position.x, global_pose.pose.position.y, tf2::getYaw(global_pose.pose.orientation));
    Eigen::Vector3f vel(global_vel.pose.position.x, global_vel.pose.position.y, tf2::getYaw(global_vel.pose.orientation));

    //reset the map for new operations
    path_map_.resetPathDist();
    goal_map_.resetPathDist();

    //temporarily remove obstacles that are within the footprint of the robot
    std::vector<base_local_planner::Position2DInt> footprint_list =
        footprint_helper_.getFootprintCells(
            pos,
            footprint_spec_,
            costmap_,
            true);

    //mark cells within the initial footprint of the robot
    for (unsigned int i = 0; i < footprint_list.size(); ++i) {
      path_map_(footprint_list[i].x, footprint_list[i].y).within_robot = true;
    }

    //make sure that we update our path based on the global plan and compute costs
    path_map_.setTargetCells(costmap_, global_plan_);
    goal_map_.setLocalGoal(costmap_, global_plan_);
    ROS_DEBUG("Path/Goal distance computed");

    //rollout trajectories and find the minimum cost one
    Trajectory best = createTrajectories(pos[0], pos[1], pos[2],
        vel[0], vel[1], vel[2],
        acc_lim_x_, acc_lim_y_, acc_lim_theta_, local_plan_test_out1, local_plan_test_out2,global_frame_);
    ROS_DEBUG("Trajectories created");

    /*
    //If we want to print a ppm file to draw goal dist
    char buf[4096];
    sprintf(buf, "base_local_planner.ppm");
    FILE *fp = fopen(buf, "w");
    if(fp){
      fprintf(fp, "P3\n");
      fprintf(fp, "%d %d\n", map_.size_x_, map_.size_y_);
      fprintf(fp, "255\n");
      for(int j = map_.size_y_ - 1; j >= 0; --j){
        for(unsigned int i = 0; i < map_.size_x_; ++i){
          int g_dist = 255 - int(map_(i, j).goal_dist);
          int p_dist = 255 - int(map_(i, j).path_dist);
          if(g_dist < 0)
            g_dist = 0;
          if(p_dist < 0)
            p_dist = 0;
          fprintf(fp, "%d 0 %d ", g_dist, 0);
        }
        fprintf(fp, "\n");
      }
      fclose(fp);
    }
    */

    if(best.cost_ < 0){
      drive_velocities.pose.position.x = 0;
      drive_velocities.pose.position.y = 0;
      drive_velocities.pose.position.z = 0;
      drive_velocities.pose.orientation.w = 1;
      drive_velocities.pose.orientation.x = 0;
      drive_velocities.pose.orientation.y = 0;
      drive_velocities.pose.orientation.z = 0;
      ROS_INFO("best.cost_ < 0");
    }
    else{
      drive_velocities.pose.position.x = best.xv_;
      drive_velocities.pose.position.y = best.yv_;
      drive_velocities.pose.position.z = 0;
      tf2::Quaternion q;
      q.setRPY(0, 0, best.thetav_);
      tf2::convert(q, drive_velocities.pose.orientation);
      //ROS_INFO("best.xv_ : %f, best.yv_ : %f best.thetav_ : %f",best.xv_,best.yv_,best.thetav_);
    }
	

	
    return best;
  }

  //we need to take the footprint of the robot into account when we calculate cost to obstacles
  double TrajectoryPlanner::footprintCost(double x_i, double y_i, double theta_i){
    //check if the footprint is legal
	
	unsigned char footprintcost_index = 0;
	
    return world_model_.footprintCost(x_i, y_i, theta_i, footprint_spec_, inscribed_radius_, circumscribed_radius_,footprintcost_index);
  }
  
  /*070521 Kangneoung Lee*/
 double TrajectoryPlanner::footprintCost_waltr(double x_i, double y_i, double theta_i,bool legcell_free_true){
    //check if the footprint is legal
	
	unsigned char footprintcost_index = 0; 
	
	if(legcell_free_true ==false)  /*legcell_free_true == false means the traversable cells with leg mode should be obstacle*/
	{
		footprintcost_index = 1;
		
       return world_model_.footprintCost(x_i, y_i, theta_i, footprint_spec_, inscribed_radius_, circumscribed_radius_,footprintcost_index);
	}
	else
	{
		footprintcost_index = 2;
	   return world_model_.footprintCost(x_i, y_i, theta_i, footprint_spec_, inscribed_radius_, circumscribed_radius_,footprintcost_index);
	}
  }
  
    /*070521 Kangneoung Lee*/
 double TrajectoryPlanner::footprintHeight_waltr(double x_i, double y_i, double theta_i){
    //check if the footprint is legal
		
       return world_model_.footprintHeight(x_i, y_i, theta_i, footprint_spec_, inscribed_radius_, circumscribed_radius_);

  }
  
  
      /*070521 Kangneoung Lee*/
 double TrajectoryPlanner::footprintGroundCost_waltr(double x_i, double y_i, double theta_i){
    //check if the footprint is legal
		
       return world_model_.footprintGroundCost(x_i, y_i, theta_i, footprint_spec_, inscribed_radius_, circumscribed_radius_);

  }
  


  void TrajectoryPlanner::getLocalGoal(double& x, double& y){
    x = path_map_.goal_x_;
    y = path_map_.goal_y_;
  }

};


