/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 *         David V. Lu!!
 *********************************************************************/
#include <boost/bind.hpp>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/cost_values.h>

namespace costmap_2d
{

char* Costmap2DPublisher::cost_translation_table_ = NULL;

Costmap2DPublisher::Costmap2DPublisher(ros::NodeHandle * ros_node, Costmap2D* costmap, std::string global_frame,
                                       std::string topic_name, bool always_send_full_costmap) :
    node(ros_node), costmap_(costmap), global_frame_(global_frame), active_(false),
    always_send_full_costmap_(always_send_full_costmap)
{
  costmap_pub_ = ros_node->advertise<nav_msgs::OccupancyGrid>(topic_name, 1,
                                                    boost::bind(&Costmap2DPublisher::onNewSubscription, this, _1));
  costmap_ground_pub_ = ros_node->advertise<nav_msgs::OccupancyGrid>(topic_name+"_ground", 1); //031422 Kangneoung Lee   groundcostmap
  heightmap_pub_ = ros_node->advertise<nav_msgs::OccupancyGrid>(topic_name+"_height", 1);  //031422 Kangneoung Lee   heightmap
  costmap_update_pub_ = ros_node->advertise<map_msgs::OccupancyGridUpdate>(topic_name + "_updates", 1);
  costmap_ground_update_pub_ = ros_node->advertise<map_msgs::OccupancyGridUpdate>(topic_name +"_ground"+ "_updates", 1);
  heightmap_update_pub_ = ros_node->advertise<map_msgs::OccupancyGridUpdate>(topic_name +"_height"+ "_updates", 1);

  if (cost_translation_table_ == NULL)
  {
    cost_translation_table_ = new char[256];

    // special values:
    cost_translation_table_[0] = 0;  // NO obstacle
    cost_translation_table_[253] = 99;  // INSCRIBED obstacle
    cost_translation_table_[254] = 100;  // LETHAL obstacle
    cost_translation_table_[255] = -1;  // UNKNOWN

    // regular cost values scale the range 1 to 252 (inclusive) to fit
    // into 1 to 98 (inclusive).
    for (int i = 1; i < 253; i++)
    {
      cost_translation_table_[ i ] = char(1 + (97 * (i - 1)) / 251);
    }
  }

  xn_ = yn_ = 0;
  x0_ = costmap_->getSizeInCellsX();
  y0_ = costmap_->getSizeInCellsY();
}

Costmap2DPublisher::~Costmap2DPublisher()
{
}

void Costmap2DPublisher::onNewSubscription(const ros::SingleSubscriberPublisher& pub)
{
  prepareGrid();
  pub.publish(grid_);
  costmap_ground_pub_.publish(groundgrid_);  //031422 Kangneoung Lee   groundcostmap
  heightmap_pub_.publish(heightgrid_); //031422 Kangneoung Lee   heightmap
  
}

// prepare grid_ message for publication.
void Costmap2DPublisher::prepareGrid()
{
  boost::unique_lock<Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
  double resolution = costmap_->getResolution();

  grid_.header.frame_id = global_frame_;
  grid_.header.stamp = ros::Time::now();
  grid_.info.resolution = resolution;

  grid_.info.width = costmap_->getSizeInCellsX();
  grid_.info.height = costmap_->getSizeInCellsY();

  double wx, wy;
  costmap_->mapToWorld(0, 0, wx, wy);
  grid_.info.origin.position.x = wx - resolution / 2;
  grid_.info.origin.position.y = wy - resolution / 2;
  grid_.info.origin.position.z = 0.0;
  grid_.info.origin.orientation.w = 1.0;
  saved_origin_x_ = costmap_->getOriginX();
  saved_origin_y_ = costmap_->getOriginY();
  
  grid_.data.resize(grid_.info.width * grid_.info.height);

  unsigned char* data = costmap_->getCharMap();
  for (unsigned int i = 0; i < grid_.data.size(); i++)
  {
    grid_.data[i] = cost_translation_table_[ data[ i ]];
  }
  
  //031422 Kangneoung Lee   groundcostmap
  groundgrid_.header.frame_id = global_frame_;
  groundgrid_.header.stamp = ros::Time::now();
  groundgrid_.info.resolution = resolution;
  
  groundgrid_.info.width = grid_.info.width;
  groundgrid_.info.height = grid_.info.height;
  
  groundgrid_.info.origin.position.x = grid_.info.origin.position.x;
  groundgrid_.info.origin.position.y = grid_.info.origin.position.y;
  groundgrid_.info.origin.position.z = grid_.info.origin.position.z;
  groundgrid_.info.origin.orientation.w = 1.0;
  
  groundgrid_.data.resize(groundgrid_.info.width * groundgrid_.info.height);
  
  data = costmap_->getGroundCharMap();
  for (unsigned int i = 0; i < groundgrid_.data.size(); i++)
  {
    groundgrid_.data[i] = cost_translation_table_[ data[ i ]];
  }
  

  //031422 Kangneoung Lee   heightmap
  heightgrid_.header.frame_id = global_frame_;
  heightgrid_.header.stamp = ros::Time::now();
  heightgrid_.info.resolution = resolution;
  
  heightgrid_.info.width = grid_.info.width;
  heightgrid_.info.height = grid_.info.height;
  
  heightgrid_.info.origin.position.x = grid_.info.origin.position.x;
  heightgrid_.info.origin.position.y = grid_.info.origin.position.y;
  heightgrid_.info.origin.position.z = grid_.info.origin.position.z;
  heightgrid_.info.origin.orientation.w = 1.0;
  
  heightgrid_.data.resize(heightgrid_.info.width * heightgrid_.info.height);
  
  data = costmap_->getHeightMap();
  for (unsigned int i = 0; i < heightgrid_.data.size(); i++)
  {
    heightgrid_.data[i] = cost_translation_table_[ data[ i ]];
  }
  
  
}

void Costmap2DPublisher::publishCostmap()
{
  if (costmap_pub_.getNumSubscribers() == 0)
  {
    // No subscribers, so why do any work?
    return;
  }

  float resolution = costmap_->getResolution();

  if (always_send_full_costmap_ || grid_.info.resolution != resolution ||
      grid_.info.width != costmap_->getSizeInCellsX() ||
      grid_.info.height != costmap_->getSizeInCellsY() ||
      saved_origin_x_ != costmap_->getOriginX() ||
      saved_origin_y_ != costmap_->getOriginY())
  {
    prepareGrid();
    costmap_pub_.publish(grid_);
	costmap_ground_pub_.publish(groundgrid_);  //031422 Kangneoung Lee   groundcostmap
    heightmap_pub_.publish(heightgrid_);  //031422 Kangneoung Lee   heightmap
  }
  else if (x0_ < xn_)
  {
    boost::unique_lock<Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
    // Publish Just an Update
    map_msgs::OccupancyGridUpdate update;
    update.header.stamp = ros::Time::now();
    update.header.frame_id = global_frame_;
    update.x = x0_;
    update.y = y0_;
    update.width = xn_ - x0_;
    update.height = yn_ - y0_;
    update.data.resize(update.width * update.height);

    unsigned int i = 0;
    for (unsigned int y = y0_; y < yn_; y++)
    {
      for (unsigned int x = x0_; x < xn_; x++)
      {
        unsigned char cost = costmap_->getCost(x, y);
        update.data[i++] = cost_translation_table_[ cost ];
      }
    }
    costmap_update_pub_.publish(update);
	
	//031422 Kangneoung Lee   groundcostmap
	map_msgs::OccupancyGridUpdate ground_update;
	ground_update.header.stamp = ros::Time::now();
	ground_update.header.frame_id = update.header.frame_id;
	ground_update.x = update.x;
	ground_update.y = update.y;	
	ground_update.width = update.width;
	ground_update.height =  update.height;	
	ground_update.data.resize(ground_update.width * ground_update.height);
	
	i = 0;
    for (unsigned int y = y0_; y < yn_; y++)
    {
      for (unsigned int x = x0_; x < xn_; x++)
      {
        unsigned char ground_cost = costmap_->getGroundCost(x, y);
        ground_update.data[i++] = cost_translation_table_[ ground_cost ];
      }
    }
    costmap_ground_update_pub_.publish(ground_update);
	
	
	//031422 Kangneoung Lee   heightmap
	map_msgs::OccupancyGridUpdate height_update;
	height_update.header.stamp = ros::Time::now();
	height_update.header.frame_id = update.header.frame_id;
	height_update.x = update.x;
	height_update.y = update.y;	
	height_update.width = update.width;
	height_update.height =  update.height;	
	height_update.data.resize(height_update.width * height_update.height);
	
	i = 0;
    for (unsigned int y = y0_; y < yn_; y++)
    {
      for (unsigned int x = x0_; x < xn_; x++)
      {
        unsigned char height_cost = costmap_->getHeight(x, y);
        height_update.data[i++] = cost_translation_table_[ height_cost ];
      }
    }
    heightmap_update_pub_.publish(height_update);
	
	
  }

  xn_ = yn_ = 0;
  x0_ = costmap_->getSizeInCellsX();
  y0_ = costmap_->getSizeInCellsY();
}

}  // end namespace costmap_2d
