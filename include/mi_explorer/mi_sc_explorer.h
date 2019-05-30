//
// Created by dcheng on 19-5-27.
//

#ifndef MI_EXPLORER_MI_SC_EXPLORER_H
#define MI_EXPLORER_MI_SC_EXPLORER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

#include <sc_mapping/gearlike_map.h>
#include <simple_map_2d/prob_map_2d_ros.h>
#include <gf_traj_gen/gf_traj_gen.h>

#include <mi_explorer/node_candidate.h>

namespace mi_explorer
{
class MISCExplorer
{
public:
  explicit MISCExplorer(std::string global_frame_name);
  
  void exploreLoop();
  
  double update_duration_;
  
private:
  void odom_cb(const nav_msgs::Odometry &odom);
  
  void initRosMsgContainer();
  
  void visPaths(const std::vector<SCPathCandidate> &path_vec);
  
  void visNodes(const std::vector<SCPathCandidate> &path_vec,
                const SCPathCandidate &selected);
  
  //ROS-related
  ros::NodeHandle nh_, private_nh_;
  ros::Subscriber odom_sub_;
  ros::Publisher path_tree_viz_pub_, path_node_viz_pub_, mi_field_viz_pub_;
  ros::Publisher acc_pub_;
  ros::Publisher beam_end_viz_;
  
  // system params
  std::string global_frame_name_;
  
  // algorithm params
  double mp_acc_step_, mp_time_step_, mp_rho_, mp_max_speed_;
  int glmap_num_r_;
  double glmap_res_d_;
  int mi_beam_num_;
  double mi_beam_length_;
  
  // data containers
  nav_msgs::Odometry odom_;
  visualization_msgs::Marker path_tree_viz_;
  
  // map pointer
public:
  boost::shared_ptr<simple_map_2d::ProbMap2DROS> prob_map_;
  // traj generator
  boost::shared_ptr<gf_traj_gen::GFTrajGen> traj_gen_;
  
};
}

#endif //MI_EXPLORER_MI_SC_EXPLORER_H
