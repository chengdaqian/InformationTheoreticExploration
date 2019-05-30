//
// Created by dcheng on 19-5-27.
//

#include <mi_explorer/mi_sc_explorer.h>
#include <mi_explorer/mi_computer.h>
#include <mi_explorer/sc_motion_primitive.h>
#include <gf_traj_gen/gf_traj_gen.h>

namespace mi_explorer
{
typedef boost::recursive_mutex mutex_t;

void MISCExplorer::exploreLoop()
{
  while (ros::ok())
  {
    cout << "spin once!" << endl;
    ros::spinOnce();
    
    if (traj_gen_->readyForNext())
    {
      cout << "into loop!" << endl;
      //! 0. prep
      boost::unique_lock<mutex_t> lock(*(prob_map_->map_->getMutex()));
      
      //! 1. sc_mapping and generate candidate trajectories
      std::vector<SCPathCandidate> path_vec;
      
      SCMotionPrimitive mp(odom_, mp_acc_step_,mp_time_step_, mp_max_speed_,
          mp_rho_, glmap_num_r_, glmap_res_d_, mi_beam_length_, prob_map_);
      cout << "init mp obj" << endl;
      if (!mp.getMotionPrimtives(path_vec))
        continue;
      
      //visPaths(path_vec);
      //visNodes(path_vec, path_vec[0]);
      cout << "gen mp" << endl;
      
      //! 2. evaluate candidate trajectories
      MIComputer mi_computer(mi_beam_num_, mi_beam_length_, prob_map_);
      cout << "gen obj" << endl;
      
      mi_computer.evalNodes(path_vec);
      
      cout << "eval nodes" << endl;
      
      
      //! 3. get the path with highest information gain
      int max_info_ind = -1;
      double max_info = 0.0;
      for (int i = 0; i < path_vec.size(); i++)
      {
        if (path_vec[i].mutual_info > max_info)
        {
          max_info = path_vec[i].mutual_info;
          max_info_ind = i;
        }
      }
      if (max_info == 0 || max_info_ind == -1)
      {
        ROS_ERROR("No New Info!");
        break;
      }
  
      cout << "select path" << endl;
  
      visPaths(path_vec);
      visNodes(path_vec, path_vec[max_info_ind]);
      
      
      //! 4. gen traj
      for (int i = 0;
           !traj_gen_->trajGen(path_vec[max_info_ind].waypoints) && i < 3;
           i++)
      {
        ROS_WARN("Failed to generate trajectory! Restarting NOW!");
        continue;
      }
      
      cout << "gen traj" << endl;
      
      //! 5. visualize stuff
      
      cout << "visualize!" << endl;
    }
    
    prob_map_->pubMap();
    ros::Duration(0.1).sleep();
  }
}

MISCExplorer::MISCExplorer(std::string global_frame_name)
: nh_("~")
, private_nh_("~")
, global_frame_name_(global_frame_name)
{
  prob_map_ = boost::shared_ptr<simple_map_2d::ProbMap2DROS>(
      new simple_map_2d::ProbMap2DROS("prob_map", global_frame_name_));
  
  traj_gen_ = boost::shared_ptr<gf_traj_gen::GFTrajGen>(
      new gf_traj_gen::GFTrajGen("traj_gen", global_frame_name_, prob_map_));
  
  odom_sub_ = private_nh_.subscribe("prob_map/odom", 2,
                                    &MISCExplorer::odom_cb, this);
  path_tree_viz_pub_ = private_nh_.advertise<visualization_msgs::Marker>
      ("path_tree_viz",3);
  path_node_viz_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>
      ("path_node_viz",3);
  mi_field_viz_pub_  = private_nh_.advertise<sensor_msgs::PointCloud2>
      ("mi_field_viz" ,3);
  acc_pub_           = private_nh_.advertise<geometry_msgs::Vector3>
      ("acc_cmd",      3);
  
  nh_.param<double>("/ground_range", mi_beam_length_, 5.0);
  
  nh_.param<double>("mp_acc_step",  mp_acc_step_ , 0.08);
  nh_.param<double>("mp_time_step", mp_time_step_, mi_beam_length_/mp_max_speed_);
  nh_.param<double>("mp_max_speed", mp_max_speed_, 1.0);
  nh_.param<double>("mp_rho",       mp_rho_,       1.0);
  
  nh_.param<int>   ("glmap_num_r",  glmap_num_r_, 40);
  nh_.param<double>("glmap_res_d",  glmap_res_d_, 0.4);
  
  nh_.param<int>   ("mi_beam_num_" , mi_beam_num_   , 8 );
  
  cout << "*** Param for sc explorer ***\n"
       << "ground_range: " << mi_beam_length_ << endl
       << "mp_time_step"<< mp_time_step_ << endl
       << "glmap_num_r" <<  glmap_num_r_ << endl
       << "glmap_res_d" <<  glmap_res_d_ << endl
       << "mi_beam_num_" << mi_beam_num_ << endl;
  
  initRosMsgContainer();
}

void MISCExplorer::odom_cb(const nav_msgs::Odometry &odom)
{
  odom_ = odom;
}

void MISCExplorer::visPaths(const std::vector<SCPathCandidate> &path_vec)
{
  path_tree_viz_.points.clear();
  geometry_msgs::Point parent, child;
  
  for (auto it = path_vec.begin(); it != path_vec.end(); it++){
    
    child.x = it->waypoints[0].real();
    child.y = it->waypoints[0].imag();
    
    for (int i = 1; i < it->waypoints.rows(); i++){
      
      parent = child;
      
      child.x = it->waypoints[i].real();
      child.y = it->waypoints[i].imag();
      
      path_tree_viz_.points.push_back(parent);
      path_tree_viz_.points.push_back(child );
    }
  }
  path_tree_viz_pub_.publish(path_tree_viz_);
}

void MISCExplorer::visNodes(const vector<mi_explorer::SCPathCandidate> &path_vec,
    const mi_explorer::SCPathCandidate &selected)
{
  pcl::PointCloud<pcl::PointXYZI> node_cloud_viz;
  pcl::PointXYZI node_point_viz(50);
  for (auto it = path_vec.begin(); it != path_vec.end(); it++){
    node_point_viz.x = (float)it->wx;
    node_point_viz.y = (float)it->wy;
    node_cloud_viz.push_back(node_point_viz);
  }
  
  node_point_viz.x = (float)selected.wx;
  node_point_viz.y = (float)selected.wy;
  node_point_viz.intensity = 80;
  node_cloud_viz.push_back(node_point_viz);
  
  sensor_msgs::PointCloud2 node_cloud_viz_output;
  pcl::toROSMsg(node_cloud_viz, node_cloud_viz_output);
  node_cloud_viz_output.header.frame_id = global_frame_name_;
  node_cloud_viz_output.header.stamp = ros::Time::now();
  path_node_viz_pub_.publish(node_cloud_viz_output);
}

void MISCExplorer::initRosMsgContainer()
{
  path_tree_viz_.header.frame_id = global_frame_name_;
  path_tree_viz_.action = visualization_msgs::Marker::ADD;
  path_tree_viz_.pose.orientation.w = 1.0;
  path_tree_viz_.id = 0;
  path_tree_viz_.type = visualization_msgs::Marker::LINE_LIST;
  path_tree_viz_.scale.x = 0.02;
  path_tree_viz_.color.r = path_tree_viz_.color.a = 1.0;
}
}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "mi_sc_explorer");
  
  mi_explorer::MISCExplorer explorer("/odom");
  
  explorer.exploreLoop();
}