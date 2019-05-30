//
// Created by dcheng on 19-5-27.
//

#ifndef MI_EXPLORER_SC_MOTION_PRIMITIVE_H
#define MI_EXPLORER_SC_MOTION_PRIMITIVE_H

#include <ros/ros.h>
#include <mi_explorer/node_candidate.h>
#include <simple_map_2d/prob_map_2d_ros.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <sc_mapping/gearlike_map.h>

namespace mi_explorer
{
class SCMotionPrimitive
{
  static const int STEP_NUM = 5;
public:
  SCMotionPrimitive(const nav_msgs::Odometry &odom, double acc_step,
                    double time_step, double max_speed, double rho,
                    int gear_num_r, double gear_res_d, double range,
                    boost::shared_ptr<simple_map_2d::ProbMap2DROS> prob_map);
  
  bool getMotionPrimtives(std::vector<SCPathCandidate> &path_vec);
  
private:
  /**
   *
   * @return 0 if the region is a disk and no need for gearlike mapping.
   * -1 if gearlike solve failed, and need to replan
   * 1 if gearlike solve succeeded, and proceed to path gen
   */
  int genGearLike();
  
  bool rangeToGearlike(Eigen::VectorXd range, Eigen::VectorXd &rad,
                       Eigen::VectorXd &arg);
  
  void genNaiveMP(double wx0, double wy0, double vx0, double vy0,
                  double acc_step, double time_step, double max_speed,
                  std::vector<mi_explorer::SCPathCandidate> &path_vec,
                  bool is_disk = false);
  
  int gear_num_r_;
  double gear_res_d_;
  
  boost::shared_ptr<simple_map_2d::ProbMap2DROS> prob_map_;
  double acc_step_, time_step_, max_speed_, rho_;
  double max_range_, range_;
  double nm_acc_step_, nm_max_speed_;
  
  double trans_R_;
  
  const nav_msgs::Odometry &odom_;
  
  boost::shared_ptr<sc_mapping::GearLikeMap> gearlike_map_;
};
}

#endif //MI_EXPLORER_SC_MOTION_PRIMITIVE_H