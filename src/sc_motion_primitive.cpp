//
// Created by dcheng on 19-5-27.
//

#include <mi_explorer/sc_motion_primitive.h>
#include <cmath>
#include <igl/slice_mask.h>
#include <ctime>

#define MAX_RAD_RATIO 3.0

namespace mi_explorer
{
using Eigen::VectorXd;
bool SCMotionPrimitive::rangeToGearlike(Eigen::VectorXd range,
    Eigen::VectorXd &rad, Eigen::VectorXd &arg)
{
  std::vector<double> vrad, varg;
  
  int n = range.rows();
  
  double min_rad = range.minCoeff();
  double max_rad = min_rad * MAX_RAD_RATIO;
  max_rad = std::max(max_rad, 2.0);
  range = (range.array() > max_rad).select(
      VectorXd::Constant(n, max_rad), range);
  
  cout << "range: \n" << range << endl;
  
  int ind1 = 0;
  while ((range(ind1) - range((ind1 + 1)%n) < gear_res_d_)
         || (range(ind1) - range((ind1+1)%n) > gear_res_d_
             && range(ind1) - range((ind1-1+n) % n) > gear_res_d_ )
         || (range((ind1+1)%n) - range(ind1) > gear_res_d_
             && range((ind1+1)%n) - range((ind1 + 2)%n) > gear_res_d_))
  {
    ind1++;
    if (ind1 == n)
    {
      max_range_ = range.maxCoeff();
      cout << "perfect circular!" << endl;
      return false;
    }
  }
  
  double arg_holder;
  
  if (range(ind1+1) > range(ind1))
    arg_holder = (ind1 + 1) * 2.0 / n;
  else
    arg_holder = (ind1) * 2.0 / n;
  
  Eigen::VectorXd range_cpy(range);
  range << range_cpy.tail(n-ind1-1), range_cpy.head(ind1+1);
  
  double rad_holder = range(0);
  double short_holder = range(0);
  
  
  for (int i = 1; i < n; i++)
  {
    double tmp_arg = 2.0 * ((i+1+ind1) % n) / n;//todo maybe have bug
    double tmp_rad = range(i);
    
    if (tmp_rad - rad_holder > gear_res_d_)
    {
      if (tmp_rad - range((i+1) % n) > gear_res_d_)
        continue;
      else
      {
        vrad.push_back(short_holder);
        vrad.push_back(short_holder);
        varg.push_back(arg_holder);
        varg.push_back(tmp_arg);
        
        rad_holder = tmp_rad;
        short_holder = tmp_rad;
        arg_holder = tmp_arg;
      }
    }
    else if (tmp_rad - rad_holder < -gear_res_d_)
    {
      vrad.push_back(short_holder);
      vrad.push_back(short_holder);
      varg.push_back(arg_holder);
      varg.push_back(tmp_arg - 2.0 / n);
      
      rad_holder = tmp_rad;
      short_holder = tmp_rad;
      arg_holder = tmp_arg - 2.0 / n;
    }
    else
    {
      short_holder = std::min(short_holder, tmp_rad);
    }
    
    if (i == n-1)// last element
    {
      vrad.push_back(short_holder);
      vrad.push_back(short_holder);
      varg.push_back(arg_holder);
      varg.push_back(varg[0]);
    }
  }
  
  int n2 = vrad.size();
  
  for (int i = 0; i < vrad.size(); i += 2)
  {
    if (vrad[i] > vrad[(i-2+n2)%n2] && vrad[i] > vrad[(i+2)%n2])
    {
      double higher = std::max(vrad[(i-2+n2)%n2], vrad[(i+2)%n2]);
      double arg_diff = varg[i+1] - varg[i];
      if (arg_diff < 0)
        arg_diff += 2.0;
      double max_rad = (arg_diff) * M_PI * higher + higher;
      if (vrad[i] > max_rad)
      {
        vrad[i]   = max_rad;
        vrad[i+1] = max_rad;
      }
    }
  }
  
  rad = VectorXd(vrad.size());
  arg = VectorXd(varg.size());
  for (int i = 0; i < rad.rows(); i++)
  {
    rad[i] = vrad[i];
    arg[i] = varg[i];
  }
  max_range_ = rad.maxCoeff();
  return true;
}

SCMotionPrimitive::SCMotionPrimitive(const nav_msgs::Odometry &odom,
    double acc_step, double time_step, double max_speed, double rho,
    int gear_num_r, double gear_res_d, double range,
    boost::shared_ptr<simple_map_2d::ProbMap2DROS> prob_map)
: acc_step_(acc_step)
, time_step_(time_step)
, max_speed_(max_speed)
, rho_(rho)
, gear_num_r_(gear_num_r)
, gear_res_d_(gear_res_d)
, range_(range)
, prob_map_(prob_map)
, odom_(odom)
{

}

bool SCMotionPrimitive::getMotionPrimtives(
    std::vector<mi_explorer::SCPathCandidate> &path_vec)
{
  //! 1. if no need for gearlike_mapping, then just fall back to naive MtnPrmtv
  int gear_code = genGearLike();
  if (gear_code == 0)
  {
    double nm_factor = 1.0 / max_range_ / 1.414;
    genNaiveMP(odom_.pose.pose.position.x, odom_.pose.pose.position.y,
               odom_.twist.twist.linear.x, odom_.twist.twist.linear.y,
               acc_step_ * nm_factor, time_step_ * nm_factor, max_speed_, path_vec);
    return true;
  }
  if (gear_code == -1)
    return false;
  
  // 2. generate normalized motion primitive on unit disk
  assert(max_range_ > 0 && "max_range is not properly initialized!");
  double nm_factor = 1.0 / max_range_ / 1.414;
  double nm_vx0 = odom_.twist.twist.linear.x * nm_factor;
  double nm_vy0 = odom_.twist.twist.linear.y * nm_factor;
  double nm_vx0_disk = nm_vx0 * cos(trans_R_) - nm_vy0 * sin(trans_R_);
  double nm_vy0_disk = nm_vx0 * sin(trans_R_) + nm_vy0 * cos(trans_R_);
  nm_acc_step_  = acc_step_ * nm_factor;
  nm_max_speed_ = max_speed_ * nm_factor;
  
  cout << "before gen naive mp" << endl;
  
  genNaiveMP(0, 0, nm_vx0_disk, nm_vy0_disk, nm_acc_step_, time_step_,
             nm_max_speed_, path_vec, true);
  
  cout << "after gen naive mp" << endl;
  
  // 3. map back to gearlike region, then translate to world frame, assign mx my
  for (int i = 0; i < path_vec.size(); i++)
  {
    cout << "*****************************\n";
    cout << i << "th traj:\n disk waypoints\n" << path_vec[i].waypoints << endl;
    Eigen::VectorXcd gear_path;
    gearlike_map_->glmap(path_vec[i].waypoints, gear_path);
    Eigen::VectorXcd world_path = gear_path.array()
        + std::complex<double>(odom_.pose.pose.position.x,
                               odom_.pose.pose.position.y);
    path_vec[i].waypoints = world_path;
    
    cout << "*** world waypoints *** \n" << path_vec[i].waypoints << endl;
    
    int wp_len = path_vec[i].waypoints.rows();
    path_vec[i].wx = path_vec[i].waypoints[wp_len - 1].real();
    path_vec[i].wy = path_vec[i].waypoints[wp_len - 1].imag();
    
    prob_map_->map_->worldToMap(path_vec[i].wx, path_vec[i].wy,
                                path_vec[i].mx, path_vec[i].my);
    
    cout << "wx: " << path_vec[i].wx << ", wy: " << path_vec[i].wy << endl;
    cout << "mx: " << path_vec[i].mx << ", my: " << path_vec[i].my << endl;
  }
  return true;
}

int SCMotionPrimitive::genGearLike()
{
  Eigen::VectorXd beam_range(gear_num_r_);
  
  double resolution = prob_map_->map_->getResolution();
  unsigned int mx, my;
  prob_map_->map_->worldToMap(odom_.pose.pose.position.x,
                              odom_.pose.pose.position.y,
                              mx, my);
  int size_x = prob_map_->map_->getSizeInCellsX();
  int size_y = prob_map_->map_->getSizeInCellsY();
  
  cout << "*** start gear like region generation ***" << endl;
  /**
   * Start generating a fucking gearlike region
   */
  //! 1. get mo-fu*king beam ranges!
  for (int i = 0; i < gear_num_r_; i++)
  {
    double angle = 2.0 * i * M_PI / gear_num_r_;
    int end_x = (int)mx + (int)ceil(range_ / resolution * cos(angle));
    int end_y = (int)my + (int)ceil(range_ / resolution * sin(angle));
  
    // if beam end coords out of bound, segment it
    if (end_x < 0){
      double truncate_scale = ((double)mx) / (mx - end_x);
      end_y = (int)my + int((end_y - (int)my) * truncate_scale);
      end_x = 0;
    }
    if (end_x >= size_x){
      end_y = (int)my + (size_x - 1 - (int)mx) * (end_y - (int)my) / (end_x - (int)mx);
      end_x = size_x - 1;
    }
  
    if (end_y < 0){
      end_x = (int)mx - ((int)mx - end_x) * (int)my / ((int)my - end_y);
      end_y = 0;
    }
    if (end_y >= size_y){
      end_x = (int)mx + (size_y - 1 - (int)my) * (end_x - (int)mx) / (end_y - (int)my);
      end_y = size_y - 1;
    }
    if (end_x < 0 || end_x >= size_x || end_y < 0 || end_y >= size_y)
      ROS_ERROR("***** Still have end coords out of bound! ******");
    
    
    std::vector<unsigned int> cell_vec;
    prob_map_->map_->raytraceLine(mx, my, end_x, end_y, cell_vec);
    
    auto it = cell_vec.begin();
    for (; it != cell_vec.end(); it++)
    {
      if (prob_map_->map_->getOddsRatio(*it) > 0.5)
        break;
    }
    unsigned int beam_end_mx, beam_end_my;
    prob_map_->map_->indexToCells(*(it-1), beam_end_mx, beam_end_my);
    double tmp_rad = (angle > M_PI / 4 && angle < 3 * M_PI / 4)
                  || (angle > 5 * M_PI / 4 && angle < 7 * M_PI / 4)
        ? abs((int)beam_end_my - (int)my) * resolution / fabs(sin(angle))
        : abs((int)beam_end_mx - (int)mx) * resolution / fabs(cos(angle));
    
    beam_range(i) = tmp_rad;
  }
  
  Eigen::VectorXd rad, arg;
  if (!rangeToGearlike(beam_range, rad, arg))
  {
    return 0;
  }
  
  trans_R_ = -arg[arg.size() - 1] * M_PI;
  
  cout << "flag 5 " << endl;
  
  cout << "final rad: \n" << rad << endl;
  cout << "final arg: \n" << arg << endl;
  
  gearlike_map_ = boost::shared_ptr<sc_mapping::GearLikeMap>(
      new sc_mapping::GearLikeMap(rad, arg));
  
  if (gearlike_map_->term_code_ != 1){
    cout << "Gearlike Map Termcode: " << gearlike_map_->term_code_ << endl;
    return -1;
    
  }
  
  
  cout << "6 " << endl;
  
  return 1;
}

void SCMotionPrimitive::genNaiveMP(
    double wx0, double wy0, double vx0, double vy0,
    double acc_step, double time_step, double max_speed,
    std::vector<mi_explorer::SCPathCandidate> &path_vec, bool is_disk)
{
  cout << "******************************\nstart gen naive on disk\n";
  cout << "vx0: " << vx0 << ", vy0: " << vy0 << ", acc_step: " << acc_step
       << ", timestep: " << time_step << ",max_speed: " <<max_speed << endl;
  path_vec.clear();
  
  double acc_x, acc_y;
  
  acc_x = -(STEP_NUM - 1) / 2 * acc_step;
  for (unsigned int ix = 0; ix < STEP_NUM; ix++, acc_x += acc_step){
    
    acc_y = -(STEP_NUM - 1) / 2 * acc_step;
    for (unsigned int iy = 0; iy < STEP_NUM; iy++, acc_y += acc_step) {
      
      // abandon node if end speed exceeds max speed
      double end_vx = vx0 + acc_x * time_step, end_vy = vy0 + acc_y * time_step;
      
      if (sqrt(end_vx * end_vx + end_vy * end_vy) > max_speed)
        continue;
      
      double end_wx = wx0 + vx0 * time_step + 0.5*acc_x * time_step * time_step;
      double end_wy = wy0 + vy0 * time_step + 0.5*acc_y * time_step * time_step;
      
      if (is_disk && sqrt(end_wx*end_wx + end_wy*end_wy) >= 1.0)
        continue;
      
      double max_vx = std::max(fabs(end_vx), fabs(vx0));
      double max_vy = std::max(fabs(end_vy), fabs(vy0));
      //double max_vy = acc_y > 0 ? end_vy : vy0;
      double max_v = std::max(max_vx, max_vy);
      
      if (max_v < 1e-4) // max_v == 0
        continue;
  
      double waypoint_timestep = prob_map_->map_->getResolution() / max_v;
      
      SCPathCandidate path;
      path.wx = end_wx;
      path.wy = end_wy;
      path.mutual_info = 0.0;
      path.travel_cost = ((acc_x * acc_x + acc_y * acc_y) + rho_) * time_step;
      prob_map_->map_->worldToMap(end_wx, end_wy, path.mx, path.my);
      
      path.waypoints = VectorXcd(int(time_step / waypoint_timestep) + 1);
      
      cout << "*** traj info:" << endl;
      cout << "acc x: " << acc_x << ", acc y: " << acc_y << endl;
      cout << "wx: " << end_wx << ", wy: " << end_wy << endl;
      cout << "maxv: " << max_v << ", waypoints size: " << path.waypoints.rows() << endl;
      
      for (int i = 0; i < path.waypoints.rows(); i++)
      {
        double time = i * waypoint_timestep;
        double iter_wx = wx0 + vx0 * time + 0.5 * acc_x * time * time;
        double iter_wy = wy0 + vy0 * time + 0.5 * acc_y * time * time;
        path.waypoints[i] = std::complex<double>(iter_wx, iter_wy);
      }
      
      path_vec.push_back(path);
    }
  }
}

}