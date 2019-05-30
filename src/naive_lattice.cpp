//
// Created by chengdaqian on 18-5-23.
//

#include <mi_explorer/naive_lattice.h>
#include <cmath>

namespace mi_explorer
{
NaiveLattice::NaiveLattice(const nav_msgs::Odometry &odom,
    double lattice_acc_step, double lattice_time_step, double lattice_max_speed,
    double lattice_rho, boost::shared_ptr<simple_map_2d::ProbMap2DROS> prob_map)
: prob_map_(prob_map)
, acc_step_(lattice_acc_step)
, time_step_(lattice_time_step)
, max_speed_(lattice_max_speed)
, rho_(lattice_rho)
, odom_(odom)
{

}

void NaiveLattice::getMotionPrimitives(std::list<NodeCandidate> &node_list) {

  node_list.clear();

  double vx0 = odom_.twist.twist.linear.x;
  double vy0 = odom_.twist.twist.linear.y;
  double wx0 = odom_.pose.pose.position.x;
  double wy0 = odom_.pose.pose.position.y;

  double acc_x, acc_y;

  acc_x = -(STEP_NUM - 1) / 2 * acc_step_;
  for (unsigned int ix = 0; ix < STEP_NUM; ix++, acc_x += acc_step_){

    acc_y = -(STEP_NUM - 1) / 2 * acc_step_;
    for (unsigned int iy = 0; iy < STEP_NUM; iy++, acc_y += acc_step_) {

      // abandon node if end speed exceeds max speed
      // TODO: controller should also limit speed
      double end_vx = vx0 + acc_x * time_step_, end_vy = vy0 + acc_y * time_step_;

      if (sqrt(end_vx * end_vx + end_vy * end_vy) > max_speed_)
        continue;

      double end_wx = wx0 + vx0 * time_step_ + 0.5 * acc_x * time_step_ * time_step_;
      double end_wy = wy0 + vy0 * time_step_ + 0.5 * acc_y * time_step_ * time_step_;

      NodeCandidate node;
      node.acc_x = acc_x;
      node.acc_y = acc_y;
      node.wx    = end_wx;
      node.wy    = end_wy;
      node.mutual_info = 0.0;
      node.travel_cost = ((acc_x * acc_x + acc_y * acc_y) + rho_) * time_step_;
      prob_map_->map_->worldToMap(end_wx, end_wy, node.mx, node.my);

      // abandon node if not free, or outside bound
      if (prob_map_->map_->getOddsRatio(node.mx, node.my) >= 1.0
        || !prob_map_->checkInBound(node.wx, node.wy)){
        continue;
      }

      node_list.push_back(node);

    }
  }

}
}