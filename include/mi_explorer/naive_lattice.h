//
// Created by chengdaqian on 18-5-23.
//

#ifndef MI_EXPLORER_NAIVE_LATTICE_H
#define MI_EXPLORER_NAIVE_LATTICE_H

#include <ros/ros.h>
#include <mi_explorer/node_candidate.h>
#include <simple_map_2d/prob_map_2d_ros.h>
#include <nav_msgs/Odometry.h>

namespace mi_explorer
{
    class NaiveLattice
    {
        static const int STEP_NUM = 5;
    public:
        NaiveLattice(nav_msgs::Odometry odom, double lattice_acc_step, double lattice_time_step, double lattice_max_speed, double lattice_rho, boost::shared_ptr<simple_map_2d::ProbMap2DROS> prob_map);

        void getMotionPrimitives(std::list<NodeCandidate> &node_list);

    private:
        boost::shared_ptr<simple_map_2d::ProbMap2DROS> prob_map_;

        double acc_step_, time_step_, max_speed_, rho_;

        nav_msgs::Odometry odom_;
    };
}

#endif //MI_EXPLORER_NAIVE_LATTICE_H
