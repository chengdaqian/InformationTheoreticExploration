//
// Created by chengdaqian on 18-5-23.
//

#ifndef MI_EXPLORER_MI_NAIVE_EXPLORER_H
#define MI_EXPLORER_MI_NAIVE_EXPLORER_H

#include <ros/ros.h>
#include <simple_map_2d/prob_map_2d_ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <mi_explorer/node_candidate.h>

namespace mi_explorer
{
    class MINaiveExplorer
    {
    public:
        MINaiveExplorer(std::string global_frame_name);

        void main_explore_loop();

        /**
         * compute mutual information for every free cell and visualize them
         */
        void test_explore_loop();

        double update_duration_;

    private:
        void odom_cb(const nav_msgs::Odometry &odom);

        void initRosMsgContainer();

        bool selectCandidate(std::list<NodeCandidate> &node_list, NodeCandidate &selected);

        bool checkAccCollisionFree(double acc_x, double acc_y);

        void visualizePaths(const std::list<NodeCandidate> &node_list);

        void visualizeNodes(const std::list<NodeCandidate> &node_list, const NodeCandidate &selected);

        void pubAccCmd(bool selected_valid, const NodeCandidate &selected_node);

    private:
        // ROS-related
        ros::NodeHandle nh_, private_nh_;
        ros::Subscriber odom_sub_;
        ros::Publisher  path_tree_viz_pub_, path_node_viz_pub_, mi_field_viz_pub_, acc_pub_;
        ros::Publisher  beam_end_viz_;

        // system_params
        std::string global_frame_name_;

        // Algorithm params
        double lattice_acc_step_, lattice_time_step_, lattice_max_speed_, lattice_rho_;
        int mi_beam_num_;
        double mi_beam_length_;

        // data containers
        nav_msgs::Odometry odom_;
        visualization_msgs::Marker path_tree_viz_;

        // map pointer
        boost::shared_ptr<simple_map_2d::ProbMap2DROS> prob_map_;
    };
}

#endif //MI_EXPLORER_MI_NAIVE_EXPLORER_H
