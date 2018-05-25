//
// Created by chengdaqian on 18-5-23.
//

#include <mi_explorer/mi_computer.h>
#include <simple_map_2d/prob_map_2d_ros.h>
#include <cmath>
#include <mi_explorer/const_values.h>

namespace mi_explorer
{
    MIComputer::MIComputer(unsigned int mi_beam_num, double mi_beam_length,
                           boost::shared_ptr<simple_map_2d::ProbMap2DROS> prob_map)
    : beam_num_(mi_beam_num)
    , beam_length_(mi_beam_length)
    , prob_map_(prob_map)
    {

    }

    void MIComputer::evalNodes(std::list<NodeCandidate> &node_list) {
        //ROS_INFO_STREAM("Single Beam MI:" << evalSingleBeam(40, 80, 40, 110));
        for (auto it = node_list.begin(); it != node_list.end(); it++){

            for (unsigned int beam_idx = 0; beam_idx < beam_num_; beam_idx ++){

                int end_x = it->mx + prob_map_->map_->cellDistance(beam_length_ * cos(beam_idx * 2 * PI / beam_num_));
                int end_y = it->my + prob_map_->map_->cellDistance(beam_length_ * sin(beam_idx * 2 * PI / beam_num_));

                // if beam end coords out of bound, segment it
                if (end_x < 0){
                    end_y *= it->mx / (it->mx - end_x);
                    end_x = 0;
                }
                if (end_x > prob_map_->map_->getSizeInCellsX()){
                    end_y *= (prob_map_->map_->getSizeInCellsX() - it->mx) / (end_x - it->mx);
                    end_x = prob_map_->map_->getSizeInCellsX() - 1;
                }
                if (end_y < 0){
                    end_x *= it->my / (it->my - end_y);
                    end_y = 0;
                }
                if (end_y > prob_map_->map_->getSizeInCellsY()){
                    end_x *= (prob_map_->map_->getSizeInCellsY() - it->my) / (end_x - it->my);
                    end_y = prob_map_->map_->getSizeInCellsY() - 1;
                }

                it->mutual_info += evalSingleBeam(it->mx, it->my, (unsigned int)end_x, (unsigned int)end_y);
            }
        }
    }

    double MIComputer::evalSingleBeam(unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1)
    {
        // get beam cells
        std::list<unsigned int> beam_cell_list;
        prob_map_->map_->raytraceLine(x0,y0,x1,y1,beam_cell_list);

        double mi_beam = 0.0;

        // get each cell's current odds ratio and probability of intersect
        std::vector<double> prob_intersect_vec(beam_cell_list.size());
        std::vector<double> odds_ratio_vec(beam_cell_list.size());
        auto it = beam_cell_list.begin();

        double prob_accumulate_intersect = 0.0;
        for (unsigned int i = 0; i < beam_cell_list.size(); i++, it++){

            odds_ratio_vec[i] = prob_map_->map_->getOddsRatio(*it);

            double pi_occ = prob_map_->map_->getProbValue(*it);
            double pi_measure = (1 - prob_accumulate_intersect) * pi_occ;
            prob_accumulate_intersect += pi_measure;
            prob_intersect_vec[i] = pi_measure;
        }

        // compute mi for each cell in the beam
        for (unsigned int i = 0; i < beam_cell_list.size(); i++){

            double mi_cell_beam = 0.0;

            // now I perform integral, or summation in this discretized case.
            // since for k < i, f = 0, they are ignored
            // for k = i:
            mi_cell_beam += prob_intersect_vec[i]
                            * (log((odds_ratio_vec[i] + 1) / (odds_ratio_vec[i] + 1.0/R_OCC))
                               - log(R_OCC) / (odds_ratio_vec[i] * R_OCC + 1));

            // for k > i:
            for (unsigned int k = i + 1; k < beam_cell_list.size(); k++){
                mi_cell_beam += prob_intersect_vec[k]
                                * (log((odds_ratio_vec[i] + 1) / (odds_ratio_vec[i] + 1.0/R_EMP))
                                   - log(R_EMP) / (odds_ratio_vec[i] * R_EMP + 1));
            }

            mi_beam += mi_cell_beam;
        }

        return mi_beam;
    }
}