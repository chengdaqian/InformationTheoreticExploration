//
// Created by chengdaqian on 18-5-23.
//

#ifndef MI_EXPLORER_MI_COMPUTER_H
#define MI_EXPLORER_MI_COMPUTER_H

#include <mi_explorer/node_candidate.h>
#include <simple_map_2d/prob_map_2d_ros.h>

namespace mi_explorer
{
    class MIComputer
    {
    public:
        MIComputer(unsigned int mi_beam_num, double mi_beam_length, boost::shared_ptr<simple_map_2d::ProbMap2DROS> prob_map);

        void evalNodes(std::list<NodeCandidate> &node_list);

    private:
        double evalSingleBeam(unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1);


    private:
        unsigned int beam_num_;
        double beam_length_;
        boost::shared_ptr<simple_map_2d::ProbMap2DROS> prob_map_;
    };
}

#endif //MI_EXPLORER_MI_COMPUTER_H
