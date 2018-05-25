//
// Created by chengdaqian on 18-5-23.
//

#ifndef MI_EXPLORER_NODE_CANDIDATE_H
#define MI_EXPLORER_NODE_CANDIDATE_H
namespace mi_explorer
{
    struct NodeCandidate
    {
    public:
        NodeCandidate()
                : mx(0), my(0), acc_x(0.0), acc_y(0.0)
                , mutual_info(0.0), travel_cost(100.0)
        {}

        unsigned int mx, my;
        double wx, wy;
        double acc_x, acc_y;
        double mutual_info;
        double travel_cost;
        double score;
    };
}
#endif //MI_EXPLORER_NODE_CANDIDATE_H
