//
// Created by chengdaqian on 18-5-23.
//

#include <mi_explorer/mi_computer.h>
#include <simple_map_2d/prob_map_2d_ros.h>
#include <cmath>
#include <mi_explorer/const_values.h>

#include <sensor_msgs/PointCloud2.h>


namespace mi_explorer
{
using std::cout;
using std::endl;
    MIComputer::MIComputer(unsigned int mi_beam_num, double mi_beam_length,
                           boost::shared_ptr<simple_map_2d::ProbMap2DROS> prob_map)
    : beam_num_(mi_beam_num)
    , beam_length_(mi_beam_length)
    , prob_map_(prob_map)
    {
    }

    void MIComputer::evalNodes(std::list<NodeCandidate> &node_list) {


        int size_x = prob_map_->map_->getSizeInCellsX(), size_y = prob_map_->map_->getSizeInCellsY();
        double resolution = prob_map_->map_->getResolution();

        for (auto it = node_list.begin(); it != node_list.end(); it++){

            for (unsigned int beam_idx = 0; beam_idx < beam_num_; beam_idx ++){

                int end_x = (int)it->mx + (int)ceil(beam_length_ / resolution * cos(beam_idx * 2 * PI / beam_num_));
                int end_y = (int)it->my + (int)ceil(beam_length_ / resolution * sin(beam_idx * 2 * PI / beam_num_));

                // if beam end coords out of bound, segment it
                if (end_x < 0){
                    double truncate_scale = ((double)it->mx) / (it->mx - end_x);
                    end_y = (int)it->my + int((end_y - (int)it->my) * truncate_scale);
                    end_x = 0;
                }
                if (end_x >= size_x){
                    end_y = (int)it->my + (size_x - 1 - (int)it->mx) * (end_y - (int)it->my) / (end_x - (int)it->mx);
                    end_x = size_x - 1;
                }

                if (end_y < 0){
                    end_x = (int)it->mx - ((int)it->mx - end_x) * (int)it->my / ((int)it->my - end_y);
                    end_y = 0;
                }
                if (end_y >= size_y){
                    end_x = (int)it->mx + (size_y - 1 - (int)it->my) * (end_x - (int)it->mx) / (end_y - (int)it->my);
                    end_y = size_y - 1;
                }
                if (end_x < 0 || end_x >= size_x || end_y < 0 || end_y >= size_y)
                    ROS_ERROR("***** Still have end coords out of bound! ******");

                it->mutual_info += evalSingleBeam(it->mx, it->my, (unsigned int)end_x, (unsigned int)end_y);
            }
        }

    }

void MIComputer::evalNodes(std::vector<SCPathCandidate> &path_vec) {
  int size_x = prob_map_->map_->getSizeInCellsX();
  int size_y = prob_map_->map_->getSizeInCellsY();
  double resolution = prob_map_->map_->getResolution();
  
  //cout << "res: " << resolution << ", sizex: " << size_x << ", sizey: " << size_y << endl;
  //cout << "beamnum: " << beam_num_ << endl;
  
  for (auto it = path_vec.begin(); it != path_vec.end(); it++){
    //cout << "mx: " << (int)it->mx << " (" << it->mx << endl;
    //cout << "my: " << (int)it->mx << " (" << it->my << endl;
    
    for (int beam_idx = 0; beam_idx < beam_num_; beam_idx ++){
      //cout << "start-";
      int end_x = (int)it->mx + (int)ceil(beam_length_
          * cos(2*M_PI * beam_idx / beam_num_) / resolution);
      int end_y = (int)it->my + (int)ceil(beam_length_
          * sin(2*M_PI * beam_idx / beam_num_) / resolution );
      
      //cout << "endx: " << end_x << ", endy: " << end_y << endl;
      
      //cout << "flag 0" << endl;
      // if beam end coords out of bound, segment it
      if (end_x < 0){
        //cout << "1";
          double truncate_scale = ((double)it->mx) / (it->mx - end_x);
          end_y = (int)it->my + int((end_y - (int)it->my) * truncate_scale);
          end_x = 0;
        //cout << "2";
      }
      //cout << "flag 1" << endl;
      if (end_x >= size_x){
        //cout << "3";
          end_y = (int)it->my + (size_x - 1 - (int)it->mx) * (end_y - (int)it->my) / (end_x - (int)it->mx);
          end_x = size_x - 1;
        //cout << "4";
      }
      //cout << "flag 2" << endl;
      if (end_y < 0){
        //cout << "5";
          end_x = (int)it->mx - ((int)it->mx - end_x) * (int)it->my / ((int)it->my - end_y);
          end_y = 0;
        //cout << "6";
      }
      //cout << "flag 3" << endl;
      if (end_y >= size_y){
        //cout << "7";
          end_x = (int)it->mx + (size_y - 1 - (int)it->my) * (end_x - (int)it->mx) / (end_y - (int)it->my);
          end_y = size_y - 1;
        //cout << "8";
      }
      //cout << "flag 4" << endl;
      if (end_x < 0 || end_x >= size_x || end_y < 0 || end_y >= size_y)
          ROS_ERROR("***** Still have end coords out of bound! ******");
      //cout << "flag 5" << endl;
      
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