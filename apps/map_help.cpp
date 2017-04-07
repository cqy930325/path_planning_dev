#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <slam/map/occupancy_grid_map/occupancy_grid_map.h>
#include <slam/map/occupancy_grid_map/syn_ogm.h>
#include <iostream>
#include <queue>
#include <set>
#include <map>
#include <glog/logging.h>

void binarize(std::shared_ptr<PI::slam::ogm::OccupancyGridMap> ogm, const double threshold){
    CHECK(threshold >= 0 && threshold <= 1) << "invalid threshold";
    for(size_t i = 0; i < ogm->grid_size().width;i++) {
        for (size_t j = 0; j < ogm->grid_size().height; j++) {
            double val;
            ogm->GetValue(i, j, &val);
            val = val < threshold ? 0 : 1;
            ogm->SetValue(i, j, val);
        }
    }
}

PI::slam::ogm::OccupancyGridMap upScale(std::shared_ptr<PI::slam::ogm::OccupancyGridMap> ogm, const int factor){
    CHECK_EQ(ogm->grid_size().width % factor, 0) << "width should be divided by factor exactly!";
    CHECK_EQ(ogm->grid_size().height % factor, 0) << "height should be divided by factor exactly!";
    cv::Size grid_size(ogm->grid_size().width/factor,ogm->grid_size().height/factor);
    PI::slam::ogm::OccupancyGridMap scale_ogm(ogm->cell_size()*factor,grid_size);
    for(size_t i = 0; i < scale_ogm.grid_size().width;i++){
        for(size_t j = 0; j < scale_ogm.grid_size().height;j++){
            double val = 1;
            for(size_t k = i*factor; k < i*factor+factor; k++){
                for(size_t l = j*factor;l < j*factor+factor;l++){
                    double tmp_val;
                    ogm->GetValue(k,l,&tmp_val);
                    val *= tmp_val;
                }
            }
            val = val == 0 ? 0 : 1;
            scale_ogm.SetValue(i,j,val);
        }
    }
    return scale_ogm;
}


int main(int argc, char* argv[]) {
    std::default_random_engine rnd(12);
    std::shared_ptr<PI::slam::ogm::OccupancyGridMap> ogm = PI::slam::ogm::CreateRandomOgm(rnd);
    cv::Mat map = ogm->Visualize();
    std::cout << ogm->grid_size() << std::endl;
    std::cout << ogm->map_size() << std::endl;
    std::cout << ogm->cell_size() << std::endl;
    cv::Mat dst;
    binarize(ogm,0.3);
    //PI::slam::ogm::OccupancyGridMap sogm = upScale(ogm,4);
    //std::cout << sogm.grid_size() << std::endl;
    //std::cout << sogm.map_size() << std::endl;
    //std::cout << sogm.cell_size() << std::endl;
    while (1) {
        cv::imshow("map", ogm->Visualize());
        int key = cv::waitKey(5);
        if (key == 27)
            break;
        continue;
    }
    return 0;
}