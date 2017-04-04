#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/cvstd.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

cv::Mat generateMap(int x, int y){
    cv::Mat map = cv::Mat(x,y,CV_8U,255);
    for(int i = 30; i < 70;i++)
        for(int j = 30;j < 90;j++)
            map.at<uchar>(j,i) = 0;
    for(int i = 140; i < 170;i++)
        for(int j = 100;j < 190;j++)
            map.at<uchar>(j,i) = 0;
    int startx = 100;
    int starty = 100;
    int level = 30;
    for(int i = 0; i < level; i++){
        for(int j = startx - i; j <= startx + i;j++){
            map.at<uchar>(starty+i,j) = 0;
        }
    }
    return map;
}
void bfs(cv::Mat *pmap, int x, int y, uchar val){
    int i = 1;
    pmap->at<uchar>(y,x) = val;
    cv::Mat *obstacleMap = new cv::Mat(pmap->clone());
    *obstacleMap = *obstacleMap / 255;
    while(x+i < pmap->cols || x-i >= 0 || y+i < pmap->rows || y-i >= 0){
        val++;
        if(x+i < pmap->cols){
            for(int j = 0; j <= i-1;j++){
                pmap->at<uchar>(y-j,x+i) = val;
                pmap->at<uchar>(y+j,x+i) = val;
            }
        }
        if(y+i < pmap->rows){
            for(int j = 0; j <= i-1;j++){
                pmap->at<uchar>(y+i,x-j) = val;
                pmap->at<uchar>(y+i,x+j) = val;
            }
        }
        if(x-i >= 0){
            for(int j = 0; j <= i-1;j++){
                pmap->at<uchar>(y+j,x-i) = val;
                pmap->at<uchar>(y-j,x-i) = val;
            }
        }
        if(y-i >= 0){
            for(int j = 0; j <= i-1;j++){
                pmap->at<uchar>(y-i,x+j) = val;
                pmap->at<uchar>(y-i,x-j) = val;
            }
        }
        if(x+i < pmap->cols && y+i < pmap->rows)
            pmap->at<uchar>(y+i,x+i) = val;
        if(x-i >= 0 && y-i >= 0)
            pmap->at<uchar>(y-i,x-i) = val;
        if(x-i >= 0 && y+i < pmap->rows)
            pmap->at<uchar>(y+i,x-i) = val;
        if(y-i >= 0 && x+i < pmap->rows)
            pmap->at<uchar>(y-i,x+i) = val;
        i++;
    }
    *pmap = pmap->dot(*obstacleMap);
}

cv::Mat getPotentialMap(cv::Mat map,std::pair<int,int> end){
    cv::Mat* pmap = new cv::Mat(map.clone());
    int x = end.first;
    int y = end.second;
    bfs(pmap,x,y,char(1));
    return *pmap;
}
std::vector<std::pair<int,int>> waterfront_pathplanner(cv::Mat map, std::pair<int,int> start, std::pair<int,int> end){
    std::vector<std::pair<int,int>> pointList;
    cv::Mat pmap = getPotentialMap(map,end);
    return pointList;
}

int main(int argc, char* argv[]){
    //cv::Mat map_c = cv::imread("./map.png");
    //
    cv::Mat map = generateMap(200,200);
    getPotentialMap(map,std::pair<int,int>(100,100));
    while(1){
        cv::imshow("map", map);
        int key = cv::waitKey(5);
        if (key == 27) {
          break;
        }
        continue;
    }
    return 0;
}
