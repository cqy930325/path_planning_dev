#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/cvstd.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

cv::Mat generateMap(int x, int y){
    cv::Mat map = cv::Mat(x,y,CV_8U,255);
    std::cout <<map << std::endl;
    /*for(int i = 30; i < 70;i++)
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
    }*/
    return map;
}
void dfs(cv::Mat *pmap, int x, int y, uchar val){
    std::cout << x << "," << y << "," << int(val) << std::endl;
    if(x >= 0 && x < pmap->cols && y >= 0 && y < pmap->rows && pmap->at<uchar>(y,x) != 0)
        pmap->at<uchar>(y,x) = pmap->at<uchar>(y,x) > val ? val : pmap->at<uchar>(y,x);
    else
        return;
    dfs(pmap,x,y+1,val+1);
    dfs(pmap,x,y-1,val+1);
    dfs(pmap,x+1,y+1,val+1);
    dfs(pmap,x+1,y,val+1);
    dfs(pmap,x+1,y-1,val+1);
    dfs(pmap,x-1,y+1,val+1);
    dfs(pmap,x-1,y,val+1);
    dfs(pmap,x-1,y-1,val+1);
}

cv::Mat getPotentialMap(cv::Mat map,std::pair<int,int> end){
    cv::Mat* pmap = new cv::Mat(map);
    std::cout << *pmap << std::endl;
    int x = end.first;
    int y = end.second;
    dfs(pmap,x,y,char(1));
    std::cout << *pmap << std::endl;
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
    cv::Mat map = generateMap(3,3);
    getPotentialMap(map,std::pair<int,int>(1,1));
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
