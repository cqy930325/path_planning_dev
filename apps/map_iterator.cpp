#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/cvstd.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>


void addTriangle(int x,int y,int level,cv::Mat &map){
    for(int i = 0; i < level; i++){
        for(int j = x - i; j <= x + i;j++){
            map.at<uchar>(y+i,j) = 0;
        }
    }
}

void addRectangle(std::pair<int,int> topLeft,std::pair<int,int> bottomRight,cv::Mat &map){
    for(int i = topLeft.first; i <= bottomRight.first;i++)
        for(int j = topLeft.second;j <= bottomRight.second;j++)
            map.at<uchar>(j,i) = 0;
}

cv::Mat generateMap(int x, int y){
    cv::Mat map = cv::Mat(x,y,CV_8U,255);
    //addTriangle(7,5,3,map);
    addRectangle(std::pair<int,int>(0,3),std::pair<int,int>(8,3),map);

    return map;
}

void dfs_help(cv::Mat *pamp,std::pair<int,int> cell,uchar val){
    int x = cell.first;
    int y = cell.second;
    std::cout << x << " " << y << std::endl;
    if(x >=0 && x < pamp->cols && y >= 0 && y < pamp->rows){
        if(pamp->at<uchar>(y,x) != 0){
            pamp->at<uchar>(y,x) = std::min(pamp->at<uchar>(y,x),val);
        }
    }
    else
        return;
    val++;
    dfs_help(pamp,std::pair<int,int>(y,x+1),val);
    std::cout << "gg" <<std::endl;
    dfs_help(pamp,std::pair<int,int>(y+1,x+1),val);
    std::cout << "hhh" <<std::endl;
    dfs_help(pamp,std::pair<int,int>(y-1,x+1),val);
    std::cout << "ffff" <<std::endl;
    dfs_help(pamp,std::pair<int,int>(y-1,x),val);
    std::cout << "iiii" <<std::endl;
    dfs_help(pamp,std::pair<int,int>(y+1,x),val);
    std::cout << "jjjj" <<std::endl;
    dfs_help(pamp,std::pair<int,int>(y-1,x-1),val);
    dfs_help(pamp,std::pair<int,int>(y+1,x-1),val);
    dfs_help(pamp,std::pair<int,int>(y,x-1),val);
}

void dfs(cv::Mat *pmap, std::pair<int,int> goal){
    cv::Mat *obstacleMap = new cv::Mat(pmap->clone());
    dfs_help(pmap,goal,1);
    //*pmap = pmap->mul(*obstacleMap);
}



void bfs(cv::Mat *pmap, int x, int y, uchar val){
    cv::Mat *obstacleMap = new cv::Mat(pmap->clone());
    int i = 1;
    pmap->at<uchar>(y,x) = val;
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
    *pmap = pmap->mul(*obstacleMap);
}

cv::Mat getPotentialMap(cv::Mat map,std::pair<int,int> goal){
    cv::Mat* pmap = new cv::Mat(map.clone());
    int x = goal.first;
    int y = goal.second;
    //bfs(pmap,x,y,char(1));
    dfs(pmap,goal);
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
    cv::Mat map = generateMap(14,14);
    getPotentialMap(map,std::pair<int,int>(9,7));
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
