#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <slam/map/occupancy_grid_map/occupancy_grid_map.h>
#include <slam/map/occupancy_grid_map/syn_ogm.h>
#include <iostream>
#include <queue>
#include <map>
#include <glog/logging.h>
#include <set>

void addTriangle(int x, int y, int level, cv::Mat &map) {
    for (int i = 0; i < level; i++) {
        for (int j = x - i; j <= x + i; j++) {
            map.at<int>(y + i, j) = 1;
        }
    }
}

std::shared_ptr<PI::slam::ogm::OccupancyGridMap>
upScale(std::shared_ptr<PI::slam::ogm::OccupancyGridMap> ogm, const int factor) {
    CHECK_EQ(ogm->grid_size().width % factor, 0) << "width should be divided by factor exactly!";
    CHECK_EQ(ogm->grid_size().height % factor, 0) << "height should be divided by factor exactly!";
    cv::Size grid_size(ogm->grid_size().width / factor, ogm->grid_size().height / factor);
    std::shared_ptr<PI::slam::ogm::OccupancyGridMap> scale_ogm(
            new PI::slam::ogm::OccupancyGridMap(ogm->cell_size() * factor, grid_size));
    for (size_t i = 0; i < scale_ogm->grid_size().width; i++) {
        for (size_t j = 0; j < scale_ogm->grid_size().height; j++) {
            double val = 1;
            for (size_t k = i * factor; k < i * factor + factor; k++) {
                for (size_t l = j * factor; l < j * factor + factor; l++) {
                    double tmp_val;
                    ogm->GetValue(k, l, &tmp_val);
                    val *= tmp_val;
                }
            }
            val = val == 0 ? 0 : 1;
            scale_ogm->SetValue(i, j, val);
        }
    }
    return scale_ogm;
}

void addRectangle(std::pair<int, int> topLeft, std::pair<int, int> bottomRight, cv::Mat &map, int val) {
    for (int i = topLeft.first; i <= bottomRight.first; i++)
        for (int j = topLeft.second; j <= bottomRight.second; j++)
            map.at<int>(i, j) = val;
}

cv::Mat generateMap(int x, int y) {
    cv::Mat map = cv::Mat(x, y, CV_32S, 1);
    addRectangle(std::pair<int, int>(1, 1), std::pair<int, int>(x - 2, y - 2), map, 0);
    addRectangle(std::pair<int, int>(10, 5), std::pair<int, int>(30, 10), map, 1);
    addRectangle(std::pair<int, int>(20, 40), std::pair<int, int>(40, 100), map, 1);
    addTriangle(50, 50, 20, map);
    return map;
}

void binarize(std::shared_ptr<PI::slam::ogm::OccupancyGridMap> ogm, const double threshold) {
    CHECK(threshold >= 0 && threshold <= 1) << "invalid threshold";
    for (size_t i = 0; i < ogm->grid_size().width; i++) {
        for (size_t j = 0; j < ogm->grid_size().height; j++) {
            double val;
            ogm->GetValue(i, j, &val);
            val = val < threshold ? 0 : 1;
            ogm->SetValue(i, j, val);
        }
    }
}

std::shared_ptr<PI::slam::ogm::OccupancyGridMap> generateMap(cv::Mat m) {
    std::shared_ptr<PI::slam::ogm::OccupancyGridMap> map(
            new PI::slam::ogm::OccupancyGridMap(1, cv::Size(m.rows, m.cols)));
    for (int i = 0; i < m.cols; i++) {
        for (int j = 0; j < m.rows; j++) {
            map->SetValue(i, j, double(m.at<int>(i, j)));
        }
    }
    return map;
}

typedef struct node {
    PI::slam::ogm::CellIndex idx;
    int distance;
    double val;
    node *prev;
} node_t;

struct cmp {
    bool operator()(node_t *n1, node_t *n2) {
        return n1->distance > n2->distance;
    }
};


std::vector<node_t *> getNeighbors(std::map<PI::slam::ogm::CellIndex, node_t *> nodes, node_t *node) {
    std::vector<node_t *> neighbors;
    int x = int(node->idx.x_idx);
    int y = int(node->idx.y_idx);
    node_t *n;
    PI::slam::ogm::CellIndex idx = node->idx;
    idx.x_idx += 1;
    if (nodes.find(idx) != nodes.end()) {
        n = nodes[idx];
        if (n->val != 1)
            neighbors.push_back(n);
    }
    idx = node->idx;
    idx.y_idx += 1;
    if (nodes.find(idx) != nodes.end()) {
        n = nodes[idx];
        if (n->val != 1)
            neighbors.push_back(n);
    }
    idx = node->idx;
    idx.y_idx -= 1;
    if (nodes.find(idx) != nodes.end()) {
        n = nodes[idx];
        if (n->val != 1)
            neighbors.push_back(n);
    }
    idx = node->idx;
    idx.x_idx -= 1;
    if (nodes.find(idx) != nodes.end()) {
        n = nodes[idx];
        if (n->val != 1)
            neighbors.push_back(n);
    }

    return neighbors;
}

std::vector<node_t *> getNeighbors(std::vector<node_t *> nodes, cv::Size grid_size, node_t *node) {
    std::vector<node_t *> neighbors;
    int x = int(node->idx.x_idx);
    int y = int(node->idx.y_idx);
    node_t *n;
    if (x + 1 < grid_size.width) {
        n = nodes[y * grid_size.width + x + 1];
        if (n->val != 1)
            neighbors.push_back(n);
    }
    if (y + 1 < grid_size.height) {
        n = nodes[(y + 1) * grid_size.width + x];
        if (n->val != 1)
            neighbors.push_back(n);
    }
    if (x - 1 >= 0) {
        n = nodes[y * grid_size.width + x - 1];
        if (n->val != 1)
            neighbors.push_back(n);
    }
    if (y - 1 >= 0) {
        n = nodes[(y - 1) * grid_size.width + x];
        if (n->val != 1)
            neighbors.push_back(n);
    }
    return neighbors;
}

std::map<PI::slam::ogm::CellIndex, node_t *>
distanceTransform(std::shared_ptr<PI::slam::ogm::OccupancyGridMap> ogm, cv::Point2d source) {
    cv::Size size = ogm->grid_size();
    cv::Mat dT(size, CV_32S);
    std::priority_queue<node_t *, std::vector<node_t *>, cmp> unvisited;
    std::vector<node_t *> nodes;
    PI::slam::ogm::CellIndex *srcidx = new PI::slam::ogm::CellIndex;
    std::set<node_t *> visited;
    std::set<node_t *> unvisitedset;
    std::map<PI::slam::ogm::CellIndex, node_t *> map;
    if (!ogm->FindCell(source, srcidx)) {
        std::cerr << "invalid location" << std::endl;
    }
    for (size_t j = 0; j < size.height; j++) {
        for (size_t i = 0; i < size.width; i++) {

            PI::slam::ogm::CellIndex tmp(i, j);
            double tmp_val = 0;
            bool got = ogm->GetValue(tmp, &tmp_val);
            CHECK(got) << "invalid index access!" << std::endl;
            node_t *node = new node_t;
            if (tmp.x_idx == srcidx->x_idx && tmp.y_idx == srcidx->y_idx) {
                node->distance = 1;
                unvisited.push(node);
                unvisitedset.insert(node);
            } else {
                node->distance = 999999999;
            }
            node->idx = tmp;
            node->val = tmp_val;
            node->prev = NULL;
            nodes.push_back(node);
        }
    }

    while (!unvisited.empty()) {
        node_t *closest = unvisited.top();
        unvisited.pop();
        visited.insert(closest);
        std::vector<node_t *> neighbors = getNeighbors(nodes, ogm->grid_size(), closest);
        for (auto neighbor : neighbors) {
            if (visited.find(neighbor) != visited.end())
                continue;
            int newDis = closest->distance + 1;
            if (unvisitedset.find(neighbor) == unvisitedset.end()) {
                unvisitedset.insert(neighbor);
                unvisited.push(neighbor);
            } else if (newDis > neighbor->distance)
                continue;
            neighbor->distance = newDis;
            neighbor->prev = closest;

        }
    }
    for (auto node: nodes) {
        if (visited.find(node) != visited.end())
            map[node->idx] = node;

    }
    return map;
}

bool generatePath(std::shared_ptr<PI::slam::ogm::OccupancyGridMap> ogm, cv::Point2d start, cv::Point2d goal,
                  std::vector<PI::slam::ogm::CellIndex> *path) {
    PI::slam::ogm::CellIndex goalidx;
    if (!ogm->FindCell(start, &goalidx))
        return false;
    PI::slam::ogm::CellIndex startidx;
    if (!ogm->FindCell(start, &startidx))
        return false;
    double value;
    if (ogm->GetValue(startidx, &value)) {
        if (value == 1)
            return false;
    } else
        return false;
    if (ogm->GetValue(goalidx, &value)) {
        if (value == 1)
            return false;
    } else
        return false;
    std::map<PI::slam::ogm::CellIndex, node_t *> transformMap = distanceTransform(ogm, goal);
    std::cout << "distanceMap generated!" << std::endl;
    std::set<node_t *> unvisited;
    for (auto it = transformMap.begin(); it != transformMap.end(); it++)
        unvisited.insert(it->second);
    if (unvisited.size() <= 1) {
        std::cerr << "invalid start point node# = 0" << std::endl;
        return false;
    }
    std::cout << "#Nodes:" << unvisited.size() << std::endl;
    node_t *node = transformMap[startidx];
    bool isfirst = true;
    while (!unvisited.empty()) {
        unvisited.erase(node);
        std::vector<node_t *> neighbors = getNeighbors(transformMap, node);
        std::sort(neighbors.begin(), neighbors.end(),
                  [](node_t *n1, node_t *n2) { return n1->distance > n2->distance; });
        node_t *next = NULL;
        for (auto neighbor: neighbors) {
            if (unvisited.find(neighbor) != unvisited.end()) {
                next = neighbor;
                break;
            }
        }
        if (next == NULL) {
            if (isfirst)
                path->push_back(node->idx);
            isfirst = false;
            node = node->prev;
        } else {
            isfirst = true;
            next->prev = node;
            path->push_back(node->idx);
            node = next;
        }
    }
    return true;
}

bool visualize(std::shared_ptr<PI::slam::ogm::OccupancyGridMap> ogm, cv::Point2d start, cv::Point2d goal, int scale,
               cv::Mat &pathMap) {

    std::vector<PI::slam::ogm::CellIndex> *path = new std::vector<PI::slam::ogm::CellIndex>;
    if (!generatePath(ogm, start, goal, path))
        return false;
    std::cout << path->size() << std::endl;
    for (auto idx : *path) {
        ogm->SetValue(idx, 0.5);
    }
    cv::resize(ogm->Visualize(), pathMap, cv::Size(), scale, scale);
    return true;
}

void randomStartTest(int n) {
    std::default_random_engine rnd(n);
    std::shared_ptr<PI::slam::ogm::OccupancyGridMap> ogm;
    try {
        ogm = PI::slam::ogm::CreateRandomOgm(rnd);
    } catch (...) {
        std::cout << "random generator error" << std::endl;
        return;
    }
    while (ogm->grid_size().width % 2 == 0 && ogm->grid_size().height % 2 == 0) {
        ogm = upScale(ogm, 2);
        std::cout << "scaled by 2" << std::endl;
    }
    while (ogm->grid_size().width % 3 == 0 && ogm->grid_size().height % 3 == 0) {
        ogm = upScale(ogm, 3);
        std::cout << "scaled by 3" << std::endl;
    }
    while (ogm->grid_size().width % 5 == 0 && ogm->grid_size().height % 5 == 0) {
        ogm = upScale(ogm, 5);
        std::cout << "scaled by 5" << std::endl;
    }
    while (ogm->grid_size().width % 7 == 0 && ogm->grid_size().height % 7 == 0) {
        ogm = upScale(ogm, 7);
        std::cout << "scaled by 7" << std::endl;
    }
    std::cout << "test map#" << n << std::endl << ogm->grid_size() << std::endl;
    std::cout << "map_size:" << ogm->map_size() << std::endl;
    binarize(ogm, 0.2);
    cv::Mat pathMap;
    std::string filename;
    for (double i = -ogm->map_size().width / 2; i < ogm->map_size().width / 2; i += (ogm->map_size().width / 10)) {
        for (double j = -ogm->map_size().height / 2;
             j < ogm->map_size().height / 2; j += (ogm->map_size().height / 10)) {
            if (!visualize(ogm, cv::Point2d(i, j), cv::Point2d(i, j), 1, pathMap)) {
                std::cout << "can not start at(" << i << "," << j << "), go to next" << std::endl;
                continue;
            }
            filename = "./data/random_start_test_" + std::to_string(i) + "," + std::to_string(j) + ".png";
            cv::imwrite(filename, pathMap);
            std::cout << "save test image:" << filename << std::endl;
        }
    }
}

void randomMapTest() {
    for (int num = 0; num < 1000; num++) {
        std::default_random_engine rnd(num);
        std::shared_ptr<PI::slam::ogm::OccupancyGridMap> ogm;
        try {
            ogm = PI::slam::ogm::CreateRandomOgm(rnd);
        } catch (...) {
            std::cout << "random generator error" << std::endl;
            break;
        }
        while (ogm->grid_size().width % 2 == 0 && ogm->grid_size().height % 2 == 0) {
            ogm = upScale(ogm, 2);
            std::cout << "scaled by 2" << std::endl;
        }
        while (ogm->grid_size().width % 3 == 0 && ogm->grid_size().height % 3 == 0) {
            ogm = upScale(ogm, 3);
            std::cout << "scaled by 3" << std::endl;
        }
        while (ogm->grid_size().width % 5 == 0 && ogm->grid_size().height % 5 == 0) {
            ogm = upScale(ogm, 5);
            std::cout << "scaled by 5" << std::endl;
        }
        while (ogm->grid_size().width % 7 == 0 && ogm->grid_size().height % 7 == 0) {
            ogm = upScale(ogm, 7);
            std::cout << "scaled by 7" << std::endl;
        }
        std::cout << "test #" << num << std::endl << ogm->grid_size() << std::endl;
        binarize(ogm, 0.2);
        std::string filename = "./data/random_map_test_" + std::to_string(num) + ".png";
        cv::Mat pathMap;
        if (!visualize(ogm, cv::Point2d(0, 0), cv::Point2d(0, 0), 1, pathMap)) {
            std::cout << "can not start at 0,0 , go to next" << std::endl;
            break;
        }
        visualize(ogm, cv::Point2d(0, 0), cv::Point2d(0, 0), 1, pathMap);
        cv::imwrite(filename, pathMap);
        std::cout << "save test image" << std::endl;
    }
}

void checkMap(int n) {
    std::default_random_engine rnd(n);
    std::shared_ptr<PI::slam::ogm::OccupancyGridMap> ogm;
    ogm = PI::slam::ogm::CreateRandomOgm(rnd);
    while (1) {
        cv::imshow("map", ogm->Visualize());
        int key = cv::waitKey(5);
        if (key == 27)
            break;
        continue;
    }
}

int main(int argc, char *argv[]) {
    //checkMap(1024);
    randomMapTest();
    randomStartTest(1024);
    return 0;
}
