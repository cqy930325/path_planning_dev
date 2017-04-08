//
// Created by perceptin on 4/7/17.
//

#include "./coverage_path_planning.h"

void AddTriangle(int x, int y, int level, cv::Mat *map) {
  for (int i = 0; i < level; ++i) {
    for (int j = x - i; j <= x + i; ++j) {
      map->at<int>(y + i, j) = 1;
    }
  }
}
void AddRectangle(std::pair<int, int> top_left, std::pair<int, int> bottom_right
  , cv::Mat *map, int val) {
  for (int i = top_left.first; i <= bottom_right.first; ++i)
    for (int j = top_left.second; j <= bottom_right.second; ++j)
      map->at<int>(i, j) = val;
}

cv::Mat GenerateMap(int x, int y) {
  cv::Mat map = cv::Mat(x, y, CV_32S, 1);
  AddRectangle(std::pair<int, int>(1, 1),
               std::pair<int, int>(x - 2, y - 2), &map, 0);
  AddRectangle(std::pair<int, int>(10, 5),
               std::pair<int, int>(30, 10), &map, 1);
  AddRectangle(std::pair<int, int>(20, 40),
               std::pair<int, int>(40, 100), &map, 1);
  AddTriangle(50, 50, 20, &map);
  return map;
}
bool Visualize(std::shared_ptr<PI::slam::ogm::OccupancyGridMap> ogm,
               cv::Point2d start, cv::Point2d goal, int scale,
               const cv::Mat &path_map) {
  std::vector<PI::slam::ogm::CellIndex> *path =
    new std::vector<PI::slam::ogm::CellIndex>;
  if (!GeneratePath(ogm, start, goal, path)) return false;
  std::cout << path->size() << std::endl;
  for (auto idx : *path) {
    ogm->SetValue(idx, 0.5);
  }
  cv::resize(ogm->Visualize(), path_map, cv::Size(), scale, scale);
  return true;
}
void RandomStartTest(int n) {
  std::default_random_engine rnd(n);
  std::shared_ptr<PI::slam::ogm::OccupancyGridMap> ogm;
  try {
    ogm = PI::slam::ogm::CreateRandomOgm(rnd);
  } catch (...) {
    std::cout << "random generator error" << std::endl;
    return;
  }
  while (ogm->grid_size().width % 2 == 0
    && ogm->grid_size().height % 2 == 0) {
    ogm = UpScale(ogm, 2);
    std::cout << "scaled by 2" << std::endl;  // Test upscale function
  }
  while (ogm->grid_size().width % 3 == 0
    && ogm->grid_size().height % 3 == 0) {
    ogm = UpScale(ogm, 3);
    std::cout << "scaled by 3" << std::endl;
  }
  while (ogm->grid_size().width % 5 == 0
    && ogm->grid_size().height % 5 == 0) {
    ogm = UpScale(ogm, 5);
    std::cout << "scaled by 5" << std::endl;
  }
  while (ogm->grid_size().width % 7 == 0
    && ogm->grid_size().height % 7 == 0) {
    ogm = UpScale(ogm, 7);
    std::cout << "scaled by 7" << std::endl;
  }
  std::cout << "test map#" << n << std::endl << ogm->grid_size() << std::endl;
  std::cout << "map_size:" << ogm->map_size() << std::endl;
  Binarize(ogm, 0.2);
  cv::Mat path_map;
  std::string file_name;
  // iterate 10*10 start points on the map
  for (double i = -ogm->map_size().width / 2;
       i < ogm->map_size().width / 2;
       i += (ogm->map_size().width / 10)) {
    for (double j = -ogm->map_size().height / 2;
         j < ogm->map_size().height / 2
      ; j += (ogm->map_size().height / 10)) {
      if (!Visualize(ogm, cv::Point2d(i, j),
                     cv::Point2d(i, j), 1, path_map)) {
        std::cout << "can not start at(" << i
                  << "," << j << "), go to next" << std::endl;
        continue;
      }
      file_name = "./data/random_start_test_" + std::to_string(i)
        + "," + std::to_string(j) + ".png";
      cv::imwrite(file_name, path_map);
      std::cout << "save test image:" << file_name << std::endl;
    }
  }
}

void RandomMapTest() {
  for (int num = 61; num < 1000; ++num) {
    std::default_random_engine rnd(num);
    std::shared_ptr<PI::slam::ogm::OccupancyGridMap> ogm;
    try {
      ogm = PI::slam::ogm::CreateRandomOgm(rnd);
    } catch (...) {
      std::cout << "random generator error" << std::endl;
      continue;
    }
    while (ogm->grid_size().width % 2 == 0
      && ogm->grid_size().height % 2 == 0) {
      ogm = UpScale(ogm, 2);
      std::cout << "scaled by 2" << std::endl;
    }
    while (ogm->grid_size().width % 3 == 0
      && ogm->grid_size().height % 3 == 0) {
      ogm = UpScale(ogm, 3);
      std::cout << "scaled by 3" << std::endl;
    }
    while (ogm->grid_size().width % 5 == 0
      && ogm->grid_size().height % 5 == 0) {
      ogm = UpScale(ogm, 5);
      std::cout << "scaled by 5" << std::endl;
    }
    while (ogm->grid_size().width % 7 == 0
      && ogm->grid_size().height % 7 == 0) {
      ogm = UpScale(ogm, 7);
      std::cout << "scaled by 7" << std::endl;
    }
    std::cout << "test #" << num << std::endl
              << ogm->grid_size() << std::endl;
    Binarize(ogm, 0.2);
    std::string file_name = "./data/random_map_test_"
      + std::to_string(num) + ".png";
    cv::Mat path_map;
    if (!Visualize(ogm, cv::Point2d(0, 0), cv::Point2d(0, 0), 1, path_map)) {
      std::cout << "can not start at 0,0 , go to next" << std::endl;
      continue;
    }
    cv::imwrite(file_name, path_map);
    std::cout << "save test image" << std::endl;
  }
}

void CheckMap(int n) {
  std::default_random_engine rnd(n);
  std::shared_ptr<PI::slam::ogm::OccupancyGridMap> ogm;
  try {
    ogm = PI::slam::ogm::CreateRandomOgm(rnd);
  } catch (...) {
    std::cout << "random generator error" << std::endl;
    return;
  }
  while (ogm->grid_size().width % 2 == 0
    && ogm->grid_size().height % 2 == 0) {
    ogm = UpScale(ogm, 2);
    std::cout << "scaled by 2" << std::endl;
  }
  while (ogm->grid_size().width % 3 == 0
    && ogm->grid_size().height % 3 == 0) {
    ogm = UpScale(ogm, 3);
    std::cout << "scaled by 3" << std::endl;
  }
  while (ogm->grid_size().width % 5 == 0
    && ogm->grid_size().height % 5 == 0) {
    ogm = UpScale(ogm, 5);
    std::cout << "scaled by 5" << std::endl;
  }
  while (ogm->grid_size().width % 7 == 0
    && ogm->grid_size().height % 7 == 0) {
    ogm = UpScale(ogm, 7);
    std::cout << "scaled by 7" << std::endl;
  }
  std::cout << "test #" << n << std::endl
            << ogm->grid_size() << std::endl;
  Binarize(ogm, 0.2);
  if (!StepVisualize(ogm, cv::Point2d(0, 0), cv::Point2d(0, 0), 10, 5)) {
    std::cout << "can not start at 0,0 , go to next" << std::endl;
    return;
  }
  std::cout << "exit" << std::endl;

}

int main(int argc, char *argv[]) {
  CheckMap(3);
  RandomMapTest();
  RandomStartTest(1024);
  return 0;
}


