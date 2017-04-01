/** PerceptIn license **/

#ifndef UTIL_INCLUDE_UTIL_KMEANS_HPP_
#define UTIL_INCLUDE_UTIL_KMEANS_HPP_

#include <random>
#include <unordered_set>

#include <glog/logging.h>

#include <util/debug.h>
#include <util/timer.h>

namespace PI {
namespace util {

template <typename TData>
const size_t KMeansClusterNode<TData>::kInvalidId =
    std::numeric_limits<size_t>::max();

/**
 * @brief Choose centers of clusters among the data.
 * @tparam TData Data type of each sample.
 * @tparam FDistance A function calculating distance between two samples in
 *                   this format:
 *            double Function(const TData &sample1, const TData &sample2);
 * @param data Data to group into clusters.
 * @param k Number of clusters.
 * @param[out] clusters Value at the centroid of the clusters.
 * @param distance_function Function calculating distance between two samples.
 * @return True if an initial set of clusters was created.
 */
template<typename TData, class FDistance>
bool InitiateKMeansClusters(const std::vector<TData> &data,
                            const size_t k,
                            std::vector<TData> *clusters,
                            FDistance distance_function) {
  CHECK_NOTNULL(clusters);
  clusters->clear();
  clusters->reserve(data.size());

  if (data.size() < k) {
    return false;
  }
  if (data.size() == k) {
    clusters->insert(clusters->end(), data.begin(), data.end());
    return true;
  }

  // KMeans++ seeding algorithm:
  // 1. Choose one center uniformly at random from among the sample points in
  //    data.
  // 2. For each sample x, compute the distance between x and the nearest center
  //    that has already been chosen.
  // 3. Add one new sample as a center. Each point x is chosen with probability
  //    proportional to D(x)^2.
  // 4. Repeat Steps 2 and 3 until k centers have been chosen.
  std::default_random_engine randomizer;
  std::unordered_set<size_t> used_idx_data;
  used_idx_data.reserve(k);

  // Uniformly sample among the sample to create the first cluster.
  std::uniform_int_distribution<size_t> random_idx_data(0, data.size() - 1);
  int idx_data = random_idx_data(randomizer);
  clusters->push_back(data[idx_data]);
  used_idx_data.insert(idx_data);

  while (clusters->size() < k) {
    // Compute distances between x and the nearest center that has already been
    // chosen.
    std::vector<double> min_squared_dists(
        data.size(), std::numeric_limits<double>::max());
    for (size_t i = 0; i < data.size(); i++) {
      for (auto const &cluster : *clusters) {
        const double dist = distance_function(data[i], cluster);
        if (dist * dist < min_squared_dists[i]) {
          min_squared_dists[i] = dist * dist;
        }
      }
    }

    // Add one new sample as center.
    const double sum_min_dists =
        std::accumulate(min_squared_dists.begin(), min_squared_dists.end(), 0.0);
    if (sum_min_dists < 1e-10) {
      return false;
    }

    std::uniform_real_distribution<double> random_dist(1e-10, sum_min_dists);
    double cut_dist = random_dist(randomizer);

    double dist_accumulate = 0;
    size_t idx_data = 0;
    for (idx_data = 0; idx_data < min_squared_dists.size(); idx_data++) {
      dist_accumulate += min_squared_dists[idx_data];
      if (dist_accumulate > cut_dist) {
        break;
      }
    }

    if (used_idx_data.insert(idx_data).second) {
      clusters->push_back(data[idx_data]);
    }
  }

  CHECK_EQ(clusters->size(), k);
  return true;
}

/**
 * @brief Choose centers of clusters among the data.
 * @tparam TData Data type of each sample.
 * @tparam FDistance A function calculating distance between two samples in
 *                   this format:
 *            double Function(const TData &sample1, const TData &sample2);
 * @param data Data to group into clusters.
 * @param[out] cluster Value at the centroid of the newly sampled cluster.
 * @param distance_function Function calculating distance between two samples.
 * @return True if a sample was created.
 */
template<typename TData, class FDistance>
bool ResampleKMeansCluster(const std::vector<TData> &data,
                           TData *cluster,
                           FDistance distance_function) {
  CHECK_NOTNULL(cluster);
  if (data.empty()) {
    return false;
  }
  std::default_random_engine randomizer;
  std::uniform_int_distribution<size_t> random_idx_data(0, data.size() - 1);
  *cluster = data[random_idx_data(randomizer)];
  return true;
}


/**
 * @brief Choose centers of clusters among the data.
 * @tparam TData Data type of each sample.
 * @tparam FDistance A function calculating distance between two samples in
 *                   this format:
 *            double Function(const TData &sample1, const TData &sample2);
 * @param data Data to group into clusters.
 * @param[out] clusters Value at the centroid of the clusters.
 * @param[out] assignments Index of the cluster where each sample belongs to.
 * @param[out] groups Indices of the samples in #data belonging to each group.
 * @param distance_function Function calculating distance between two samples.
 * @return True if a sample was created.
 */
template<typename TData, class FDistance>
void AssociateDataWithClusters(const std::vector<TData> &data,
                               const std::vector<TData> &clusters,
                               std::vector<size_t> *assignmens,
                               std::vector<std::vector<size_t>> *groups,
                               FDistance distance_function) {
  CHECK(!clusters.empty());
  CHECK_NOTNULL(assignmens);
  assignmens->clear();
  assignmens->resize(data.size(), clusters.size());
  CHECK_NOTNULL(groups);
  groups->clear();
  groups->resize(clusters.size(), std::vector<size_t>());

  // Assign each sample in data to a cluster.
  for (size_t idx_data = 0; idx_data < data.size(); idx_data++) {
    const size_t idx_cluster =
        KMeansTransform(data[idx_data], clusters, distance_function);
    assignmens->at(idx_data) = idx_cluster;
    groups->at(idx_cluster).push_back(idx_data);
  }
}

template<class TData, class FDistance, class FMean>
bool KMeansCluster(const std::vector<TData> &data,
                   const size_t k,
                   std::vector<TData> *clusters,
                   std::vector<size_t> *assignments,
                   std::vector<std::vector<size_t>> *groups,
                   FDistance distance_function,
                   FMean mean_function) {
  CHECK_GT(k, 0);
  CHECK_NOTNULL(clusters);
  clusters->clear();
  clusters->reserve(k);
  CHECK_NOTNULL(assignments);
  assignments->clear();
  assignments->reserve(data.size());
  CHECK_NOTNULL(groups);
  groups->clear();
  groups->reserve(k);

  // If the number of samples is less than or equal to the desired number of
  // clusters, can't cluster.
  if (data.size() <= k) {
    return false;
  }

  // Number of data points is greater than the number of clusters. Do KMeans.
  if (!InitiateKMeansClusters(data, k, clusters, distance_function)) {
    return false;
  }
  CHECK_EQ(clusters->size(), k);

  bool converged = false;
  std::vector<size_t> assignments_1(data.size(), k);
  std::vector<size_t> assignments_2(data.size(), k);
  std::vector<size_t> *assignments_current = &assignments_1;
  std::vector<size_t> *assignments_previous = &assignments_2;
  while (!converged) {
    AssociateDataWithClusters(
        data, *clusters, assignments_current, groups, distance_function);

    // Check convergence.
    CHECK_EQ(assignments_previous->size(), assignments_current->size());
    converged = true;
    for (unsigned int i = 0; i < assignments_current->size(); i++) {
      if (assignments_current->at(i) != assignments_previous->at(i)) {
        converged = false;
        break;
      }
    }

    // Calculate cluster centers according to the groupings.
    for (size_t c = 0; c < clusters->size(); c++) {
      if (groups->at(c).empty()) {
        CHECK(
            ResampleKMeansCluster(data, &(clusters->at(c)), distance_function));
        continue;
      }

      std::vector<TData> data_in_this_cluster;
      data_in_this_cluster.reserve(groups->at(c).size());
      for (auto const & idx_data : groups->at(c)) {
        data_in_this_cluster.push_back(data[idx_data]);
      }
      clusters->at(c) = mean_function(data_in_this_cluster);
    }

    // Current assignment becomes previous assignment.
    std::vector<size_t> *tmp = assignments_previous;
    assignments_previous = assignments_current;
    assignments_current = tmp;
  }

  // Prepare for output.
  for (auto const &i : *assignments_current) {
    assignments->push_back(i);
  }

  return true;
}

/**
 * @brief Construct a KMeans clustering tree from a set of data. This is a
 *        helper function for HierarchicalKMeansCluster().
 * @tparam TData Data type of each sample.
 * @tparam FDistance A function calculating distance between two samples in
 *                   this format:
 *            double Function(const TData &sample1, const TData &sample2);
 * @tparam FMean A function calculating the mean of a vector of samples in
 *               this format:
 *            TData Function(const std::vector<TData> &samples);
 * @param data Data to group into clusters.
 * @param branching_factor Branching factor of a layer in the KMeans tree.
 * @param max_depth Maximum depth of the KMeans tree.
 * @param distance_function Function calculating distance between two samples.
 * @param mean_function Function calculating the mean of a vector of samples.
 * @param[in, out] tree The root of a sub-tree.
 * @param current_depth The depth where the current sub-tree is at.
 * @param[in, out] next_id The ID to assign to the next generated node.
 */
template<class TData, class FDistance, class FMean>
void ConstructKMeansClusterTree(
    const std::vector<TData> &data,
    const size_t branching_factor,
    const size_t max_depth,
    FDistance distance_function,
    FMean mean_function,
    std::shared_ptr<KMeansClusterNode<TData>> tree,
    const size_t current_depth,
    size_t *next_id) {
  CHECK_GE(branching_factor, 2)
      << "For branching factor < 2, use regular K-means.";
  CHECK_GE(max_depth, 2) << "For depth < 2, use regular K-means.";
  CHECK(tree);
  CHECK_NOTNULL(next_id);

  // If this is the last level, assign ID to the tree, and then stop.
  if (current_depth == max_depth) {
    return;
  }

  // Use K-means to cluster the set of data.
  std::vector<TData> clusters;
  std::vector<size_t> assignments;
  std::vector<std::vector<size_t>> groups;
  if (!util::KMeansCluster(data, branching_factor, &clusters,
      &assignments, &groups, distance_function, mean_function)) {
    return;
  }
  CHECK_EQ(clusters.size(), groups.size());

  // Construct the child nodes.
  for (size_t g = 0; g < groups.size(); g++) {
    // Collect descriptors in this group.
    if (groups[g].empty()) {
      INTERNAL(LOG(ERROR) << "No data in cluster [" << g << "].");
      continue;
    }

    // Create the child node.
    std::shared_ptr<KMeansClusterNode<TData>> child =
        std::make_shared<KMeansClusterNode<TData>>();
    child->id = (*next_id)++;
    child->centroid = clusters[g];
    child->children.reserve(branching_factor);  // Reserve memory for children.

    // Further cluster the data in this cluster into groups if there are
    // sufficient number of data.
    const std::vector<size_t> &idx_data_in_group = groups[g];
    std::vector<TData> data_in_group;
    data_in_group.reserve(idx_data_in_group.size());
    for (const size_t idx_data : idx_data_in_group) {
      CHECK_LT(idx_data, data.size());
      data_in_group.push_back(data.at(idx_data));
    }
    ConstructKMeansClusterTree(data_in_group, branching_factor, max_depth,
        distance_function, mean_function, child, current_depth + 1, next_id);
    tree->children.push_back(child);
  }
}

template<class TData, class FDistance, class FMean>
std::shared_ptr<KMeansClusterNode<TData>> HierarchicalKMeansCluster(
    const std::vector<TData> &data,
    const size_t branching_factor,
    const size_t max_depth,
    FDistance distance_function,
    FMean mean_function) {
  ScopedMicrosecondTimer timer(__FUNCTION__);  // Time this function.

  CHECK_GE(branching_factor, 2)
      << "For branching factor < 2, use regular K-means.";
  CHECK_GE(max_depth, 2) << "For depth < 2, use regular K-means.";

  std::shared_ptr<KMeansClusterNode<TData>> tree =
      std::make_shared<KMeansClusterNode<TData>>();
  tree->id = 0;
  tree->centroid = mean_function(data);
  tree->children.reserve(branching_factor);  // Reserve memory for children.
  size_t next_id = tree->id + 1;
  ConstructKMeansClusterTree(data, branching_factor, max_depth,
      distance_function, mean_function, tree, 0, &next_id);
  return tree;
}

template<typename TData, class FDistance>
size_t KMeansTransform(const TData &sample,
                       const std::vector<TData> &clusters,
                       FDistance distance_function) {
  CHECK_GT(clusters.size(), 0);

  double best_dist = std::numeric_limits<double>::max();
  size_t best_idx_cluster = clusters.size();
  for (size_t idx_cluster = 0; idx_cluster < clusters.size(); idx_cluster++) {
    const double dist = distance_function(sample, clusters[idx_cluster]);
    if (dist < best_dist) {
      best_dist = dist;
      best_idx_cluster = idx_cluster;
    }
  }
  CHECK_LT(best_idx_cluster, clusters.size());
  return best_idx_cluster;
}

template<class TData, class FDistance>
std::shared_ptr<const KMeansClusterNode<TData>> KMeansTransform(
    const TData &sample,
    std::shared_ptr<const KMeansClusterNode<TData>> tree,
    FDistance distance_function) {
  CHECK(tree);

  std::shared_ptr<const KMeansClusterNode<TData>> node = tree;
  while (!node->children.empty()) {
    // Find the best cluster among its children.
    std::vector<TData> children_clusters;
    children_clusters.reserve(node->children.size());
    for (auto const &child : node->children) {
      CHECK(child);
      children_clusters.push_back(child->centroid);
    }
    const size_t idx_child =
        KMeansTransform(sample, children_clusters, distance_function);
    CHECK_LT(idx_child, node->children.size());
    node = node->children.at(idx_child);
  }
  return node;
}

}  // namespace util
}  // namespace PI

#endif  // UTIL_INCLUDE_UTIL_KMEANS_HPP_
