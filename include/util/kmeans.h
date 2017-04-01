/** PerceptIn license **/

#ifndef UTIL_INCLUDE_UTIL_KMEANS_H_
#define UTIL_INCLUDE_UTIL_KMEANS_H_

#include <memory>
#include <vector>

namespace PI {
namespace util {

/**
 * @brief Use K-means to group data into K clusters.
 * @tparam TData Data type of each sample.
 * @tparam FDistance A function calculating distance between two samples in
 *                   this format:
 *            double Function(const TData &sample1, const TData &sample2);
 * @tparam FMean A function calculating the mean of a vector of samples in
 *               this format:
 *            TData Function(const std::vector<TData> &samples);
 * @param data Data to group into clusters.
 * @param k Number of clusters.
 * @param[out] clusters Value at the centroid of each clusters.
 * @param[out] assignments Index of the cluster where each sample belongs to.
 * @param[out] groups Indices of the samples in #data belonging to each group.
 * @param distance_function Function calculating distance between two samples.
 * @param mean_function Function calculating the mean of a vector of samples.
 * @return True if data are clustered.
 */
template<class TData, class FDistance, class FMean>
bool KMeansCluster(const std::vector<TData> &data,
                   const size_t k,
                   std::vector<TData> *clusters,
                   std::vector<size_t> *assignments,
                   std::vector<std::vector<size_t>> *groups,
                   FDistance distance_function,
                   FMean mean_function);

/**
 * Define a node in a hierarchical KMeans clustering tree.
 */
template<typename TData>
struct KMeansClusterNode {
  // Define an invalid ID.
  static const size_t kInvalidId;

  /// The children of this node.
  std::vector<std::shared_ptr<KMeansClusterNode<TData>>> children;
  /// Cenroid of this cluster node.
  TData centroid;
  /// ID of this node. Default: kInvalidId.
  size_t id;

  KMeansClusterNode<TData>() :
      id(kInvalidId) {
  }
};

/**
 * @brief Use hierarchical K-means to cluster data into a KMeans clustering tree.
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
 * @return The root of the KMeans tree.
 */
template<class TData, class FDistance, class FMean>
std::shared_ptr<KMeansClusterNode<TData>> HierarchicalKMeansCluster(
    const std::vector<TData> &data,
    const size_t branching_factor,
    const size_t max_depth,
    FDistance distance_function,
    FMean mean_function);

/**
 * @brief Obtain which KMenas cluster a sample belongs to.
 * @tparam FDistance A function calculating distance between two samples in
 *                   this format:
 *            double Function(const TData &sample1, const TData &sample2);
 * @param data The query data.
 * @param clusters Centroid of the KMeans clusters.
 * @param distance_function Function calculating distance between two samples.
 * @return Index of the cluster where this sample belongs to.
 */
template<typename TData, class FDistance>
size_t KMeansTransform(const TData &sample,
                       const std::vector<TData> &clusters,
                       FDistance distance_function);

/**
 * @brief Obtain which cluster a sample belongs to in a KMeans clustering
 *        tree.
 * @tparam FDistance A function calculating distance between two samples in
 *                   this format:
 *            double Function(const TData &sample1, const TData &sample2);
 * @param data The query data.
 * @param tree The (root of) the KMeans clustering tree.
 * @param distance_function Function calculating distance between two samples.
 * @return The (leaf) node where this sample belongs to.
 */
template<class TData, class FDistance>
std::shared_ptr<const KMeansClusterNode<TData>> KMeansTransform(
    const TData &sample,
    std::shared_ptr<const KMeansClusterNode<TData>> tree,
    FDistance distance_function);

}  // namespace util
}  // namespace PI


// Add implementation of the functions.
#include <util/kmeans.hpp>

#endif  // UTIL_INCLUDE_UTIL_KMEANS_H_
