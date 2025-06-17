#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <thread>
#include <mutex>
#include <numeric>
#include <cmath>
#include <string>

using namespace std;
using namespace Eigen;

using PointType = pcl::PointXYZ;

std::string file_path;
int THR_NUM;

double computeEntropy(pcl::PointCloud<PointType>::Ptr cloud)
{
  Eigen::Vector4f centroid;
  Eigen::Matrix3f covarianceMatrixNormalized = Eigen::Matrix3f::Identity();
  pcl::compute3DCentroid(*cloud, centroid);
  pcl::computeCovarianceMatrixNormalized(*cloud, centroid, covarianceMatrixNormalized);
  double determinant = static_cast<double>(((2 * M_PI * M_E) * covarianceMatrixNormalized).determinant());
  return 0.5f * log(determinant);
}

void PC2Entropy(double& Entropy, int part_num)
{
  pcl::PointCloud<PointType>::Ptr full_cloud(new pcl::PointCloud<PointType>);
  pcl::io::loadPCDFile(file_path, *full_cloud);

  pcl::KdTreeFLANN<PointType> kdtree;
  kdtree.setInputCloud(full_cloud);
  double Entropy_ = 0;

  int partSize = full_cloud->points.size() / THR_NUM;
  pcl::PointCloud<PointType>::Ptr pc(new pcl::PointCloud<PointType>);
  for (size_t i = part_num * partSize; i < (part_num + 1) * partSize; i++)
    pc->points.push_back(full_cloud->points[i]);

  for (size_t i = 0; i < pc->points.size(); i++)
  {
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    int numberOfNeighbors = kdtree.radiusSearch(pc->points[i], 0.3, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    double localEntropy = 0;
    if (numberOfNeighbors > 15)
    {
      pcl::PointCloud<PointType>::Ptr localCloud(new pcl::PointCloud<PointType>);
      for (size_t iz = 0; iz < pointIdxRadiusSearch.size(); ++iz)
        localCloud->points.push_back(full_cloud->points[pointIdxRadiusSearch[iz]]);
      localEntropy = computeEntropy(localCloud);
    }
    Entropy_ += localEntropy;
  }

  Entropy = Entropy_ / pc->points.size();
  std::cout << "Thread " << part_num << " complete" << std::endl;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("calculate_mme");

  node->declare_parameter<std::string>("file_path", "");
  node->declare_parameter<int>("THR_NUM", 4);
  node->get_parameter("file_path", file_path);
  node->get_parameter("THR_NUM", THR_NUM);

  std::vector<std::thread*> mthreads(THR_NUM);
  std::vector<double> Entropys(THR_NUM);

  for (int i = 0; i < THR_NUM; i++)
    mthreads[i] = new std::thread(PC2Entropy, std::ref(Entropys[i]), i);

  for (int i = 0; i < THR_NUM; i++)
  {
    mthreads[i]->join();
    delete mthreads[i];
  }

  double sum = std::accumulate(Entropys.begin(), Entropys.end(), 0.0);
  double mean = sum / Entropys.size();
  double accum = 0.0;

  for (const double& d : Entropys)
    accum += (d - mean) * (d - mean);

  double stdev = std::sqrt(accum / (Entropys.size() - 1));
  std::cout << "MME mean: " << mean << " std: " << stdev << std::endl;

  rclcpp::shutdown();
  return 0;
}
