#include <ros/ros.h>
// PCL specific includes
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
//header for filtering
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
//header for clustering purpose
#include <pcl/segmentation/conditional_euclidean_clustering.h>
//header for subscribing to kinect
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//header for model coeffs-->optional
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
//header planar segmentation
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/radius_outlier_removal.h>

ros::Publisher pub;
ros::Publisher coef_pub, ind_pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ> voxel_filtered;
  pcl::PointCloud<pcl::PointXYZ> cloud_segmented;
  pcl::PointCloud<pcl::PointXYZ> in_future;

  pcl::fromROSMsg(*input, *cloud);

  // Get rid of points that are too far or close
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.8);
  pass.filter (*cloud_filtered);

  // Downsample + clean up data a bit
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(cloud_filtered);
  vg.setLeafSize(0.01, 0.01, 0.01);
  vg.filter(*cloud_filtered);

  // Planar segmentation
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setOptimizeCoefficients(true);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.01);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_table(new pcl::PointCloud<pcl::PointXYZ> ());

  // Segment the largest planar component from the remaining cloud
  seg.setInputCloud (cloud_filtered);
  seg.segment (*inliers, *coefficients);
  if (inliers->indices.size () == 0)
  {
    std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    // break;
  }

  // Extract the planar inliers from the input cloud
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*cloud_table);
  std::cout << "PointCloud representing the planar component: " << cloud_table->points.size () << " data points." << std::endl;

  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*cloud_filtered);

  // Find clusters
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.02); // 2cm
  ec.setMinClusterSize(100);
  ec.setMaxClusterSize(25000);
  ec.setInputCloud(cloud_table);
  ec.extract(cluster_indices);

  // Find largest cluster
  int largest_cluster_size = 0;
  int largest_idx = 0;
  for (int i=0; i < cluster_indices.size(); i++) {
    int cluster_size = cluster_indices[i].indices.size();
    if (cluster_size > largest_cluster_size) {
      largest_cluster_size = cluster_size;
      largest_idx = i;
    }
  }

  // Keep largest cluster
  pcl::PointIndices::Ptr largest_cluster(new pcl::PointIndices);
  largest_cluster->indices = cluster_indices[largest_idx].indices;
  extract.setInputCloud(cloud_table);
  extract.setIndices(largest_cluster);
  extract.setNegative(false);
  extract.filter(*cloud_table);

  // pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
  // outrem.setInputCloud(cloud_table);
  // outrem.setRadiusSearch(0.2);
  // outrem.setMinNeighborsInRadius(15);
  // outrem.filter (*cloud_table);

  // Project the model inliers
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setInputCloud(cloud_table);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);

  // Create a Concave Hull representation of the projected inliers
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ConvexHull<pcl::PointXYZ> chull;
  chull.setDimension(2);
  chull.setInputCloud(cloud_table);
  chull.reconstruct(*cloud_hull);

  pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
  prism.setInputCloud(cloud_filtered);
  prism.setInputPlanarHull(cloud_hull);
  prism.setHeightLimits(0, 0.1);
  prism.segment(*inliers);

  // pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*cloud_filtered);

  // cloud_filtered = cloud_projected;
  // Publish the model coefficients
  // pcl_msgs::ModelCoefficients ros_coefficients;
  // pcl_conversions::fromPCL(coefficients, ros_coefficients);
  // coef_pub.publish(ros_coefficients);

  // Publish the Point Indices
  // pcl_msgs::PointIndices ros_inliers;
  // pcl_conversions::fromPCL(*inliers, ros_inliers);
  // ind_pub.publish(ros_inliers);

  // Create the filtering object
  // pcl::ExtractIndices<pcl::PointXYZ> extract;
  // extract.setInputCloud(cloud);
  // extract.setIndices(inliers);
  // extract.setNegative(false);
  // extract.filter(cloud_segmented);

  //Publish the new cloud
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*cloud_filtered, output);
  output.header.frame_id = input->header.frame_id;
  output.header.stamp=ros::Time::now();
  pub.publish(output);
}


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "new_tuto");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output model coefficients
  //pub = nh.advertise<pcl_msgs::ModelCoefficients> ("output", 1);
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  coef_pub = nh.advertise<pcl_msgs::ModelCoefficients>("planar_coef", 1);
  ind_pub = nh.advertise<pcl_msgs::PointIndices>("point_indices", 1);
  // Spin
  ros::spin ();
}
