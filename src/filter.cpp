#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/lexical_cast.hpp> 
#include <Eigen/Dense>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/common/centroid.h>

ros::Publisher pub;
ros::Publisher coef_pub, ind_pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg(*input, *cloud);

  // Get rid of points that are too far or close
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.8);
  pass.filter (*cloud);

  // Downsample + clean up data a bit
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(0.0075, 0.0075, 0.0075);
  //// not support in pcl 1.7.0 :(
  // vg.setMinimumPointsNumberPerVoxel(10);
  vg.filter(*cloud);

  // Planar segmentation
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setOptimizeCoefficients(true);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.02);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_table(new pcl::PointCloud<pcl::PointXYZ> ());

  // Segment the largest planar component from the remaining cloud
  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);

  // Extract the planar inliers from the input cloud
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*cloud_table);
  std::cout << "PointCloud representing the planar component: " << cloud_table->points.size () << " data points." << std::endl;

  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*cloud);

  // Isolate the table (assuming it's the largest cluster in our plane)
  std::vector<pcl::PointIndices> plane_cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> plane_ec;
  plane_ec.setClusterTolerance(0.02); // 2cm
  plane_ec.setMinClusterSize(100);
  plane_ec.setMaxClusterSize(25000);
  plane_ec.setInputCloud(cloud_table);
  plane_ec.extract(plane_cluster_indices);

  uint16_t largest_idx = 0;
  for (int i=1; i < plane_cluster_indices.size(); i++) {
    if (plane_cluster_indices[i].indices.size() > plane_cluster_indices[largest_idx].indices.size()) {
      largest_idx = i;
    }
  }

  pcl::PointIndices::Ptr largest_cluster(new pcl::PointIndices());
  largest_cluster->indices = plane_cluster_indices[largest_idx].indices;
  extract.setInputCloud(cloud_table);
  extract.setIndices(largest_cluster);
  extract.setNegative(false);
  extract.filter(*cloud_table);

  // Project the model inliers
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setInputCloud(cloud_table);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);

  // Isolate objects on the table
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ConvexHull<pcl::PointXYZ> chull;
  chull.setDimension(2);
  chull.setInputCloud(cloud_table);
  chull.reconstruct(*cloud_hull);

  pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
  prism.setInputCloud(cloud);
  prism.setInputPlanarHull(cloud_hull);
  prism.setHeightLimits(0, 0.1);
  prism.segment(*inliers);

  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*cloud);

  // Remove isolated points (probably noise)
  // pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
  // outrem.setInputCloud(cloud);
  // outrem.setRadiusSearch(0.03);
  // outrem.setMinNeighborsInRadius(5);
  // outrem.filter(*cloud);

  // Search for marshmallows
  std::vector<pcl::PointIndices> mm_cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> mm_ec;
  mm_ec.setClusterTolerance(0.02); // 2cm
  mm_ec.setMinClusterSize(20);
  mm_ec.setMaxClusterSize(1000);
  mm_ec.setInputCloud(cloud);
  mm_ec.extract(mm_cluster_indices);

  static tf::TransformBroadcaster br;
  std::string frame = input->header.frame_id;

  for (int i=0; i < mm_cluster_indices.size(); i++) {
    if (mm_cluster_indices[i].indices.size() < 10)
      break;

    Eigen::Vector4f min_pt;
    Eigen::Vector4f max_pt;
    pcl::getMinMax3D(*cloud, mm_cluster_indices[i].indices, min_pt, max_pt);

    Eigen::Vector4f center = (min_pt + max_pt) / 2.0;
    float x = center[0];
    float y = center[1];
    float z = center[2];
    std::cout << x << "|" << y << "|" << z << std::endl;

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, z));
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    std::string mm_name = "marshmellow_" + boost::lexical_cast<std::string>(i);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame, mm_name));

    // pcl::CentroidPoint<pcl::PointXYZ> centroid;
    // for (int j = 0; j < mm_cluster_indices[i].indices.size(); j++) {
    //   centroid.add(cloud->points[mm_cluster_indices[i].indices[j]]);
    // }
    // pcl::PointXYZ c;
    // centroid.get(c);
    // std::cout << c.x << "|" << c.y << "|" << c.z << std::endl;
  }

  //Publish the new cloud
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*cloud, output);
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
